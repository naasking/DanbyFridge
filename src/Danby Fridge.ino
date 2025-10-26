#include <DHT.h>
#include <Adafruit_ST7735.h>
#include <Fonts/FreeSans12pt7b.h>
#include <SPI.h>                 // ESP32 core SPI
#include <Preferences.h>         // for saving data
#include "rotary.h"
#include <esp_sleep.h>
#include <esp_pm.h>
#include "driver/rmt.h"
#include "soc/rmt_reg.h"
#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"
#include "async.h"

#define DHTTYPE DHT22
// Preferences instance for non-volatile storage on ESP32
static Preferences prefs;

// === ESP32-C3 Super mini Pin Mapping ===
#define DHTPIN          10  // Safe
#define RELAY_PIN       2   // Safe -- connected to HW-040 relay module

// Rotary encoder -- grouped on the right side
#define ROTARY_SW       9  // Strapping: rotary encoder button press at start boots into download mode
#define ROTARY_CLK      7  // Safe
#define ROTARY_DT       6  // Safe

// TFT SPI -- grouped on the left side
#define TFT_DC          0   // Safe
#define BACKLIGHT_PIN   1   // Strapping: defaults HIGH
#define TFT_CS          3   // Safe
#define TFT_MOSI        4   // Safe (SDA on TFT7735)
#define TFT_SCK         5   // Safe (SCL on TFT7735)
#define TFT_MISO       -1   // Not used

// === Notes ===
// - All pins are 3.3V only (no 5V tolerance).

#define DISPLAY_UPDATE_INTERVAL 200
#define DHT_READ_INTERVAL 1000
#define CONTROL_INTERVAL 10000
#define HYSTERESIS_TENTHS 5      // 0.5°C hysteresis
#define ENCODER_STEP_TENTHS 1    // 0.1°C per encoder step
#define ENCODER_PULSES_PER_STEP 1 // quadrature pulses per mechanical detent (tune if needed)
#define BUTTON_DEBOUNCE_MS 50
#define LONG_PRESS_MS 1000 // ms for long press to toggle units

/* Persistence (Preferences) */
#define SAVE_INTERVAL_MS 5000    // minimum interval between non-volatile writes

// Power management
#define WDT_SLEEP_S 8            // WDT sleep interval in seconds (LowPower SLEEP_8S)
#define CONTROL_INTERVAL_S 10    // control interval in seconds (approximately)
#define DISPLAY_ON_AFTER_WAKE_MS 3000 // keep display on for some time after wake for user feedback
#define COMPRESSOR_MIN_OFF_MS (4UL * 60UL * 1000UL) // 4 minutes minimum off time for compressor starter
/* DHT read retry settings and RMT parsing thresholds (tunable) */
#define DHT_MAX_RETRIES 3
//#define DHT_RETRY_DELAY_MS 200
#define DHT_RETRY_DELAY_MS 2000
#define DHT_MAX_RETRIES    3
#define DHT_TIMEOUT_MS     300
#define DHT_RCV_ITEMS      100

/* RMT parsing thresholds (microseconds) */
#define DHT_RMT_HIGH_MIN_US 20        // ignore highs shorter than this
#define DHT_RMT_ONE_THRESHOLD_US 50   // high-duration >= this -> bit=1
#define DHT_RMT_TIMEOUT_MS 300        // timeout for a single RMT attempt (ms)
#define DHT22_BITS 40                 // bits in the RMT stream

#define println(text)
//#define println(text) if (Serial && Serial.isConnected()) Serial.println(text)

DHT dht(DHTPIN, DHTTYPE);
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, -1); // RST tied HIGH on module; pass -1 to disable software reset

 // Internal state stored in tenths of °C to avoid non-atomic float access
volatile int16_t targetTenthsC = 250; // tenths of a degree C = 25.0°C

// Quadrature decoder state (4-bit history) and pulse accumulator.
// ISR calls rotary_step() and increments encoderPulseCount.
volatile uint8_t rot_state = 0;
volatile int16_t encoderPulseCount = 0;
 
 // Persistence helpers
unsigned long lastSaveMs = 0;

// status codes for the DHT subsystem
typedef enum  {
  DHT_OK                   =  0, // device in a good state
  DHT_UNINITIALIZED        =  1, // device uninitialized
  DHT_ERR_DRIVER_INSTALL   =  2, // couldn't install driver
  DHT_ERR_RMT_CONFIG       =  3, // couldn't configure RMT
  DHT_ERR_BUFFER           =  4, // couldn't allocate buffer
  DHT_ERR_RMT_START        =  5, // Failed to start RMT RX
  DHT_ERR_NO_ITEMS         =  6, // Ringbuffer returned NULL or len == 0
  DHT_ERR_NOT_ENOUGH_ITEMS =  7, // Too few items to represent a full frame
  DHT_ERR_CHECKSUM         =  8, // Checksum mismatch
  DHT_ERR_PARSE            =  9, // Parse failed (timing/format error)
  DHT_ERR_TIMEOUT          = 10, // Timed out waiting for data
  DHT_ERR_RETRIES_EXHAUSTED= 11, // All retries used up without success
} DHT_STATUS;

// Compressor/relay safety
bool relayOn = false;
unsigned long lastRelayOffMs = 0;

// DHT read state helpers
static bool controlReadPending = false;
bool displayCelsius = true;
unsigned long lastDHTReadMs = 0;
unsigned long displayOnUntilMs = 0;
unsigned long wdtWakeCount = 0;
float lastValidTempC = NAN;
bool lastButtonState = HIGH;
unsigned long lastButtonMs = 0;
unsigned long pressStartMs = 0;

// fixed width text bounds
uint16_t textWidth, textHeight;

typedef struct {
    async_state;
    int retriesLeft;
    unsigned long lastAttemptMs;
    RingbufHandle_t rb;
    DHT_STATUS status;
    rmt_channel_t channel;
    size_t itemSize;
    rmt_item32_t* items;
} AsyncDhtState;

AsyncDhtState dhtState = { .status = DHT_UNINITIALIZED };

void initDhtRmt() {
    rmt_config_t rmt_rx = {};
    rmt_rx.rmt_mode = RMT_MODE_RX;
    rmt_rx.channel = RMT_CHANNEL_2;
    rmt_rx.gpio_num = (gpio_num_t)DHTPIN;
    rmt_rx.clk_div = 80; // 1us resolution
    rmt_rx.mem_block_num = 2;

    rmt_rx.rx_config.filter_en = true;
    rmt_rx.rx_config.filter_ticks_thresh = 10;  // 10 µs, keeps 50us pulses
    rmt_rx.rx_config.idle_threshold = 10000;    // 10 ms

    // Apply config and install driver once
    size_t rcvBufferSize = RMT_MEM_ITEM_NUM * rmt_rx.mem_block_num * sizeof(rmt_item32_t);
    if (rmt_config(&rmt_rx) != ESP_OK) {
      dhtState.status = DHT_ERR_RMT_CONFIG;
    } else if (rmt_driver_install(rmt_rx.channel, rcvBufferSize, 0) != ESP_OK) {
      dhtState.status = DHT_ERR_DRIVER_INSTALL;
    } else if (rmt_get_ringbuf_handle(rmt_rx.channel, &dhtState.rb) != ESP_OK || !dhtState.rb) {
      dhtState.status = DHT_ERR_BUFFER;
    } else {
      dhtState.status = DHT_OK;
    }
    // initialize the async state
    async_init(&dhtState);
}

void IRAM_ATTR encoderISR() {
  // Minimal ISR: use rotary_step_s16 to update encoderPulseCount directly.
  // rotary_step_s16 expects (count, state, rotb, rota).
  // Read DT (rotb) first, then CLK (rota), and pass in that order to avoid swapped inputs.
  uint8_t a = digitalRead(ROTARY_DT);  // rota (LSB)
  uint8_t b = digitalRead(ROTARY_CLK); // rotb (MSB)
  rotary_step_s16(&encoderPulseCount, &rot_state, b, a);
}

// Wake-only ISR for the rotary button: intentionally empty so its only effect
// is to wake the MCU from sleep. Debounce and button handling remain in loop().
void IRAM_ATTR wakeISR() {
  // no-op: wake-only
}

void setup() {
  //Serial.begin(115200, SERIAL_8N1, -1);
  // Serial.begin(115200);
  // Serial.setTxTimeoutMs(0);
  //delay(3000);
  //println("Program started...");

  // Initialize RMT for DHT non-blocking reads once (driver + ringbuffer)
  initDhtRmt();

  // Initialize SPI with explicit pins for ESP32-C3
  SPI.begin(TFT_SCK, TFT_MISO, TFT_MOSI, TFT_CS);

  // Ensure control pins are configured and stable before calling library init.
  pinMode(TFT_CS, OUTPUT);
  pinMode(TFT_DC, OUTPUT);

  // Initialize SPI with a slower clock speed so it's more stable, otherwise
  // the scren sometimes blanks out when receiving many rotary pulses.
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
  digitalWrite(TFT_CS, HIGH);
  digitalWrite(TFT_DC, HIGH);
  SPI.endTransaction();

  delay(10); // allow peripheral to settle

  // Call library init (use MINI160x80 PLUGIN for this module)
  tft.initR(INITR_MINI160x80_PLUGIN);
  tft.setFont(&FreeSans12pt7b);
  {
    int16_t x, y;
    char text[] = "-CC.CCC";
    tft.getTextBounds(text, 0, 100, &x, &y, &textWidth, &textHeight);
  }
  tft.setTextSize(1);
  tft.setRotation(0);
  tft.enableDisplay(true);
  tft.fillScreen(ST7735_BLACK);

  pinMode(RELAY_PIN, OUTPUT);
  pinMode(ROTARY_CLK, INPUT_PULLUP);
  pinMode(ROTARY_DT, INPUT_PULLUP);
  pinMode(ROTARY_SW, INPUT_PULLUP);

  // Backlight control pin (module BLK has onboard transistor)
  pinMode(BACKLIGHT_PIN, OUTPUT);
  digitalWrite(BACKLIGHT_PIN, HIGH);

  unsigned long now = millis();
  displayOnUntilMs = now + DISPLAY_ON_AFTER_WAKE_MS;

  attachInterrupt(digitalPinToInterrupt(ROTARY_CLK), encoderISR, CHANGE);
  // Also attach to DT so the ISR runs on changes of either channel (better capture on fast/cheap encoders)
  attachInterrupt(digitalPinToInterrupt(ROTARY_DT), encoderISR, CHANGE);
  // Wake-only interrupt for rotary pushbutton (FALLING edge). ISR does nothing
  // beyond waking the MCU; actual button processing is done in the loop.
  attachInterrupt(digitalPinToInterrupt(ROTARY_SW), wakeISR, FALLING);

  // initialize relay OFF (assumes active HIGH; invert logic if needed)
  //FIXME: should maybe check if relay is already on and needs to be?
  digitalWrite(RELAY_PIN, LOW);

  lastDHTReadMs = now;

  // Load persisted target temperature (tenths of °C). Validate range, otherwise set to current measured temp if available.
  prefs.begin("fridge", false);

  // Try to load saved target
  int16_t saved = prefs.getShort("targetC", INT16_MIN);
  targetTenthsC = saved >= -400 && saved <= 500
    ? saved
    : 250;  // default 25.0C

  // keep Preferences open for the lifetime of the program (do not end here)
  // prefs.end(); // removed to keep prefs active

  // initialize save timestamp to avoid immediate write
  lastSaveMs = now;

  // ensure display shows current values at startup
  updateDisplay(lastValidTempC, targetTenthsC);
}

void show(float temp, int16_t x, int16_t y, uint16_t color) {
  // bounds calculation is slightly off
  tft.setTextColor(color);
  tft.fillRect(x, y - textHeight, textWidth + 20, textHeight + 1, ST77XX_BLACK);
  if (isnan(temp)) {
    // error code should print out 7 chars to match expected length
    tft.setCursor(x, y);
    tft.printf("E:%04d", dhtState.status);
  } else {
    char buf[16];
    char symbol = displayCelsius ? 'C' : 'F';
    if (!displayCelsius) {
      temp = temp * 9.0 / 5.0 + 32.0;
    }
    // numeric field: width 5 (space/sign + up to 2 digits + '.' + 1 decimal) + " C" = 7 chars
    snprintf(buf, sizeof(buf), "%5.1f %c", temp, symbol);
    // right-align the text
    int16_t x1, y1;
    uint16_t w, h;
    tft.getTextBounds(buf, x, y, &x1, &y1, &w, &h);
    tft.setCursor(78 - w, y);
    tft.print(buf);
    tft.drawCircle(x + textWidth - 4, y - textHeight / 2 + 6, 2, color);
  }
}

void updateDisplay(float currentC, int16_t targetTenths) {
  // Full-screen redraw but use color to distinguish current (white) and target (light blue).
  //println("updating display...");

  // Target temperature (right) in light blue - also fixed-width
  uint16_t lightBlue = tft.color565(120, 120, 255); // light blue (RGB: 120,120,255)
  float tgtC = targetTenths / 10.0;
  show(tgtC, 0, 50, lightBlue);

  // Current temperature (left) - print fixed-width numeric + unit to avoid residual chars
  show(currentC, 0, 140, ST77XX_WHITE);

  // Compressor running indicator (snowflake) between the values
  // Draw a simple snowflake icon centered at (80, 20)
  int cx = 40, cy = 80, size = 10;
  if (relayOn) {
    uint16_t color = lightBlue;
    tft.drawPixel(cx, cy, color);

    // main arms (horizontal & vertical)
    tft.drawLine(cx - size, cy, cx + size, cy, color); // horizontal
    tft.drawLine(cx, cy - size, cx, cy + size, color); // vertical

    // diagonals (scaled to ~70% of size for aesthetics)
    int diag = size * 0.7;
    tft.drawLine(cx - diag, cy - diag, cx + diag, cy + diag, color);
    tft.drawLine(cx - diag, cy + diag, cx + diag, cy - diag, color);

    // small branches: one pixel offset perpendicular to each arm end
    // Horizontal arms
    int bsize = size / 3;
    tft.drawPixel(cx - size, cy - bsize, color);
    tft.drawPixel(cx - size, cy + bsize, color);
    tft.drawPixel(cx + size, cy - bsize, color);
    tft.drawPixel(cx + size, cy + bsize, color);

    // Vertical arms
    tft.drawPixel(cx - bsize, cy - size, color);
    tft.drawPixel(cx + bsize, cy - size, color);
    tft.drawPixel(cx - bsize, cy + size, color);
    tft.drawPixel(cx + bsize, cy + size, color);
  } else {
    // If compressor off, optionally draw a faint outline or nothing. We'll draw nothing.
  }
}

async dhtRead(AsyncDhtState* s, unsigned long now, float* temperature, float* humidity) {
    async_begin(s);
    // initialize retry state
    s->retriesLeft = 3;
    while (s->retriesLeft > 0) {
      // try every second so I can monitor the error codes
      await(now - s->lastAttemptMs >= 1000);
      s->lastAttemptMs = now;

      // Start the RMT transaction (non-blocking)
      if (rmt_rx_start(s->channel, true) != ESP_OK) {
          s->retriesLeft--;
          s->status = DHT_ERR_RMT_START;
          async_yield; // let caller loop again
          continue;
      }
      // --- Wait for RMT data in ringbuffer ---
      await((s->items = (rmt_item32_t*)xRingbufferReceive(s->rb, &s->itemSize, 0)) != NULL);

      // Stop RX and parse
      rmt_rx_stop(s->channel);

      if (s->itemSize > 0) {
        s->status = dht22_parse_items(s->items, s->itemSize / sizeof(rmt_item32_t), temperature, humidity);
        vRingbufferReturnItem(s->rb, s->items);
        if (s->status == DHT_OK)
          async_exit; // success, end coroutine
      } else {
        s->status = DHT_ERR_NO_ITEMS;
        vRingbufferReturnItem(s->rb, s->items);
      }
      // failed parse or item extraction, decrement retries
      s->retriesLeft--;
      async_yield; // yield before retry
    }
    // If we reach here, all retries failed
    s->status = DHT_ERR_RETRIES_EXHAUSTED;
    async_end;
}

// Return DHT status enum indicating success or failure reason
DHT_STATUS dht22_parse_items(rmt_item32_t* items, size_t itemsCount,
                            float* temperature, float* humidity) {
    if (itemsCount < (DHT22_BITS * 2 + 4)) {
        return DHT_ERR_NOT_ENOUGH_ITEMS;
    }
    uint8_t data[5] = {0};

    // Skip initial response pulses (~80us low + 80us high)
    size_t idx = 2;
    for (int bit = 0; bit < DHT22_BITS; bit++, idx += 2) {
        if (idx + 1 >= itemsCount)
          return DHT_ERR_NOT_ENOUGH_ITEMS;
        // Each bit: items[idx] = low (~50us), items[idx+1] = high (26us or 70us)
        uint32_t high_ticks = items[idx+1].duration0;
        // Convert ticks to microseconds (assuming 1 tick = 1us if RMT clock set that way)
        uint32_t high_us = high_ticks;

        int value = (high_us > 40) ? 1 : 0; // threshold ~40us
        data[bit / 8] <<= 1;
        data[bit / 8] |= value;
    }
    // Verify checksum
    uint8_t sum = data[0] + data[1] + data[2] + data[3];
    if (sum != data[4]) {
        return DHT_ERR_CHECKSUM;
    }
    // Humidity: 16 bits
    uint16_t raw_hum = (data[0] << 8) | data[1];
    // Temperature: 16 bits, MSB may be sign
    uint16_t raw_temp = (data[2] << 8) | data[3];

    *humidity = raw_hum / 10.0f;
    if (raw_temp & 0x8000) {
        raw_temp &= 0x7FFF;
        *temperature = -(raw_temp / 10.0f);
    } else {
        *temperature = raw_temp / 10.0f;
    }
    return DHT_OK;
}

void controlTemperature(unsigned long now, float currentC, int16_t targetTenths) {
  // If the reading is invalid, skip control to avoid spurious changes.
  if (isnan(currentC))
    return;

  int16_t currentTenths = (int16_t)round(currentC * 10.0);

  // Need to turn ON when current is below target minus hysteresis.
  if (currentTenths <= (targetTenths - HYSTERESIS_TENTHS)) {
    // Only allow starting compressor if it is currently off and the minimum off time has elapsed.
    if (!relayOn) {
      if ((now - lastRelayOffMs) >= COMPRESSOR_MIN_OFF_MS) {
        digitalWrite(RELAY_PIN, HIGH);
        relayOn = true;
      }
    }
  } else if (currentTenths >= (targetTenths + HYSTERESIS_TENTHS)) {
    // Need to turn OFF when current is above target plus hysteresis.
    if (relayOn) {
      digitalWrite(RELAY_PIN, LOW);
      relayOn = false;
      lastRelayOffMs = now;
    }
  }
}

void loop() {
  bool targetDirty = false;
  bool displayDirty = false;
  unsigned long now = millis();
  int16_t pulses = 0;

  // Consume encoder pulses produced by the quadrature ISR and convert pulses -> steps
  {
    noInterrupts();
    pulses = encoderPulseCount;
    encoderPulseCount = 0;
    interrupts();

    if (pulses != 0) {
      static int16_t remainder = 0;
      remainder += pulses;
      int16_t steps = remainder / ENCODER_PULSES_PER_STEP;
      remainder = remainder % ENCODER_PULSES_PER_STEP;

      if (steps != 0) {
        int16_t newTarget = targetTenthsC + (int16_t)(steps * ENCODER_STEP_TENTHS);
        if (newTarget < -400) newTarget = -400;
        if (newTarget > 500)  newTarget = 500;
        if (newTarget != targetTenthsC) {
          targetTenthsC = newTarget;
          targetDirty = true;
          displayDirty = true;
        }
      }
    }
  }

  // Persist target temperature occasionally (rate-limited to SAVE_INTERVAL_MS)
  if (targetDirty && (now - lastSaveMs >= SAVE_INTERVAL_MS)) {
    int16_t toSave = targetTenthsC; // local copy
    println("Save target C");
    // Preferences is already opened in setup and kept open; just write the value
    prefs.putShort("targetC", toSave);
    lastSaveMs = now;
  }

  // button press to wake board from sleep and turn on screen
  // long-press to switch temperature units
  bool sw = digitalRead(ROTARY_SW);
  if (sw == LOW) {
    if (lastButtonState == HIGH && (now - lastButtonMs) > BUTTON_DEBOUNCE_MS) {
      // Debounced button press
      pressStartMs = now;
      lastButtonMs = now;
      displayDirty = true;
    }
    if ((now - pressStartMs) >= LONG_PRESS_MS) {
      // Long press: toggle units
      displayCelsius = !displayCelsius;
      pressStartMs = now; // reset the hold period
      displayDirty = true;
    }
  }
  if (sw == HIGH && lastButtonState == LOW && (now - lastButtonMs) > BUTTON_DEBOUNCE_MS) {
    // Button released
    lastButtonMs = now;
  }
  lastButtonState = sw;
  
  // Keep display on for a short period after any updates for user feedback
  bool displayOn = displayOnUntilMs - now < DISPLAY_ON_AFTER_WAKE_MS;
  digitalWrite(BACKLIGHT_PIN, displayOn ? HIGH : LOW);
  
  //  Put MCU to low-power sleep if idle: wake sources are external INT (encoder/button) and timer (approx WDT)
  //  Only sleep if no recent activity and display not required.
  bool idle = (pulses == 0) && !targetDirty && !displayDirty;
  if (!idle) {
    displayOnUntilMs = now + DISPLAY_ON_AFTER_WAKE_MS;
  } else if (!displayOn) {
    // // Configure timer wake for approximately WDT_SLEEP_S seconds (esp_light_sleep)
    // esp_sleep_enable_timer_wakeup((uint64_t)WDT_SLEEP_S * 1000000ULL);
    // // Enter light sleep; external interrupts attached with attachInterrupt() will also wake the chip
    // esp_light_sleep_start();
    // Woke up (either via timer or external INT)
    // wdtWakeCount++;
    // keep display on briefly after wake for feedback
    // displayOnUntilMs = now + DISPLAY_ON_AFTER_WAKE_MS;
  }

  // // Start non-blocking DHT read sequence when interval reached, and poll progress
  // if (now - lastDHTReadMs >= DHT_READ_INTERVAL) {
  //   startDhtRetries();
  //   lastDHTReadMs = now;
  // }

  // // Poll any in-progress non-blocking DHT attempt (RMT); handle initial/control outcomes
  // {
  //   float dhtTemp;
  //   if (dhtRetryActive) {
  //     if (pollDhtRetries(now, &dhtTemp)) {
  //       if (!isnan(dhtTemp)) {
  //         lastValidTempC = dhtTemp;
  //         if (controlReadPending) {
  //           controlReadPending = false;
  //           controlTemperature(now, lastValidTempC, targetTenthsC);
  //           updateDisplay(lastValidTempC, targetTenthsC);
  //         }
  //       } else {
  //         //println("DHT read failed (retries)");
  //         // clear pending flags on failure
  //         controlReadPending = false;
  //       }
  //     }
  //   }
  // }

  // // 5) Control relay periodically using WDT wake count approximation
  // // WDT interval roughly WDT_SLEEP_S seconds per LowPower wake.
  // // Add elapsed seconds since last WDT wake (approx)
  // // Use wdtWakeCount as a coarse timer: multiply by WDT_SLEEP_S
  // unsigned long elapsedS = wdtWakeCount * WDT_SLEEP_S;
  // if (elapsedS >= CONTROL_INTERVAL_S) {
  //   // Clear accumulated wake count
  //   wdtWakeCount = 0;
  //   // Start a non-blocking DHT read for control if not already active
  //   if (!dhtRetryActive && !controlReadPending) {
  //     startDhtRetries();
  //     controlReadPending = true;
  //   }
  // }

  float t, h;
  if (dhtRead(&dhtState, millis(), &t, &h) == ASYNC_DONE) {
    // preserve last valid temp in case of read errors, only update when DHT_OK
    if (dhtState.status == DHT_OK) {
      lastValidTempC = t;
      displayDirty = true;
    }
    // else {
    //     // process error?
    // }
    async_init(&dhtState); // reset for next loop
  }// else if (dhtState.status == SOME_ERROR) { displayDirty = true; }


  // Single display update point: update once per loop if any condition requested it.
  if (displayDirty) {
    updateDisplay(lastValidTempC, targetTenthsC);
  }
}
