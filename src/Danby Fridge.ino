#include <DHT.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>                 // ESP32 core SPI
#include <stdio.h>
#include <Preferences.h>         // replaces EEPROM.h
#include "rotary.h"
#include <esp_sleep.h>
#include <esp_pm.h>
#include "driver/rmt.h"
#include "soc/rmt_reg.h"
#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"

#define DHTTYPE DHT22
// Preferences instance for non-volatile storage on ESP32
static Preferences prefs;

// === ESP32-C3 Super mini Pin Mapping ===
#define DHTPIN          21  // Safe
#define RELAY_PIN       1   // Safe -- connected to HW-040 relay module

// Rotary encoder -- grouped on the right side
#define ROTARY_SW       9  // Strapping: rotary encoder button press at start boots into download mode
#define ROTARY_CLK      7  // Safe
#define ROTARY_DT       6  // Safe

// TFT SPI -- grouped on the left side
#define TFT_DC          0   // Safe
#define BACKLIGHT_PIN   2   // Strapping: defaults HIGH
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
#define BUTTON_DEBOUNCE_MS 250
#define ENCODER_DEBOUNCE_MS 10   // debounce period for encoder (ms)
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
#define DHT_RETRY_DELAY_MS 200

/* RMT parsing thresholds (microseconds) */
#define DHT_RMT_HIGH_MIN_US 20        // ignore highs shorter than this
#define DHT_RMT_ONE_THRESHOLD_US 50   // high-duration >= this -> bit=1
#define DHT_RMT_TIMEOUT_MS 300        // timeout for a single RMT attempt (ms)

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
bool targetDirty = false;
bool displayDirty = false;

// RMT-based async DHT read state (non-blocking receive)
static const rmt_channel_t DHT_RMT_CHANNEL = RMT_CHANNEL_0;
static RingbufHandle_t dhtRmtRb = NULL;
static bool dhtRmtActive = false;
static unsigned long dhtRmtStartMs = 0;
static rmt_config_t dhtRmtConfig;

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
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);
  delay(3000);
  println("Program started...");
  // println("*** I2C Pins ");

  // // Continue normal initialization
  // dht.begin();

  // // Initialize RMT for DHT non-blocking reads once (driver + ringbuffer)
  // {
  //   rmt_config_t rmt_rx;
  //   rmt_rx.rmt_mode = RMT_MODE_RX;
  //   rmt_rx.channel = DHT_RMT_CHANNEL;
  //   rmt_rx.gpio_num = (gpio_num_t)DHTPIN;
  //   rmt_rx.clk_div = 80; // 1us resolution
  //   rmt_rx.mem_block_num = 2;
  //   // Apply config and install driver once
  //   if (rmt_config(&rmt_rx) == ESP_OK) {
  //     if (rmt_driver_install(rmt_rx.channel, 1000, 0) == ESP_OK) {
  //       // obtain ringbuffer handle (used by pollRmtDhtAttempt)
  //       rmt_get_ringbuf_handle(rmt_rx.channel, &dhtRmtRb);
  //       // keep driver installed for subsequent reads; do not uninstall until program end
  //     } else {
  //       println("RMT driver install failed");
  //     }
  //   } else {
  //     println("RMT config failed");
  //   }
  //   // Note: dhtRmtRb may be NULL if driver/ringbuf not available; startRmtDhtAttempt will check
  // }

  // Initialize SPI with explicit pins for ESP32-C3
  SPI.begin(TFT_SCK, TFT_MISO, TFT_MOSI, TFT_CS);

  // Initialize SPI with a slower clock speed so it's more stable, otherwise
  // the scren sometimes blanks out when receiving many rotary pulses.
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(TFT_CS, LOW);
  // send commands/data
  digitalWrite(TFT_CS, HIGH);
  SPI.endTransaction();

  // Initialize display and let the Adafruit library manage SPI.
  println("ST7735: init start");
  // Ensure control pins are configured and stable before calling library init.
  pinMode(TFT_CS, OUTPUT);
  digitalWrite(TFT_CS, HIGH);
  pinMode(TFT_DC, OUTPUT);
  digitalWrite(TFT_DC, HIGH);
  delay(10); // allow peripheral to settle
  // Call library init (use MINI160x80 PLUGIN for this module)
  tft.initR(INITR_MINI160x80_PLUGIN);
  tft.setRotation(3);
  tft.enableDisplay(true);
  println("ST7735: clear screen");
  tft.fillScreen(ST7735_BLACK);

  // Ensure screen is cleared to black and initial UI drawn
  //updateDisplay(lastValidTempC, targetTenthsC);
  //println("ST7735: initial UI drawn");

  pinMode(RELAY_PIN, OUTPUT);
  pinMode(ROTARY_CLK, INPUT_PULLUP);
  pinMode(ROTARY_DT, INPUT_PULLUP);
  pinMode(ROTARY_SW, INPUT_PULLUP);

  // Backlight control pin (module BLK has onboard transistor)
  pinMode(BACKLIGHT_PIN, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ROTARY_CLK), encoderISR, CHANGE);
  // Also attach to DT so the ISR runs on changes of either channel (better capture on fast/cheap encoders)
  attachInterrupt(digitalPinToInterrupt(ROTARY_DT), encoderISR, CHANGE);
  // Wake-only interrupt for rotary pushbutton (FALLING edge). ISR does nothing
  // beyond waking the MCU; actual button processing is done in the loop.
  attachInterrupt(digitalPinToInterrupt(ROTARY_SW), wakeISR, FALLING);

  // initialize relay OFF (assumes active HIGH; invert logic if needed)
  digitalWrite(RELAY_PIN, LOW);

  lastDHTReadMs = millis();

  // Load persisted target temperature (tenths of °C). Validate range, otherwise set to current measured temp if available.
  prefs.begin("fridge", false);

  // Try to load saved target
  int16_t saved = prefs.getShort("targetC", INT16_MIN);
  if (saved >= -400 && saved <= 500) {
    // Valid persisted value
    targetTenthsC = saved;
  } else {
    // No valid saved value: use default target (25.0 °C)
    targetTenthsC = 250;
  }
  // keep Preferences open for the lifetime of the program (do not end here)
  // prefs.end(); // removed to keep prefs active

  // initialize save timestamp to avoid immediate write
  lastSaveMs = millis();

  // ensure display shows current values at startup
  updateDisplay(lastValidTempC, targetTenthsC);
}

void updateDisplay(float currentC, int16_t targetTenths) {
  // Full-screen redraw but use color to distinguish current (white) and target (light blue).
  println("updating display...");
  // Filling the screen causes flicker, use the setTextColor overload that accepts
  // a background colour already paints the background thus overwriting the space
  //tft.fillScreen(ST7735_BLACK);

  tft.setTextSize(2);

  // Current temperature (left) - print fixed-width numeric + unit to avoid residual chars
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.setCursor(8, 18);
  if (isnan(currentC)) {
    // pad "Err" to match numeric+unit width ("%5.1f C" -> 7 chars)
    tft.print("Err    ");
  } else {
    char buf[16];
    if (displayCelsius) {
      // numeric field: width 5 (space/sign + up to 2 digits + '.' + 1 decimal) + " C" = 7 chars
      snprintf(buf, sizeof(buf), "%5.1f C", currentC);
      tft.print(buf);
    } else {
      float f = currentC * 9.0 / 5.0 + 32.0;
      snprintf(buf, sizeof(buf), "%5.1f F", f);
      tft.print(buf);
    }
  }

  // Target temperature (right) in light blue - also fixed-width
  uint16_t lightBlue = tft.color565(173, 216, 230); // light blue (RGB: 173,216,230)
  tft.setTextColor(lightBlue, ST77XX_BLACK);
  tft.setCursor(70, 18);
  float tgtC = targetTenths / 10.0;
  {
    char buf2[16];
    if (displayCelsius) {
      snprintf(buf2, sizeof(buf2), "%5.1f C", tgtC);
      tft.print(buf2);
    } else {
      float tf = tgtC * 9.0 / 5.0 + 32.0;
      snprintf(buf2, sizeof(buf2), "%5.1f F", tf);
      tft.print(buf2);
    }
  }

  // // Compressor running indicator (snowflake) between the values
  // // Draw a simple snowflake icon centered at (80, 20)
  // int cx = 80;
  // int cy = 24;
  // if (relayOn) {
  //   uint16_t iconColor = lightBlue;
  //   // central dot
  //   tft.drawPixel(cx, cy, iconColor);
  //   // main arms
  //   tft.drawLine(cx - 6, cy, cx + 6, cy, iconColor); // horizontal
  //   tft.drawLine(cx, cy - 6, cx, cy + 6, iconColor); // vertical
  //   // diagonals
  //   tft.drawLine(cx - 4, cy - 4, cx + 4, cy + 4, iconColor);
  //   tft.drawLine(cx - 4, cy + 4, cx + 4, cy - 4, iconColor);
  //   // small branches on arms
  //   tft.drawPixel(cx - 6, cy - 1, iconColor);
  //   tft.drawPixel(cx - 6, cy + 1, iconColor);
  //   tft.drawPixel(cx + 6, cy - 1, iconColor);
  //   tft.drawPixel(cx + 6, cy + 1, iconColor);
  //   tft.drawPixel(cx - 1, cy - 6, iconColor);
  //   tft.drawPixel(cx + 1, cy - 6, iconColor);
  //   tft.drawPixel(cx - 1, cy + 6, iconColor);
  //   tft.drawPixel(cx + 1, cy + 6, iconColor);
  // } else {
  //   // If compressor off, optionally draw a faint outline or nothing. We'll draw nothing.
  // }

}

static bool startRmtDhtAttempt() {
  // Start an RMT receive attempt using the driver and ringbuffer already initialized in setup().
  // Returns true if the attempt was successfully initiated.
  if (!dhtRmtRb) {
    // RMT driver or ringbuffer not available
    return false;
  }

  // Send start signal: pull pin low for 2 ms, then high ~40 us
  pinMode(DHTPIN, OUTPUT);
  digitalWrite(DHTPIN, LOW);
  ets_delay_us(2000); // 2 ms
  digitalWrite(DHTPIN, HIGH);
  ets_delay_us(40); // 40 us
  pinMode(DHTPIN, INPUT_PULLUP);

  // Start receiving on preconfigured channel
  if (rmt_rx_start(DHT_RMT_CHANNEL, true) != ESP_OK) {
    // ensure ringbuffer is clean if start failed
    if (dhtRmtRb) {
      // drain any available item
      size_t rx_size = 0;
      void* item = xRingbufferReceive(dhtRmtRb, &rx_size, 0);
      if (item) vRingbufferReturnItem(dhtRmtRb, item);
    }
    return false;
  }

  dhtRmtActive = true;
  dhtRmtStartMs = millis();
  return true;
}

// Non-blocking poll: returns NAN if no result yet, or temperature if data ready, or NaN when attempt exhausted.
static float pollRmtDhtAttempt() {
  if (!dhtRmtActive) return NAN;

  // Try to receive available buffer without blocking
  size_t rx_size = 0;
  rmt_item32_t* items = (rmt_item32_t*) xRingbufferReceive(dhtRmtRb, &rx_size, 0);
  if (!items) {
    // No data yet. Check timeout (300 ms) to consider attempt failed.
      if (millis() - dhtRmtStartMs > 300) {
      // timeout: clean up and mark not active
      rmt_rx_stop(DHT_RMT_CHANNEL);
      // drain any pending item if present
      size_t _sz = 0;
      void* _it = xRingbufferReceive(dhtRmtRb, &_sz, 0);
      if (_it) vRingbufferReturnItem(dhtRmtRb, _it);
      dhtRmtActive = false;
      return NAN;
    }
    return NAN; // still waiting
  }

  int item_count = rx_size / sizeof(rmt_item32_t);

  // Collect high durations into a fixed buffer (avoid heap allocations)
  uint32_t highs[128];
  int highs_cnt = 0;
  for (int i = 0; i < item_count; ++i) {
    if (items[i].level0 == 1 && highs_cnt < 128) highs[highs_cnt++] = items[i].duration0;
    if (items[i].level1 == 1 && highs_cnt < 128) highs[highs_cnt++] = items[i].duration1;
  }

  // Return RMT buffer
  vRingbufferReturnItem(dhtRmtRb, (void*)items);

  // Stop receiver for this attempt; keep driver installed for subsequent reads
  rmt_rx_stop(DHT_RMT_CHANNEL);
  dhtRmtActive = false;

  // Filter relevant highs into a fixed buffer
  uint32_t bitHighs[64];
  int bitHighs_cnt = 0;
  for (int i = 0; i < highs_cnt; ++i) {
    uint32_t d = highs[i];
    if (d > DHT_RMT_HIGH_MIN_US && bitHighs_cnt < 64) bitHighs[bitHighs_cnt++] = d;
  }
  if (bitHighs_cnt < 40) return NAN;

  uint8_t bits[40];
  int bits_cnt = 0;
  for (int i = 0; i < bitHighs_cnt && bits_cnt < 40; ++i) {
    uint32_t dur = bitHighs[i];
    bits[bits_cnt++] = (dur > DHT_RMT_ONE_THRESHOLD_US) ? 1 : 0;
  }
  if (bits_cnt < 40) return NAN;

  uint8_t data[5] = {0};
  for (int i = 0; i < 40; ++i) {
    data[i/8] <<= 1;
    data[i/8] |= bits[i];
  }
  uint8_t sum = data[0] + data[1] + data[2] + data[3];
  if (sum != data[4]) return NAN;

  int16_t rawHum = (data[0] << 8) | data[1];
  int16_t rawTemp = (data[2] << 8) | data[3];
  float tempC = 0.0f;
  if (rawTemp & 0x8000) {
    rawTemp &= 0x7FFF;
    tempC = -(rawTemp / 10.0f);
  } else {
    tempC = rawTemp / 10.0f;
  }
  return tempC;
}

// Non-blocking retry manager: starts attempts and polls them over multiple loop iterations.
// Call startDhtRetries() to begin; call pollDhtRetries(&outTemp) repeatedly until it returns true (done).
static int dhtRetriesLeft = 0;
static unsigned long dhtNextAttemptMs = 0;
static bool dhtRetryActive = false;

static void startDhtRetries() {
  dhtRetriesLeft = DHT_MAX_RETRIES;
  dhtNextAttemptMs = millis();
  dhtRetryActive = true;
  dhtRmtActive = false;
}

static bool pollDhtRetries(float* outTemp) {
  if (!dhtRetryActive) return false;

  unsigned long now = millis();

  if (dhtRmtActive) {
    float res = pollRmtDhtAttempt();
    if (!isnan(res)) {
      *outTemp = res;
      dhtRetryActive = false;
      return true;
    }
    // still waiting for RMT data
    return false;
  } else {
    if (now >= dhtNextAttemptMs) {
      // Start a new RMT attempt
      if (startRmtDhtAttempt()) {
        // attempt started, poll in subsequent loop iterations
        return false;
      } else {
        // failed to start RMT; treat as attempt used
        dhtRetriesLeft--;
        if (dhtRetriesLeft <= 0) {
          dhtRetryActive = false;
          *outTemp = NAN;
          return true;
        }
        dhtNextAttemptMs = now + DHT_RETRY_DELAY_MS;
        return false;
      }
    }
    return false;
  }
}

void controlTemperature(float currentC, int16_t targetTenths) {
  // If the reading is invalid, skip control to avoid spurious changes.
  if (isnan(currentC)) return;

  int16_t currentTenths = (int16_t)round(currentC * 10.0);
  unsigned long now = millis();

  // Need to turn ON when current is below target minus hysteresis.
  if (currentTenths <= (targetTenths - HYSTERESIS_TENTHS)) {
    // Only allow starting compressor if it is currently off and the minimum off time has elapsed.
    if (!relayOn) {
      if ((now - lastRelayOffMs) >= COMPRESSOR_MIN_OFF_MS) {
        digitalWrite(RELAY_PIN, HIGH);
        relayOn = true;
      } else {
        // Still in minimum-off window: do not start yet.
      }
    }
  }
  // Need to turn OFF when current is above target plus hysteresis.
  else if (currentTenths >= (targetTenths + HYSTERESIS_TENTHS)) {
    if (relayOn) {
      digitalWrite(RELAY_PIN, LOW);
      relayOn = false;
      lastRelayOffMs = now;
    }
  }
}

void loop() {
  //println("in loop...");
  //delay(500);
  unsigned long now = millis();

  // 1) Consume encoder pulses produced by the quadrature ISR and convert pulses -> steps
  {
    int16_t pulses = 0;
    noInterrupts();
    pulses = encoderPulseCount;
    encoderPulseCount = 0;
    interrupts();

    if (pulses != 0) {
      println("Detected pulses");
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
          targetDirty = true; // mark for persistence
          // defer display update to the single update at the end of loop()
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
    targetDirty = false;
  }
  
  // // Keep display on for a short period after any updates for user feedback
  // if (now < displayOnUntilMs) {
  //   digitalWrite(BACKLIGHT_PIN, HIGH);
  // } else {
  //   digitalWrite(BACKLIGHT_PIN, LOW);
  // }
  
  // //  Put MCU to low-power sleep if idle: wake sources are external INT (encoder/button) and timer (approx WDT)
  // //  Only sleep if no recent activity and display not required.
  // bool idle = (encoderPulseCount == 0) && !targetDirty;
  // if (idle) {
  //   // Configure timer wake for approximately WDT_SLEEP_S seconds (esp_light_sleep)
  //   esp_sleep_enable_timer_wakeup((uint64_t)WDT_SLEEP_S * 1000000ULL);
  //   // Enter light sleep; external interrupts attached with attachInterrupt() will also wake the chip
  //   esp_light_sleep_start();
  //   // Woke up (either via timer or external INT)
  //   wdtWakeCount++;
  //   // keep display on briefly after wake for feedback
  //   displayOnUntilMs = millis() + DISPLAY_ON_AFTER_WAKE_MS;
  // }

  // (quadrature ISR handles bounce; no encoder lock needed)

  // 2) Button handled and debounced in loop (no millis() in ISR)
  bool sw = digitalRead(ROTARY_SW);

  if (sw == LOW && lastButtonState == HIGH && (now - lastButtonMs) > BUTTON_DEBOUNCE_MS) {
    // Button pressed (debounced start)
    println("Detected button edge");
    pressStartMs = now;
    lastButtonMs = now;
  }

  if (sw == HIGH && lastButtonState == LOW) {
    // Button released, check duration
    unsigned long pressDuration = now - pressStartMs;
    if (pressDuration >= LONG_PRESS_MS) {
      println("Detected button press");
      displayCelsius = !displayCelsius;
      // mark the display dirty; actual redraw happens once at end of loop()
      displayDirty = true;
    }
    lastButtonMs = now;
  }

  lastButtonState = sw;

  // // 3) Start non-blocking DHT read sequence when interval reached, and poll progress
  // if (now - lastDHTReadMs >= DHT_READ_INTERVAL) {
  //   startDhtRetries();
  //   lastDHTReadMs = now;
  // }

  // // Poll any in-progress non-blocking DHT attempt (RMT); handle initial/control outcomes
  // {
  //   float dhtTemp;
  //   if (dhtRetryActive) {
  //     if (pollDhtRetries(&dhtTemp)) {
  //       if (!isnan(dhtTemp)) {
  //         lastValidTempC = dhtTemp;
  //         if (controlReadPending) {
  //           controlReadPending = false;
  //           controlTemperature(lastValidTempC, targetTenthsC);
  //           updateDisplay(lastValidTempC, targetTenthsC);
  //         }
  //       } else {
  //         println("DHT read failed (retries)");
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

  // Single display update point: update once per loop if any condition requested it.
  if (displayDirty) {
    updateDisplay(lastValidTempC, targetTenthsC);
    displayDirty = false;
  }
}
