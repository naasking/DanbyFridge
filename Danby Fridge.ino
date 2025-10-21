#include <DHT.h>
#include <Adafruit_ST77xx.h>
#include <SPI.h>                 // ESP32 core SPI
#include <Preferences.h>         // replaces EEPROM.h
#include "rotary.h"
#include <esp_sleep.h>
#include <esp_pm.h>
#include "driver/rmt.h"
#include "soc/rmt_reg.h"
#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"
#include <vector>

#define DHTTYPE DHT22
// Preferences instance for non-volatile storage on ESP32
static Preferences prefs;

// // === For Arduino Nano ===
// #define DHTPIN 4
// #define DHTTYPE DHT22
// #define RELAY_PIN 5
// #define ROTARY_CLK 2
// #define ROTARY_DT 6
// #define ROTARY_SW 3
// #define TFT_CS 10
// #define TFT_RST 9
// #define TFT_DC 8
// // #define TFT_MOSI        11   // GPIO4 (fixed MOSI - SDA on TFT)
// // #define TFT_SCK         13   // GPIO6 (fixed SCK - SCL on TFT)
// #define BACKLIGHT_PIN 7          // pin to control TFT backlight (default D7)

// === ESP32-C3 Mini Pin Remapping ===
#define DHTPIN          0   // GPIO0
#define RELAY_PIN       1   // GPIO1
#define ROTARY_CLK      2   // GPIO2
#define ROTARY_DT       3   // GPIO3
#define ROTARY_SW       10  // GPIO10 (has pull-up, interrupt capable)
#define TFT_CS          7   // GPIO7
#define TFT_DC          8   // GPIO8
#define TFT_RST         9   // GPIO9
#define TFT_MOSI        4   // GPIO4 (fixed MOSI)
#define TFT_SCK         6   // GPIO6 (fixed SCK)
#define BACKLIGHT_PIN   5   // GPIO5 (PWM-capable)

// === Notes ===
// - All pins are 3.3V only (no 5V tolerance).
// - GPIO0 is strapping: keep it pulled high at boot if used for DHT.
// - If your board exposes GPIO18/19, you can swap them in for less conflict.
// - Replace EEPROM/LowPower libraries with Preferences + esp_sleep APIs.

#define DISPLAY_UPDATE_INTERVAL 200
#define DHT_READ_INTERVAL 1000
#define CONTROL_INTERVAL 10000
#define HYSTERESIS_TENTHS 5      // 0.5°C hysteresis
#define ENCODER_STEP_TENTHS 1    // 0.1°C per encoder step
#define ENCODER_PULSES_PER_STEP 4 // quadrature pulses per mechanical detent (tune if needed)
#define BUTTON_DEBOUNCE_MS 250
#define ENCODER_DEBOUNCE_MS 10   // debounce period for encoder (ms)
#define LONG_PRESS_MS 1000 // ms for long press to toggle units

// EEPROM persistence
#define EEPROM_ADDR_TARGET 0
#define SAVE_INTERVAL_MS 5000    // minimum interval between EEPROM writes

// Power management
#define WDT_SLEEP_S 8            // WDT sleep interval in seconds (LowPower SLEEP_8S)
#define CONTROL_INTERVAL_S 10    // control interval in seconds (approximately)
#define DISPLAY_ON_AFTER_WAKE_MS 3000 // keep display on for some time after wake for user feedback
#define COMPRESSOR_MIN_OFF_MS (4UL * 60UL * 1000UL) // 4 minutes minimum off time for compressor starter
// DHT read retry settings
#define DHT_MAX_RETRIES 3
#define DHT_RETRY_DELAY_MS 200

DHT dht(DHTPIN, DHTTYPE);
Adafruit_ST77xx tft = Adafruit_ST77xx(160, 80, TFT_CS, TFT_DC, TFT_RST);

 // Internal state stored in tenths of °C to avoid non-atomic float access
volatile int16_t targetTenthsC = 250; // tenths of a degree C = 25.0°C

// Quadrature decoder state (4-bit history) and pulse accumulator.
// ISR calls rotary_step() and increments encoderPulseCount.
volatile uint8_t rot_state = 0;
volatile int16_t encoderPulseCount = 0;
 
 // Persistence helpers
unsigned long lastSaveMs = 0;
bool targetDirty = false;

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
  // rotary_step_s16 performs the state update and increments/decrements the counter.
  uint8_t a = digitalRead(ROTARY_CLK); // rota (LSB)
  uint8_t b = digitalRead(ROTARY_DT);  // rotb (MSB)
  rotary_step_s16(&encoderPulseCount, &rot_state, b, a);
}

// Wake-only ISR for the rotary button: intentionally empty so its only effect
// is to wake the MCU from sleep. Debounce and button handling remain in loop().
void IRAM_ATTR wakeISR() {
  // no-op: wake-only
}

void setup() {
  Serial.begin(115200);
  dht.begin();

  // Initialize RMT for DHT non-blocking reads once (driver + ringbuffer)
  {
    rmt_config_t rmt_rx;
    rmt_rx.rmt_mode = RMT_MODE_RX;
    rmt_rx.channel = DHT_RMT_CHANNEL;
    rmt_rx.gpio_num = (gpio_num_t)DHTPIN;
    rmt_rx.clk_div = 80; // 1us resolution
    rmt_rx.mem_block_num = 2;
    // Apply config and install driver once
    if (rmt_config(&rmt_rx) == ESP_OK) {
      if (rmt_driver_install(rmt_rx.channel, 1000, 0) == ESP_OK) {
        // obtain ringbuffer handle (used by pollRmtDhtAttempt)
        rmt_get_ringbuf_handle(rmt_rx.channel, &dhtRmtRb);
        // keep driver installed for subsequent reads; do not uninstall until program end
      } else {
        Serial.println("RMT driver install failed");
      }
    } else {
      Serial.println("RMT config failed");
    }
    // Note: dhtRmtRb may be NULL if driver/ringbuf not available; startRmtDhtAttempt will check
  }

  // Initialize SPI with explicit pins for ESP32-C3
  SPI.begin(TFT_SCK, -1, TFT_MOSI, TFT_CS);

  // Initialize display inside an SPI transaction for consistent bus settings
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  tft.initSPI();
  tft.setRotation(2);
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  SPI.endTransaction();

  pinMode(RELAY_PIN, OUTPUT);
  pinMode(ROTARY_CLK, INPUT_PULLUP);
  pinMode(ROTARY_DT, INPUT_PULLUP);
  pinMode(ROTARY_SW, INPUT_PULLUP);

  // Backlight control pin
  pinMode(BACKLIGHT_PIN, OUTPUT);
  digitalWrite(BACKLIGHT_PIN, HIGH); // turn on display backlight at startup

  attachInterrupt(digitalPinToInterrupt(ROTARY_CLK), encoderISR, CHANGE);
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
  // Wrap display operations in an SPI transaction to ensure the bus runs at the desired speed
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));

  // Full-screen redraw but use color to distinguish current (white) and target (light blue).
  tft.fillScreen(ST77XX_BLACK);

  tft.setTextSize(2);

  // Current temperature (left)
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(8, 18);
  if (isnan(currentC)) {
    tft.print("Err");
  } else {
    if (displayCelsius) {
      tft.print(currentC, 1);
      tft.print(" C");
    } else {
      float f = currentC * 9.0 / 5.0 + 32.0;
      tft.print(f, 1);
      tft.print(" F");
    }
  }

  // Target temperature (right) in light blue
  uint16_t lightBlue = tft.color565(173, 216, 230); // light blue (RGB: 173,216,230)
  tft.setTextColor(lightBlue);
  tft.setCursor(96, 18);
  float tgtC = targetTenths / 10.0;
  if (displayCelsius) {
    tft.print(tgtC, 1);
    tft.print(" C");
  } else {
    float tf = tgtC * 9.0 / 5.0 + 32.0;
    tft.print(tf, 1);
    tft.print(" F");
  }

  // Compressor running indicator (snowflake) between the values
  // Draw a simple snowflake icon centered at (80, 20)
  int cx = 80;
  int cy = 24;
  if (relayOn) {
    uint16_t iconColor = lightBlue;
    // central dot
    tft.drawPixel(cx, cy, iconColor);
    // main arms
    tft.drawLine(cx - 6, cy, cx + 6, cy, iconColor); // horizontal
    tft.drawLine(cx, cy - 6, cx, cy + 6, iconColor); // vertical
    // diagonals
    tft.drawLine(cx - 4, cy - 4, cx + 4, cy + 4, iconColor);
    tft.drawLine(cx - 4, cy + 4, cx + 4, cy - 4, iconColor);
    // small branches on arms
    tft.drawPixel(cx - 6, cy - 1, iconColor);
    tft.drawPixel(cx - 6, cy + 1, iconColor);
    tft.drawPixel(cx + 6, cy - 1, iconColor);
    tft.drawPixel(cx + 6, cy + 1, iconColor);
    tft.drawPixel(cx - 1, cy - 6, iconColor);
    tft.drawPixel(cx + 1, cy - 6, iconColor);
    tft.drawPixel(cx - 1, cy + 6, iconColor);
    tft.drawPixel(cx + 1, cy + 6, iconColor);
  } else {
    // If compressor off, optionally draw a faint outline or nothing. We'll draw nothing.
  }

  SPI.endTransaction();
}

static bool startRmtDhtAttempt() {
  // Configure and start an RMT receive attempt; returns true if started
  rmt_config_t rmt_rx;
  rmt_rx.rmt_mode = RMT_MODE_RX;
  rmt_rx.channel = DHT_RMT_CHANNEL;
  rmt_rx.gpio_num = (gpio_num_t)DHTPIN;
  rmt_rx.clk_div = 80; // 1us resolution
  rmt_rx.mem_block_num = 2;
  // Use default rx config fields
  if (rmt_config(&rmt_rx) != ESP_OK) return false;
  if (rmt_driver_install(rmt_rx.channel, 1000, 0) != ESP_OK) return false;

  // Get ring buffer handle
  rmt_get_ringbuf_handle(rmt_rx.channel, &dhtRmtRb);
  if (!dhtRmtRb) {
    rmt_driver_uninstall(rmt_rx.channel);
    return false;
  }

  // Send start signal: pull pin low for 2 ms, then high ~40 us
  pinMode(DHTPIN, OUTPUT);
  digitalWrite(DHTPIN, LOW);
  ets_delay_us(2000); // 2 ms
  digitalWrite(DHTPIN, HIGH);
  ets_delay_us(40); // 40 us
  pinMode(DHTPIN, INPUT_PULLUP);

  // Start receiving
  if (rmt_rx_start(rmt_rx.channel, true) != ESP_OK) {
    vRingbufferReturnItem(dhtRmtRb, NULL); // ensure clean
    rmt_driver_uninstall(rmt_rx.channel);
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
      vRingbufferReturnItem(dhtRmtRb, NULL);
      rmt_driver_uninstall(DHT_RMT_CHANNEL);
      dhtRmtActive = false;
      return NAN;
    }
    return NAN; // still waiting
  }

  int item_count = rx_size / sizeof(rmt_item32_t);

  // Collect high durations
  std::vector<uint32_t> highs;
  highs.reserve(64);
  for (int i = 0; i < item_count; ++i) {
    if (items[i].level0 == 1) highs.push_back(items[i].duration0);
    if (items[i].level1 == 1) highs.push_back(items[i].duration1);
  }

  // Return RMT buffer
  vRingbufferReturnItem(dhtRmtRb, (void*)items);

  // Stop and uninstall driver
  rmt_rx_stop(DHT_RMT_CHANNEL);
  rmt_driver_uninstall(DHT_RMT_CHANNEL);
  dhtRmtActive = false;

  // Filter relevant highs
  std::vector<uint32_t> bitHighs;
  for (auto d : highs) {
    if (d > 20) bitHighs.push_back(d);
  }
  if (bitHighs.size() < 40) return NAN;

  std::vector<uint8_t> bits;
  for (size_t i = 0; i < bitHighs.size() && bits.size() < 40; ++i) {
    uint32_t dur = bitHighs[i];
    bits.push_back(dur > 50 ? 1 : 0);
  }
  if (bits.size() < 40) return NAN;

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
  unsigned long now = millis();

  // 1) Consume encoder pulses produced by the quadrature ISR and convert pulses -> steps
  {
    int16_t pulses = 0;
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
          targetDirty = true; // mark for persistence
          // immediate feedback to user
          updateDisplay(lastValidTempC, targetTenthsC);
        }
      }
    }
  }

  // Persist target temperature occasionally (rate-limited to SAVE_INTERVAL_MS)
  if (targetDirty && (millis() - lastSaveMs >= SAVE_INTERVAL_MS)) {
    int16_t toSave = targetTenthsC; // local copy
    // Preferences is already opened in setup and kept open; just write the value
    prefs.putShort("targetC", toSave);
    lastSaveMs = millis();
    targetDirty = false;
  }
  
  // Keep display on for a short period after any updates for user feedback
  if (millis() < displayOnUntilMs) {
    digitalWrite(BACKLIGHT_PIN, HIGH);
  } else {
    digitalWrite(BACKLIGHT_PIN, LOW);
  }
  
  //  Put MCU to low-power sleep if idle: wake sources are external INT (encoder/button) and timer (approx WDT)
  //  Only sleep if no recent activity and display not required.
  bool idle = (encoderPulseCount == 0) && !targetDirty;
  if (idle) {
    // Configure timer wake for approximately WDT_SLEEP_S seconds (esp_light_sleep)
    esp_sleep_enable_timer_wakeup((uint64_t)WDT_SLEEP_S * 1000000ULL);
    // Enter light sleep; external interrupts attached with attachInterrupt() will also wake the chip
    esp_light_sleep_start();
    // Woke up (either via timer or external INT)
    wdtWakeCount++;
    // keep display on briefly after wake for feedback
    displayOnUntilMs = millis() + DISPLAY_ON_AFTER_WAKE_MS;
  }

  // (quadrature ISR handles bounce; no encoder lock needed)

  // 2) Button handled and debounced in loop (no millis() in ISR)
  bool sw = digitalRead(ROTARY_SW);

  if (sw == LOW && lastButtonState == HIGH && (now - lastButtonMs) > BUTTON_DEBOUNCE_MS) {
    // Button pressed (debounced start)
    pressStartMs = now;
    lastButtonMs = now;
  }

  if (sw == HIGH && lastButtonState == LOW) {
    // Button released, check duration
    unsigned long pressDuration = now - pressStartMs;
    if (pressDuration >= LONG_PRESS_MS) {
      displayCelsius = !displayCelsius;
      updateDisplay(lastValidTempC, targetTenthsC);
    }
    lastButtonMs = now;
  }

  lastButtonState = sw;

  // 3) Start non-blocking DHT read sequence when interval reached, and poll progress
  if (now - lastDHTReadMs >= DHT_READ_INTERVAL) {
    startDhtRetries();
    lastDHTReadMs = now;
  }

  // Poll any in-progress non-blocking DHT attempt (RMT); handle initial/control outcomes
  {
    float dhtTemp;
    if (dhtRetryActive) {
      if (pollDhtRetries(&dhtTemp)) {
        if (!isnan(dhtTemp)) {
          lastValidTempC = dhtTemp;
          if (controlReadPending) {
            controlReadPending = false;
            controlTemperature(lastValidTempC, targetTenthsC);
            updateDisplay(lastValidTempC, targetTenthsC);
          }
        } else {
          Serial.println("DHT read failed (retries)");
          // clear pending flags on failure
          controlReadPending = false;
        }
      }
    }
  }


  // 5) Control relay periodically using WDT wake count approximation
  // WDT interval roughly WDT_SLEEP_S seconds per LowPower wake.
  // Add elapsed seconds since last WDT wake (approx)
  // Use wdtWakeCount as a coarse timer: multiply by WDT_SLEEP_S
  unsigned long elapsedS = wdtWakeCount * WDT_SLEEP_S;
  if (elapsedS >= CONTROL_INTERVAL_S) {
    // Clear accumulated wake count
    wdtWakeCount = 0;
    // Start a non-blocking DHT read for control if not already active
    if (!dhtRetryActive && !controlReadPending) {
      startDhtRetries();
      controlReadPending = true;
    }
  }

}
