#include <DHT.h>
#include <Adafruit_ST77xx.h>
#include <SPI.h>
#include "rotary.h"
#include <EEPROM.h>

#define DHTPIN 4
#define DHTTYPE DHT22
#define RELAY_PIN 5
#define ROTARY_CLK 2
#define ROTARY_DT 6
#define ROTARY_SW 3
#define TFT_CS 10
#define TFT_RST 9
#define TFT_DC 8

#define DISPLAY_UPDATE_INTERVAL 200
#define DHT_READ_INTERVAL 1000
#define CONTROL_INTERVAL 10000
#define HYSTERESIS_TENTHS 5      // 0.5°C hysteresis
#define ENCODER_STEP_TENTHS 1    // 0.1°C per encoder step
#define ENCODER_PULSES_PER_STEP 4 // quadrature pulses per mechanical detent (tune if needed)
#define BUTTON_DEBOUNCE_MS 250
#define ENCODER_DEBOUNCE_MS 10   // debounce period for encoder (ms)

// EEPROM persistence
#define EEPROM_ADDR_TARGET 0
#define SAVE_INTERVAL_MS 5000    // minimum interval between EEPROM writes

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

bool displayCelsius = true;
unsigned long lastDHTReadMs = 0;
unsigned long lastDisplayMs = 0;
unsigned long lastControlMs = 0;

float lastValidTempC = NAN;
bool lastButtonState = HIGH;
unsigned long lastButtonMs = 0;
unsigned long lastEncoderProcessedMs = 0;

void encoderISR() {
  // Minimal ISR: use rotary_step_s16 to update encoderPulseCount directly.
  // rotary_step_s16 performs the state update and increments/decrements the counter.
  uint8_t a = digitalRead(ROTARY_CLK); // rota (LSB)
  uint8_t b = digitalRead(ROTARY_DT);  // rotb (MSB)
  rotary_step_s16(&encoderPulseCount, &rot_state, b, a);
}

void setup() {
  Serial.begin(115200);
  dht.begin();

  tft.initSPI();
  tft.setRotation(2);
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);

  pinMode(RELAY_PIN, OUTPUT);
  pinMode(ROTARY_CLK, INPUT_PULLUP);
  pinMode(ROTARY_DT, INPUT_PULLUP);
  pinMode(ROTARY_SW, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ROTARY_CLK), encoderISR, CHANGE);

  // initialize relay OFF (assumes active HIGH; invert logic if needed)
  digitalWrite(RELAY_PIN, LOW);

  lastDHTReadMs = millis();
  lastDisplayMs = millis();
  lastControlMs = millis();

  // Load persisted target temperature (tenths of °C). Validate range, otherwise set to current measured temp if available.
  int16_t saved = 0;
  EEPROM.get(EEPROM_ADDR_TARGET, saved);
  if (saved >= -400 && saved <= 500) {
    targetTenthsC = saved;
  } else {
    // No valid saved value: try to initialize target from the current DHT reading.
    float initTempC = dht.readTemperature();
    if (isnan(initTempC)) {
      // sensor may need a short time after power-up; try once more
      delay(200);
      initTempC = dht.readTemperature();
    }
    if (!isnan(initTempC)) {
      targetTenthsC = (int16_t)round(initTempC * 10.0);
      lastValidTempC = initTempC;
    }
    // otherwise keep the compiled-in default targetTenthsC
  }
  // initialize save timestamp to avoid immediate write
  lastSaveMs = millis();
}

void updateDisplay(float currentC, int16_t targetTenths) {
  // Simple full redraw. For less flicker, update only changed fields.
  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(4, 8);
  tft.setTextSize(2);
  tft.print("Cur: ");
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
  tft.setCursor(4, 36);
  tft.print("Tgt: ");
  float tgtC = targetTenths / 10.0;
  if (displayCelsius) {
    tft.print(tgtC, 1);
    tft.print(" C");
  } else {
    float tf = tgtC * 9.0 / 5.0 + 32.0;
    tft.print(tf, 1);
    tft.print(" F");
  }
}

void controlTemperature(float currentC, int16_t targetTenths) {
  if (isnan(currentC)) return; // do not change relay on sensor error
  int16_t currentTenths = (int16_t)round(currentC * 10.0);

  // Hysteresis control
  if (currentTenths <= (targetTenths - HYSTERESIS_TENTHS)) {
    digitalWrite(RELAY_PIN, HIGH); // ON
  } else if (currentTenths >= (targetTenths + HYSTERESIS_TENTHS)) {
    digitalWrite(RELAY_PIN, LOW);  // OFF
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
    EEPROM.put(EEPROM_ADDR_TARGET, toSave);
    lastSaveMs = millis();
    targetDirty = false;
  }

  // (quadrature ISR handles bounce; no encoder lock needed)

  // 2) Button handled and debounced in loop (no millis() in ISR)
  bool sw = digitalRead(ROTARY_SW);
  if (sw == LOW && lastButtonState == HIGH && (now - lastButtonMs) > BUTTON_DEBOUNCE_MS) {
    displayCelsius = !displayCelsius;
    lastButtonMs = now;
    updateDisplay(lastValidTempC, targetTenthsC);
  }
  lastButtonState = sw;

  // 3) Read DHT at interval, save last valid
  if (now - lastDHTReadMs >= DHT_READ_INTERVAL) {
    float tempC = dht.readTemperature();
    if (!isnan(tempC)) lastValidTempC = tempC;
    else Serial.println("DHT read failed");
    lastDHTReadMs = now;
  }

  // 4) Periodic display update
  if (now - lastDisplayMs >= DISPLAY_UPDATE_INTERVAL) {
    updateDisplay(lastValidTempC, targetTenthsC);
    lastDisplayMs = now;
  }

  // 5) Control relay periodically
  if (now - lastControlMs >= CONTROL_INTERVAL) {
    controlTemperature(lastValidTempC, targetTenthsC);
    lastControlMs = now;
  }
}
