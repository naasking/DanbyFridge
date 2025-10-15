#include <DHT.h>
#include <Adafruit_ST77xx.h>
#include <SPI.h>
#include "rotary.h"

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
#define BUTTON_DEBOUNCE_MS 250
#define ENCODER_DEBOUNCE_MS 10   // debounce period for encoder (ms)

DHT dht(DHTPIN, DHTTYPE);
Adafruit_ST77xx tft = Adafruit_ST77xx(160, 80, TFT_CS, TFT_DC, TFT_RST);

 // Internal state stored in tenths of °C to avoid non-atomic float access
volatile int16_t targetTenthsC = 250; // tenths of a degree C = 25.0°C

// Quadrature decoder state (4-bit history) and pulse accumulator.
// ISR calls rotary_step() and increments encoderPulseCount.
volatile uint8_t rot_state = 0;
volatile int16_t encoderPulseCount = 0;

bool displayCelsius = true;
unsigned long lastDHTReadMs = 0;
unsigned long lastDisplayMs = 0;
unsigned long lastControlMs = 0;

float lastValidTempC = NAN;
bool lastButtonState = HIGH;
unsigned long lastButtonMs = 0;
unsigned long lastEncoderProcessedMs = 0;

void encoderISR() {
  // Minimal ISR: use rotary_step for quadrature decoding (rotb = DT, rota = CLK)
  uint8_t a = digitalRead(ROTARY_CLK); // rota (LSB)
  uint8_t b = digitalRead(ROTARY_DT);  // rotb (MSB)
  int8_t step = rotary_step(&rot_state, b, a);
  if (step) {
    encoderPulseCount += step;
  }
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
        targetTenthsC = newTarget;
        // immediate feedback to user
        updateDisplay(lastValidTempC, targetTenthsC);
      }
    }
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
