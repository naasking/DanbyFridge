#include <DHT.h>
#include <Adafruit_ST77xx.h>
#include <SPI.h>

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
volatile int16_t targetCentidegreesC = 250; // tenths of a degee C = 25.0°C
volatile int8_t encoderDelta = 0;           // set from ISR
volatile bool encoderChanged = false;

// Encoder lock to prevent ISR cascade during bounce.
// ISR sets encoderLocked=true immediately to block further ISR processing.
// Loop clears encoderLocked after ENCODER_DEBOUNCE_MS have passed since processing.
volatile bool encoderLocked = false;

bool displayCelsius = true;
unsigned long lastDHTReadMs = 0;
unsigned long lastDisplayMs = 0;
unsigned long lastControlMs = 0;

float lastValidTempC = NAN;
bool lastButtonState = HIGH;
unsigned long lastButtonMs = 0;
unsigned long lastEncoderProcessedMs = 0;

void encoderISR() {
  // Minimal ISR: prevent cascade by checking encoderLocked,
  // then record direction and set flags.
  if (encoderLocked) return;

  bool clk = digitalRead(ROTARY_CLK);
  bool dt  = digitalRead(ROTARY_DT);

  if (clk != dt) {
    encoderDelta++; // CW
  } else {
    encoderDelta--; // CCW
  }
  encoderChanged = true;
  encoderLocked = true; // immediately lock further ISR processing until loop clears it
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

  // 1) Consume encoder changes (handled in loop to allow debouncing/time logic)
  if (encoderChanged) {
    noInterrupts();
    int8_t d = encoderDelta;
    encoderDelta = 0;
    encoderChanged = false;
    interrupts();

    if (d != 0) {
      int16_t newTarget = targetTenthsC + (int16_t)(d * ENCODER_STEP_TENTHS);
      if (newTarget < -400) newTarget = -400;
      if (newTarget > 500)  newTarget = 500;
      targetTenthsC = newTarget;

      // record when we processed this change so we can clear encoderLocked after debounce
      lastEncoderProcessedMs = now;

      // immediate feedback to user
      updateDisplay(lastValidTempC, targetTenthsC);
    }
  }

  // 1.b) Clear encoder lock after debounce period so ISR can accept new stable steps
  if (encoderLocked && (now - lastEncoderProcessedMs >= ENCODER_DEBOUNCE_MS)) {
    encoderLocked = false;
  }

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
