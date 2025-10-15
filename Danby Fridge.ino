#include <DHT.h>
#include <Adafruit_ST77xx.h>
#include <SPI.h>

// Pin Assignments adjusted for Nano
#define DHTPIN 4          // Moved DHT to D4 to free up interrupt pins
#define DHTTYPE DHT22
#define RELAY_PIN 5
#define ROTARY_CLK 2      // Using D2 (interrupt pin)
#define ROTARY_DT 6
#define ROTARY_SW 3       // Using D3 (interrupt pin)
#define TFT_CS 10         // SPI pins remain the same as they are hardware SPI
#define TFT_RST 9
#define TFT_DC 8

// Constants for debouncing and timing
#define DEBOUNCE_DELAY 50
#define BUTTON_DEBOUNCE_DELAY 250
#define DISPLAY_UPDATE_INTERVAL 500
#define CONTROL_INTERVAL 10000

DHT dht(DHTPIN, DHTTYPE);
Adafruit_ST77xx tft = Adafruit_ST77xx(160, 80, TFT_CS, TFT_DC, TFT_RST);

volatile float targetTemperature = 25.0;
volatile unsigned long lastInterruptTime = 0;
volatile bool displayNeedsUpdate = false;
volatile bool isCelsius = true;

void setup() {
  dht.begin();

  tft.initSPI();  // Initialize ST77355 display
  //tft.init(240, 240);       //for the ST7788
  tft.setRotation(2);
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);

  pinMode(RELAY_PIN, OUTPUT);
  pinMode(ROTARY_CLK, INPUT_PULLUP);
  pinMode(ROTARY_DT, INPUT_PULLUP);
  pinMode(ROTARY_SW, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ROTARY_CLK), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ROTARY_SW), toggleTemperatureUnit, FALLING);
}

void updateEncoder() {
  unsigned long interruptTime = millis();
  
  if (interruptTime - lastInterruptTime > DEBOUNCE_DELAY) {
    int clkState = digitalRead(ROTARY_CLK);
    int dtState = digitalRead(ROTARY_DT);

    if (clkState != dtState) {
      targetTemperature += 0.1;
    } else {
      targetTemperature -= 0.1;
    }

    displayNeedsUpdate = true;
    lastInterruptTime = interruptTime;
  }
}

void toggleTemperatureUnit() {
  static unsigned long lastButtonPress = 0;
  unsigned long currentTime = millis();
  
  if (currentTime - lastButtonPress > BUTTON_DEBOUNCE_DELAY) {
    isCelsius = !isCelsius;
    
    // Convert temperature when switching units
    if (isCelsius) {
      targetTemperature = (targetTemperature - 32.0) * (5.0 / 9.0);
    } else {
      targetTemperature = (targetTemperature * (9.0 / 5.0)) + 32.0;
    }
    
    displayNeedsUpdate = true;
    lastButtonPress = currentTime;
  }
}

void updateDisplay(float currentTemp, float targetTemp) {
  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(10, 50);
  tft.print(currentTemp, 1);
  tft.print(isCelsius ? " C" : " F");
  //tft.print("<-");
  tft.setCursor(10, 100);
  tft.print(targetTemp, 1);
  tft.print(isCelsius ? " C" : " F");
}

void controlTemperature(float currentTemp, float targetTemp) {
  digitalWrite(RELAY_PIN, currentTemp < targetTemp);
}

void loop() {
  static unsigned long lastDisplayUpdate = 0;
  static unsigned long lastControlTime = 0;
  
  float currentTemperature = isCelsius ? 
    dht.readTemperature() : 
    dht.readTemperature(true);

  // Update display if needed or periodically
  if (displayNeedsUpdate || (millis() - lastDisplayUpdate > DISPLAY_UPDATE_INTERVAL)) {
    updateDisplay(currentTemperature, targetTemperature);
    displayNeedsUpdate = false;
    lastDisplayUpdate = millis();
  }

  // Control temperature every 10 seconds
  if (millis() - lastControlTime >= CONTROL_INTERVAL) {
    controlTemperature(currentTemperature, targetTemperature);
    lastControlTime = millis();
  }
}
