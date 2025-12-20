#include <Adafruit_ST7735.h>
#include <Fonts/FreeSans12pt7b.h>
#include <SPI.h>                 // ESP32 core SPI
#include <Preferences.h>         // for saving data
#include "rotary.h"
#include <esp_sleep.h>
#include <esp_pm.h>
#include "async.h"

// Preferences instance for non-volatile storage on ESP32
static Preferences prefs;

// === ESP32-C3 Super mini Pin Mapping ===
#define THERM_PIN       0  // Safe -- connected to thermistor
#define RELAY_PIN       21 // Safe -- connected to HW-040 relay module
#define VREF_PIN        1  // Safe -- connected to voltage reference to correct for voltage dips

// Rotary encoder -- grouped on the right side
#define ROTARY_SW       2  // Strapping: should default high
#define ROTARY_CLK      3  // Safe
#define ROTARY_DT       4  // Safe

// TFT SPI -- grouped on the left side
#define TFT_DC          7   // Safe
#define BACKLIGHT_PIN   5   // Safe
#define TFT_CS          6   // Safe
#define TFT_MOSI        8   // SDA-Strapping if GPIO2 is low
#define TFT_SCK         10  // SCL-Safe
#define TFT_MISO       -1   // Not used

// === Notes ===
// - All pins are 3.3V only (no 5V tolerance).

// ADC config
// measured 2.4V on a 3.3V rail as the reference
#define VREF_K          2.41f/3.3f
#define SAMPLES_TOTAL   64
#define SAMPLE_DELAY_MS 10

// Thermistor constants -- use a 100K NTC 3950
// #define R_FIXED 94200.0f    // ~100kΩ reference resistor
// #define R0      125000.0f   // 100kΩ at 25 °C
#define R_FIXED 100000.0f    // ~100kΩ reference resistor
#define R0      100000.0f   // 100kΩ at 25 °C
#define BETA    3950.0f
#define T0      298.15f     // 25 °C in Kelvin

// EMA smoothing factor
#define EMA_ALPHA 0.1f   // adjust between 0.05–0.2 depending on smoothness

#define DISPLAY_UPDATE_INTERVAL 200
#define CONTROL_INTERVAL 10000
#define HYSTERESIS_TENTHS 5      // 0.5°C hysteresis
#define ENCODER_STEP_TENTHS 1    // 0.1°C per encoder step
#define ENCODER_PULSES_PER_STEP 2 // quadrature pulses per mechanical detent (tune if needed)
#define BUTTON_DEBOUNCE_MS 50
#define LONG_PRESS_MS 1000 // ms for long press to toggle units

/* Persistence (Preferences) */
#define SAVE_INTERVAL_MS 5000    // minimum interval between non-volatile writes

// Power management
#define WDT_SLEEP_S 8            // WDT sleep interval in seconds (LowPower SLEEP_8S)
#define CONTROL_INTERVAL_S 10    // control interval in seconds (approximately)
#define DISPLAY_ON_AFTER_WAKE_MS 10000 // keep display on for some time after wake for user feedback
#define COMPRESSOR_MIN_OFF_MS (4UL * 60UL * 1000UL) // 4 minutes minimum off time for compressor starter
#define THERM_START_DELAY 10000

#define _println(text)
#define _printf(fmt, arg0)
//#define _println(text) if (Serial && Serial.isConnected()) Serial._println(text)
//#define _printf(fmt, arg0) if (Serial && Serial.isConnected()) Serial.printf(fmt, arg0)

// RST tied HIGH on module; pass -1 to disable software reset
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, -1);

// Thermistor async function state
typedef struct {
    async_state;                // the async state
    int sampleCount;            // the number of samples taken
    uint32_t temp;              // accumulated temperature reading
    unsigned long lastSampleMs; // last time a sample was taken
    bool expMovingAvg;          // smoothing is enabled
    float vRef;                 // voltage divider reference
} ThermistorState;

ThermistorState thermState;

// Internal state stored in tenths of °C to avoid non-atomic float access
volatile int16_t targetTenthsC = 250; // tenths of a degree C = 25.0°C

// Quadrature decoder state (4-bit history) and pulse accumulator.
// ISR calls rotary_step() and increments encoderPulseCount.
volatile uint8_t rot_state = 0;
volatile int16_t encoderPulseCount = 0;
 
// Persistence helpers
unsigned long lastSaveMs = 0;

// Compressor/relay safety
bool relayOn = false;
unsigned long lastRelayOffMs = 0;

bool displayCelsius = true;
unsigned long displayOnMs = 0;
unsigned long wdtWakeCount = 0;
float lastValidTempC = NAN;
bool lastButtonState = HIGH;
unsigned long lastButtonMs = 0;
unsigned long pressStartMs = 0;

/// @brief Flag indicating that the target temperature has changed.
/// @remarks Must be a global because changes could happen before the last save window elapses
bool targetDirty = false;
bool unitsDirty = false; // true when the units selection needs persisting

// fixed width text bounds
uint16_t textWidth, textHeight;

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
  // set this LOW immediately so the compressor doesn't immediately turn on
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);

  //Serial.begin(115200, SERIAL_8N1, -1);
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);
  //delay(3000);
  _println("Program started...");

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

  pinMode(THERM_PIN, INPUT);
  pinMode(VREF_PIN, INPUT);
  pinMode(ROTARY_CLK, INPUT_PULLUP);
  pinMode(ROTARY_DT, INPUT_PULLUP);
  pinMode(ROTARY_SW, INPUT_PULLUP);

  // Backlight control pin (module BLK has onboard transistor)
  pinMode(BACKLIGHT_PIN, OUTPUT);
  digitalWrite(BACKLIGHT_PIN, HIGH);

  unsigned long now = millis();
  displayOnMs = now + DISPLAY_ON_AFTER_WAKE_MS;

  attachInterrupt(digitalPinToInterrupt(ROTARY_CLK), encoderISR, CHANGE);
  // Also attach to DT so the ISR runs on changes of either channel (better capture on fast/cheap encoders)
  attachInterrupt(digitalPinToInterrupt(ROTARY_DT), encoderISR, CHANGE);
  // Wake-only interrupt for rotary pushbutton (FALLING edge). ISR does nothing
  // beyond waking the MCU; actual button processing is done in the loop.
  attachInterrupt(digitalPinToInterrupt(ROTARY_SW), wakeISR, FALLING);

  // initialize relay OFF (assumes active HIGH; invert logic if needed)
  //FIXME: should maybe check if relay is already on and needs to be?
  digitalWrite(RELAY_PIN, LOW);

  // Load persisted target temperature (tenths of °C). Validate range, otherwise set to current measured temp if available.
  prefs.begin("fridge", false);

  // Try to load saved target
  int16_t saved = prefs.getShort("targetC", INT16_MIN);
  targetTenthsC = saved >= -400 && saved <= 500
    ? saved
    : 250;  // default 25.0C

  // Load persisted units selection (true = Celsius). Default to Celsius if missing.
  displayCelsius = prefs.getBool("unitsC", true);

  // keep Preferences open for the lifetime of the program (do not end here)
  // prefs.end(); // removed to keep prefs active

  // initialize save timestamp to avoid immediate write
  lastSaveMs = now;

  // initialize the thermistor task
  async_init(&thermState);
  thermState.expMovingAvg = false;

  // let system settle for THERM_START_DELAY before reading
  thermState.lastSampleMs = now + THERM_START_DELAY;

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
    tft.printf(" - - - - - ");
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
  //_println("updating display...");

  // Target temperature (right) in light blue - also fixed-width
  uint16_t lightBlue = tft.color565(120, 120, 255); // light blue (RGB: 120,120,255)
  float tgtC = targetTenths / 10.0;
  show(tgtC, 0, 50, lightBlue);

  // Current temperature (left) - print fixed-width numeric + unit to avoid residual chars
  show(currentC, 0, 130, ST77XX_WHITE);

  // Compressor running indicator (snowflake) between the values
  // Draw a simple snowflake icon centered at (40, 80)
  int cx = 40, cy = 80, size = 10;
  //tft.fillRect(0, cy - size - 1, 80, size + 1, ST77XX_RED);
  tft.fillRect(0, cy - size, 80, 2*size, ST77XX_BLACK);
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

/// @brief  Read the thermistor value.
/// @param s    The thermistor state.
/// @param now  The current time, bound to mills().
/// @return     The async function status.
async readThermistor(ThermistorState *s, unsigned long now, float* temp) {
    async_begin(s);

    s->sampleCount = 0;
    s->temp = 0;
    s->vRef = 0;

    while (s->sampleCount < SAMPLES_TOTAL) {
        // wait between samples for SAMPLE_DELAY_MS
        // also wait at boot for THERM_START_DELAY to start taking measurements
        // to let the system settle
        await(now - s->lastSampleMs < THERM_START_DELAY
           && now - s->lastSampleMs >= SAMPLE_DELAY_MS);
        s->lastSampleMs = now;

        // Read thermistor node
        (void)analogRead(THERM_PIN);   // throw-away after mux switch
        delayMicroseconds(300);
        s->temp += analogRead(THERM_PIN);
        
        // Read 3.3V sense node
        (void)analogRead(VREF_PIN);    // throw-away after mux switch
        delayMicroseconds(300);
        s->vRef += analogRead(VREF_PIN);
        s->sampleCount++;
    }

    // Compute ratiometric average ADC value using a voltage reference on another ADC pin
    {
      float vTemp = (float)s->temp / SAMPLES_TOTAL;
      float vRef = (float)s->vRef / SAMPLES_TOTAL;

      // Compute thermistor resistance
      float rTherm = R_FIXED * (vRef / (vTemp * VREF_K) - 1.0f);

      // Beta equation
      float invT = (1.0f / T0) + (1.0f / BETA) * logf(rTherm / R0);
      float tempK = 1.0f / invT;
      float tempC = tempK - 273.15f - 1.0f; // +1.0C bias as measured by ice slurry
      _printf("Temp: %f", tempC);
      if (s->expMovingAvg) {
        *temp = EMA_ALPHA * tempC  + (1.0f - EMA_ALPHA) * *temp;
      } else {
        s->expMovingAvg = true;
        *temp = tempC;
      }
    }
    async_end;
}

/// @brief Relay control function
/// @returns True if the comrpessor has changed state, false otherwise.
bool controlTemperature(unsigned long now, float currentC, int16_t targetTenths) {
  // Skip if the reading is invalid or if the relay is off and it was on is before the min cool-off
  if (isnan(currentC) || !relayOn && (now - lastRelayOffMs) < COMPRESSOR_MIN_OFF_MS)
    return false;

  int16_t currentTenths = (int16_t)round(currentC * 10.0);

  // Need to turn ON when current is above target plus hysteresis.
  if (!relayOn && currentTenths >= (targetTenths + HYSTERESIS_TENTHS)) {
    // Only allow starting compressor if it is currently off and the minimum off time has elapsed.
    digitalWrite(RELAY_PIN, HIGH);
    relayOn = true;
    return true;
  } else if (relayOn && currentTenths <= (targetTenths - HYSTERESIS_TENTHS)) {
    // Need to turn OFF when current is below target minus hysteresis.
    digitalWrite(RELAY_PIN, LOW);
    relayOn = false;
    lastRelayOffMs = now;
    return true;
  }
  return false;
}

void loop() {
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
        if (newTarget < -100) newTarget = -100;
        if (newTarget > 500)  newTarget = 500;
        if (newTarget != targetTenthsC) {
          targetTenthsC = newTarget;
          targetDirty = true;
          displayDirty = true;
        }
      }
    }
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
      unitsDirty = true;            // mark units changed so we persist it
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
  // This MUST be a <= comparison as two subsequent iterations might have the same `now` value.
  // While pulses are being registered, this keeps pushing `displayOnMs` forwards and the subsequent
  // loop iterations pull the pin LOW, eventually pushing it HIGH once no pulses are recorded and
  // `now` advances slightly.  This ends up looking like a PWM signal and the screen noticeably
  // dims as a result.
  bool displayOn = (displayOnMs - now) <= (unsigned long)DISPLAY_ON_AFTER_WAKE_MS;
  digitalWrite(BACKLIGHT_PIN, displayOn ? HIGH : LOW);
  
  //  Put MCU to low-power sleep if idle: wake sources are external INT (encoder/button) and timer (approx WDT)
  //  Only sleep if no recent activity and display not required.
  bool idle = (pulses == 0) && !targetDirty && !displayDirty;
  if (!idle) {
    displayOnMs = now + DISPLAY_ON_AFTER_WAKE_MS;
  } else if (!displayOn) {
    // // Configure timer wake for approximately WDT_SLEEP_S seconds (esp_light_sleep)
    // esp_sleep_enable_timer_wakeup((uint64_t)WDT_SLEEP_S * 1000000ULL);
    // // Enter light sleep; external interrupts attached with attachInterrupt() will also wake the chip
    // esp_light_sleep_start();
    // Woke up (either via timer or external INT)
    // wdtWakeCount++;
    // keep display on briefly after wake for feedback
    // displayOnMs = now + DISPLAY_ON_AFTER_WAKE_MS;
  }

  // measure the current temperature
  float t = lastValidTempC;
  if (readThermistor(&thermState, now, &t) == ASYNC_DONE) {
    displayDirty = isnan(lastValidTempC) || abs(t - lastValidTempC) > 0.1f;
    lastValidTempC = t;
    async_init(&thermState);
  }
  
  // Persist target temperature occasionally (rate-limited to SAVE_INTERVAL_MS)
  if ((targetDirty || unitsDirty) && (now - lastSaveMs >= SAVE_INTERVAL_MS)) {
    // Local copies to avoid races with ISRs/other logic
    int16_t toSave = targetTenthsC;
    bool unitsToSave = displayCelsius;
    _println("Save settings");
    // Preferences is already opened in setup and kept open; just write the values
    prefs.putShort("targetC", toSave);
    prefs.putBool("unitsC", unitsToSave);
    lastSaveMs = now;
    targetDirty = false;   // reset dirty state
    unitsDirty = false;
  }
  
  // Relay control based on temperature -- it should monitor temperature regardless
  // of whether something has changed, it should only depend on the amount of time
  // the relay has been off. Mark the display as dirty if the relay changes state.
  displayDirty |= controlTemperature(now, lastValidTempC, targetTenthsC);

  // Single display update point: update once per loop if any condition requested it.
  if (displayDirty) {
    updateDisplay(lastValidTempC, targetTenthsC);
  }
}
