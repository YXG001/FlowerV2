#include <Arduino.h>
#include <math.h>

// FlowerV2 firmware prototype
// ESP32-S3 + 49E-family linear Hall sensors + potentiometer + Max/MSP
// Frame: seed_state kettle_state windmill_state knob_value seed_value kettle_value windmill_rpm knob_raw
//
// IMPORTANT: Confirm pinout, supply voltage and ADC-safe output of each Hall sensor.
// 49E-family parts from different vendors are not guaranteed to be interchangeable.

// ============================================================================
// USER CONFIGURATION / 用户常用配置
// ============================================================================
// ADC input choices on ESP32-S3:
//   ADC1: GPIO1=CH0, GPIO2=CH1, GPIO3=CH2 (*), GPIO4=CH3, GPIO5=CH4,
//         GPIO6=CH5, GPIO7=CH6, GPIO8=CH7, GPIO9=CH8, GPIO10=CH9.
//   Recommended: GPIO1, GPIO2, GPIO4-GPIO10. GPIO3 is a strapping pin (*).
//   ADC2: GPIO11-GPIO20 (check Wi-Fi/peripheral conflicts before using).
//   GPIO19/20 are also native USB D-/D+: avoid when using native USB.
//   GPIO0/45/46 are strapping pins and are not usable ADC inputs here.
//   Board variants may reserve some pins. All four analog pins must be unique.
//   Verify actual 49E sensor pinout and output voltage before wiring:
//   ESP32-S3 ADC pins are NOT 5V tolerant.
// ============================================================================

// 1. GPIO / 引脚定义（优先修改）
static constexpr uint8_t PIN_SEED     = 4;  // ADC1_CH3
static constexpr uint8_t PIN_KETTLE   = 5;  // ADC1_CH4
static constexpr uint8_t PIN_WINDMILL = 6;  // ADC1_CH5
static constexpr uint8_t PIN_KNOB     = 7;  // ADC1_CH6

// 2. Seed / Kettle / 种子与水壶的触发、释放阈值（%）
static constexpr float SEED_ON_PERCENT    = 5.0f;
static constexpr float SEED_OFF_PERCENT   = 3.0f;
static constexpr float KETTLE_ON_PERCENT  = 5.0f;
static constexpr float KETTLE_OFF_PERCENT = 3.0f;

// 3. Windmill / 风车磁铁与检测阈值
static constexpr uint8_t WINDMILL_PULSES_PER_REV = 3; // 3 magnets; each must produce 1 detected pulse
static constexpr int8_t  WINDMILL_PEAK_POLARITY  = 1;
static constexpr float   WINDMILL_ON_COUNTS      = 30.0f; // ADC counts (hardware trial)
static constexpr float   WINDMILL_OFF_COUNTS     = 10.0f;
static constexpr uint32_t WINDMILL_MIN_TIMEOUT_US     = 1500000;

// 4. Knob / 电位器分档
static constexpr int KNOB_ADC_MIN = 0;
static constexpr int KNOB_ADC_MAX = 4095;
static constexpr int KNOB_HYSTERESIS_COUNTS = 50;

// Threshold notes:
// - Seed/kettle ON must be higher than OFF (percent of no-magnet baseline).
// - Windmill ON must be higher than OFF (ADC counts, not percent).
// - WINDMILL_PULSES_PER_REV = real number of detected pulses per revolution.
// - WINDMILL_MIN_TIMEOUT_US controls the minimum stop-detection delay.
// - KNOB_ADC_MIN/MAX define usable potentiometer range; three equal sections.
// ============================================================================
// END USER CONFIGURATION / 下方是内部默认值，通常不需要修改
// ============================================================================

// Internal timing, filtering, ADC and calibration defaults
// These constants are intentionally outside the normal user configuration.
static constexpr uint32_t SERIAL_BAUD               = 115200;
static constexpr uint32_t SENSOR_SAMPLE_INTERVAL_MS = 20;    // 50 Hz
static constexpr uint32_t OUTPUT_INTERVAL_MS        = 50;    // 20 Hz
static constexpr uint32_t WINDMILL_SAMPLE_US        = 2000;  // nominal 500 Hz
static constexpr float    SENSOR_FILTER_ALPHA       = 0.20f; // FlowerV1 IIR
static constexpr float    WINDMILL_FILTER_ALPHA     = 0.35f;
static constexpr uint8_t  ADC_RESOLUTION_BITS        = 12;
static constexpr auto     ADC_INPUT_ATTENUATION      = ADC_11db;
static constexpr uint32_t SERIAL_STARTUP_DELAY_MS   = 500;
static constexpr uint32_t MAIN_LOOP_YIELD_MS        = 1;
static constexpr float   WINDMILL_BASELINE_ALPHA = 0.002f; // slow moving reference
static constexpr float   RPM_FILTER_ALPHA        = 0.35f;
static constexpr float   WINDMILL_STOP_PERIOD_FACTOR = 2.5f;
static constexpr uint32_t WINDMILL_REFRACTORY_US     = 8000;
static constexpr uint32_t WINDMILL_INITIAL_TIMEOUT_US = 8000000;
static constexpr uint32_t WINDMILL_MAX_TIMEOUT_US     = 20000000;
static constexpr uint16_t CALIBRATION_SAMPLES = 200;
static constexpr uint32_t CALIBRATION_INTERVAL_MS = 10;
static constexpr uint32_t CALIBRATION_SETTLE_DELAY_MS = 1000;
static constexpr uint16_t CALIBRATION_LED_TOGGLE_SAMPLES = 25;
static constexpr float HALL_ZERO_MIN_COUNTS = 1.0f;
static constexpr uint8_t STATUS_LED_BRIGHTNESS = 24;

constexpr bool isAdcCapableGpio(uint8_t pin) {
  return pin >= 1 && pin <= 20;
}
static_assert(isAdcCapableGpio(PIN_SEED) && isAdcCapableGpio(PIN_KETTLE) &&
              isAdcCapableGpio(PIN_WINDMILL) && isAdcCapableGpio(PIN_KNOB),
              "Selected GPIO must support ESP32-S3 ADC");
static_assert(PIN_SEED != PIN_KETTLE && PIN_SEED != PIN_WINDMILL &&
              PIN_SEED != PIN_KNOB && PIN_KETTLE != PIN_WINDMILL &&
              PIN_KETTLE != PIN_KNOB && PIN_WINDMILL != PIN_KNOB,
              "Each ADC input must use a unique GPIO");

static_assert(WINDMILL_PULSES_PER_REV > 0, "At least one pulse/revolution required");
static_assert(SEED_OFF_PERCENT < SEED_ON_PERCENT, "Seed thresholds reversed");
static_assert(KETTLE_OFF_PERCENT < KETTLE_ON_PERCENT, "Kettle thresholds reversed");
static_assert(WINDMILL_OFF_COUNTS < WINDMILL_ON_COUNTS, "Windmill thresholds reversed");
static_assert(KNOB_ADC_MAX > KNOB_ADC_MIN, "Knob ADC range invalid");

struct HallInput {
  explicit HallInput(uint8_t gpio) : pin(gpio) {}
  uint8_t pin;
  uint16_t raw = 0;
  float filtered = 0.0f;
  float zero = 0.0f;
  float percent = 0.0f;
  bool active = false;
};

static HallInput seed{PIN_SEED};
static HallInput kettle{PIN_KETTLE};

struct WindmillInput {
  uint16_t raw = 0;
  float filtered = 0.0f;
  float baseline = 0.0f;
  bool pulseHigh = false;
  bool hasPulse = false;
  bool periodValid = false;
  uint32_t lastPulseUs = 0;
  float filteredPeriodUs = 0.0f;
  float rpm = 0.0f;
  bool rotating = false;
};

static WindmillInput windmill;
static uint16_t knobRaw = 0; // latest unfiltered ADC count (0..4095)
static float knobFiltered = 0.0f;
static uint8_t knobValue = 0;

// 'd' toggles a diagnostic stream for the windmill Hall sensor.
// Normal operation continues to use the unchanged eight-field Max/MSP frame.
static bool windmillDebug = false;
static uint16_t windmillRawMin = 4095;
static uint16_t windmillRawMax = 0;
static float windmillResponseMin = 4095.0f;
static float windmillResponseMax = -4095.0f;
static float windmillLastResponse = 0.0f;
static uint32_t windmillDebugPulseCount = 0;

static uint32_t lastHallSampleMs = 0;
static uint32_t lastWheelSampleUs = 0;
static uint32_t lastOutputMs = 0;

// -----------------------------------------------------------------------------
// RGB status LED: same optional-board-variant approach as FlowerV1.
// -----------------------------------------------------------------------------
static void setRgb(uint8_t r, uint8_t g, uint8_t b) {
#if defined(RGB_BUILTIN)
  neopixelWrite(RGB_BUILTIN, r, g, b);
#else
  (void)r;
  (void)g;
  (void)b;
#endif
}

static void ledCalibrating(bool on) {
  setRgb(0, 0, on ? STATUS_LED_BRIGHTNESS : 0);
}

static void ledReady() {
  setRgb(0, STATUS_LED_BRIGHTNESS, 0);
}

static void ledError() {
  setRgb(STATUS_LED_BRIGHTNESS, 0, 0);
}

// -----------------------------------------------------------------------------
// Calibration
// Magnet-free state required for seed and kettle; do not rotate the windmill.
// Calibration pauses outgoing serial frames (about three seconds).
// -----------------------------------------------------------------------------
static bool calibrateSensors() {
  uint64_t seedSum = 0;
  uint64_t kettleSum = 0;
  uint64_t wheelSum = 0;

  ledCalibrating(true);
  delay(CALIBRATION_SETTLE_DELAY_MS);  // Allow the user to move magnetic objects away.

  for (uint16_t i = 0; i < CALIBRATION_SAMPLES; ++i) {
    if ((i % CALIBRATION_LED_TOGGLE_SAMPLES) == 0) {
      ledCalibrating(((i / CALIBRATION_LED_TOGGLE_SAMPLES) % 2) == 0);
    }
    seedSum += (uint16_t)analogRead(PIN_SEED);
    kettleSum += (uint16_t)analogRead(PIN_KETTLE);
    wheelSum += (uint16_t)analogRead(PIN_WINDMILL);
    delay(CALIBRATION_INTERVAL_MS);
  }

  seed.zero = (float)seedSum / CALIBRATION_SAMPLES;
  kettle.zero = (float)kettleSum / CALIBRATION_SAMPLES;
  const float wheelZero = (float)wheelSum / CALIBRATION_SAMPLES;

  seed.filtered = seed.zero;
  kettle.filtered = kettle.zero;
  seed.raw = (uint16_t)seed.zero;
  kettle.raw = (uint16_t)kettle.zero;
  seed.percent = 0.0f;
  kettle.percent = 0.0f;
  seed.active = false;
  kettle.active = false;

  windmill = WindmillInput{};
  windmill.filtered = wheelZero;
  windmill.baseline = wheelZero;
  windmill.raw = (uint16_t)wheelZero;

  knobRaw = (uint16_t)analogRead(PIN_KNOB);
  knobFiltered = (float)knobRaw;
  knobValue = 0;

  // Reset all timers so the blocked calibration interval is never a "pulse gap".
  lastHallSampleMs = millis();
  lastWheelSampleUs = micros();
  lastOutputMs = millis();

  // ADC readings near zero can indicate a disconnected sensor or incorrect wiring.
  const bool ok = seed.zero > HALL_ZERO_MIN_COUNTS &&
      kettle.zero > HALL_ZERO_MIN_COUNTS && wheelZero > HALL_ZERO_MIN_COUNTS;
  if (ok) ledReady();
  else ledError();
  return ok;
}

// -----------------------------------------------------------------------------
// Seed/kettle: FlowerV1-style low-pass, percent-from-baseline and hysteresis.
// -----------------------------------------------------------------------------
static void updateHall(HallInput &ch, float onPercent, float offPercent) {
  ch.raw = (uint16_t)analogRead(ch.pin);
  ch.filtered += SENSOR_FILTER_ALPHA * ((float)ch.raw - ch.filtered);
  ch.percent = (fabsf(ch.zero) > HALL_ZERO_MIN_COUNTS)
      ? ((ch.filtered - ch.zero) / ch.zero * 100.0f)
      : 0.0f;

  const float magnitude = fabsf(ch.percent);
  if (!ch.active && magnitude >= onPercent) {
    ch.active = true;
  } else if (ch.active && magnitude <= offPercent) {
    ch.active = false;
  }
}

// -----------------------------------------------------------------------------
// Potentiometer -> positions 0, 1, 2 with a small hysteresis around boundaries.
// -----------------------------------------------------------------------------
static void updateKnob() {
  knobRaw = (uint16_t)analogRead(PIN_KNOB);
  knobFiltered += SENSOR_FILTER_ALPHA * ((float)knobRaw - knobFiltered);

  const float step = (float)(KNOB_ADC_MAX - KNOB_ADC_MIN) / 3.0f;
  const float t01 = KNOB_ADC_MIN + step;
  const float t12 = KNOB_ADC_MIN + 2.0f * step;

  // Supports passing directly from one extreme to the other.
  while (knobValue < 2) {
    const float boundary = (knobValue == 0) ? t01 : t12;
    if (knobFiltered < boundary + KNOB_HYSTERESIS_COUNTS) break;
    ++knobValue;
  }
  while (knobValue > 0) {
    const float boundary = (knobValue == 2) ? t12 : t01;
    if (knobFiltered >= boundary - KNOB_HYSTERESIS_COUNTS) break;
    --knobValue;
  }
}

// -----------------------------------------------------------------------------
// Windmill: positive-peak detection with a slow reference (AC-like tracking).
// Unlike calibrating to a fixed rotor angle, the moving reference can gradually
// recover if the wheel stops with a magnet directly beside the Hall sensor.
// Requires separate, well-resolved peaks and suitable Hall/magnet polarity.
// -----------------------------------------------------------------------------
static void updateWindmill(uint32_t nowUs) {
  windmill.raw = (uint16_t)analogRead(PIN_WINDMILL);
  if (windmillDebug) {
    if (windmill.raw < windmillRawMin) windmillRawMin = windmill.raw;
    if (windmill.raw > windmillRawMax) windmillRawMax = windmill.raw;
  }
  windmill.filtered += WINDMILL_FILTER_ALPHA
      * ((float)windmill.raw - windmill.filtered);
  windmill.baseline += WINDMILL_BASELINE_ALPHA
      * (windmill.filtered - windmill.baseline);

  const float response = WINDMILL_PEAK_POLARITY
      * (windmill.filtered - windmill.baseline);
  if (windmillDebug) {
    windmillLastResponse = response;
    if (response < windmillResponseMin) windmillResponseMin = response;
    if (response > windmillResponseMax) windmillResponseMax = response;
  }

  if (windmill.pulseHigh) {
    if (response <= WINDMILL_OFF_COUNTS) {
      windmill.pulseHigh = false;
    }
    return;
  }
  if (response < WINDMILL_ON_COUNTS) return;

  if (windmill.hasPulse &&
      (uint32_t)(nowUs - windmill.lastPulseUs) < WINDMILL_REFRACTORY_US) {
    return;
  }

  windmill.pulseHigh = true;
  if (windmillDebug) ++windmillDebugPulseCount;
  if (windmill.hasPulse) {
    const uint32_t elapsedUs = (uint32_t)(nowUs - windmill.lastPulseUs);
    // A pulse after a long stop starts a new measurement.
    if (elapsedUs < WINDMILL_MAX_TIMEOUT_US) {
      if (!windmill.periodValid) {
        windmill.filteredPeriodUs = (float)elapsedUs;
        windmill.periodValid = true;
      } else {
        windmill.filteredPeriodUs += RPM_FILTER_ALPHA
            * ((float)elapsedUs - windmill.filteredPeriodUs);
      }
      windmill.rpm = 60000000.0f
          / (WINDMILL_PULSES_PER_REV * windmill.filteredPeriodUs);
    } else {
      windmill.periodValid = false;
      windmill.rpm = 0.0f;
    }
  }

  windmill.lastPulseUs = nowUs;
  windmill.hasPulse = true;
  windmill.rotating = true;
}

static void updateWindmillStop(uint32_t nowUs) {
  if (!windmill.hasPulse) {
    windmill.rotating = false;
    windmill.rpm = 0.0f;
    return;
  }

  uint32_t timeoutUs = WINDMILL_INITIAL_TIMEOUT_US;
  if (windmill.periodValid) {
    float adaptiveTimeout = windmill.filteredPeriodUs * WINDMILL_STOP_PERIOD_FACTOR;
    if (adaptiveTimeout < WINDMILL_MIN_TIMEOUT_US) {
      adaptiveTimeout = WINDMILL_MIN_TIMEOUT_US;
    }
    if (adaptiveTimeout > WINDMILL_MAX_TIMEOUT_US) {
      adaptiveTimeout = WINDMILL_MAX_TIMEOUT_US;
    }
    timeoutUs = (uint32_t)adaptiveTimeout;
  }

  if ((uint32_t)(nowUs - windmill.lastPulseUs) > timeoutUs) {
    windmill.rotating = false;
    windmill.rpm = 0.0f;
    windmill.hasPulse = false;
    windmill.periodValid = false;
    // Do not reset the magnetic filter/baseline: they adapt while stopped.
  }
}

// -----------------------------------------------------------------------------
// Max/MSP protocol: 8 ASCII numeric fields, one LF-terminated line.
// Fields 1-7 unchanged; field 8 is raw potentiometer ADC for calibration.
// No log messages are printed on the data serial connection.
// -----------------------------------------------------------------------------
static void sendFrame(uint32_t nowUs) {
  updateWindmillStop(nowUs);

  if (windmillDebug) {
    // W raw raw_min raw_max filtered baseline response response_min
    //   response_max pulses_since_debug_enable rotating rpm
    // Min/max summarize all ~500 Hz samples since the previous 20 Hz frame.
    Serial.print("W ");
    Serial.print(windmill.raw);
    Serial.print(' ');
    Serial.print(windmillRawMin);
    Serial.print(' ');
    Serial.print(windmillRawMax);
    Serial.print(' ');
    Serial.print(windmill.filtered, 1);
    Serial.print(' ');
    Serial.print(windmill.baseline, 1);
    Serial.print(' ');
    Serial.print(windmillLastResponse, 1);
    Serial.print(' ');
    Serial.print(windmillResponseMin, 1);
    Serial.print(' ');
    Serial.print(windmillResponseMax, 1);
    Serial.print(' ');
    Serial.print(windmillDebugPulseCount);
    Serial.print(' ');
    Serial.print(windmill.rotating ? 1 : 0);
    Serial.print(' ');
    Serial.println(windmill.rpm, 1);
    windmillRawMin = 4095;
    windmillRawMax = 0;
    windmillResponseMin = 4095.0f;
    windmillResponseMax = -4095.0f;
    return;
  }

  Serial.print(seed.active ? 1 : 0);
  Serial.print(' ');
  Serial.print(kettle.active ? 1 : 0);
  Serial.print(' ');
  Serial.print(windmill.rotating ? 1 : 0);
  Serial.print(' ');
  Serial.print(knobValue);
  Serial.print(' ');
  Serial.print(seed.percent, 2);
  Serial.print(' ');
  Serial.print(kettle.percent, 2);
  Serial.print(' ');
  Serial.print(windmill.rpm, 1);
  Serial.print(' ');
  Serial.println(knobRaw);
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(SERIAL_STARTUP_DELAY_MS);

  analogReadResolution(ADC_RESOLUTION_BITS);
  analogSetPinAttenuation(PIN_SEED, ADC_INPUT_ATTENUATION);
  analogSetPinAttenuation(PIN_KETTLE, ADC_INPUT_ATTENUATION);
  analogSetPinAttenuation(PIN_WINDMILL, ADC_INPUT_ATTENUATION);
  analogSetPinAttenuation(PIN_KNOB, ADC_INPUT_ATTENUATION);

  pinMode(PIN_SEED, INPUT);
  pinMode(PIN_KETTLE, INPUT);
  pinMode(PIN_WINDMILL, INPUT);
  pinMode(PIN_KNOB, INPUT);

  calibrateSensors();
}

void loop() {
  // Same c/C recalibration command as FlowerV1.
  while (Serial.available() > 0) {
    const char command = (char)Serial.read();
    if (command == 'c' || command == 'C') {
      calibrateSensors();
    } else if (command == 'd' || command == 'D') {
      windmillDebug = !windmillDebug;
      windmillDebugPulseCount = 0;
      windmillRawMin = 4095;
      windmillRawMax = 0;
      windmillResponseMin = 4095.0f;
      windmillResponseMax = -4095.0f;
    }
  }

  const uint32_t nowUs = micros();
  const uint32_t nowMs = millis();

  if ((uint32_t)(nowUs - lastWheelSampleUs) >= WINDMILL_SAMPLE_US) {
    lastWheelSampleUs = nowUs;
    updateWindmill(nowUs);
  }

  if ((uint32_t)(nowMs - lastHallSampleMs) >= SENSOR_SAMPLE_INTERVAL_MS) {
    lastHallSampleMs = nowMs;
    updateHall(seed, SEED_ON_PERCENT, SEED_OFF_PERCENT);
    updateHall(kettle, KETTLE_ON_PERCENT, KETTLE_OFF_PERCENT);
    updateKnob();
  }

  if ((uint32_t)(nowMs - lastOutputMs) >= OUTPUT_INTERVAL_MS) {
    lastOutputMs = nowMs;
    sendFrame(nowUs);
  }

  delay(MAIN_LOOP_YIELD_MS);  // Yield to ESP32; wheel ADC target remains ~500 Hz.
}
