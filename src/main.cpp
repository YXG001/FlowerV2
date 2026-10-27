#include <Arduino.h>
#include <math.h>

// FlowerV2 firmware prototype
// ESP32-S3 + 49E-family linear Hall sensors + potentiometer + Max/MSP
// Frame: seed_state kettle_state windmill_state knob_value seed_value kettle_value windmill_rpm
//
// IMPORTANT: Confirm pinout, supply voltage and ADC-safe output of each Hall sensor.
// 49E-family parts from different vendors are not guaranteed to be interchangeable.

// -----------------------------------------------------------------------------
// Hardware: default pins are ESP32-S3 ADC1-capable GPIOs. Change to match wiring.
// -----------------------------------------------------------------------------
static constexpr uint8_t PIN_SEED     = 4;  // ADC1_CH3
static constexpr uint8_t PIN_KETTLE   = 5;  // ADC1_CH4
static constexpr uint8_t PIN_WINDMILL = 6;  // ADC1_CH5
static constexpr uint8_t PIN_KNOB     = 7;  // ADC1_CH6

// -----------------------------------------------------------------------------
// Communication and sampling (FlowerV1-compatible framing)
// -----------------------------------------------------------------------------
static constexpr uint32_t SERIAL_BAUD               = 115200;
static constexpr uint32_t SENSOR_SAMPLE_INTERVAL_MS = 20;    // 50 Hz
static constexpr uint32_t OUTPUT_INTERVAL_MS        = 50;    // 20 Hz
static constexpr uint32_t WINDMILL_SAMPLE_US        = 2000;  // nominal 500 Hz
static constexpr float    SENSOR_FILTER_ALPHA       = 0.20f; // FlowerV1 IIR
static constexpr float    WINDMILL_FILTER_ALPHA     = 0.35f;

// Initial thresholds: EXAMPLES ONLY. Tune these with the actual magnets/sensors.
// Percent is relative to each Hall sensor's startup baseline (same as FlowerV1).
static constexpr float SEED_ON_PERCENT    = 5.0f;
static constexpr float SEED_OFF_PERCENT   = 3.0f;
static constexpr float KETTLE_ON_PERCENT  = 5.0f;
static constexpr float KETTLE_OFF_PERCENT = 3.0f;

// Windmill: assume one positive magnetic peak per magnet in the wheel.
// Use -1 if the selected magnet polarity gives negative peaks.
// Magnet count MUST equal the number of reliably counted pulses per revolution.
static constexpr uint8_t WINDMILL_PULSES_PER_REV = 8; // tentative 8-magnet design
static constexpr int8_t  WINDMILL_PEAK_POLARITY  = 1;
static constexpr float   WINDMILL_ON_COUNTS      = 65.0f; // ADC counts
static constexpr float   WINDMILL_OFF_COUNTS     = 30.0f;
static constexpr float   WINDMILL_BASELINE_ALPHA = 0.002f; // slow moving reference
static constexpr float   RPM_FILTER_ALPHA        = 0.35f;
static constexpr uint32_t WINDMILL_REFRACTORY_US     = 8000;
static constexpr uint32_t WINDMILL_INITIAL_TIMEOUT_US = 8000000;
static constexpr uint32_t WINDMILL_MIN_TIMEOUT_US     = 1500000;
static constexpr uint32_t WINDMILL_MAX_TIMEOUT_US     = 20000000;

// Knob: analog potentiometer mapped to three discrete positions 0/1/2.
// Tune ADC endpoints when the real potentiometer is connected.
static constexpr int KNOB_ADC_MIN = 0;
static constexpr int KNOB_ADC_MAX = 4095;
static constexpr int KNOB_HYSTERESIS_COUNTS = 50;

// FlowerV1-style startup calibration and ASCII 'c'/'C' recalibration.
static constexpr uint16_t CALIBRATION_SAMPLES = 200;
static constexpr uint32_t CALIBRATION_INTERVAL_MS = 10;

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

static HallInput seed = {PIN_SEED};
static HallInput kettle = {PIN_KETTLE};

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
static float knobFiltered = 0.0f;
static uint8_t knobValue = 0;

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
  setRgb(0, 0, on ? 24 : 0);
}

static void ledReady() {
  setRgb(0, 24, 0);
}

static void ledError() {
  setRgb(24, 0, 0);
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
  delay(1000);  // Allow the user to move magnetic objects away.

  for (uint16_t i = 0; i < CALIBRATION_SAMPLES; ++i) {
    if ((i % 25) == 0) {
      ledCalibrating(((i / 25) % 2) == 0);
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

  knobFiltered = (float)analogRead(PIN_KNOB);
  knobValue = 0;

  // Reset all timers so the blocked calibration interval is never a "pulse gap".
  lastHallSampleMs = millis();
  lastWheelSampleUs = micros();
  lastOutputMs = millis();

  // ADC readings near zero can indicate a disconnected sensor or incorrect wiring.
  const bool ok = seed.zero > 1.0f && kettle.zero > 1.0f && wheelZero > 1.0f;
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
  ch.percent = (fabsf(ch.zero) > 1.0f)
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
  const float raw = (float)analogRead(PIN_KNOB);
  knobFiltered += SENSOR_FILTER_ALPHA * (raw - knobFiltered);

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
  windmill.filtered += WINDMILL_FILTER_ALPHA
      * ((float)windmill.raw - windmill.filtered);
  windmill.baseline += WINDMILL_BASELINE_ALPHA
      * (windmill.filtered - windmill.baseline);

  const float response = WINDMILL_PEAK_POLARITY
      * (windmill.filtered - windmill.baseline);

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
    float adaptiveTimeout = windmill.filteredPeriodUs * 2.5f;
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
// Max/MSP protocol: EXACTLY 7 ASCII numeric fields, one LF-terminated line.
// No log messages are printed on the data serial connection.
// -----------------------------------------------------------------------------
static void sendFrame(uint32_t nowUs) {
  updateWindmillStop(nowUs);

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
  Serial.println(windmill.rpm, 1);
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(500);

  analogReadResolution(12);
  analogSetPinAttenuation(PIN_SEED, ADC_11db);
  analogSetPinAttenuation(PIN_KETTLE, ADC_11db);
  analogSetPinAttenuation(PIN_WINDMILL, ADC_11db);
  analogSetPinAttenuation(PIN_KNOB, ADC_11db);

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

  delay(1);  // Yield to the ESP32 system; wheel ADC target remains ~500 Hz.
}
