// ================================================================
//  ROBOT FOLLOWER — with CSV serial output for analysis
//  Baud: 115200
//  CSV columns: time_ms, dist, error, v_base, pwm_L, pwm_R,
//               drift, rms_error, iae, overshoot
// ================================================================
#include "hardware/gpio.h"

// ================================================================
//  PINS
// ================================================================
#define ENCA_L   2
#define ENCB_L   3
#define ENCA_R   14
#define ENCB_R   15
#define M1_IN1   16
#define M1_IN2   17
#define M2_IN3   18
#define M2_IN4   19
#define TRIG_PIN 20
#define ECHO_PIN 21

// ================================================================
//  ENCODER
// ================================================================
volatile int32_t ticksLeft  = 0;
volatile int32_t ticksRight = 0;
volatile uint8_t prevLeft   = 0;
volatile uint8_t prevRight  = 0;

const int8_t quadTable[16] = {
   0, +1, -1,  0,
  -1,  0,  0, +1,
  +1,  0,  0, -1,
   0, -1, +1,  0
};

inline uint8_t readLeft()  { return (gpio_get(ENCA_L) << 1) | gpio_get(ENCB_L); }
inline uint8_t readRight() { return (gpio_get(ENCA_R) << 1) | gpio_get(ENCB_R); }

void isrLeft() {
  uint8_t curr = readLeft();
  ticksLeft += quadTable[(prevLeft << 2) | curr];
  prevLeft = curr;
}
void isrRight() {
  uint8_t curr = readRight();
  ticksRight += quadTable[(prevRight << 2) | curr];
  prevRight = curr;
}

// ================================================================
//  TUNING
// ================================================================
const float DESIRED_CM  = 25.0f;
const float Kp_dist     = 13.0f;
const float Ki_dist     = 0.0f;
const float Kd_dist     = 0.0f;
const float Kp_align    = 0.3f;

// ================================================================
//  STATE
// ================================================================
float filteredDist   = 25.0f;
float v_base         = 0.0f;
float integral_dist  = 0.0f;
float prevError_dist = 0.0f;

int32_t tickDrift      = 0;
int32_t prevLeftTicks  = 0;
int32_t prevRightTicks = 0;

unsigned long lastTime  = 0;
unsigned long lastUS    = 0;
unsigned long startTime = 0;

// ================================================================
//  ERROR METRICS STATE
// ================================================================
float  sumSquaredError = 0.0f;
float  iae             = 0.0f;     // integral of absolute error
int    errorCount      = 0;
float  peakDist        = 0.0f;    // for overshoot calculation
float  rmsError        = 0.0f;    // updated every 1s
float  overshootPct    = 0.0f;
bool   testPhase       = false;   // flag: set true when you start a test
float  phaseStartDist  = 0.0f;   // distance at start of test phase

// ================================================================
//  MOTOR
// ================================================================
void setMotor(float L, float R) {
  L = constrain(L, -240, 240);
  R = constrain(R, -240, 240);
  analogWrite(M1_IN1, L > 0 ? (int)L : 0);
  analogWrite(M1_IN2, L < 0 ? (int)(-L) : 0);
  analogWrite(M2_IN3, R > 0 ? (int)R : 0);
  analogWrite(M2_IN4, R < 0 ? (int)(-R) : 0);
}

void stopMotors() {
  analogWrite(M1_IN1, 0); analogWrite(M1_IN2, 0);
  analogWrite(M2_IN3, 0); analogWrite(M2_IN4, 0);
}

// ================================================================
//  ULTRASONIC
// ================================================================
void updateUltrasonic() {
  if (millis() - lastUS < 25) return;
  lastUS = millis();

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long dur = pulseIn(ECHO_PIN, HIGH, 15000UL);

  float d = (dur > 0) ? dur * 0.0343f / 2.0f : 999.0f;

  if (d < 2.0f || d > 80.0f) {
    filteredDist   = 80.0f;
    v_base         = 240.0f;
    prevError_dist = 80.0f - DESIRED_CM;
    integral_dist  = 100.0f;
    return;
  }

  filteredDist = 0.3f * d + 0.7f * filteredDist;

  float error      = filteredDist - DESIRED_CM;
  float derivative = error - prevError_dist;
  prevError_dist   = error;

  integral_dist   += error;
  integral_dist    = constrain(integral_dist, -100.0f, 100.0f);

  v_base = Kp_dist * error
         + Ki_dist * integral_dist * 0.025f
         + Kd_dist * derivative    / 0.025f;

  v_base = constrain(v_base, -240.0f, 240.0f);
}

// ================================================================
//  ERROR METRICS — update every loop iteration
// ================================================================
void updateMetrics(float error) {
  sumSquaredError += error * error;
  iae             += fabsf(error) * 0.01f;   // × dt (100 Hz)
  errorCount++;

  // Track peak distance for overshoot (only positive direction)
  if (filteredDist > peakDist) peakDist = filteredDist;

  // Overshoot % relative to desired
  if (peakDist > DESIRED_CM)
    overshootPct = (peakDist - DESIRED_CM) / DESIRED_CM * 100.0f;

  // Update RMS every 1 second (100 iterations)
  if (errorCount >= 100) {
    rmsError = sqrt(sumSquaredError / errorCount);
    sumSquaredError = 0;
    errorCount      = 0;
  }
}

// ================================================================
//  SETUP
// ================================================================
void setup() {
  Serial.begin(115200);
  delay(500);

  // Print CSV header
  Serial.println("time_ms,dist_cm,error_cm,v_base,pwm_L,pwm_R,drift,rms_error,iae,overshoot_pct");

  pinMode(ENCA_L, INPUT_PULLUP); pinMode(ENCB_L, INPUT_PULLUP);
  pinMode(ENCA_R, INPUT_PULLUP); pinMode(ENCB_R, INPUT_PULLUP);

  prevLeft  = readLeft();
  prevRight = readRight();

  attachInterrupt(digitalPinToInterrupt(ENCA_L), isrLeft,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCB_L), isrLeft,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCA_R), isrRight, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCB_R), isrRight, CHANGE);

  pinMode(M1_IN1, OUTPUT); pinMode(M1_IN2, OUTPUT);
  pinMode(M2_IN3, OUTPUT); pinMode(M2_IN4, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  stopMotors();
  delay(2000);

  startTime = millis();
  lastTime  = millis();
  lastUS    = millis();
}

// ================================================================
//  MAIN LOOP — 100 Hz
// ================================================================
void loop() {
  if (millis() - lastTime < 10) return;
  lastTime += 10;

  updateUltrasonic();

  noInterrupts();
  int32_t left  = ticksLeft;
  int32_t right = ticksRight;
  interrupts();

  int32_t dL = left  - prevLeftTicks;
  int32_t dR = right - prevRightTicks;
  prevLeftTicks  = left;
  prevRightTicks = right;

  tickDrift += dL - dR;

  float v_corr = constrain(Kp_align * tickDrift, -40.0f, 40.0f);

  float pwm_L = v_base - v_corr;
  float pwm_R = v_base + v_corr;

  setMotor(pwm_L, pwm_R);

  float error = filteredDist - DESIRED_CM;
  updateMetrics(error);

  // ── CSV output every iteration ──
  Serial.print(millis() - startTime);  Serial.print(",");
  Serial.print(filteredDist, 2);       Serial.print(",");
  Serial.print(error, 2);              Serial.print(",");
  Serial.print(v_base, 1);            Serial.print(",");
  Serial.print(constrain(pwm_L, -240, 240), 1); Serial.print(",");
  Serial.print(constrain(pwm_R, -240, 240), 1); Serial.print(",");
  Serial.print(tickDrift);             Serial.print(",");
  Serial.print(rmsError, 3);          Serial.print(",");
  Serial.print(iae, 3);               Serial.print(",");
  Serial.println(overshootPct, 2);
}
