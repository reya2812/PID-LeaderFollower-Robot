// orginal without adaptive

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
//  TUNING — start here
// ================================================================
const float DESIRED_CM  = 25.0f;

// Distance PID → v_base (PWM Base)
// Kp: main driving force, raise until it reaches target fast
// Ki: removes steady-state offset
// Kd: brakes before reaching target, reduces overshoot. Raise if still overshooting
const float Kp_dist = 15.2f;
const float Ki_dist = 4.0f;
const float Kd_dist = 3.7f;

// Alignment: cumulative tick drift → steering correction
// raise if robot curves, lower if it oscillates left-right
const float Kp_align = 0.3f;

// ================================================================
//  STATE
// ================================================================
float filteredDist = 0.0f;
float v_base       = 0.0f;

float integral_dist  = 0.0f;
float prevError_dist = 0.0f;

int32_t tickDrift      = 0;
int32_t prevLeftTicks  = 0;
int32_t prevRightTicks = 0;

unsigned long lastTime = 0;
unsigned long lastUS   = 0;

// ================================================================
//  MOTOR
// ================================================================
void setMotor(float L, float R) {
  L = constrain(L, -220, 220);
  R = constrain(R, -220, 220);
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
//  ULTRASONIC — 40 Hz
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
  if (dur == 0) {
    v_base = 220.0f;
    prevError_dist = 200.0f - DESIRED_CM;  // keep derivative consistent
    integral_dist  = constrain(integral_dist, -100.0f, 100.0f);
    return;
  }

  float d = dur * 0.0343f / 2.0f;

  // Out of range = same treatment as no echo
  if (d < 2.0f || d > 200.0f) {
    v_base = 220.0f;
    prevError_dist = 200.0f - DESIRED_CM;
    return;
  }

  // EMA filter
  filteredDist = 0.3f * d + 0.7f * filteredDist;

  // Distance PID
  float error = filteredDist - DESIRED_CM;

  // Derivative on error (D brakes as robot approaches target)
  float derivative = error - prevError_dist;   // per 50ms step, no /dt needed for tuning
  prevError_dist   = error;

  // Integral with simple clamp anti-windup
  integral_dist += error;
  integral_dist  = constrain(integral_dist, -100.0f, 100.0f);

  v_base = Kp_dist * error
         + Ki_dist * integral_dist * 0.025f   // 0.05 = dt of outer loop (40Hz)
         + Kd_dist * derivative / 0.025f;     // /dt for proper D scaling

  v_base = constrain(v_base, -220.0f, 220.0f);
}

// ================================================================
//  SETUP
// ================================================================
void setup() {
  Serial.begin(115200);

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

  lastTime = millis();
  lastUS   = millis();
}

// ================================================================
//  MAIN LOOP — 100 Hz
// ================================================================
void loop() {
  if (millis() - lastTime < 10) return;
  lastTime += 10;

  // Distance PID → v_base (40 Hz)
  updateUltrasonic();

  // Read encoders
  noInterrupts();
  int32_t left  = ticksLeft;
  int32_t right = ticksRight;
  interrupts();

  // Cumulative drift for alignment
  int32_t dL = left  - prevLeftTicks;
  int32_t dR = right - prevRightTicks;
  prevLeftTicks  = left;
  prevRightTicks = right;

  tickDrift += dL - dR;

  // Alignment correction — capped so it never fights v_base too hard
  float v_corr = constrain(Kp_align * tickDrift, -40.0f, 40.0f);

  float pwm_L = v_base - v_corr;
  float pwm_R = v_base + v_corr;

  setMotor(pwm_L, pwm_R);

  Serial.print("Dist: ");     Serial.print(filteredDist, 1);
  Serial.print("  err: ");    Serial.print(filteredDist - DESIRED_CM, 1);
  Serial.print("  v_base: "); Serial.print(v_base, 1);
  Serial.print("  drift: ");  Serial.print(tickDrift);
  Serial.print("  pwm_L: ");  Serial.print(pwm_L, 1);
  Serial.print("  pwm_R: ");  Serial.println(pwm_R, 1);
}