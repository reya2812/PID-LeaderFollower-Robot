// SDC Lab task 2 version 2
// Straight Line Position PID control

// Cascaded PID Control (Position + Alignment + Velocity)

// Distance error (ed):
// ed = targetCounts − (left + right)/2
// Position PID gives target velocity (v)

// Alignment error (ea):
// ea = left − right
// Alignment PID gives correction (Δv)

// Target velocities:
// leftVel  = v − Δv
// rightVel = v + Δv

// Velocity PID (per motor):
// PWM_left  = PID(leftVel  − actualVel_left)
// PWM_right = PID(rightVel − actualVel_right)

// ---------------- ENCODERS ----------------
const int encoder1A = 13;
const int encoder1B = 12;
const int encoder2A = 19;
const int encoder2B = 18;

volatile long encoderCount1 = 0;
volatile long encoderCount2 = 0;
volatile byte prevState1 = 0;
volatile byte prevState2 = 0;

// ---------------- MOTORS ----------------
const int M1_IN1 = 15;
const int M1_IN2 = 14;
const int M2_IN3 = 16;
const int M2_IN4 = 17;

// ---------------- ROBOT PARAMS ----------------
const float radius = 3.5; // cm
const long countsPerRev = 1320;
float distPerCount = (2 * PI * radius) / countsPerRev;

// ---------------- PID GAINS ----------------
// Position PID → gives velocity target
float Kp_pos = 0.8;
float Ki_pos = 0.0;
float Kd_pos = 0.2;

// Alignment PID → gives velocity correction
float Kp_align = 0.5;
float Ki_align = 0.0;
float Kd_align = 0.1;

// Velocity PID (per motor)
float Kp_vel = 2.0;
float Ki_vel = 0.5;
float Kd_vel = 0.05;

// ---------------- STATE ----------------
float prevError_pos = 0, integral_pos = 0;

float prevError_align = 0, integral_align = 0;

float prevError_vel_L = 0, integral_vel_L = 0;
float prevError_vel_R = 0, integral_vel_R = 0;

long prevLeft = 0, prevRight = 0;

unsigned long prevTime = 0;

// ---------------- SETUP ----------------
void setup() {
  pinMode(encoder1A, INPUT);
  pinMode(encoder1B, INPUT);
  pinMode(encoder2A, INPUT);
  pinMode(encoder2B, INPUT);

  prevState1 = (digitalRead(encoder1A) << 1) | digitalRead(encoder1B);
  prevState2 = (digitalRead(encoder2A) << 1) | digitalRead(encoder2B);

  attachInterrupt(digitalPinToInterrupt(encoder1A), updateEncoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder1B), updateEncoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder2A), updateEncoder2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder2B), updateEncoder2, CHANGE);

  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);
  pinMode(M2_IN3, OUTPUT);
  pinMode(M2_IN4, OUTPUT);

  delay(2000);
}

// ---------------- MAIN ----------------
void loop() {
  moveStraightCascaded(20.0); // move 20 cm
  while (true);
}

// ---------------- MAIN CONTROL ----------------
void moveStraightCascaded(float target_cm) {

  long startLeft = encoderCount1;
  long startRight = encoderCount2;

  float targetCounts = target_cm / distPerCount;

  prevTime = millis();
  prevLeft = 0;
  prevRight = 0;

  while (true) {

    // -------- TIME --------
    unsigned long now = millis();
    float dt = (now - prevTime) / 1000.0;
    prevTime = now;

    // -------- ENCODER READ --------
    noInterrupts();
    long left = encoderCount1 - startLeft;
    long right = encoderCount2 - startRight;
    interrupts();

    float avgCounts = (left + right) / 2.0;

    // -------- POSITION PID --------
    float error_pos = targetCounts - avgCounts;

    integral_pos += error_pos * dt;
    float derivative_pos = (error_pos - prevError_pos) / dt;

    float targetVel = Kp_pos * error_pos + Ki_pos * integral_pos + Kd_pos * derivative_pos;

    prevError_pos = error_pos;

    // -------- ALIGNMENT PID --------
    float error_align = left - right;

    integral_align += error_align * dt;
    float derivative_align = (error_align - prevError_align) / dt;

    float correction = Kp_align * error_align + Ki_align * integral_align + Kd_align * derivative_align;

    prevError_align = error_align;

    // -------- TARGET VELOCITIES --------
    float targetVel_L = targetVel - correction;
    float targetVel_R = targetVel + correction;

    // -------- ACTUAL VELOCITIES --------
    float vel_L = (left - prevLeft) / dt;
    float vel_R = (right - prevRight) / dt;

    prevLeft = left;
    prevRight = right;

    // -------- VELOCITY PID (LEFT) --------
    float error_vel_L = targetVel_L - vel_L;

    integral_vel_L += error_vel_L * dt;
    float derivative_vel_L = (error_vel_L - prevError_vel_L) / dt;

    float pwm_L = Kp_vel * error_vel_L + Ki_vel * integral_vel_L + Kd_vel * derivative_vel_L;

    prevError_vel_L = error_vel_L;

    // -------- VELOCITY PID (RIGHT) --------
    float error_vel_R = targetVel_R - vel_R;

    integral_vel_R += error_vel_R * dt;
    float derivative_vel_R = (error_vel_R - prevError_vel_R) / dt;

    float pwm_R = Kp_vel * error_vel_R + Ki_vel * integral_vel_R + Kd_vel * derivative_vel_R;

    prevError_vel_R = error_vel_R;

    // -------- CLAMP PWM --------
    pwm_L = constrain(pwm_L, -255, 255);
    pwm_R = constrain(pwm_R, -255, 255);

    setMotor(pwm_L, pwm_R);

    // -------- STOP CONDITION --------
    if (abs(error_pos) < 5) break;
  }

  stopMotor();
}

// ---------------- MOTOR DRIVER ----------------
void setMotor(float leftPWM, float rightPWM) {

  if (leftPWM > 0) {
    analogWrite(M1_IN1, leftPWM);
    analogWrite(M1_IN2, 0);
  } else {
    analogWrite(M1_IN1, 0);
    analogWrite(M1_IN2, -leftPWM);
  }

  if (rightPWM > 0) {
    analogWrite(M2_IN3, rightPWM);
    analogWrite(M2_IN4, 0);
  } else {
    analogWrite(M2_IN3, 0);
    analogWrite(M2_IN4, -rightPWM);
  }
}

void stopMotor() {
  analogWrite(M1_IN1, 0);
  analogWrite(M1_IN2, 0);
  analogWrite(M2_IN3, 0);
  analogWrite(M2_IN4, 0);
}

// ---------------- ENCODER ISR ----------------
void updateEncoder1() {
  byte state = (digitalRead(encoder1A) << 1) | digitalRead(encoder1B);
  byte combined = (prevState1 << 2) | state;

  if (combined == 1 || combined == 7 || combined == 14 || combined == 8) encoderCount1++;
  else if (combined == 2 || combined == 4 || combined == 13 || combined == 11) encoderCount1--;

  prevState1 = state;
}

void updateEncoder2() {
  byte state = (digitalRead(encoder2A) << 1) | digitalRead(encoder2B);
  byte combined = (prevState2 << 2) | state;

  if (combined == 1 || combined == 7 || combined == 14 || combined == 8) encoderCount2++;
  else if (combined == 2 || combined == 4 || combined == 13 || combined == 11) encoderCount2--;

  prevState2 = state;
}