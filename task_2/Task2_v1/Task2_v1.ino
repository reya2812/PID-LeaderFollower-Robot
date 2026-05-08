// 23/3/26
// SDC Lab task 2 version 1
// Straight Line Position PID control

// Two levels of PID - Distance (ed) and Alignment (ea)
// ed = target - (left + right)/2
// ea = left - right

// leftPWM = base − correction
// rightPWM = base + correction

// base PWM given by distance PID and correction PWM given by Alignment PID


// ---------------- ENCODER PINS ----------------
const int encoder1A = 13;
const int encoder1B = 12;
const int encoder2A = 19;
const int encoder2B = 18;

volatile long encoderCount1 = 0;
volatile long encoderCount2 = 0;
volatile byte prevState1 = 0;
volatile byte prevState2 = 0;

// ---------------- MOTOR PINS ----------------
const int M1_IN1 = 15;
const int M1_IN2 = 14;
const int M2_IN3 = 16;
const int M2_IN4 = 17;

// ---------------- ROBOT PARAMETERS ----------------
const float radius = 3.5; // cm
const long countsPerRev = 1320;

float distPerCount = (2 * PI * radius) / countsPerRev;

// ---------------- PID GAINS ----------------
// Distance PID
float Kp_d = 2.0;
float Ki_d = 0.01;
float Kd_d = 0.5;

// Alignment PID
float Kp_a = 1.5;
float Ki_a = 0.0;
float Kd_a = 0.2;

// ---------------- PID VARIABLES ----------------
float prevError_d = 0, integral_d = 0;
float prevError_a = 0, integral_a = 0;

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

// ---------------- MAIN LOOP ----------------
void loop() {
  moveStraightPID(20.0); // move 20 cm
  while(true); // stop after one run
}

// ---------------- PID MOVE FUNCTION ----------------
void moveStraightPID(float target_cm) {

  long startLeft = encoderCount1;
  long startRight = encoderCount2;

  float targetCounts = target_cm / distPerCount;

  prevTime = millis();

  while (true) {

    // Time
    unsigned long currentTime = millis();
    float dt = (currentTime - prevTime) / 1000.0;
    prevTime = currentTime;

    // Read encoders safely
    noInterrupts();
    long left = encoderCount1 - startLeft;
    long right = encoderCount2 - startRight;
    interrupts();

    float avgCounts = (left + right) / 2.0;

    // -------- DISTANCE PID --------
    float error_d = targetCounts - avgCounts;

    integral_d += error_d * dt;
    float derivative_d = (error_d - prevError_d) / dt;

    float baseSpeed = Kp_d * error_d + Ki_d * integral_d + Kd_d * derivative_d;

    prevError_d = error_d;

    // -------- ALIGNMENT PID --------
    float error_a = left - right;

    integral_a += error_a * dt;
    float derivative_a = (error_a - prevError_a) / dt;

    float correction = Kp_a * error_a + Ki_a * integral_a + Kd_a * derivative_a;

    prevError_a = error_a;

    // -------- FINAL MOTOR OUTPUT --------
    float leftPWM = baseSpeed - correction;
    float rightPWM = baseSpeed + correction;

    // Clamp PWM
    leftPWM = constrain(leftPWM, -255, 255);
    rightPWM = constrain(rightPWM, -255, 255);

    setMotor(leftPWM, rightPWM);

    // -------- STOP CONDITION --------
    if (abs(error_d) < 5) break;
  }

  stopMotor();
}

// ---------------- MOTOR DRIVER ----------------
void setMotor(float leftPWM, float rightPWM) {

  // LEFT MOTOR
  if (leftPWM > 0) {
    analogWrite(M1_IN1, leftPWM);
    analogWrite(M1_IN2, 0);
  } else {
    analogWrite(M1_IN1, 0);
    analogWrite(M1_IN2, -leftPWM);
  }

  // RIGHT MOTOR
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