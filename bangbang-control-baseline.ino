// --- Encoder Pins ---
// M1: Left Motor Encoder
const int encoder1A = 13; // LeftC1
const int encoder1B = 12; // LeftC2

// M2: Right Motor Encoder (Uses Mega pins 18, 19)
const int encoder2A = 19; // RightC1 
const int encoder2B = 18; // RightC2 

volatile long encoderCount1 = 0; // Left Motor Count
volatile long encoderCount2 = 0; // Right Motor Count
volatile byte prevState1 = 0;
volatile byte prevState2 = 0;

// --- Motor Control Pins (All pins must be PWM capable for speed control) ---
// Motor 1 (Left Side)
const int M1_IN1 = 15; // Left Forward Speed (PWM) -> Acts as s1
const int M1_IN2 = 14; // Left Backward Speed (PWM) -> Acts as s2

// Motor 2 (Right Side)
const int M2_IN3 = 16; // Right Forward Speed (PWM) -> Acts as s3
const int M2_IN4 = 17; // Right Backward Speed (PWM) -> Acts as s4

//IR PINS
const int LeftIR=0;
const int RightIR=1;


const float radius = 3.5; // In CENTIMETER
const float wheelBase = 20; // In CENTIMETER
const long numOfEncCntPerRev_R = 1320;
const long numOfEncCntPerRev_L = 1320;
float turnDistance = ( PI * wheelBase) / 4.0; 
float distPerCountLeft  = (2 * PI * radius) / numOfEncCntPerRev_L;
float distPerCountRight = (2 * PI * radius) / numOfEncCntPerRev_R;
const long numOfEncCntFor_1m_R = 100 * (numOfEncCntPerRev_R/ (2*PI*radius)) ;
const long numOfEncCntFor_1m_L = 100 * (numOfEncCntPerRev_L/ (2*PI*radius)) ;
const long numOfEncCntFor_T_L  = (turnDistance / distPerCountLeft) ;
const long numOfEncCntFor_T_R = (turnDistance / distPerCountRight) ;

    
void setup() {
  //Serial.begin(115200);
  //while(!Serial){;}

  // --- Encoder Setup ---
  pinMode(encoder1A, INPUT);
  pinMode(encoder1B, INPUT);
  pinMode(encoder2A, INPUT);
  pinMode(encoder2B, INPUT);

  // IR setup
  pinMode(LeftIR,INPUT);
  pinMode(RightIR,INPUT);

  // Initialize previous states
  prevState1 = (digitalRead(encoder1A) << 1) | digitalRead(encoder1B);
  prevState2 = (digitalRead(encoder2A) << 1) | digitalRead(encoder2B);

  // Attach interrupts for both channels of each encoder
  // Note: Pins 18 and 19 (for M2) are external interrupts on Mega (INT5, INT4)
  attachInterrupt(digitalPinToInterrupt(encoder1A), updateEncoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder1B), updateEncoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder2A), updateEncoder2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder2B), updateEncoder2, CHANGE);

  // --- Motor Setup: All control pins are now outputs ---
  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);
  pinMode(M2_IN3, OUTPUT);
  pinMode(M2_IN4, OUTPUT);

  // Stop motors initially
  //motor_move(0, 0); 
}

void loop() {
  // Get initial encoder counts safely

  //Rotate 90 degrees
  delay(2000);
  motor_move(75, 100, encoderCount1, encoderCount2, false);
  delay(400);
  motor_turn((PI/2) + 0.044 , 100, encoderCount1, encoderCount2);
  delay(400); // was 400
  motor_move(160, 100, encoderCount1, encoderCount2, false); // og 117
  delay(400);
  motor_turn((PI/2) + 0.044 , 100, encoderCount1, encoderCount2);
  delay(400);
  motor_move(50, 100, encoderCount1, encoderCount2, true);
  motor_move(-50, 100, encoderCount1, encoderCount2, true );
  delay(400);
  motor_turn(-((PI/2) + 0.044) , 100, encoderCount1, encoderCount2);
  delay(400);
  motor_move(160, 100, encoderCount1, encoderCount2, false);

  
  /*motor_move(30.5, 100, encoderCount1, encoderCount2, false);
  delay(200);
  motor_turn(-(PI/2 + 0.087) , 100, encoderCount1, encoderCount2);
  delay(200);
  motor_turn(PI/2 - 0.0345 , 100, encoderCount1, encoderCount2);
  delay(200);
  motor_move(60, 100, encoderCount1, encoderCount2, true);
  motor_move(-40, 100, encoderCount1, encoderCount2, false);
  delay(300);
  motor_turn(-(PI/2 + 0.085), 100, encoderCount1, encoderCount2);
  delay(200);
  motor_move(45, 100, encoderCount1, encoderCount2, false);
  delay(100);
  while(true){motor_turn(2*PI, 100,  encoderCount1, encoderCount2);}*/

  delay(5000);

  // Wait 2 seconds

  
  

//while(true){}
} 



// ---------------- ISR FUNCTIONS ------------------

void updateEncoder1() { // Left Motor
  byte state = (digitalRead(encoder1A) << 1) | digitalRead(encoder1B);
  byte combined = (prevState1 << 2) | state;

  switch (combined) {
    case 0b0001:
    case 0b0111:
    case 0b1110:
    case 0b1000:
      encoderCount1++;
      break;
    case 0b0010:
    case 0b0100:
    case 0b1101:
    case 0b1011:
      encoderCount1--;
      break;
    default:
      break;
  }
  prevState1 = state;
}

void updateEncoder2() { // Right Motor
  byte state = (digitalRead(encoder2A) << 1) | digitalRead(encoder2B);
  byte combined = (prevState2 << 2) | state;

  switch (combined) {
    case 0b0001:
    case 0b0111:
    case 0b1110:
    case 0b1000:
      encoderCount2++;
      break;
    case 0b0010:
    case 0b0100:
    case 0b1101:
    case 0b1011:
      encoderCount2--;
      break;
    default:
      break;
  }
  prevState2 = state;
}




// ---------------- MOVEMENT FUNCTION ------------------


void motor_move( float dist, float speed, int leftCount, int rightCount, bool IR) {
  // Speed Scaling: 0-100 to 0-255
  int pwmSpeed = (int)(speed * 2.55);

  // Ensure speed is within the 0-255 PWM range
  if (pwmSpeed > 255) pwmSpeed = 255;
  if (pwmSpeed < 0) pwmSpeed = 0;
  // dist is an input in CENTIMETER
  // Initialize all four speeds to zero (stop state)
  int s1 = 0, s2 = 0, s3 = 0, s4 = 0;
  int leftCountinit=leftCount;
  int rightCountinit=rightCount;
  float numEncCntFor_distCM_R = abs(dist) * (numOfEncCntPerRev_R/ (2*PI*radius));
  float numEncCntFor_distCM_L = abs(dist) * (numOfEncCntPerRev_L/ (2*PI*radius));

  if(IR){
      while(true){

      noInterrupts();
      long leftcount = encoderCount1; // live update
      long rightcount = encoderCount2;
      interrupts();

      long changeLeft=abs(leftcount - leftCountinit);
      long changeRight=abs(rightcount - rightCountinit);

      int leftSensor=digitalRead(LeftIR);
      int rightSensor=digitalRead(RightIR);



      if (changeLeft >= numEncCntFor_distCM_L && changeRight >= numEncCntFor_distCM_R || leftSensor==HIGH || rightSensor==HIGH){
        break;
      }

    
      

      if(dist>0){
          s1 = 0, s2 = 0, s3 = 0, s4 = 0;
          //Serial.println(changeRight-changeLeft);
          if ((changeLeft-changeRight)>5){
            s1 = pwmSpeed*0.9; // Left Forward
            s3 = pwmSpeed; // Right Forward
          }else if((changeRight-changeLeft)>5){
            s1 = pwmSpeed; // Left Forward
            s3 = pwmSpeed*0.9; // Right Forward
          }else{
            s1 = pwmSpeed; // Left Forward
            s3 = pwmSpeed; // Right Forward
          }
          analogWrite(M1_IN1, s1); 
          analogWrite(M2_IN3, s3*0.9718); 
          analogWrite(M1_IN2, s2); 
          analogWrite(M2_IN4, s4); //0.9286 was og
        }else{
          //Serial.println("hi reverse");
          s1 = 0, s2 = 0, s3 = 0, s4 = 0;
          //Serial.println(changeRight-changeLeft);
          if ((changeLeft-changeRight)>5){
            s2 = pwmSpeed*0.9; // Left Forward
            s4 = pwmSpeed; // Right Forward
          }else if((changeRight-changeLeft)>5){
            s2 = pwmSpeed; // Left Forward
            s4 = pwmSpeed*0.9; // Right Backward
          }else{
            s2 = pwmSpeed; // Left Backward
            s4 = pwmSpeed; // Right Backward
          }
          analogWrite(M1_IN1, s1); 
          analogWrite(M2_IN3, s3);
          analogWrite(M1_IN2, s2*0.9718); 
          analogWrite(M2_IN4, s4); 

        }
    /*Serial.print("Change: ");
    Serial.println(changeRight - changeLeft);
    Serial.print("Left: ");
    Serial.print(numEncCntFor_distCM_L);
    Serial.print(" ");
    Serial.print(changeLeft);
    Serial.print("  Right: ");
    Serial.print(numEncCntFor_distCM_R);
    Serial.print(" ");
    Serial.println(changeRight);*/
    }

  }else{
    while(true){

      noInterrupts();
      long leftcount = encoderCount1; // live update
      long rightcount = encoderCount2;
      interrupts();

      long changeLeft=abs(leftcount - leftCountinit);
      long changeRight=abs(rightcount - rightCountinit);



      if (changeLeft >= numEncCntFor_distCM_L && changeRight >= numEncCntFor_distCM_R){
        break;
      }

    
      

      if(dist>0){
          s1 = 0, s2 = 0, s3 = 0, s4 = 0;
          //Serial.println(changeRight-changeLeft);
          if ((changeLeft-changeRight)>5){
            s1 = pwmSpeed*0.9; // Left Forward
            s3 = pwmSpeed; // Right Forward
          }else if((changeRight-changeLeft)>5){
            s1 = pwmSpeed; // Left Forward
            s3 = pwmSpeed*0.9; // Right Forward
          }else{
            s1 = pwmSpeed; // Left Forward
            s3 = pwmSpeed; // Right Forward
          }
          analogWrite(M1_IN1, s1); 
          analogWrite(M2_IN3, s3*0.9718); 
          analogWrite(M1_IN2, s2); 
          analogWrite(M2_IN4, s4); //0.9286 was og
        }else{
          //Serial.println("hi reverse");
          s1 = 0, s2 = 0, s3 = 0, s4 = 0;
          //Serial.println(changeRight-changeLeft);
          if ((changeLeft-changeRight)>5){
            s2 = pwmSpeed*0.9; // Left Forward
            s4 = pwmSpeed; // Right Forward
          }else if((changeRight-changeLeft)>5){
            s2 = pwmSpeed; // Left Forward
            s4 = pwmSpeed*0.9; // Right Backward
          }else{
            s2 = pwmSpeed; // Left Backward
            s4 = pwmSpeed; // Right Backward
          }
          analogWrite(M1_IN1, s1); 
          analogWrite(M2_IN3, s3);
          analogWrite(M1_IN2, s2*0.9718); 
          analogWrite(M2_IN4, s4); 

        }
        /*Serial.print("Change: ");
        Serial.println(changeRight - changeLeft);
        Serial.print("Left: ");
        Serial.print(numEncCntFor_distCM_L);
        Serial.print(" ");
        Serial.print(changeLeft);
        Serial.print("  Right: ");
        Serial.print(numEncCntFor_distCM_R);
        Serial.print(" ");
        Serial.println(changeRight);


        */


    /*Serial.print("Left: ");
    Serial.print(numEncCntFor_distCM_L);
    Serial.print(" ");
    Serial.print(changeLeft);
    Serial.print("  Right: ");
    Serial.print(numEncCntFor_distCM_R);
    Serial.print(" ");
    Serial.println(changeRight);*/
    }
  }

  analogWrite(M1_IN1, 0); 
  analogWrite(M1_IN2, 0); 
  analogWrite(M2_IN3, 0); 
  analogWrite(M2_IN4, 0); 


  // --- Motor Logic: Defines which pin gets the PWM signal --- 
   /* if ((leftCount-rightCount)>20){
    s1 = pwmSpeed*0.984; // Left Forward
    s3 = pwmSpeed; // Right Forward
    }else if((rightCount-leftCount)>10){
    s1 = pwmSpeed; // Left Forward
    s3 = pwmSpeed*0.984; // Right Forward
    }else{
    s1 = pwmSpeed; // Left Forward
    s3 = pwmSpeed; // Right Forward
    }


  
  
  
  // If dirn == 0, all s1-s4 remain 0 (stop)

  // Apply PWM signal to the corresponding motor input pins
  analogWrite(M1_IN1, s1); 
  analogWrite(M1_IN2, s2); 
  analogWrite(M2_IN3, s3*0.9718); //0.9286 was og
  analogWrite(M2_IN4, s4*0.9718); */
}


void motor_turn(float angle, int speed, int leftCount, int rightCount){
  int leftCountinit=leftCount;
  int rightCountinit=rightCount;
  float arb = 180;
  float ticksLeft= (((abs(angle)*wheelBase)/2)/ distPerCountLeft) - arb;
  float ticksRight= (((abs(angle)*wheelBase)/2)/ distPerCountRight) - arb;
  int pwmSpeed = (int)(speed * 2.55);


  
  // Ensure speed is within the 0-255 PWM range
  if (pwmSpeed > 255) pwmSpeed = 255;
  if (pwmSpeed < 0) pwmSpeed = 0;
  // Initialize all four speeds to zero (stop state)
  int s1 = 0, s2 = 0, s3 = 0, s4 = 0;
  while(true){
    noInterrupts();
    long leftcount = encoderCount1; // live update
    long rightcount = encoderCount2;
    interrupts();

    long changeLeft=abs(leftcount - leftCountinit);
    long changeRight=abs(rightcount - rightCountinit);


    if (changeLeft >= ticksLeft && changeRight >= ticksRight){
      break;
    }
    // --- Motor Logic: Defines which pin gets the PWM signal --- 
      if(angle>0){
        s1 = 0, s2 = 0, s3 = 0, s4 = 0;
        //Serial.println(changeRight-changeLeft);
        if ((changeLeft-changeRight)>5){
          s1 = pwmSpeed*0.9; // Left Forward
          s4 = pwmSpeed; // Right Forward
        }else if((changeRight-changeLeft)>5){
          s1 = pwmSpeed; // Left Forward
          s4 = pwmSpeed*0.9; // Right Forward
        }else{
          s1 = pwmSpeed; // Left Forward
          s4 = pwmSpeed; // Right Forward
        }
      }
      else{
        s1 = 0, s2 = 0, s3 = 0, s4 = 0;
       // Serial.println(changeRight-changeLeft);
        if ((changeLeft-changeRight)>5){
          s2 = pwmSpeed*0.9; // Left Forward
          s3 = pwmSpeed; // Right Forward
        }else if((changeRight-changeLeft)>5){
          s2 = pwmSpeed; // Left Forward
          s3 = pwmSpeed*0.9; // Right Forward
        }else{
          s2 = pwmSpeed; // Left Forward
          s3 = pwmSpeed; // Right Forward
        }


      }


  
  
  
  // If dirn == 0, all s1-s4 remain 0 (stop)

  // Apply PWM signal to the corresponding motor input pins
  analogWrite(M1_IN1, s1); 
  analogWrite(M1_IN2, s2); 
  analogWrite(M2_IN3, s3*0.9718); //0.9286 was og
  analogWrite(M2_IN4, s4*0.9718); 

  /*ISerial.print("Change: ");
  Serial.println(changeRight - changeLeft);Serial.print("Left: ");
  Serial.print(ticksLeft);
  Serial.print(" ");
  Serial.print(changeLeft);
  Serial.print("  Right: ");
  Serial.print(ticksRight);
  Serial.print(" ");
  Serial.println(changeRight);*/
  
  
}
  analogWrite(M1_IN1, 0); 
  analogWrite(M1_IN2, 0); 
  analogWrite(M2_IN3, 0); 
  analogWrite(M2_IN4, 0); 
}








void motor_stop() {
  analogWrite(M1_IN1, 0); 
  analogWrite(M1_IN2, 0); 
  analogWrite(M2_IN3, 0); 
  analogWrite(M2_IN4, 0); 
}