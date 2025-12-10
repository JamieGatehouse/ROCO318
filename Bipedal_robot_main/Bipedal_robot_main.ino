/*#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  90   // 'minimum' pulse length count (out of 4096)
#define SERVOMAX  490  // 'maximum' pulse length count (out of 4096)
#define SERVO_FREQ 50  // Analog servos run at ~50 Hz updates

void setup() {
  Serial.begin(9600);
  Serial.println("Inital Gait Pattern");

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);

  delay(10);
}

void setServoAngle(uint8_t servonum, float angle) {

  if (angle < 0) angle = 0;
  if (angle > 285) angle = 285;

  int pulselen = SERVOMIN + (angle / 285.0) * (SERVOMAX - SERVOMIN);
  pwm.setPWM(servonum, 0, pulselen);
}

void loop() {
  
  //frame 1
  setServoAngle(0, 75);
  setServoAngle(1, 45);
  setServoAngle(2, 45);
  setServoAngle(3, 81);
  setServoAngle(4, 90);
  setServoAngle(5, 45);
  setServoAngle(6, 35);
  setServoAngle(7, 90);
  delay(2000);

 
  //frame 2
  setServoAngle(0, 90);
  setServoAngle(1, 45);
  setServoAngle(2, 45);
  setServoAngle(3, 90);
  setServoAngle(4, 115);
  setServoAngle(5, 45);
  setServoAngle(6, 35);
  setServoAngle(7, 100);
  delay(2000);


  //frame 3
  setServoAngle(0, 95);
  setServoAngle(1, 135);
  setServoAngle(2, 135);
  setServoAngle(3, 95);
  setServoAngle(4, 115);
  setServoAngle(5, 90);
  setServoAngle(6, 90);
  setServoAngle(7, 100);
  delay(2000);


  //frame 4
  setServoAngle(0, 95);
  setServoAngle(1, 135);
  setServoAngle(2, 135);
  setServoAngle(3, 95);
  setServoAngle(4, 115);
  setServoAngle(5, 135);
  setServoAngle(6, 135);
  setServoAngle(7, 100);
  delay(2000);


  //frame 5
  setServoAngle(0, 75);
  setServoAngle(1, 135);
  setServoAngle(2, 135);
  setServoAngle(3, 81);
  setServoAngle(4, 90);
  setServoAngle(5, 135);
  setServoAngle(6, 135);
  setServoAngle(7, 90);
  delay(2000);


  //frame 6
  setServoAngle(0, 75);
  setServoAngle(1, 90);
  setServoAngle(2, 90);
  setServoAngle(3, 81);
  setServoAngle(4, 90);
  setServoAngle(5, 45);
  setServoAngle(6, 35);
  setServoAngle(7, 90);
  delay(2000);
 
  //frame 7
  setServoAngle(0, 75);
  setServoAngle(1, 45);
  setServoAngle(2, 45);
  setServoAngle(3, 81);
  setServoAngle(4, 90);
  setServoAngle(5, 45);
  setServoAngle(6, 35);
  setServoAngle(7, 90);
  delay(2000);
}
*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  90   // minimum pulse length
#define SERVOMAX  490  // maximum pulse length
#define SERVO_FREQ 50  // 50Hz servo rate

// store current servo angles
float currentAngles[8] = {90, 90, 90, 90, 90, 90, 90, 90};

void setup() {
  Serial.begin(9600);
  Serial.println("Smooth Gait Pattern");

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);

  delay(10);
}

void setServoAngleRaw(uint8_t servonum, float angle) {
  if (angle < 0) angle = 0;
  if (angle > 285) angle = 285;

  int pulselen = SERVOMIN + (angle / 285.0) * (SERVOMAX - SERVOMIN);
  pwm.setPWM(servonum, 0, pulselen);
}

// --------------------------------------------------------
// SMOOTH MOVEMENT FUNCTION
// --------------------------------------------------------
void smoothSetServoAngle(uint8_t servonum, float targetAngle, int steps, int stepDelay) {
  float startAngle = currentAngles[servonum];
  float stepSize = (targetAngle - startAngle) / (float)steps;

  for (int i = 0; i < steps; i++) {
    float newAngle = startAngle + stepSize * i;
    setServoAngleRaw(servonum, newAngle);
    delay(stepDelay);
  }

  // Final write & update internal angle tracker
  setServoAngleRaw(servonum, targetAngle);
  currentAngles[servonum] = targetAngle;
}

// --------------------------------------------------------
// MOVE ALL SERVOS TO A "FRAME" SMOOTHLY
// --------------------------------------------------------
void moveToFrame(float target[8], int steps, int stepDelay) {
  for (int s = 0; s < steps; s++) {
    for (int i = 0; i < 8; i++) {
      float start = currentAngles[i];
      float diff  = target[i] - start;
      float newAngle = start + diff * ((float)s / steps);
      setServoAngleRaw(i, newAngle);
    }
    delay(stepDelay);
  }

  // finalize
  for (int i = 0; i < 8; i++) {
    currentAngles[i] = target[i];
  }
}

void loop() {

  //float frame1[8] = {97,95,95,90,97,92,87,93};

  float frame1[8] = {97,120,120,90,97,117,112,93};
  float frame2[8] = {78,120,120,90,78,117,112,74};
  float frame3[8] = {97,95,95,100,78,92,87,100};
  float frame4[8] = {107,70,70,90,107,70,62,93};
  float frame5[8] = {120,70,70,109,107,70,62,93};
  float frame6[8] = {120,90,70,90,107,62,62,93};
  float frame7[8] = {120,95,95,90,107,122,122,93};
  float frame8[8] = {120,95,95,90,107,122,122,93};


  //float frame1[8] = {75,45,45,81,90,45,35,90};
  //float frame2[8] = {90,45,45,90,115,45,35,100};
  //float frame3[8] = {95,135,135,95,115,90,90,100};
  //float frame4[8] = {95,135,135,95,115,135,135,100};
  //float frame5[8] = {75,135,135,81,90,135,135,90};
  //float frame6[8] = {75,90,90,81,90,45,35,90};
  //float frame7[8] = {75,45,45,81,90,45,35,90};

  // smoother: more steps = slower/more fluid
  int transitionSteps = 40;   // try 20–60
  int stepDelay = 10;         // ms delay per step

  moveToFrame(frame1, transitionSteps, stepDelay);
  Serial.println("1");
  delay(2000);

  moveToFrame(frame2, transitionSteps, stepDelay);
  Serial.println("2");
  delay(2000);

  moveToFrame(frame3, transitionSteps, stepDelay);
  Serial.println("3");
  delay(2000);

  moveToFrame(frame4, transitionSteps, stepDelay);
  Serial.println("4");
  delay(2000);

  moveToFrame(frame5, transitionSteps, stepDelay);
  Serial.println("5");
  delay(2000);

  moveToFrame(frame6, transitionSteps, stepDelay);
  Serial.println("6");
  delay(2000);

  moveToFrame(frame7, transitionSteps, stepDelay);
  Serial.println("7");
  delay(2000);
  
  moveToFrame(frame8, transitionSteps, stepDelay);
  Serial.println("8");
  delay(2000);
  
}
