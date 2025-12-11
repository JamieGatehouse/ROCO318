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
  float frame2[8] = {80,120,120,80,65,117,112,80};
  float frame3[8] = {80,95,95,100,78,92,87,100};
  float frame4[8] = {107,70,70,90,107,70,62,93};
  float frame5[8] = {120,70,70,109,107,70,62,93};
  float frame6[8] = {120,90,70,90,107,62,62,93};
  float frame7[8] = {120,95,95,90,107,122,122,93};
  float frame8[8] = {120,95,95,90,107,122,122,93};




  // smoother: more steps = slower/more fluid
  int transitionSteps = 40;   // try 20–60
  int stepDelay = 10;         // ms delay per step

  moveToFrame(frame1, transitionSteps, stepDelay);
  Serial.println("1");
  delay(200);

  moveToFrame(frame2, transitionSteps, stepDelay);
  Serial.println("2");
  delay(200);

  moveToFrame(frame3, transitionSteps, stepDelay);
  Serial.println("3");
  delay(200);

  moveToFrame(frame4, transitionSteps, stepDelay);
  Serial.println("4");
  delay(200);

  moveToFrame(frame5, transitionSteps, stepDelay);
  Serial.println("5");
  delay(200);

  moveToFrame(frame6, transitionSteps, stepDelay);
  Serial.println("6");
  delay(200);

  moveToFrame(frame7, transitionSteps, stepDelay);
  Serial.println("7");
  delay(100);
  
  moveToFrame(frame8, transitionSteps, stepDelay);
  Serial.println("8");
  delay(100);
  
}




/*#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
Adafruit_MPU6050 mpu;

#define SERVOMIN  90   // minimum pulse length
#define SERVOMAX  490  // maximum pulse length
#define SERVO_FREQ 50  // 50Hz servo rate

// store current servo angles
float currentAngles[8] = {90, 90, 90, 90, 90, 90, 90, 90};

// timing for MPU readings
unsigned long lastMPURead = 0;
const unsigned long mpuReadInterval = 100; // read every 100ms

void setup() {
  Serial.begin(9600);
  Serial.println("Smooth Gait Pattern with MPU-6050");

  // Initialize PWM servo driver
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  // Configure MPU6050 settings
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("");
  delay(100);
}

void setServoAngleRaw(uint8_t servonum, float angle) {
  if (angle < 0) angle = 0;
  if (angle > 285) angle = 285;

  int pulselen = SERVOMIN + (angle / 285.0) * (SERVOMAX - SERVOMIN);
  pwm.setPWM(servonum, 0, pulselen);
}

// --------------------------------------------------------
// READ AND DISPLAY MPU6050 DATA
// --------------------------------------------------------
void readMPU6050() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  Serial.println("=== MPU-6050 Readings ===");
  
  // Accelerometer data (m/s^2)
  Serial.print("Accel X: "); Serial.print(a.acceleration.x, 2);
  Serial.print(" | Y: "); Serial.print(a.acceleration.y, 2);
  Serial.print(" | Z: "); Serial.println(a.acceleration.z, 2);
  
  // Gyroscope data (rad/s)
  Serial.print("Gyro X: "); Serial.print(g.gyro.x, 2);
  Serial.print(" | Y: "); Serial.print(g.gyro.y, 2);
  Serial.print(" | Z: "); Serial.println(g.gyro.z, 2);
  
  // Temperature
  Serial.print("Temp: "); Serial.print(temp.temperature); Serial.println(" °C");
  Serial.println("========================");
  Serial.println();
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
    
    // Read MPU data periodically during movement
    if (millis() - lastMPURead >= mpuReadInterval) {
      readMPU6050();
      lastMPURead = millis();
    }
  }

  // finalize
  for (int i = 0; i < 8; i++) {
    currentAngles[i] = target[i];
  }
}

void loop() {

  float frame1[8] = {97,120,120,90,97,117,112,93};
  float frame2[8] = {80,120,120,80,65,117,112,80};
  float frame3[8] = {80,95,95,100,78,92,87,100};
  float frame4[8] = {107,70,70,90,107,70,62,93};
  float frame5[8] = {120,70,70,109,107,70,62,93};
  float frame6[8] = {120,90,70,90,107,62,62,93};
  float frame7[8] = {120,95,95,90,107,122,122,93};
  float frame8[8] = {120,95,95,90,107,122,122,93};

  // smoother: more steps = slower/more fluid
  int transitionSteps = 40;   // try 20–60
  int stepDelay = 10;         // ms delay per step

  moveToFrame(frame1, transitionSteps, stepDelay);
  Serial.println("Frame 1");
  readMPU6050();
  delay(200);

  moveToFrame(frame2, transitionSteps, stepDelay);
  Serial.println("Frame 2");
  readMPU6050();
  delay(200);

  moveToFrame(frame3, transitionSteps, stepDelay);
  Serial.println("Frame 3");
  readMPU6050();
  delay(200);

  moveToFrame(frame4, transitionSteps, stepDelay);
  Serial.println("Frame 4");
  readMPU6050();
  delay(200);

  moveToFrame(frame5, transitionSteps, stepDelay);
  Serial.println("Frame 5");
  readMPU6050();
  delay(200);

  moveToFrame(frame6, transitionSteps, stepDelay);
  Serial.println("Frame 6");
  readMPU6050();
  delay(200);

  moveToFrame(frame7, transitionSteps, stepDelay);
  Serial.println("Frame 7");
  readMPU6050();
  delay(100);
  
  moveToFrame(frame8, transitionSteps, stepDelay);
  Serial.println("Frame 8");
  readMPU6050();
  delay(100);
}*/
