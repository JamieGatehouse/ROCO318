#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
Adafruit_MPU6050 mpu;

#define SERVOMIN  90   // minimum pulse length
#define SERVOMAX  490  // maximum pulse length
#define SERVO_FREQ 50  // 50Hz servo rate

// Servo mapping
#define RIGHT_ANKLE_ROLL 0
#define RIGHT_KNEE 1
#define RIGHT_HIP_TILT 2
#define RIGHT_HIP_ROLL 3
#define LEFT_ANKLE_ROLL 4
#define LEFT_KNEE 5
#define LEFT_HIP_TILT 6
#define LEFT_HIP_ROLL 7

// store current servo angles
float currentAngles[8] = {90, 90, 90, 90, 90, 90, 90, 90};

// PID Controller variables
float targetPitch = 0.0;        // Target pitch angle (0 = level)
float currentPitch = 0.0;       // Current pitch from MPU
float pitchError = 0.0;
float lastPitchError = 0.0;
float pitchErrorSum = 0.0;      // Integral term

// PID Gains - TUNE THESE VALUES
float Kp = 2.0;    // Proportional gain
float Ki = 0.1;    // Integral gain
float Kd = 0.5;    // Derivative gain

// PID limits
float maxPitchCorrection = 20.0;  // Maximum correction angle in degrees
float maxIntegral = 100.0;         // Anti-windup limit

// timing for MPU readings
unsigned long lastMPURead = 0;
unsigned long lastPIDUpdate = 0;
const unsigned long mpuReadInterval = 20;  // read every 20ms (50Hz)
const unsigned long pidUpdateInterval = 20; // update PID every 20ms

// Calibration offset
float pitchOffset = 0.0;

void setup() {
  Serial.begin(9600);
  Serial.println("Quadruped PID Pitch Stabilization");

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

  Serial.println("Calibrating... Keep robot level!");
  delay(1000);
  calibratePitch();
  
  Serial.println("Calibration complete!");
  Serial.println("Starting walk cycle with PID stabilization...");
  delay(500);
}

void calibratePitch() {
  float sumPitch = 0;
  int samples = 50;
  
  for (int i = 0; i < samples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    
    float pitch = atan2(a.acceleration.x, 
                        sqrt(a.acceleration.y * a.acceleration.y + 
                             a.acceleration.z * a.acceleration.z)) * 180.0 / PI;
    sumPitch += pitch;
    delay(10);
  }
  
  pitchOffset = sumPitch / samples;
  Serial.print("Pitch offset: ");
  Serial.println(pitchOffset);
}

void setServoAngleRaw(uint8_t servonum, float angle) {
  if (angle < 0) angle = 0;
  if (angle > 285) angle = 285;

  int pulselen = SERVOMIN + (angle / 285.0) * (SERVOMAX - SERVOMIN);
  pwm.setPWM(servonum, 0, pulselen);
}

// Calculate pitch angle from accelerometer
float calculatePitch() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  // Calculate pitch (rotation around Y-axis)
  float pitch = atan2(a.acceleration.x, 
                      sqrt(a.acceleration.y * a.acceleration.y + 
                           a.acceleration.z * a.acceleration.z)) * 180.0 / PI;
  
  return pitch - pitchOffset;
}

// PID Controller - returns correction angle
float updatePID() {
  currentPitch = calculatePitch();
  pitchError = targetPitch - currentPitch;
  
  // Proportional term
  float P = Kp * pitchError;
  
  // Integral term with anti-windup
  pitchErrorSum += pitchError;
  if (pitchErrorSum > maxIntegral) pitchErrorSum = maxIntegral;
  if (pitchErrorSum < -maxIntegral) pitchErrorSum = -maxIntegral;
  float I = Ki * pitchErrorSum;
  
  // Derivative term
  float D = Kd * (pitchError - lastPitchError);
  lastPitchError = pitchError;
  
  // Calculate total correction
  float correction = P + I + D;
  
  // Limit correction
  if (correction > maxPitchCorrection) correction = maxPitchCorrection;
  if (correction < -maxPitchCorrection) correction = -maxPitchCorrection;
  
  return correction;
}

// Display current status
void displayStatus() {
  Serial.print("Pitch: ");
  Serial.print(currentPitch, 2);
  Serial.print("° | Error: ");
  Serial.print(pitchError, 2);
  Serial.print("° | P: ");
  Serial.print(Kp * pitchError, 2);
  Serial.print(" | I: ");
  Serial.print(Ki * pitchErrorSum, 2);
  Serial.print(" | D: ");
  Serial.println(Kd * (pitchError - lastPitchError), 2);
}

// Move all servos to a frame with PID correction applied
void moveToFrame(float target[8], int steps, int stepDelay) {
  for (int s = 0; s < steps; s++) {
    // Update PID if enough time has passed
    float pitchCorrection = 0;
    if (millis() - lastPIDUpdate >= pidUpdateInterval) {
      pitchCorrection = updatePID();
      lastPIDUpdate = millis();
    }
    
    for (int i = 0; i < 8; i++) {
      float start = currentAngles[i];
      float diff  = target[i] - start;
      float newAngle = start + diff * ((float)s / steps);
      
      // Apply PID correction to hip tilt servos (2 and 6)
      // Positive pitch (tilting forward) -> increase hip tilt angles to compensate
      if (i == RIGHT_HIP_TILT || i == LEFT_HIP_TILT) {
        newAngle += pitchCorrection;
      }
      
      setServoAngleRaw(i, newAngle);
    }
    delay(stepDelay);
  }

  // Finalize angles
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

  int transitionSteps = 40;
  int stepDelay = 10;

  moveToFrame(frame1, transitionSteps, stepDelay);
  displayStatus();
  delay(200);

  moveToFrame(frame2, transitionSteps, stepDelay);
  displayStatus();
  delay(200);

  moveToFrame(frame3, transitionSteps, stepDelay);
  displayStatus();
  delay(200);

  moveToFrame(frame4, transitionSteps, stepDelay);
  displayStatus();
  delay(200);

  moveToFrame(frame5, transitionSteps, stepDelay);
  displayStatus();
  delay(200);

  moveToFrame(frame6, transitionSteps, stepDelay);
  displayStatus();
  delay(200);

  moveToFrame(frame7, transitionSteps, stepDelay);
  displayStatus();
  delay(100);
  
  moveToFrame(frame8, transitionSteps, stepDelay);
  displayStatus();
  delay(100);
}
