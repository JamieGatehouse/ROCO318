#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVOMIN  90
#define SERVOMAX  490
#define SERVO_FREQ 50

//this array stores the current angles for all 8 servos to track their positions between movements
float currentAngles[8] = {90, 90, 90, 90, 90, 90, 90, 90};

//this is the setup function and it initialises the serial communication and PWM driver for controlling the servos
void setup() {
Serial.begin(9600);        //this initialises serial communication at 9600 baud rate
Serial.println("Smooth Gait Pattern");        //this prints a startup message to the serial monitor
pwm.begin();        //this initialises the PWM driver
pwm.setOscillatorFrequency(27000000);        //this sets the internal oscillator frequency to 27MHz for accurate PWM timing
pwm.setPWMFreq(SERVO_FREQ);        //this sets the PWM frequency to 50Hz which is standard for servos
delay(10);        //this delays for 10ms to allow the PWM driver to stabilise
}

//this function converts a servo angle to a PWM pulse length and sends it to the specified servo
void setServoAngleRaw(uint8_t servonum, float angle) {
if (angle < 0) angle = 0;        //this ensures the angle doesnt go below 0 degrees (as that is impossible)
if (angle > 285) angle = 285;        //this ensures the angle doesnt exceed 285 degrees (as that is the maximum range)
int pulselen = SERVOMIN + (angle / 285.0) * (SERVOMAX - SERVOMIN);        //this converts the angle to a pulse length value between SERVOMIN and SERVOMAX
pwm.setPWM(servonum, 0, pulselen);        //this sends the PWM signal to the specified servo
}

//this function smoothly moves a single servo from its current position to a target angle using interpolation
void smoothSetServoAngle(uint8_t servonum, float targetAngle, int steps, int stepDelay) {
float startAngle = currentAngles[servonum];        //this gets the current angle of the servo
float stepSize = (targetAngle - startAngle) / (float)steps;        //this calculates the angle increment for each step
for (int i = 0; i < steps; i++) {        //this loops through all the steps to create smooth movement
float newAngle = startAngle + stepSize * i;        //this calculates the intermediate angle for this step
setServoAngleRaw(servonum, newAngle);        //this moves the servo to the intermediate angle
delay(stepDelay);        //this delays between steps to control the speed of movement
  }
setServoAngleRaw(servonum, targetAngle);        //this moves the servo to the final target angle
currentAngles[servonum] = targetAngle;        //this updates the stored current angle for this servo
}

//this function moves all 8 servos to their target positions simultaneously with smooth interpolation
void moveToFrame(float target[8], int steps, int stepDelay) {
for (int s = 0; s < steps; s++) {        //this loops through each step of the movement
for (int i = 0; i < 8; i++) {        //this loops through all 8 servos
float start = currentAngles[i];        //this gets the starting angle for this servo
float diff  = target[i] - start;        //this calculates the total angle change needed
float newAngle = start + diff * ((float)s / steps);        //this calculates the intermediate angle for this step using linear interpolation
setServoAngleRaw(i, newAngle);        //this moves the servo to the intermediate angle
    }
delay(stepDelay);        //this delays between steps to control the speed of movement
  }
for (int i = 0; i < 8; i++) {        //this loops through all servos to finalise their positions
currentAngles[i] = target[i];        //this updates the stored current angle to the target angle
  }
}

//this is the main loop function and it executes a walking gait pattern by moving through 8 different frames continuously
void loop() {
float frame1[8] = {97,120,120,90,97,117,112,93};        //this defines the servo angles for frame 1 of the gait cycle
float frame2[8] = {80,120,120,80,65,117,112,80};        //this defines the servo angles for frame 2 of the gait cycle
float frame3[8] = {80,95,95,100,78,92,87,100};        //this defines the servo angles for frame 3 of the gait cycle
float frame4[8] = {107,70,70,90,107,70,62,93};        //this defines the servo angles for frame 4 of the gait cycle
float frame5[8] = {120,70,70,109,107,70,62,93};        //this defines the servo angles for frame 5 of the gait cycle
float frame6[8] = {120,90,70,90,107,62,62,93};        //this defines the servo angles for frame 6 of the gait cycle
float frame7[8] = {120,95,95,90,107,122,122,93};        //this defines the servo angles for frame 7 of the gait cycle
float frame8[8] = {120,95,95,90,107,122,122,93};        //this defines the servo angles for frame 8 of the gait cycle

int transitionSteps = 40;        //this sets the number of interpolation steps for smooth transitions between frames
int stepDelay = 10;        //this sets the delay in milliseconds between each interpolation step

moveToFrame(frame1, transitionSteps, stepDelay);        //this moves all servos to frame 1 smoothly
Serial.println("1");        //this prints frame 1 to the serial monitor
delay(200);        //this delays for 200ms before moving to the next frame

moveToFrame(frame2, transitionSteps, stepDelay);        //this moves all servos to frame 2 smoothly
Serial.println("2");        //this prints frame 2 to the serial monitor
delay(200);        //this delays for 200ms before moving to the next frame

moveToFrame(frame3, transitionSteps, stepDelay);        //this moves all servos to frame 3 smoothly
Serial.println("3");        //this prints frame 3 to the serial monitor
delay(200);        //this delays for 200ms before moving to the next frame

moveToFrame(frame4, transitionSteps, stepDelay);        //this moves all servos to frame 4 smoothly
Serial.println("4");        //this prints frame 4 to the serial monitor
delay(200);        //this delays for 200ms before moving to the next frame

moveToFrame(frame5, transitionSteps, stepDelay);        //this moves all servos to frame 5 smoothly
Serial.println("5");        //this prints frame 5 to the serial monitor
delay(200);        //this delays for 200ms before moving to the next frame

moveToFrame(frame6, transitionSteps, stepDelay);        //this moves all servos to frame 6 smoothly
Serial.println("6");        //this prints frame 6 to the serial monitor
delay(200);        //this delays for 200ms before moving to the next frame

moveToFrame(frame7, transitionSteps, stepDelay);        //this moves all servos to frame 7 smoothly
Serial.println("7");        //this prints frame 7 to the serial monitor
delay(100);        //this delays for 100ms before moving to the next frame

moveToFrame(frame8, transitionSteps, stepDelay);        //this moves all servos to frame 8 smoothly
Serial.println("8");        //this prints frame 8 to the serial monitor
delay(100);        //this delays for 100ms before repeating the gait cycle
}



*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
Adafruit_MPU6050 mpu;

#define SERVOMIN  90
#define SERVOMAX  490
#define SERVO_FREQ 50

#define RIGHT_ANKLE_ROLL 0
#define RIGHT_KNEE 1
#define RIGHT_HIP_TILT 2
#define RIGHT_HIP_ROLL 3
#define LEFT_ANKLE_ROLL 4
#define LEFT_KNEE 5
#define LEFT_HIP_TILT 6
#define LEFT_HIP_ROLL 7

float currentAngles[8] = {90, 90, 90, 90, 90, 90, 90, 90};

float targetPitch = 0.0;
float currentPitch = 0.0;
float pitchError = 0.0;
float lastPitchError = 0.0;
float pitchErrorSum = 0.0;

float Kp = 2.0;
float Ki = 0.1;
float Kd = 0.5;

float maxPitchCorrection = 20.0;
float maxIntegral = 100.0;

unsigned long lastMPURead = 0;
unsigned long lastPIDUpdate = 0;
const unsigned long mpuReadInterval = 20;
const unsigned long pidUpdateInterval = 20;

float pitchOffset = 0.0;

//this is the setup function and it initialises the PWM servo driver and MPU6050 sensor then performs calibration
void setup() {
  Serial.begin(9600);
  Serial.println("Quadruped PID Pitch Stabilization");

  pwm.begin();      //this initialises the PWM servo driver
  pwm.setOscillatorFrequency(27000000);       //this sets the oscillator frequency to 27MHz for accurate PWM
  pwm.setPWMFreq(SERVO_FREQ);       //this sets the PWM frequency to 50Hz for servo control

  if (!mpu.begin()) {       //this checks if the MPU6050 sensor is connected
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);        //this loops indefinitely if the sensor is not found
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);       //this sets the accelerometer range to +/- 8G
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);        //this sets the gyroscope range to +/- 500 degrees per second
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);     //this sets the filter bandwidth to 21Hz to reduce noise

  Serial.println("Calibrating... Keep robot level!");
  delay(1000);
  calibratePitch();     //this calls the calibration function to determine the pitch offset
  
  Serial.println("Calibration complete!");
  Serial.println("Starting walk cycle with PID stabilization...");
  delay(500);
}

//this function calibrates the pitch by taking 50 samples and calculating the average offset
void calibratePitch() {
  float sumPitch = 0;       //this initialises the sum of pitch values
  int samples = 50;     //this sets the number of calibration samples to take
  
  for (int i = 0; i < samples; i++) {       //this loops through all calibration samples
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);        //this reads the current sensor data from the MPU6050
    
    float pitch = atan2(a.acceleration.x, 
                        sqrt(a.acceleration.y * a.acceleration.y + 
                             a.acceleration.z * a.acceleration.z)) * 180.0 / PI;       //this calculates the pitch angle from accelerometer data
    sumPitch += pitch;      //this adds the pitch value to the running sum
    delay(10);      //this waits 10ms before taking the next sample
  }
  
  pitchOffset = sumPitch / samples;     //this calculates the average pitch offset
  Serial.print("Pitch offset: ");
  Serial.println(pitchOffset);      //this prints the calculated offset to the console
}

//this function sets a servo to a specific angle by converting the angle to a PWM pulse length
void setServoAngleRaw(uint8_t servonum, float angle) {
  if (angle < 0) angle = 0;     //this ensures the angle wont go below 0 degrees
  if (angle > 285) angle = 285;     //this ensures the angle wont go above 285 degrees

  int pulselen = SERVOMIN + (angle / 285.0) * (SERVOMAX - SERVOMIN);       //this converts the angle to a pulse length between SERVOMIN and SERVOMAX
  pwm.setPWM(servonum, 0, pulselen);        //this sends the pulse length to the specified servo
}

//this function calculates the current pitch angle from the accelerometer data
float calculatePitch() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);      //this reads the current sensor data from the MPU6050
  
  float pitch = atan2(a.acceleration.x, 
                      sqrt(a.acceleration.y * a.acceleration.y + 
                           a.acceleration.z * a.acceleration.z)) * 180.0 / PI;     //this calculates the pitch angle from accelerometer readings
  
  return pitch - pitchOffset;       //this returns the pitch angle with the calibration offset applied
}

//this function implements the PID controller and returns a correction angle to stabilise the pitch
float updatePID() {
  currentPitch = calculatePitch();      //this gets the current pitch angle from the sensor
  pitchError = targetPitch - currentPitch;      //this calculates the error between target and current pitch
  
  float P = Kp * pitchError;        //this calculates the proportional term
  
  pitchErrorSum += pitchError;      //this accumulates the error for the integral term
  if (pitchErrorSum > maxIntegral) pitchErrorSum = maxIntegral;     //this prevents the integral term from exceeding the maximum limit
  if (pitchErrorSum < -maxIntegral) pitchErrorSum = -maxIntegral;       //this prevents the integral term from going below the minimum limit
  float I = Ki * pitchErrorSum;     //this calculates the integral term
  
  float D = Kd * (pitchError - lastPitchError);     //this calculates the derivative term
  lastPitchError = pitchError;      //this stores the current error for the next derivative calculation
  
  float correction = P + I + D;     //this calculates the total correction angle
  
  if (correction > maxPitchCorrection) correction = maxPitchCorrection;     //this limits the correction to the maximum allowed value
  if (correction < -maxPitchCorrection) correction = -maxPitchCorrection;       //this limits the correction to the minimum allowed value
  
  return correction;        //this returns the calculated correction angle
}

//this function displays the current pitch status and PID terms to the serial console
void displayStatus() {
  Serial.print("Pitch: ");
  Serial.print(currentPitch, 2);        //this prints the current pitch angle
  Serial.print("° | Error: ");
  Serial.print(pitchError, 2);      //this prints the pitch error
  Serial.print("° | P: ");
  Serial.print(Kp * pitchError, 2);     //this prints the proportional term
  Serial.print(" | I: ");
  Serial.print(Ki * pitchErrorSum, 2);      //this prints the integral term
  Serial.print(" | D: ");
  Serial.println(Kd * (pitchError - lastPitchError), 2);        //this prints the derivative term
}

//this function smoothly moves all servos to a target frame with PID correction applied to hip tilt servos
void moveToFrame(float target[8], int steps, int stepDelay) {
  for (int s = 0; s < steps; s++) {     //this loops through all transition steps
    float pitchCorrection = 0;      //this initialises the pitch correction value
    if (millis() - lastPIDUpdate >= pidUpdateInterval) {        //this checks if enough time has passed for a PID update
      pitchCorrection = updatePID();        //this calculates the pitch correction using the PID controller
      lastPIDUpdate = millis();     //this updates the last PID update time
    }
    
    for (int i = 0; i < 8; i++) {       //this loops through all servos
      float start = currentAngles[i];       //this gets the starting angle for this servo
      float diff  = target[i] - start;      //this calculates the difference between target and current angle
      float newAngle = start + diff * ((float)s / steps);       //this calculates the intermediate angle for this step
      
      if (i == RIGHT_HIP_TILT || i == LEFT_HIP_TILT) {      //this checks if the servo is a hip tilt servo
        newAngle += pitchCorrection;        //this applies the PID correction to the hip tilt servos
      }
      
      setServoAngleRaw(i, newAngle);        //this moves the servo to the calculated angle
    }
    delay(stepDelay);       //this waits before the next step
  }

  for (int i = 0; i < 8; i++) {     //this loops through all servos
    currentAngles[i] = target[i];       //this updates the stored current angles to the target values
  }
}

//this is the main loop function and it executes the walking gait cycle with 8 frames
void loop() {
  float frame1[8] = {97,120,120,90,97,117,112,93};      //this defines the servo angles for frame 1
  float frame2[8] = {80,120,120,80,65,117,112,80};      //this defines the servo angles for frame 2
  float frame3[8] = {80,95,95,100,78,92,87,100};        //this defines the servo angles for frame 3
  float frame4[8] = {107,70,70,90,107,70,62,93};        //this defines the servo angles for frame 4
  float frame5[8] = {120,70,70,109,107,70,62,93};       //this defines the servo angles for frame 5
  float frame6[8] = {120,90,70,90,107,62,62,93};        //this defines the servo angles for frame 6
  float frame7[8] = {120,95,95,90,107,122,122,93};      //this defines the servo angles for frame 7
  float frame8[8] = {120,95,95,90,107,122,122,93};      //this defines the servo angles for frame 8

  int transitionSteps = 40;     //this sets the number of steps for smooth transitions between frames
  int stepDelay = 10;       //this sets the delay between each transition step in milliseconds

  moveToFrame(frame1, transitionSteps, stepDelay);      //this moves to frame 1
  displayStatus();      //this displays the current pitch status
  delay(200);       //this waits 200ms before the next frame

  moveToFrame(frame2, transitionSteps, stepDelay);      //this moves to frame 2
  displayStatus();      //this displays the current pitch status
  delay(200);       //this waits 200ms before the next frame

  moveToFrame(frame3, transitionSteps, stepDelay);      //this moves to frame 3
  displayStatus();      //this displays the current pitch status
  delay(200);       //this waits 200ms before the next frame

  moveToFrame(frame4, transitionSteps, stepDelay);      //this moves to frame 4
  displayStatus();      //this displays the current pitch status
  delay(200);       //this waits 200ms before the next frame

  moveToFrame(frame5, transitionSteps, stepDelay);      //this moves to frame 5
  displayStatus();      //this displays the current pitch status
  delay(200);       //this waits 200ms before the next frame

  moveToFrame(frame6, transitionSteps, stepDelay);      //this moves to frame 6
  displayStatus();      //this displays the current pitch status
  delay(200);       //this waits 200ms before the next frame

  moveToFrame(frame7, transitionSteps, stepDelay);      //this moves to frame 7
  displayStatus();      //this displays the current pitch status
  delay(100);       //this waits 100ms before the next frame
  
  moveToFrame(frame8, transitionSteps, stepDelay);      //this moves to frame 8
  displayStatus();      //this displays the current pitch status
  delay(100);       //this waits 100ms before repeating the cycle
}
*/