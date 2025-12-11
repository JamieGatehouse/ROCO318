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

