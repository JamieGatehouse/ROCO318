#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

//this creates an instance of the PWM servo driver using the default I2C address
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  90   // this defines the minimum pulse length count (out of 4096) for 0 degree position
#define SERVOMAX  490  // this defines the maximum pulse length count (out of 4096) for 285 degree position
#define SERVO_FREQ 50  // this sets the PWM frequency to 50 Hz which is standard for analog servos

uint8_t servonum = 0;  // this stores the servo channel number on the PCA9685 board

void setup() {
  Serial.begin(9600);     //this initialises serial communication at 9600 baud rate for debugging output
  Serial.println("Servo angle test!");     //this prints the message to the serial monitor

  pwm.begin();     //this initialises the PWM driver and prepares it for communication
  pwm.setOscillatorFrequency(27000000);     //this sets the internal oscillator frequency to 27MHz for accurate timing
  pwm.setPWMFreq(SERVO_FREQ);     //this sets the PWM frequency to 50Hz which is required for servo control
  delay(10);     //this provides a short delay to ensure the PWM driver is fully initialised before use
}

//this setServoAngle function converts an angle in degrees to a PWM pulse length and sends it to the specified servo
//it takes the servo channel number and desired angle as parameters and constrains the angle to valid range
void setServoAngle(uint8_t servonum, float angle) {
  if (angle < 0) angle = 0;     //this ensures the angle wont go below 0 degrees (as that is impossible)
  if (angle > 285) angle = 285;     //this ensures the angle wont go above 285 degrees (as that is the maximum for this servo)

  int pulselen = SERVOMIN + (angle / 285.0) * (SERVOMAX - SERVOMIN);     //this calculates the pulse length by mapping the angle to the pulse range
  pwm.setPWM(servonum, 0, pulselen);     //this sends the PWM signal to the servo with the calculated pulse length
}

//this loop function runs continuously and cycles the servo through the different angles
//each angle position is held for 2 seconds before moving to the next position
void loop() {
  setServoAngle(servonum, 45);     //this moves the servo to 45 degrees
  delay(2000);     //this waits for 2 seconds at 45 degrees

  setServoAngle(servonum, 90);     //this moves the servo to 90 degrees
  delay(2000);     //this waits for 2 seconds at 90 degrees

  setServoAngle(servonum, 180);     //this moves the servo to 180 degrees
  delay(2000);     //this waits for 2 seconds at 180 degrees

  setServoAngle(servonum, 0);     //this moves the servo back to 0 degrees
  delay(2000);     //this waits for 2 seconds at 0 degrees before the loop repeats
}