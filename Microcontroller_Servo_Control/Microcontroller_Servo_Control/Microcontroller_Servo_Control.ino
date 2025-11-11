//**********Task 1: Understanding and Controlling RC Servomotors*************
//Implement a basic control program on a microcontroller to move an RC servomotor to specified angles
//Demonstrate control of a single servo by setting it to various positions using PWM signals.
//***************************************************************************


#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

void setup() {
  Serial.begin(9600);
  Serial.write("Microcontroller Servo Control");
  
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);

  delay(10);
}

void loop() {
  Serial.println(servoAngle);

}
