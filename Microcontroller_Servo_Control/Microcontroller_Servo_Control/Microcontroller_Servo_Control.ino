#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  90   // 'minimum' pulse length count (out of 4096)
#define SERVOMAX  490  // 'maximum' pulse length count (out of 4096)
#define SERVO_FREQ 50  // Analog servos run at ~50 Hz updates

uint8_t servonum = 0;  // Servo channel number on PCA9685

void setup() {
  Serial.begin(9600);
  Serial.println("Servo angle test!");

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
  setServoAngle(servonum, 45);
  delay(2000);

  setServoAngle(servonum, 90);
  delay(2000);

  setServoAngle(servonum, 180);
  delay(2000);

  setServoAngle(servonum, 0);
  delay(2000);
}

