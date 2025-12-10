#include <Wire.h>
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
  setServoAngle(0, 81);
  setServoAngle(1, 45);
  setServoAngle(2, 45);
  setServoAngle(3, 81);
  setServoAngle(4, 90);
  setServoAngle(5, 45);
  setServoAngle(6, 45);
  setServoAngle(7, 90);
  delay(2000);

 
  //frame 2
  setServoAngle(0, 90);
  setServoAngle(1, 45);
  setServoAngle(2, 45);
  setServoAngle(3, 90);
  setServoAngle(4, 115);
  setServoAngle(5, 45);
  setServoAngle(6, 45);
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
  setServoAngle(0, 81);
  setServoAngle(1, 135);
  setServoAngle(2, 135);
  setServoAngle(3, 81);
  setServoAngle(4, 90);
  setServoAngle(5, 135);
  setServoAngle(6, 135);
  setServoAngle(7, 90);
  delay(2000);


  //frame 6
  setServoAngle(0, 81);
  setServoAngle(1, 90);
  setServoAngle(2, 90);
  setServoAngle(3, 81);
  setServoAngle(4, 90);
  setServoAngle(5, 45);
  setServoAngle(6, 45);
  setServoAngle(7, 90);
  delay(2000);
 
  //frame 7
  setServoAngle(0, 81);
  setServoAngle(1, 45);
  setServoAngle(2, 45);
  setServoAngle(3, 81);
  setServoAngle(4, 90);
  setServoAngle(5, 45);
  setServoAngle(6, 45);
  setServoAngle(7, 90);
  delay(2000);
}
