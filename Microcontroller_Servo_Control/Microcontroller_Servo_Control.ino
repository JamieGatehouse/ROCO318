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
