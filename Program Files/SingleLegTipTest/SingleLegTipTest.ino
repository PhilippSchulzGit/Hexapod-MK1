/**
 * Program to test out the PWM board for the robot ant
 * Created:   01.09.2021
 * Modified:  01.09.2021
 * Author:    Philipp Schulz
 */
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x40);              //object of the pwm driver board

#define FREQUENCY 50

#define SERVOMIN1  644  // This is the 'minimum' pulse length count (out of 4096) // was 650
#define SERVOMAX1  2633 // This is the 'maximum' pulse length count (out of 4096) // was 2350

long value = 0;
long minimum = 0;
long maximum = 130;
int timeDelay = 1000;
int setDelay = 10;
bool direction = true;

int pulseWidth1(int angle){          //converts given angle to a number in 4096
  int pulse_wide, analog_value;
  pulse_wide = map(angle, 0, 180, SERVOMIN1, SERVOMAX1);
  analog_value = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096);
  return analog_value;
}

void setup() {
  Serial.begin(9600);
  pwm1.begin();
  pwm1.setPWMFreq(FREQUENCY);
  pwm1.setPWM(5,0,pulseWidth1(value));
  delay(timeDelay);
}

void loop() {
    Serial.println(value);
    pwm1.setPWM(5,0,pulseWidth1(value));
    if(direction) {
      value++;
    } else {
      value--;
    }
    if(value == maximum) {
      direction = false;
    } else if(value == minimum) {
      direction = true;
    }
    delay(setDelay);
}
