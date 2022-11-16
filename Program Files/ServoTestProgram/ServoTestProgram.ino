/**
 * Program to test out the PWM board for the robot ant
 * Created:   01.09.2021
 * Modified:  01.09.2021
 * Author:    Philipp Schulz
 */
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x40);              //object of the pwm driver board
Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x41);              //object of the pwm driver board

#define FREQUENCY 50

#define SERVOMIN1  644  // This is the 'minimum' pulse length count (out of 4096) // was 650
#define SERVOMIN2  697  // This is the 'minimum' pulse length count (out of 4096) // was 650
#define SERVOMAX1  2633 // This is the 'maximum' pulse length count (out of 4096) // was 2350
#define SERVOMAX2  2741 // This is the 'maximum' pulse length count (out of 4096) // was 2350

// --------------------------used PWM pins on boards----------------------
/* Leg Nr. | Direction | Joint | Board Nr. | Pin Nr.
      1    |    left   |  hip  |     1     |    14
      1         left     knee        1          13
      1         left     foot        1          15
      1        right      hip        1           9
      1        right     knee        1          10
      1        right     foot        1          11
      2         left      hip        1           2
      2         left     knee        1           1
      2         left     foot        1           0
      2        right      hip        2          14
      2        right     knee        2          13
      2        right     foot        2          15
      3         left      hip        2           2
      3         left     knee        2           1
      3         left     foot        2           0
      3        right      hip        2           6
      3        right     knee        2           5
      3        right     foot        2           4
// --------------------------used PWM pins on boards----------------------*/

long value = 90;
int timeDelay = 1000;
int setDelay = 500;

int pulseWidth1(int angle){          //converts given angle to a number in 4096
  int pulse_wide, analog_value;
  pulse_wide = map(angle, 0, 180, SERVOMIN1, SERVOMAX1);
  analog_value = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096);
  return analog_value;
}
int pulseWidth2(int angle){          //converts given angle to a number in 4096
  int pulse_wide, analog_value;
  pulse_wide = map(angle, 0, 180, SERVOMIN2, SERVOMAX2);
  analog_value = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096);
  return analog_value;
}

void setup() {
  pwm1.begin();
  pwm2.begin();
  //pwm1.setOscillatorFrequency(27000000);  // The int.osc. is closer to 27MHz  
  pwm1.setPWMFreq(FREQUENCY);
  pwm2.setPWMFreq(FREQUENCY);
  Serial.begin(9600);
  pwm1.sleep();
  pwm2.sleep();
  pwm1.wakeup();
  pwm1.setPWM(0,0,pulseWidth1(value));
  delay(timeDelay);
  pwm1.setPWM(1,0,pulseWidth1(value));
  delay(timeDelay);
  pwm1.setPWM(2,0,pulseWidth1(value));
  delay(timeDelay);
  pwm1.setPWM(9,0,pulseWidth1(value));
  delay(timeDelay);
  pwm1.setPWM(10,0,pulseWidth1(value));
  delay(timeDelay);
  pwm1.setPWM(11,0,pulseWidth1(value));
  delay(timeDelay);
  pwm1.setPWM(13,0,pulseWidth1(value));
  delay(timeDelay);
  pwm1.setPWM(14,0,pulseWidth1(value));
  delay(timeDelay);
  pwm1.setPWM(15,0,pulseWidth1(value));
  delay(timeDelay);
  pwm2.wakeup();
  pwm2.setPWM(0,0,pulseWidth2(value));
  delay(timeDelay);
  pwm2.setPWM(1,0,pulseWidth2(value));
  delay(timeDelay);
  pwm2.setPWM(2,0,pulseWidth2(value));
  delay(timeDelay);
  pwm2.setPWM(4,0,pulseWidth2(value));
  delay(timeDelay);
  pwm2.setPWM(5,0,pulseWidth2(value));
  delay(timeDelay);
  pwm2.setPWM(6,0,pulseWidth2(value));
  delay(timeDelay);
  pwm2.setPWM(13,0,pulseWidth2(value));
  delay(timeDelay);
  pwm2.setPWM(14,0,pulseWidth2(value));
  delay(timeDelay);
  pwm2.setPWM(15,0,pulseWidth2(value));
  delay(timeDelay);
  Serial.println("ready");
}

void loop() {
  if(Serial.available()) {
    value = Serial.parseFloat();
    // 1. board
    pwm1.setPWM(0,0,pulseWidth1(value));
    delay(setDelay);
    pwm1.setPWM(1,0,pulseWidth1(value));
    delay(setDelay);
    pwm1.setPWM(2,0,pulseWidth1(value));
    delay(setDelay);
    pwm1.setPWM(9,0,pulseWidth1(value));
    delay(setDelay);
    pwm1.setPWM(10,0,pulseWidth1(value));
    delay(setDelay);
    pwm1.setPWM(11,0,pulseWidth1(value));
    delay(setDelay);
    pwm1.setPWM(13,0,pulseWidth1(value));
    delay(setDelay);
    pwm1.setPWM(14,0,pulseWidth1(value));
    delay(setDelay);
    pwm1.setPWM(15,0,pulseWidth1(value));
    delay(setDelay);
    Serial.println("board 1 set");
    //2. board
    pwm2.setPWM(0,0,pulseWidth2(value));
    delay(setDelay);
    pwm2.setPWM(1,0,pulseWidth2(value));
    delay(setDelay);
    pwm2.setPWM(2,0,pulseWidth2(value));
    delay(setDelay);
    pwm2.setPWM(4,0,pulseWidth2(value));
    delay(setDelay);
    pwm2.setPWM(5,0,pulseWidth2(value));
    delay(setDelay);
    pwm2.setPWM(6,0,pulseWidth2(value));
    delay(setDelay);
    pwm2.setPWM(13,0,pulseWidth2(value));
    delay(setDelay);
    pwm2.setPWM(14,0,pulseWidth2(value));
    delay(setDelay);
    pwm2.setPWM(15,0,pulseWidth2(value));
    delay(setDelay);
    Serial.println("board 2 set");
    Serial.println(value);
    Serial.println(map(value, 0, 180, SERVOMIN1, SERVOMAX1));
    Serial.println(map(value, 0, 180, SERVOMIN2, SERVOMAX2));
    value = Serial.parseFloat();
  }
}
