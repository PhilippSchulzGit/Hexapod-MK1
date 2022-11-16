/**
 * Program to test out the individual servos of the robot ant
 * Created:   07.09.2021
 * Modified:  07.09.2021
 * Author:    Philipp Schulz
 */
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x40);              //object of the pwm driver board
Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x41);              //object of the pwm driver board

#define FREQUENCY 50

#define SERVOMIN1  644  // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMIN2  697  // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX1  2633 // This is the 'maximum' pulse length count (out of 4096)
#define SERVOMAX2  2741 // This is the 'maximum' pulse length count (out of 4096)

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

// https://stackoverflow.com/questions/9072320/split-string-into-string-array
String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length()-1;

  for(int i=0; i<=maxIndex && found<=index; i++){
    if(data.charAt(i)==separator || i==maxIndex){
        found++;
        strIndex[0] = strIndex[1]+1;
        strIndex[1] = (i == maxIndex) ? i+1 : i;
    }
  }

  return found>index ? data.substring(strIndex[0], strIndex[1]) : "";
}

void moveServoByNumber(int servoNumber, int angle) {
  int PWMBoard = 1;
  int servoPin = 0;
  bool inputValid = true;
  // find out which servo to use
  if(servoNumber == 1) {
    servoPin = 9;
  } else if(servoNumber == 2) {
    servoPin = 10;
  } else if(servoNumber == 3) {
    servoPin = 11;
  } else if(servoNumber == 4) {
    servoPin = 14;
  } else if(servoNumber == 5) {
    servoPin = 13;
  } else if(servoNumber == 6) {
    servoPin = 15;
  } else if(servoNumber == 7) {
    servoPin = 14;
    PWMBoard = 2;
  } else if(servoNumber == 8) {
    servoPin = 13;
    PWMBoard = 2;
  } else if(servoNumber == 9) {
    servoPin = 15;
    PWMBoard = 2;
  } else if(servoNumber == 10) {
    servoPin = 2;
  } else if(servoNumber == 11) {
    servoPin = 1;
  } else if(servoNumber == 12) {
    servoPin = 0;
  } else if(servoNumber == 13) {
    servoPin = 6;
    PWMBoard = 2;
  } else if(servoNumber == 14) {
    servoPin = 5;
    PWMBoard = 2;
  } else if(servoNumber == 15) {
    servoPin = 4;
    PWMBoard = 2;
  } else if(servoNumber == 16) {
    servoPin = 2;
    PWMBoard = 2;
  } else if(servoNumber == 17) {
    servoPin = 1;
    PWMBoard = 2;
  } else if(servoNumber == 18) {
    servoPin = 0;
    PWMBoard = 2;
  } else {
    inputValid = false; //set flag
  }
  //if a valid input was given
  if(inputValid) {
    // decide on which PWM board to use
    if(PWMBoard == 1) {
      //set PWM value
      pwm1.setPWM(servoPin,0,pulseWidth1(angle));
    } else if(PWMBoard == 2) {
      //set PWM value
      pwm2.setPWM(servoPin,0,pulseWidth2(angle));
    }
  }
}

void setup() {
  pwm1.begin();
  pwm2.begin();
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
    String serialData = Serial.readString();
    moveServoByNumber(getValue(serialData,' ',0).toInt(),getValue(serialData,' ',1).toInt());
    Serial.println(getValue(serialData,' ',0)+" : "+getValue(serialData,' ',1));
  }
}
