/**
 * Program to test out smoother movements and standing up
 * Created:   17.12.2021
 * Modified:  17.12.2021
 * Author:    Philipp Schulz
 */
/* --------------------------used PWM pins on boards----------------------
   Leg Nr. | Direction | Joint | Board Nr. | Pin Nr. | Servo Nr.
      1    |    left   |  hip  |     1     |    14   |      4
      1    |    left   | knee  |     1     |    13   |      5
      1    |    left   | foot  |     1     |    15   |      6
      1    |   right   |  hip  |     1     |     9   |      1
      1    |   right   | knee  |     1     |    10   |      2
      1    |   right   | foot  |     1     |    11   |      3
      2    |    left   |  hip  |     1     |     2   |     10
      2    |    left   | knee  |     1     |     1   |     11
      2    |    left   | foot  |     1     |     0   |     12
      2    |   right   |  hip  |     2     |    14   |      7
      2    |   right   | knee  |     2     |    13   |      8
      2    |   right   | foot  |     2     |    15   |      9
      3    |    left   |  hip  |     2     |     2   |     16
      3    |    left   | knee  |     2     |     1   |     17
      3    |    left   | foot  |     2     |     0   |     18
      3    |   right   |  hip  |     2     |     6   |     13
      3    |   right   | knee  |     2     |     5   |     14
      3    |   right   | foot  |     2     |     4   |     15
// --------------------------used PWM pins on boards----------------------*/

// library imports
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <ESP8266WiFi.h>

// objects of PWM driver boards
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x40);              //object of the pwm driver board
Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x41);              //object of the pwm driver board

// frequency at which the PWM boards run
#define FREQUENCY 50

// calibration values for PWM boards
#define SERVOMIN1  644                                    // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMIN2  697                                    // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX1  2633                                   // This is the 'maximum' pulse length count (out of 4096)
#define SERVOMAX2  2741                                   // This is the 'maximum' pulse length count (out of 4096)

// variables to track states of all servos

// NEW CONSTANTS FOR X-AXIS
int alpha = 65;
int beta = 20;

float cycleTime = 0.01;                                   // time between individual cycles of movements [s]
float homingDelay = 750;                                  // delay between individual homing of servos at startup [ms]
long previousTime = 0;                                    // timestamp of previous cycle
float PT1_K = 1;                                          // proportional component of simulated PT1
float PT1_T = 0.15;                                       // time component of simulated PT1 [s]
float c1 = 0;                                             // c1 component of DZG of discrete PT1
float d0 = 0;                                             // d0 component of DZG of discrete PT1
float d1 = 0;                                             // d1 component of DZG of discrete PT1
// current target values of all servos
float target1 = 110;                                      // target value for servo 1
float target2 = 100;                                      // target value for servo 2
float target3 = 95;                                       // target value for servo 3
float target4 = 80;                                       // target value for servo 4
float target5 = 100;                                      // target value for servo 5
float target6 = 90;                                       // target value for servo 6
float target7 = 80;                                       // target value for servo 7
float target8 = 100;                                      // target value for servo 8
float target9 = 90;                                       // target value for servo 9
float target10 = 100;                                     // target value for servo 10
float target11 = 90;                                      // target value for servo 11
float target12 = 95;                                      // target value for servo 12
float target13 = 100;                                     // target value for servo 13
float target14 = 80;                                      // target value for servo 14
float target15 = 80;                                      // target value for servo 15
float target16 = 100;                                     // target value for servo 16
float target17 = 90;                                       // target value for servo 17
float target18 = 90;                                       // target value for servo 18
// previous target values of all servos
float oldTarget1 = target1;                               // old target value for servo 1
float oldTarget2 = target2;                               // old target value for servo 2
float oldTarget3 = target3;                               // old target value for servo 3
float oldTarget4 = target4;                               // old target value for servo 4
float oldTarget5 = target5;                               // old target value for servo 5
float oldTarget6 = target6;                               // old target value for servo 6
float oldTarget7 = target7;                               // old target value for servo 7
float oldTarget8 = target8;                               // old target value for servo 8
float oldTarget9 = target9;                               // old target value for servo 9
float oldTarget10 = target10;                             // old target value for servo 10
float oldTarget11 = target11;                             // old target value for servo 11
float oldTarget12 = target12;                             // old target value for servo 12
float oldTarget13 = target13;                             // old target value for servo 13
float oldTarget14 = target14;                             // old target value for servo 14
float oldTarget15 = target15;                             // old target value for servo 15
float oldTarget16 = target16;                             // old target value for servo 16
float oldTarget17 = target17;                             // old target value for servo 17
float oldTarget18 = target18;                             // old target value for servo 18
// current values of all servos
float current1 = target1;                                 // current value for servo 1
float current2 = target2;                                 // current value for servo 2
float current3 = target3;                                 // current value for servo 3
float current4 = target4;                                 // current value for servo 4
float current5 = target5;                                 // current value for servo 5
float current6 = target6;                                 // current value for servo 6
float current7 = target7;                                 // current value for servo 7
float current8 = target8;                                 // current value for servo 8
float current9 = target9;                                 // current value for servo 9
float current10 = target10;                               // current value for servo 10
float current11 = target11;                               // current value for servo 11
float current12 = target12;                               // current value for servo 12
float current13 = target13;                               // current value for servo 13
float current14 = target14;                               // current value for servo 14
float current15 = target15;                               // current value for servo 15
float current16 = target16;                               // current value for servo 16
float current17 = target17;                               // current value for servo 17
float current18 = target18;                               // current value for servo 18
// previous values of all servos
float old1 = current1;                                    // old value for servo 1
float old2 = current2;                                    // old value for servo 2
float old3 = current3;                                    // old value for servo 3
float old4 = current4;                                    // old value for servo 4
float old5 = current5;                                    // old value for servo 5
float old6 = current6;                                    // old value for servo 6
float old7 = current7;                                    // old value for servo 7
float old8 = current8;                                    // old value for servo 8
float old9 = current9;                                    // old value for servo 9
float old10 = current10;                                  // old value for servo 10
float old11 = current11;                                  // old value for servo 11
float old12 = current12;                                  // old value for servo 12
float old13 = current13;                                  // old value for servo 13
float old14 = current14;                                  // old value for servo 14
float old15 = current15;                                  // old value for servo 15
float old16 = current16;                                  // old value for servo 16
float old17 = current17;                                  // old value for servo 17
float old18 = current18;                                  // old value for servo 18

// method to convert a given angle into a number in 4096 for PWM board 1
int pulseWidth1(int angle){
  int pulse_wide, analog_value;
  pulse_wide = map(angle, 0, 180, SERVOMIN1, SERVOMAX1);
  analog_value = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096);
  return analog_value;
}

// method to convert a given angle into a number in 4096 for PWM board 2
int pulseWidth2(int angle){
  int pulse_wide, analog_value;
  pulse_wide = map(angle, 0, 180, SERVOMIN2, SERVOMAX2);
  analog_value = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096);
  return analog_value;
}

// method to home all servos based on the initial target values
void homeServos() {
  Serial.println("starting homing");
  setServoByNumber(2,target2);                            // home knee joint front right
  setServoByNumber(3,target3);                            // home foot joint front right
  delay(homingDelay);
  setServoByNumber(5,target5);                            // home knee joint front left
  setServoByNumber(6,target6);                            // home foot joint front left
  delay(homingDelay);
  setServoByNumber(8,target8);                            // home knee joint middle right
  setServoByNumber(9,target9);                            // home foot joint middle right
  delay(homingDelay);
  setServoByNumber(11,target11);                          // home knee joint middle left
  setServoByNumber(12,target12);                          // home foot joint middle left
  delay(homingDelay);
  setServoByNumber(14,target14);                          // home knee joint back right
  setServoByNumber(15,target15);                          // home foot joint back right
  delay(homingDelay);
  setServoByNumber(17,target17);                          // home knee joint back left
  setServoByNumber(18,target18);                          // home foot joint back left
  target17 = 0;                                           // target value for servo 17
  target18 = 0;                                           // target value for servo 18
  delay(homingDelay);
  setServoByNumber(1,target1);                            // home hip joint front right
  delay(homingDelay);
  setServoByNumber(4,target4);                            // home hip joint front left
  delay(homingDelay);
  setServoByNumber(7,target7);                            // home hip joint middle right
  delay(homingDelay);
  setServoByNumber(10,target10);                          // home hip joint middle left
  delay(homingDelay);
  setServoByNumber(13,target13);                          // home hip joint back right
  delay(homingDelay);
  setServoByNumber(16,target16);                          // home hip joint back left
  delay(homingDelay);
}

// method to set a servo by its number to a new targetPosition
void setServoByNumber(int servoNumber, float targetPosition) {
  switch(servoNumber) {                                   // go over all possible servo numbers
    case 1:
      pwm1.setPWM(9,0,pulseWidth1(targetPosition));       // move servo 1 in position
      old1 = current1;                                    // save new old position
      current1 = targetPosition;                          // save new current position
      break;
    case 2:
      pwm1.setPWM(10,0,pulseWidth1(targetPosition));      // move servo 2 in position
      old2 = current2;                                    // save new old position
      current2 = targetPosition;                          // save new current position
      break;
    case 3:
      pwm1.setPWM(11,0,pulseWidth1(targetPosition));      // move servo 3 in position
      old3 = current3;                                    // save new old position
      current3 = targetPosition;                          // save new current position
      break;
    case 4:
      pwm1.setPWM(14,0,pulseWidth1(targetPosition));      // move servo 4 in position
      old4 = current4;                                    // save new old position
      current4 = targetPosition;                          // save new current position
      break;
    case 5:
      pwm1.setPWM(13,0,pulseWidth1(targetPosition));      // move servo 5 in position
      old5 = current5;                                    // save new old position
      current5 = targetPosition;                          // save new current position
      break;
    case 6:
      pwm1.setPWM(15,0,pulseWidth1(targetPosition));      // move servo 6 in position
      old6 = current6;                                    // save new old position
      current6 = targetPosition;                          // save new current position
      break;
    case 7:
      pwm2.setPWM(14,0,pulseWidth2(targetPosition));      // move servo 7 in position
      old7 = current7;                                    // save new old position
      current7 = targetPosition;                          // save new current position
      break;
    case 8:
      pwm2.setPWM(13,0,pulseWidth2(targetPosition));      // move servo 8 in position
      old8 = current8;                                    // save new old position
      current8 = targetPosition;                          // save new current position
      break;
    case 9:
      pwm2.setPWM(15,0,pulseWidth2(targetPosition));      // move servo 9 in position
      old9 = current9;                                    // save new old position
      current9 = targetPosition;                          // save new current position
      break;
    case 10:
      pwm1.setPWM(2,0,pulseWidth1(targetPosition));       // move servo 10 in position
      old10 = current10;                                  // save new old position
      current10 = targetPosition;                         // save new current position
      break;
    case 11:
      pwm1.setPWM(1,0,pulseWidth1(targetPosition));       // move servo 11 in position
      old11 = current11;                                  // save new old position
      current11 = targetPosition;                         // save new current position
      break;
    case 12:
      pwm1.setPWM(0,0,pulseWidth1(targetPosition));       // move servo 12 in position
      old12 = current12;                                  // save new old position
      current12 = targetPosition;                         // save new current position
      break;
    case 13:
      pwm2.setPWM(6,0,pulseWidth2(targetPosition));       // move servo 13 in position
      old13 = current13;                                  // save new old position
      current13 = targetPosition;                         // save new current position
      break;
    case 14:
      pwm2.setPWM(5,0,pulseWidth2(targetPosition));       // move servo 14 in position
      old14 = current14;                                  // save new old position
      current14 = targetPosition;                         // save new current position
      break;
    case 15:
      pwm2.setPWM(4,0,pulseWidth2(targetPosition));       // move servo 15 in position
      old15 = current15;                                  // save new old position
      current15 = targetPosition;                         // save new current position
      break;
    case 16:
      pwm2.setPWM(2,0,pulseWidth2(targetPosition));       // move servo 16 in position
      old16 = current16;                                  // save new old position
      current16 = targetPosition;                         // save new current position
      break;
    case 17:
      pwm2.setPWM(1,0,pulseWidth2(targetPosition));       // move servo 17 in position
      old17 = current17;                                  // save new old position
      current17 = targetPosition;                         // save new current position
      break;
    case 18:
      pwm2.setPWM(0,0,pulseWidth2(targetPosition));       // move servo 18 in position
      old18 = current18;                                  // save new old position
      current18 = targetPosition;                         // save new current position
      break;
  }
}

// method to implement the discrete model for the servo
float servoModel(float previousPosition, float previousTarget, float currentTarget) {
  return (d0*currentTarget)+(d1*previousTarget)-(c1*previousPosition);
}

// method to calculate required values for servo model based on given constants
void initializeServoModel() {
  c1 = (cycleTime - 2*PT1_T)/(cycleTime + 2*PT1_T);       // calculate c1 component of discrete PT1
  d0 = (PT1_K * cycleTime)/(2*PT1_T + cycleTime);         // calculate d0 component of discrete PT1
  d1 = d0;                                                // calculate d1 component of discrete PT1
  Serial.print("c1: ");
  Serial.print(c1,5);
  Serial.print(" d0: ");
  Serial.print(d0,5);
  Serial.print(" d1: ");
  Serial.println(d1,5);
}

// method for setting a new target for a specific servo
void setNewTarget(int number, float newTarget) {
  switch(number) {                                        // go over all possible servo numbers
    case 1:                                               // set new target for servo 1
      oldTarget1 = target1;                               // assign new old target position 1
      target1 = newTarget;                                // assign new target position 1
      break;
    case 2:                                               // set new target for servo 2
      oldTarget2 = target2;                               // assign new old target position 2
      target2 = newTarget;                                // assign new target position 2
      break;
    case 3:                                               // set new target for servo 3
      oldTarget3 = target3;                               // assign new old target position 3
      target3 = newTarget;                                // assign new target position 3
      break;
    case 4:                                               // set new target for servo 4
      oldTarget4 = target4;                               // assign new old target position 4
      target4 = newTarget;                                // assign new target position 4
      break;
    case 5:                                               // set new target for servo 5
      oldTarget5 = target5;                               // assign new old target position 5
      target5 = newTarget;                                // assign new target position 5
      break;
    case 6:                                               // set new target for servo 6
      oldTarget6 = target6;                               // assign new old target position 6
      target6 = newTarget;                                // assign new target position 6
      break;
    case 7:                                               // set new target for servo 7
      oldTarget7 = target7;                               // assign new old target position 7
      target7 = newTarget;                                // assign new target position 7
      break;
    case 8:                                               // set new target for servo 8
      oldTarget8 = target8;                               // assign new old target position 8
      target8 = newTarget;                                // assign new target position 8
      break;
    case 9:                                               // set new target for servo 9
      oldTarget9 = target9;                               // assign new old target position 9
      target9 = newTarget;                                // assign new target position 9
      break;
    case 10:                                              // set new target for servo 10
      oldTarget10 = target10;                             // assign new old target position 10
      target10 = newTarget;                               // assign new target position 10
      break;
    case 11:                                              // set new target for servo 11
      oldTarget11 = target11;                             // assign new old target position 11
      target11 = newTarget;                               // assign new target position 11
      break;
    case 12:                                              // set new target for servo 12
      oldTarget12 = target12;                             // assign new old target position 12
      target12 = newTarget;                               // assign new target position 12
      break;
    case 13:                                              // set new target for servo 13
      oldTarget13 = target13;                             // assign new old target position 13
      target13 = newTarget;                               // assign new target position 13
      break;
    case 14:                                              // set new target for servo 14
      oldTarget14 = target14;                             // assign new old target position 14
      target14 = newTarget;                               // assign new target position 14
      break;
    case 15:                                              // set new target for servo 15
      oldTarget15 = target15;                             // assign new old target position 15
      target15 = newTarget;                               // assign new target position 15
      break;
    case 16:                                              // set new target for servo 16
      oldTarget16 = target16;                             // assign new old target position 16
      target16 = newTarget;                               // assign new target position 16
      break;
    case 17:                                              // set new target for servo 17
      oldTarget17 = target17;                             // assign new old target position 17
      target17 = newTarget;                               // assign new target position 17
      break;
    case 18:                                              // set new target for servo 18
      oldTarget18 = target18;                             // assign new old target position 18
      target18 = newTarget;                               // assign new target position 18
      break;
  }
}

//method controlling the 'main' program. This is what should be done inside the cycles created from the loop method
void runProgram() {
  // NEW STUFF FOR INVERSE KINEMATICS
  if(Serial.available()) {  // if Serial input was given
    String input = Serial.readString();   // get input
    Serial.println("input: "+input);
    if(input.indexOf("x")>=0)   // if x-coordinate should be changed
    {
      int newPos = input.substring(input.indexOf(" ")).toInt();

      // calculate new angles
      double newAlpha = acos(double(sq(75) + sq(85) - sq(newPos))/double(2*75*85))*180/PI;
      double newBeta = acos(double(sq(newPos) + sq(75) - sq(85))/double(2*newPos*85))*180/PI;
      Serial.println("alpha: "+String(newAlpha)+" beta: "+String(newBeta));

      // set new targets
      setNewTarget(17,newBeta+beta);                                   // set new target position 17
      setNewTarget(18,newAlpha+alpha);                                 // set new target position 18
      
    } else if(input.indexOf("a")>=0) {  // if start value of alpha should be changed
      alpha = input.substring(input.indexOf(" ")).toInt();    // set new starting angle for alpha
      
    } else if(input.indexOf("b")>=0) {  // if start value of beta should be changed
      beta = input.substring(input.indexOf(" ")).toInt();     // set new starting angle for beta
    }
  }
  if(millis()-previousTime >= cycleTime*1000) {
    // UPDATE ALL SERVO POSITIONS
    setServoByNumber(1,servoModel(old1, oldTarget1, target1)); // set new servo value 1
    setServoByNumber(2,servoModel(old2, oldTarget2, target2)); // set new servo value 2
    setServoByNumber(3,servoModel(old3, oldTarget3, target3)); // set new servo value 3
    setServoByNumber(4,servoModel(old4, oldTarget4, target4)); // set new servo value 4
    setServoByNumber(5,servoModel(old5, oldTarget5, target5)); // set new servo value 5
    setServoByNumber(6,servoModel(old6, oldTarget6, target6)); // set new servo value 6
    setServoByNumber(7,servoModel(old7, oldTarget7, target7)); // set new servo value 7
    setServoByNumber(8,servoModel(old8, oldTarget8, target8)); // set new servo value 8
    setServoByNumber(9,servoModel(old9, oldTarget9, target9)); // set new servo value 9
    setServoByNumber(10,servoModel(old10, oldTarget10, target10)); // set new servo value 10
    setServoByNumber(11,servoModel(old11, oldTarget11, target11)); // set new servo value 11
    setServoByNumber(12,servoModel(old12, oldTarget12, target12)); // set new servo value 12
    setServoByNumber(13,servoModel(old13, oldTarget13, target13)); // set new servo value 13
    setServoByNumber(14,servoModel(old14, oldTarget14, target14)); // set new servo value 14
    setServoByNumber(15,servoModel(old15, oldTarget15, target15)); // set new servo value 15
    setServoByNumber(16,servoModel(old16, oldTarget16, target16)); // set new servo value 16
    setServoByNumber(17,servoModel(old17, oldTarget17, target17+beta)); // set new servo value 17
    setServoByNumber(18,servoModel(old18, oldTarget18, target18+alpha)); // set new servo value 18
    previousTime = millis();
  }
}

// method to return the 'index'th substring separated by the given separator
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

void setup() {
  WiFi.forceSleepBegin();                                 // WIFI is not required, turn it off to save power
  Serial.begin(9600);                                   // begin Serial
  delay(1000);
  pwm1.begin();                                           // start up PWM board 1
  pwm2.begin();                                           // start up PWM board 2
  pwm1.setPWMFreq(FREQUENCY);                             // set PWM frequency for PWM board 1
  pwm2.setPWMFreq(FREQUENCY);                             // set PWM frequency for PWM board 2
  homeServos();                                           // home all servos
  Serial.println("homing done, calculating model");
  delay(1000);
  initializeServoModel();                                 // calculate required values for servo model
  Serial.println("ready");                                // send out signal that tests can start
  Serial.println("change the x-coordinate with this command: x 100");
  Serial.println("adjust starting angle of alpha with this command: a 45");
  Serial.println("adjust starting angle of beta with this command: b 45");
}

void loop() {
    runProgram();                                         // call the cycle function to run the program
}
