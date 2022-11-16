/**
 * Program to test out smoother movements and standing up
 * Created:   21.12.2021
 * Modified:  30.12.2021
 * Author:    Philipp Schulz
 */
/* --------------------------used PWM pins on boards----------------------
   Leg Nr. | Direction | Joint | Board Nr. | Pin Nr. | Servo Nr.
      1    |    left   |  hip  |     1     |    14   |      4
      1    |    left   | knee  |     1     |    13   |      5
      1    |    left   | foot  |     1     |    15   |      6
      2    |   right   |  hip  |     1     |     9   |      1
      2    |   right   | knee  |     1     |    10   |      2
      2    |   right   | foot  |     1     |    11   |      3
      3    |    left   |  hip  |     1     |     2   |     10
      3    |    left   | knee  |     1     |     1   |     11
      3    |    left   | foot  |     1     |     0   |     12
      4    |   right   |  hip  |     2     |    14   |      7
      4    |   right   | knee  |     2     |    13   |      8
      4    |   right   | foot  |     2     |    15   |      9
      5    |    left   |  hip  |     2     |     2   |     16
      5    |    left   | knee  |     2     |     1   |     17
      5    |    left   | foot  |     2     |     0   |     18
      6    |   right   |  hip  |     2     |     6   |     13
      6    |   right   | knee  |     2     |     5   |     14
      6    |   right   | foot  |     2     |     4   |     15
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

// variables to track states of all servos

// calibration values for servo pule length counts (out of 4096); [board1 board2]
int servoMin[] = {644, 697};
int servoMax[] = {2633,2741};

// X-Axis angles, calculated via inverse kinematic model, index corresponds with leg number-1
int alphaX[] = {0,0,0,0,0,0};
int betaX[] =  {0,0,0,0,0,0};
// Y-Axis angles, calculated via inverse kinematic model, index corresponds with leg number-1
int alphaY[] = {0,0,0,0,0,0};
// Z-Axis angles, calculated via inverse kinematic model, index corresponds with leg number-1
int alphaZ[] = {0,0,0,0,0,0};

// model parameters to convert inverse kinematic model angles into the actual servo angles
// index:     1  2   3  4  5   6       7  8   9     10 11  12 13 14  15 16 17  18
float m[] = {-1, 1, -1, 1, 1, -1,-0.9444, 1, -1,0.9722, 1, -1,-1, 1, -1, 1, 1, -1};
float b[] = {90,95,165,75,95,160,     65,90,160,   100,85,165,90,90,145,90,90,160};

// constants of the physical body
float offsetX = 38.2;                                     // offset of the X-axis [mm]
float legLength = 74.5;                                   // length of the middle part of each leg [mm]
float footLength = 85.15;                                 // length of the foot of each leg [mm]

// control loop variables
float cycleTime = 0.01;                                   // time between individual cycles of movements [s]
float homingDelay = 500;                                  // delay between individual homing of servos at startup [ms]
long previousTime = 0;                                    // timestamp of previous cycle
float PT1_K = 1;                                          // proportional component of simulated PT1
float PT1_T = 0.15;                                       // time component of simulated PT1 [s]
float c1 = 0;                                             // c1 component of DZG of discrete PT1
float d0 = 0;                                             // d0 component of DZG of discrete PT1
float d1 = 0;                                             // d1 component of DZG of discrete PT1

// current target values of all servos
//                 1,2, 3, 4, 5 6, 7,8, 9, 10,11,12,13,14,15,16,17,18
float targets[] = {0,45,90,0,45,90,0,45,90, 0,45,90, 0,45,90, 0,45,90};
// previous target values of all servos
float targetsOld[] = {targets[0],targets[1],targets[2],targets[3],targets[4],targets[5],targets[6],
                      targets[7],targets[8],targets[9],targets[10],targets[11],targets[12],targets[13],
                      targets[14],targets[15],targets[16],targets[17]};
// current values of all servos
float currents[] = {targets[0],targets[1],targets[2],targets[3],targets[4],targets[5],targets[6],
                    targets[7],targets[8],targets[9],targets[10],targets[11],targets[12],targets[13],
                    targets[14],targets[15],targets[16],targets[17]};
// previous values of all servos
float currentsOld[] = {currents[0],currents[1],currents[2],currents[3],currents[4],currents[5],currents[6],
                       currents[7],currents[8],currents[9],currents[10],currents[11],currents[12],currents[13],
                       currents[14],currents[15],currents[16],currents[17]};

// method to convert a given angle into a number in 4096 for PWM board 1
int pulseWidth(int angle, int boardNumber){
  int pulse_wide, analog_value;
  pulse_wide = map(angle, 0, 180, servoMin[boardNumber], servoMax[boardNumber]);
  analog_value = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096);
  return analog_value;
}

// method to home all servos based on the initial target values
void homeServos() {
  Serial.println("starting homing");
  setServoByNumber(2,targets[2-1]);                            // home knee joint front right
  setServoByNumber(3,targets[3-1]);                            // home foot joint front right
  delay(homingDelay);
  setServoByNumber(5,targets[5-1]);                            // home knee joint front left
  setServoByNumber(6,targets[6-1]);                            // home foot joint front left
  delay(homingDelay);
  setServoByNumber(8,targets[8-1]);                            // home knee joint middle right
  setServoByNumber(9,targets[9-1]);                            // home foot joint middle right
  delay(homingDelay);
  setServoByNumber(11,targets[11-1]);                          // home knee joint middle left
  setServoByNumber(12,targets[12-1]);                          // home foot joint middle left
  delay(homingDelay);
  setServoByNumber(14,targets[14-1]);                          // home knee joint back right
  setServoByNumber(15,targets[15-1]);                          // home foot joint back right
  delay(homingDelay);
  setServoByNumber(17,targets[17-1]);                          // home knee joint back left
  setServoByNumber(18,targets[18-1]);                          // home foot joint back left
  delay(homingDelay);
  setServoByNumber(1,targets[1-1]);                            // home hip joint front right
  delay(homingDelay);
  setServoByNumber(4,targets[4-1]);                            // home hip joint front left
  delay(homingDelay);
  setServoByNumber(7,targets[7-1]);                            // home hip joint middle right
  delay(homingDelay);
  setServoByNumber(10,targets[10-1]);                          // home hip joint middle left
  delay(homingDelay);
  setServoByNumber(13,targets[13-1]);                          // home hip joint back right
  delay(homingDelay);
  setServoByNumber(16,targets[16-1]);                          // home hip joint back left
  delay(homingDelay);
}

// method to set a servo by its number to a new targetPosition
void setServoByNumber(int servoNumber, float targetPosition) {
  switch(servoNumber) {                                   // go over all possible servo numbers
    case 1:
      pwm1.setPWM(9,0,pulseWidth(modelConversionIKM(servoNumber,targetPosition),0));// move servo 1 in position
      break;
    case 2:
      pwm1.setPWM(10,0,pulseWidth(modelConversionIKM(servoNumber,targetPosition),0));// move servo 2 in position
      break;
    case 3:
      pwm1.setPWM(11,0,pulseWidth(modelConversionIKM(servoNumber,targetPosition),0));// move servo 3 in position
      break;
    case 4:
      pwm1.setPWM(14,0,pulseWidth(modelConversionIKM(servoNumber,targetPosition),0));// move servo 4 in position
      break;
    case 5:
      pwm1.setPWM(13,0,pulseWidth(modelConversionIKM(servoNumber,targetPosition),0));// move servo 5 in position
      break;
    case 6:
      pwm1.setPWM(15,0,pulseWidth(modelConversionIKM(servoNumber,targetPosition),0));// move servo 6 in position
      break;
    case 7:
      pwm2.setPWM(14,0,pulseWidth(modelConversionIKM(servoNumber,targetPosition),1));// move servo 7 in position
      break;
    case 8:
      pwm2.setPWM(13,0,pulseWidth(modelConversionIKM(servoNumber,targetPosition),1));// move servo 8 in position
      break;
    case 9:
      pwm2.setPWM(15,0,pulseWidth(modelConversionIKM(servoNumber,targetPosition),1));// move servo 9 in position
      break;
    case 10:
      pwm1.setPWM(2,0,pulseWidth(modelConversionIKM(servoNumber,targetPosition),0));// move servo 10 in position
      break;
    case 11:
      pwm1.setPWM(1,0,pulseWidth(modelConversionIKM(servoNumber,targetPosition),0));// move servo 11 in position
      break;
    case 12:
      pwm1.setPWM(0,0,pulseWidth(modelConversionIKM(servoNumber,targetPosition),0));// move servo 12 in position
      break;
    case 13:
      pwm2.setPWM(6,0,pulseWidth(modelConversionIKM(servoNumber,targetPosition),1));// move servo 13 in position
      break;
    case 14:
      pwm2.setPWM(5,0,pulseWidth(modelConversionIKM(servoNumber,targetPosition),1));// move servo 14 in position
      break;
    case 15:
      pwm2.setPWM(4,0,pulseWidth(modelConversionIKM(servoNumber,targetPosition),1));// move servo 15 in position
      break;
    case 16:
      pwm2.setPWM(2,0,pulseWidth(modelConversionIKM(servoNumber,targetPosition),1));// move servo 16 in position
      break;
    case 17:
      pwm2.setPWM(1,0,pulseWidth(modelConversionIKM(servoNumber,targetPosition),1));// move servo 17 in position
      break;
    case 18:
      pwm2.setPWM(0,0,pulseWidth(modelConversionIKM(servoNumber,targetPosition),1));// move servo 18 in position
      break;
  }
  currentsOld[servoNumber-1] = currents[servoNumber-1];   // save new old position
  currents[servoNumber-1] = targetPosition;             // save new current position
}

// method to implement the discrete model for the servo
float servoModel(float previousPosition, float previousTarget, float currentTarget) {
  return (d0*currentTarget)+(d1*previousTarget)-(c1*previousPosition);
}

// method to convert the angle calculated by the inverse kinematic model into acutal usable angles
float modelConversionIKM(int servoNumber, float angle) {
  return (angle*m[servoNumber-1] + b[servoNumber-1]);
}

// method to calculate required values for servo model based on given constants
void initializeServoModel() {
  c1 = (cycleTime - 2*PT1_T)/(cycleTime + 2*PT1_T);       // calculate c1 component of discrete PT1
  d0 = (PT1_K * cycleTime)/(2*PT1_T + cycleTime);         // calculate d0 component of discrete PT1
  d1 = d0;                                                // calculate d1 component of discrete PT1
  //print out coefficients for testing
  Serial.print("c1: ");
  Serial.print(c1,5);
  Serial.print(" d0: ");
  Serial.print(d0,5);
  Serial.print(" d1: ");
  Serial.println(d1,5);
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

//method controlling the 'main' program. This is what should be done inside the cycles created from the loop method
void runProgram() {
  // check if new inputs are given, handle those accordingly
  if(Serial.available()) {  // if Serial input was given
    String input = Serial.readString();   // get input

    Serial.println(input);
    // get new coordinates from Serial input
    int newX = getValue(input, ' ', 0).toInt();
    int newY = getValue(input, ' ', 1).toInt();
    int newZ = getValue(input, ' ', 2).toInt();
    Serial.println("input: X: "+String(newX)+" Y: "+String(newY)+" Z: "+String(newZ));

    // calculate new Y-axis angles for all legs
    double newXFromY = sqrt(sq(newX) + sq(newY));
    double newAlphaY = asin(double(newY) / double(newXFromY))*180/PI;

    // calculate new Z-axis angles for all legs
    double newXFromZ = sqrt(sq(newXFromY) + sq(newZ));
    double newAlphaZ = asin(double(newZ) / double(newXFromZ))*180/PI;
      
    // calculate new X-axis angles for all legs
    double newAlphaX = acos(double(sq(legLength) + sq(footLength) - sq(newXFromZ-offsetX))/double(2*legLength*footLength))*180/PI;
    double newBetaX = acos(double(sq(newXFromZ-offsetX) + sq(legLength) - sq(footLength))/double(2*(newXFromZ-offsetX)*legLength))*180/PI;
    Serial.println("alphaX: "+String(newAlphaX)+" betaX: "+String(newBetaX));
    Serial.println("alphaY: "+String(newAlphaY));
    Serial.println("alphaZ: "+String(newAlphaZ));

    // save new angles for all legs
    for(int i=0;i<6;i++) {
      alphaX[i] = newAlphaX;
      betaX[i] = newBetaX;
      alphaY[i] = newAlphaY;
      alphaZ[i] = newAlphaZ;
    }

    // set new targets
    // front left
    targets[4-1] = alphaY[0];                                   // set new target angle 4
    targets[5-1] = betaX[0]+alphaZ[0];                          // set new target angle 5
    targets[6-1] = alphaX[0];                                   // set new target angle 6
    // front right
    targets[1-1] = alphaY[1];                                   // set new target angle 1
    targets[2-1] = betaX[1]+alphaZ[1];                          // set new target angle 2
    targets[3-1] = alphaX[1];                                   // set new target angle 3
    // middle left
    targets[10-1] = alphaY[2];                                  // set new target angle 10
    targets[11-1] = betaX[2]+alphaZ[2];                         // set new target angle 11
    targets[12-1] = alphaX[2];                                  // set new target angle 12
    // middle right
    targets[7-1] = alphaY[3];                                   // set new target angle 7
    targets[8-1] = betaX[3]+alphaZ[3];                          // set new target angle 8
    targets[9-1] = alphaX[3];                                   // set new target angle 9
    // back left
    targets[13-1] = alphaY[5];                                  // set new target angle 13
    targets[14-1] = betaX[5]+alphaZ[5];                         // set new target angle 14
    targets[15-1] = alphaX[5];                                  // set new target angle 15
    // back right
    targets[16-1] = alphaY[4];                                  // set new target angle 16
    targets[17-1] = betaX[4]+alphaZ[4];                         // set new target angle 17
    targets[18-1] = alphaX[4];                                  // set new target angle 18
  }  
  
  // check if cycle time for servo models is reached
  if(millis()-previousTime >= cycleTime*1000) {
    // update all servo models
    for(int i=1;i<19;i++){
      setServoByNumber(i,servoModel(currentsOld[i-1], targetsOld[i-1], targets[i-1])); // set new servo value 1
    }
    previousTime = millis();
  }
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
  Serial.println("change the coordinates with this command: [x y z] -> '100 100 100'");
}

void loop() {
    runProgram();                                         // call the cycle function to run the program
}
