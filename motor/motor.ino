#include <IRremote.h>
#include <IRremoteInt.h>

// To support more than 5 receivers, remember to change the define
// IR_PARAMS_MAX in IRremoteInt.h as well.
#define IRamount 0
IRrecv *irrecvs[IRamount];
decode_results results;

const int Serial1Baud = 9600; //UART port speed
const int SerialBaud = 19200; //UART port speed
unsigned long loopStartTime;
int timeLabelCurrent = 0;
const float voltageDividerCoeff = 4.02;
const float minBatteryValue = 10.5; //volt
const float lowBatteryValue = 11; //volt
const float blowingStartBatteryValue = 10.5; //volt
const float blowingStopBatteryValue = 10.0; //volt
const float maxBatteryValue = 13; //volt
const float okBatteryValue = 12.4; //volt Used for stuck calculation
const float arefCoeff = 0.919;
float speedChosen;

int stepDurationReal;
const int stepDurationOrder = 200; // ms for step
const int timeBeforeBlowing = 1000;
const int timeBackwardMoving = 600;
const int timeRotatingMoving = 500;

float voltageAverage = 0;
float rightSonarDistance = 0;
float leftSonarDistance = 0;
float currentBatteryVoltage = 0;
bool wasCollision = 0;

const int encoderSmoothingTime = 1700;
const int encoderSmoothingAmount = (int)encoderSmoothingTime/stepDurationOrder;
int encoderLeftArray[encoderSmoothingAmount + 1];
int encoderRightArray[encoderSmoothingAmount + 1];

const int voltageSmoothingTime = 3400;
const int voltageSmoothingAmount = (int)voltageSmoothingTime/stepDurationOrder;
float voltageArray[voltageSmoothingAmount + 1];    // the readings from the analog input

float motorCurrentAverage = 0;
const int motorCurrentSmoothingTime = 7;
const int motorCurrentSmoothingAmount = (int)motorCurrentSmoothingTime/stepDurationOrder;
float motorCurrentArray[motorCurrentSmoothingAmount + 1];    // the readings from the analog input


//Sonar properties
//int sonarPins[] = {1, 2};//Analog Pin Nums to sonar sensor Pin AN
//float samples[sizeof(sonarPins)][SamplesAmount];
//int sampleIndex[sizeof(sonarPins)];

//right side
const int pinRightMotorDirection = 4; //this can be marked on motor shield as "DIR A"
const int pinRightMotorSpeed = 5; //this can be marked on motor shield as "PWM A"
const int pinRightBumper = 15; //where the right bumper is connected

//left side
const int pinLeftMotorDirection = 7; //this can be marked on motor shield as "DIR B"
const int pinLeftMotorSpeed = 6; //this can be marked on motor shield as "PWM B"
const int pinLeftBumper = 16; //where the right bumper is connected

//sonar left
const int pinLeftSonarEcho = 42;
const int pinLeftSonarTrig = 40;

//sonar right
const int pinRightSonarEcho = 52;
const int pinRightSonarTrig = 50;
const int distanceInCollision = 10; //cm

const int pinLed = 13;
const int pinRX0 = 19;
const int pinVoltage = 15;
const int pinMotorCurrent = 9;
const int pinMotorOrder = 17;
const int pinOwnPowerSwitch = 22;
const int pinBlower = 14;

const int pinBuzzer = 24;

const int pinIR1 = 10;
const int pinIR2 = 11;
const int pinIR3 = 12;
const int pinIR4 = 13;
const int pinIR5 = 14;

// Board	int.0	int.1	int.2	int.3	int.4	int.5
// Mega2560	2		3		21		20		19		18
const int interruptNumberLeft = 3;
const int pinEncoderLeftA = 20; //corresponds the interrupt number!
const int pinEncoderLeftB = 30;

const int interruptNumberRight = 2;
const int pinEncoderRightA = 21; //corresponds the interrupt number!
const int pinEncoderRightB = 31;

long int encoderLeftPosition_prev = 0;
volatile long int encoderLeftPosition = 0;
volatile unsigned long monitorTimeLeft = 0;

long int encoderRightPosition_prev = 0;
volatile long int encoderRightPosition = 0;
volatile unsigned long monitorTimeRight = 0;
bool motorPower = 0;

const int turnRightTimeout = 400;
const int turnLeftTimeout = 300;

int stuckState = 0;

#define FullSpeed 255
#define HalfSpeed 200
#define QuarterSpeed 170
#define MinSpeed 140

#define rotateLeft -1
#define stright 0
#define rotateRight 1

#define None 0
#define Joystick 1
#define Auto 2
#define Spiral 3
#define Wall 4
#define LowBattery 5
int mode = Spiral;

//enum modes { None, Joystick, Auto, Off, LowBattery};
//modes mode=None;
void initPins() {
  pinMode(pinRightMotorDirection, OUTPUT);
  pinMode(pinRightMotorSpeed, OUTPUT);
  pinMode(pinRightBumper, INPUT);
  digitalWrite(pinRightBumper, HIGH);

  pinMode(pinLeftMotorDirection, OUTPUT);
  pinMode(pinLeftMotorSpeed, OUTPUT);
  pinMode(pinLeftBumper, INPUT);
  digitalWrite(pinLeftBumper, HIGH);

  pinMode(pinLeftSonarEcho, INPUT);
  pinMode(pinLeftSonarTrig, OUTPUT);
  pinMode(pinRightSonarEcho, INPUT);
  pinMode(pinRightSonarTrig, OUTPUT);
  pinMode(pinLed, OUTPUT);

  pinMode(pinRX0, INPUT);
  digitalWrite(pinRX0, HIGH);

  pinMode(pinOwnPowerSwitch, OUTPUT);
  pinMode(pinBlower, OUTPUT);
  digitalWrite(pinBlower, HIGH);

  pinMode(pinVoltage, INPUT);
  digitalWrite(pinVoltage, HIGH);

  pinMode(pinMotorCurrent, INPUT);
  digitalWrite(pinMotorCurrent, LOW);

  pinMode(pinMotorOrder, INPUT);
  digitalWrite(pinMotorOrder, LOW);

  pinMode(pinEncoderLeftA, INPUT);
  digitalWrite(pinEncoderLeftA, HIGH);
  pinMode(pinEncoderLeftB, INPUT);
  digitalWrite(pinEncoderLeftB, HIGH);

  pinMode(pinEncoderRightA, INPUT);
  digitalWrite(pinEncoderRightA, HIGH);
  pinMode(pinEncoderRightB, INPUT);
  digitalWrite(pinEncoderRightB, HIGH);

}
//Initialization
void setup() {
  Serial.begin(SerialBaud);
  Serial1.begin(Serial1Baud);
  //analogReference(EXTERNAL);

  initPins();
  lockPower();

  attachInterrupt (interruptNumberLeft, encoderMonitorLeft, CHANGE);
  attachInterrupt (interruptNumberRight, encoderMonitorRight, CHANGE);

  for (int index = 1; index <= encoderSmoothingAmount; index++) {
    encoderLeftArray[index] = 10;
    encoderRightArray[index] = 10;
  }

  for (int index = 1; index <= voltageSmoothingAmount; index++)
    voltageArray[index] = okBatteryValue;

  for (int index = 1; index <= motorCurrentSmoothingAmount; index++)
    motorCurrentArray[index] = 0;

  irrecvs[0] = new IRrecv(pinIR1); // Receiver #0: pin 2
  irrecvs[1] = new IRrecv(pinIR2); // Receiver #1: pin 3
  // irrecvs[2] = new IRrecv(pinIR3); // Receiver #2: pin 4
  // irrecvs[3] = new IRrecv(pinIR4); // Receiver #3: pin 5
  // irrecvs[4] = new IRrecv(pinIR5); // Receiver #4: pin 6

  for (int i = 0; i < IRamount; i++)
    irrecvs[i]->enableIRIn();

  //  playRandomSong();//uncomment to play a music
  getStartSound();
  delay(1000);
}

//Main loop
void loop() {
  loopStartTime = millis();
  timeLabelCurrent = loopStartTime;
  //printTimeLabel(1);

  currentBatteryVoltage = getBatteryVoltage();
  checkBattery(currentBatteryVoltage);
  setMode();
  speedChosen = getSpeed();

  BluetoothCheck();

  if (mode == Auto || mode == Spiral) {
    setBlowerState(currentBatteryVoltage);
    checkForStuck();
    if (!stuckState) {
      processMovement();
    }
  }

  readIR();

  forceMovementHandler();
  if (stuckState) incrementAtTheEnd(stuckState);


  Serial.println(" ");
  encoderLeftPosition_prev = encoderLeftPosition;
  encoderRightPosition_prev = encoderRightPosition;

  stepDurationReal = millis() - loopStartTime;
  Serial.print("Real loop duration: ");
  Serial.println(stepDurationReal);

  if (stepDurationReal < stepDurationOrder) {
    delay(stepDurationOrder - stepDurationReal);
  }
  else {
    Serial.print("Warning! Step duration is greater than required.");
  }
}

void setMode () {
  if (mode == Spiral && wasCollision) {
    mode = Auto;
  }
  return;
}


void readIR() {
  for (int i = 0; i < IRamount; i++)
  {
    if (irrecvs[i]->decode(&results))
    {
		if(results.value == 0xE0E0E01F || results.value == 0x8F7708F ) {
		// if(1 ) {
			tone(pinBuzzer, 1900, 10);
		  Serial.print("Receiver #");
		  Serial.print(i);
		  Serial.print(":");
		  Serial.println(results.value, HEX);
		  //Serial.println(results.value, DEC);
		}
      //IRinfo(&results);
      irrecvs[i]->resume();
    }
  }
  return;
}


void IRinfo(decode_results *results) {
  // Dumps out the decode_results structure.
  // Call this after IRrecv::decode()
  int count = results->rawlen;
  if (results->decode_type == UNKNOWN) {
    Serial.print("Unknown encoding: ");
  }
  else if (results->decode_type == NEC) {
    Serial.print("Decoded NEC: ");
  }
  else if (results->decode_type == SONY) {
    Serial.print("Decoded SONY: ");
  }
  else if (results->decode_type == RC5) {
    Serial.print("Decoded RC5: ");
  }
  else if (results->decode_type == RC6) {
    Serial.print("Decoded RC6: ");
  }
  else if (results->decode_type == PANASONIC) {
    Serial.print("Decoded PANASONIC - Address: ");
    Serial.print(results->address, HEX);
    Serial.print(" Value: ");
  }
  else if (results->decode_type == LG) {
    Serial.print("Decoded LG: ");
  }
  else if (results->decode_type == JVC) {
    Serial.print("Decoded JVC: ");
  }
  else if (results->decode_type == AIWA_RC_T501) {
    Serial.print("Decoded AIWA RC T501: ");
  }
  else if (results->decode_type == WHYNTER) {
    Serial.print("Decoded Whynter: ");
  }
  Serial.print(results->value, HEX);
  Serial.print(" (");
  Serial.print(results->bits, DEC);
  Serial.println(" bits)");
  Serial.print("Raw (");
  Serial.print(count, DEC);
  Serial.print("): ");

  Serial.println();
}


void encoderMonitorLeft () {
  bool isDecreasing;
  bool valueB;
  if ( millis() - monitorTimeLeft >= 1 ) {
    valueB = digitalRead (pinEncoderLeftB);

    if (digitalRead (pinEncoderLeftA)) {
      isDecreasing = valueB;
    }
    else {
      isDecreasing = !valueB;
    }
    if (isDecreasing)
      encoderLeftPosition--;
    else
      encoderLeftPosition++;
  }
  monitorTimeLeft = millis();
  return;
}


void encoderMonitorRight () {
  bool isDecreasing;
  bool valueB;
  if ( millis() - monitorTimeRight >= 1 ) {
    valueB = digitalRead (pinEncoderRightB);

    if (digitalRead (pinEncoderRightA)) {
      isDecreasing = valueB;
    }
    else {
      isDecreasing = !valueB;
    }
    if (isDecreasing)
      encoderRightPosition--;
    else
      encoderRightPosition++;
  }
  monitorTimeRight = millis();
  return;
}


void checkForStuck() {
  if ( encodersStuck(speedChosen) || stuckState ) {
    if (!stuckState) {
      stuckState = 1;
    }
    Serial.print("stuckState: ");
    Serial.println(stuckState);

    switch ( stuckState ) {
      case 1:
        startForceMovement(0, -1, timeBackwardMoving); break;	//backward
      case 2:
        startForceMovement(-1, 1, timeRotatingMoving); break;	//left
      default:
        Serial.println("Error! StuckState is outside the range."); break;
    }

    if ( stuckState == 3 ) {
      if ( !encodersStuck(speedChosen)) {
        Serial.println("Released. Continue the movement.");
        stuckState = 0;
        return;
      }
    }

    switch ( stuckState ) {
      case 3:
        startForceMovement(0, 1, 600); break;	//forward
      case 4:
        startForceMovement(-1, 1, timeRotatingMoving); break;	//left
      case 5:
        startForceMovement(0, -1, timeBackwardMoving); break;	//backward
      case 6:
        startForceMovement(-1, 1, timeRotatingMoving); break;	//left
      default:
        Serial.println("Error! StuckState is outside the range."); break;
    }

    if ( stuckState == 7 ) {
      if ( !encodersStuck(speedChosen)) {
        Serial.println("Released. Continue the movement.");
        stuckState = 0;
      }
      else {
        Serial.println("Cannot release! Power off.");
        goToSleep();
      }
    }

  }
  return;
}

float encoderLeftAverage;
float encoderLeftSum;
float encoderRightAverage;
float encoderRightSum;

bool encodersStuck(float speedOrder) {
  motorPower = digitalRead(pinMotorOrder);
  if (speedOrder > 50 && motorPower) {
    encoderLeftSum = 0;
    encoderRightSum = 0;

    for (int index = 2; index <= encoderSmoothingAmount; index++) {
      encoderLeftArray[index - 1] = encoderLeftArray[index];
      encoderRightArray[index - 1] = encoderRightArray[index];
    }
    encoderLeftArray[encoderSmoothingAmount] = encoderLeftLastStep ();
    encoderRightArray[encoderSmoothingAmount] = encoderRightLastStep();

    for (int index = 1; index <= encoderSmoothingAmount; index++) {
      encoderLeftSum = encoderLeftSum + abs(encoderLeftArray[index]);
      encoderRightSum = encoderRightSum + abs(encoderRightArray[index]);
    }
    encoderLeftAverage = encoderLeftSum / (encoderSmoothingAmount + 1);
    encoderRightAverage = encoderRightSum / (encoderSmoothingAmount + 1);

    //Serial.print ("encoderLeftPosition = ");
    //Serial.print  (encoderLeftPosition);
    //Serial.print (",  encoderRightPosition = ");
    //Serial.println (encoderRightPosition);


    // Serial.print("encoderLeftAverage ");
    // Serial.print(encoderLeftAverage, 2);
    // Serial.print(",  encoderRightAverage ");
    // Serial.println(encoderLeftAverage, 2);
    if (encoderLeftAverage < 1 || encoderRightAverage < 1) {
      return 1;
    }
  }
  return 0;
}

//smoothingAverageAbs(array, somoothingAmount, 1500);
//float smoothingAverageAbs (float arr[], int arrayAmount, int time) {
//  float arraySum=0;
//  int requiredItems=min(arrayAmount, time/stepDurationOrder);

//  for (int index = arrayAmount-requiredItems+1; index <= arrayAmount; index++) {
//    arraySum = arraySum + abs(arr[index]);
//  }
//  return arraySum/(requiredItems+1);
//}

int encoderLeftLastStep () {
  return encoderLeftPosition_prev - encoderLeftPosition;
}

int encoderRightLastStep () {
  return encoderRightPosition_prev - encoderRightPosition;
}


unsigned long forceMoveStartTime = 0;
float courseOrderRelTime = 0;
float speedOrderRelTime = 0;
bool isForceMoveLastStep = 0;
int durationTillLastStep = 0;
int durationTillEnd = 0;

void forceMovementHandler() {
  isForceMoveLastStep = 0;

  // Serial.print("courseOrderRelTime=");
  // Serial.println(courseOrderRelTime);
  
  // Serial.print("speedOrderRelTime=");
  // Serial.println(speedOrderRelTime);
  
  if ( isForceMovementActive() ) {
    // All steps except last
    if (isInTheRange(millis(), forceMoveStartTime, forceMoveStartTime + durationTillLastStep)) {
      moveSmart(courseOrderRelTime, speedOrderRelTime);
      forceMovementLogging();
    }

    //last step only
    if (isInTheRange(millis(), forceMoveStartTime + durationTillLastStep, forceMoveStartTime + durationTillEnd)) {
      moveSmart(courseOrderRelTime, speedOrderRelTime);
      isForceMoveLastStep = 1;
      stopForceMovement();
      Serial.println("Last force movement step");
    }

    //check for small step
    if (durationTillLastStep < 0) {
      moveSmart(courseOrderRelTime, speedOrderRelTime);
      isForceMoveLastStep = 1;
      stopForceMovement();

      Serial.println("Movement time less then real step!");
      tone(pinBuzzer, 1900, 10);
    }
  }
  return;
}

void stopForceMovement() {
  forceMoveStartTime = 0;
  return;
}

void forceMovementLogging () {
  Serial.print(millis() - forceMoveStartTime);
  Serial.print("/");
  Serial.print(durationTillEnd);
  Serial.println(". Continue movement..");
  return;
}

//starts if force movement is not active!
void startForceMovement(float courseOrderRel, float speedOrderRel, int time) {
  if (forceMoveStartTime == 0) {
    forceMoveStartTime = millis();
    courseOrderRelTime = courseOrderRel;
    speedOrderRelTime = speedOrderRel;
    durationTillLastStep = time - 3*stepDurationOrder / 2;
    durationTillEnd = time - stepDurationOrder / 2;
  }
  return;
}


bool isForceMovementActive() {
  if (forceMoveStartTime != 0) return 1;
  return 0;
}


//incrementAtTheEnd(i);
void incrementAtTheEnd(int &i) {
  if (isForceMoveLastStep) {
    i++;
  }
  return;
}

void printTimeLabel(int label) {
  Serial.print("Time label ");
  Serial.print(label);
  Serial.print(". ");
  Serial.println(millis() - timeLabelCurrent);
  timeLabelCurrent = millis();
  return;
}

float  getBatteryVoltage() {
  float voltageSum = 0;
  float strangeKoeff = 1;
  int OutNow = analogRead(pinVoltage);

  if (OutNow == 0) {
    Serial.println("Battery pin is not connected");
    tone(pinBuzzer, 2700, 50);
    goToSleep();
  }
  if (OutNow == 1023) {
    Serial.println("Battery voltage is greater then allowed!");
    tone(pinBuzzer, 2700, 5);
    goToSleep();
  }
  float VoltageNow = OutNow * voltageDividerCoeff * 5 / 1023 * arefCoeff * strangeKoeff;

  for (int index = 2; index <= voltageSmoothingAmount; index++) {
    voltageArray[index - 1] = voltageArray[index];
  }
  voltageArray[voltageSmoothingAmount] = VoltageNow;
  for (int index = 1; index <= voltageSmoothingAmount; index++) {
    voltageSum = voltageSum + voltageArray[index];
  }
  voltageAverage = voltageSum / (voltageSmoothingAmount + 1);

  Serial.print("Battery voltage: ");
  Serial.println(voltageAverage, 2);

  return voltageAverage;
}


void setBlowerState(float currentVoltage) {
  if ( currentVoltage > blowingStartBatteryValue && !stuckState ) {
    StartBlowing();
  }
  if ( currentVoltage < blowingStopBatteryValue || stuckState ) {
    StopBlowing();
  }
  return;
}

void checkBattery(float currentVoltage) {
  if (currentVoltage < lowBatteryValue) {
    Serial.print("WARNING! Low battery voltage - ");
    Serial.println(currentVoltage);
    //StopBlowing();
    tone(pinBuzzer, 1700, 5);
  }
  if (currentVoltage < minBatteryValue) {
    Serial.print("ALARM! Minimal battery voltage - ");
    Serial.println(currentBatteryVoltage);
    tone(pinBuzzer, 2700, 50);
    goToSleep();
  }
  if (currentVoltage > maxBatteryValue) {
    Serial.print("ALARM! Maximal battery voltage - ");
    Serial.println(currentBatteryVoltage);
    tone(pinBuzzer, 2000, 500);
    goToSleep();
  }
  return;
}

void lockPower() {
  digitalWrite(pinOwnPowerSwitch, 255);
  Serial.println("Started");
  return;
}


void goToSleep() {
  StopBlowing();
  //tone(pinBuzzer, 1700, 3000);
  digitalWrite(pinOwnPowerSwitch, 0);
  delay(1000);
  digitalWrite(pinOwnPowerSwitch, 1);
  return;
}

unsigned long timeBlowingStart = 0;
void StartBlowing() {
  if (digitalRead(pinBlower)) {
    if (timeBlowingStart == 0) {
      timeBlowingStart = millis();
    }
    if (millis() - timeBlowingStart > timeBeforeBlowing) {
      digitalWrite(pinBlower, LOW);
      timeBlowingStart = 0;
    }
  }
  return;
}

void StopBlowing() {
  digitalWrite(pinBlower, HIGH);
  return;
}
void ChangeBlowing() {
  digitalWrite(pinBlower, !digitalRead(pinBlower));
  return;
}

void getStartSound() {
  const int DelaySound = 100;
  tone(pinBuzzer, 1915);  delay(DelaySound);
  tone(pinBuzzer, 1700);  delay(DelaySound);
  tone(pinBuzzer, 1519);  delay(DelaySound);
  tone(pinBuzzer, 1432);  delay(DelaySound);
  tone(pinBuzzer, 1275);  delay(DelaySound);
  tone(pinBuzzer, 1136);  delay(DelaySound);
  tone(pinBuzzer, 1014);  delay(DelaySound);
  noTone(pinBuzzer);
}

const float spiralStep = 0.005;
const float spiralDirectionLimit = 0.2;

float increment = spiralStep;
float spiralDirectionOrder = spiralDirectionLimit;

float getDirection() {
  if (mode == Spiral) {
    if (spiralDirectionOrder >= abs(spiralDirectionLimit)) {
      increment = -abs(spiralStep);
    }
    if (spiralDirectionOrder < -abs(spiralDirectionLimit)) {
      //increment=abs(spiralStep);
      mode = Auto;
    }
    spiralDirectionOrder = spiralDirectionOrder + increment;
    Serial.print("                          spiralDirectionOrder=");
    Serial.println(spiralDirectionOrder);
    return spiralDirectionOrder;
  }
  return stright;
}

int  getSpeed() {
  if (rightSonarDistance == 0) {
    Serial.println("Warning! Right sonar is not available!");
    tone(pinBuzzer, 900, 10);
  }
  if (leftSonarDistance == 0) {
    Serial.println("Warning! Left sonar is not available!");
    tone(pinBuzzer, 900, 10);
  }

  if (isInTheRange(rightSonarDistance, 1, 20) || isInTheRange(leftSonarDistance, 1, 20)) {
    return QuarterSpeed;
  }
  if (isInTheRange(rightSonarDistance, 20, 50) || isInTheRange(leftSonarDistance, 20, 50)) {
    return HalfSpeed;
  }
  //if rotating - full speed manually.
  // if sonarDistance < 1cm or > 50cm - FullSpeed
  return FullSpeed;
}

//isInTheRange(rightSonarDistance, 50, 9999)
boolean isInTheRange(float parameter, float minimum, float maximum) {
  if (parameter >= minimum && parameter <= maximum) {
    return true;
  }
  return false;
}

void processMovement() {
	
   // Serial.print("Right bumper: ");
   // Serial.println(isBumperPressed(pinRightBumper));
   // Serial.print("Left bumper: ");
   // Serial.println(isBumperPressed(pinLeftBumper));
	
   // Serial.print("Right sensor: ");
   // Serial.println(isRightSideCollision());
   // Serial.print("Left sensor: ");
   // Serial.println(isLeftSideCollision());
	
  if (isLeftSideCollision()) {
    startForceMovement(rotateRight, FullSpeed/255, turnRightTimeout);
    Serial.println("Right side backward");
    return;
  }

  if (isRightSideCollision()) {
    startForceMovement(rotateLeft, FullSpeed/255, turnLeftTimeout);
    Serial.println("Left side backward");
    return;
  }

  if (!isForceMovementActive()) {
    moveSmart(getDirection(), speedChosen / 255);
  }
  return;
}

bool isRightSideCollision () {
  rightSonarDistance = checkTheDistance (pinRightSonarTrig, pinRightSonarEcho, "Right");
  if (checkTheCollision (rightSonarDistance, "Right") || isBumperPressed(pinRightBumper)) {
    Serial.println("Right side collision.");
    wasCollision = 1;
    return true;
  }
  return false;
}

bool isLeftSideCollision () {
  leftSonarDistance = checkTheDistance (pinLeftSonarTrig, pinLeftSonarEcho, "Left");
  if (checkTheCollision (leftSonarDistance, "Left") || isBumperPressed(pinLeftBumper)) {
    Serial.println("Left side collision.");
    wasCollision = 1;
    return true;
  }
  return false;
}

float checkTheDistance (int pinSonarTrig, int pinSonarEcho, char descr[ ]) {
  int duration, cm;
  digitalWrite(pinSonarTrig, LOW);
  delayMicroseconds(2);
  digitalWrite(pinSonarTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(pinSonarTrig, LOW);
  int startTime;
  startTime = millis();
  duration = pulseIn(pinSonarEcho, HIGH, 50000);
  // Serial.print("Pulse duration: ");
  // Serial.print(descr);
  // Serial.println(duration);
  // int pulseDuration;
  // pulseDuration = millis() - startTime;
  cm = duration / 58;
  return cm;
}

bool checkTheCollision (float distance, char descr[ ]) {
  if (distance <= distanceInCollision  && distance > 0 )
  {
    return true;
  }
  return false;
}



String readSerial1 ()
{
  char inChar;
  inChar = ' ';
  String inputString = "";         // a string to hold incoming data
  //boolean stringComplete = false;  // whether the string is complete
  // reserve 200 bytes for the inputString:
  inputString.reserve(200);
  while (Serial1.available() > 0) {
    inChar = (char)Serial1.read();
    inputString = ""; //Remove to get a whole string
    inputString += inChar;
    //while ( strlen(inChar) != 0 ) {
    //  inputString = ""; //Remove to get a whole string
    //  inputString += inChar;
    //}
    //if (inChar != 0x00) {
    //stringComplete = true;
    //  return inputString;
    //}
  }
  return inputString;
}

bool BluetoothCheck ()
{
  String inputString = readSerial1 ();
  char input=inputString[0];
  if ( mode == Joystick )
  {
	switch ( input ) {
    case 'u':
	  moveSmart(0, 1); 
	  tone(pinBuzzer, 1700, 5); break;
	  
    case 'd':
	  moveSmart(0, -1); 
	  tone(pinBuzzer, 1700, 5); break;
	  
    case 'l':
	  moveSmart(-1, 1); 
	  tone(pinBuzzer, 1700, 5); break;
	  
    case 'r':
	  moveSmart(1, 1); 
	  tone(pinBuzzer, 1700, 5); break;
	    
    case '\0':
	  Serial.println("Stop");
	  stopMotors(); break;
    }
  }
  if ( input == 'm')
  {
    Serial.println("Manual mode");
    tone(pinBuzzer, 1700, 5);
    mode = Joystick;
    delay(1000);
    return true;
  }
  if ( input == 'a')
  {
    Serial.println("Auto mode");
    tone(pinBuzzer, 1700, 5);
    delay(1000);
    mode = Auto;
    return true;
  }

  if ( input == 'w')
  {
    Serial.println("Wall mode");
    tone(pinBuzzer, 1700, 5);
    delay(1000);
    mode = Wall;
    return true;
  }
  if ( input == 's')
  {
    Serial.println("Spiral mode");
    tone(pinBuzzer, 1700, 5);
    delay(1000);
    mode = Spiral;
    return true;
  }
  if ( input == 'x')
  {
    Serial.println("Stop all");
    tone(pinBuzzer, 1700, 5);
    delay(1000);
    mode = None;
    return true;
  }

  if ( input == 'b')
  {
    Serial.println("Blowing");
    tone(pinBuzzer, 1700, 5);
    ChangeBlowing();
    delay(1000);
    return true;
  }

  if ( input == 't') //turn
  {
    Serial.println("Spin");
    tone(pinBuzzer, 1700, 5);
    delay(1000);
    //mode = None;
    return true;
  }
  return false;
}

//moveSmart(0,1)- forward full speed
//moveSmart(0,-1)- backward full speed
//moveSmart(-1,1)- full left full speed
//moveSmart(0.5,1)- half right full speed

//courseOrderRel - [-1;1], speedOrderRel - [-1;1]
float courseOrderRel_prev;
float speedOrderRel_prev;
int leftVelocityPure_prev;
int rightVelocityPure_prev;

void moveSmart(float courseOrderRel, float speedOrderRel)
{
	
  float leftVelocityPureRel;
  float rightVelocityPureRel;
  int leftVelocityPure;
  int rightVelocityPure;

  float courseDeviationReal = 1;
  float courseDeviationOrder = 1;
  float courseCorrection = 1;
  float leftCorrection = 1;
  float rightCorrection = 1;

  
  
  rightVelocityPureRel = 1 - courseOrderRel; //[0;2]
  leftVelocityPureRel = 1 + courseOrderRel; //[0;2]

  if (rightVelocityPureRel > 1) {
    leftVelocityPureRel = leftVelocityPureRel - rightVelocityPureRel + 1; //[-1;1]
    rightVelocityPureRel = 1;
  }

  if (leftVelocityPureRel > 1) {
    rightVelocityPureRel = rightVelocityPureRel - leftVelocityPureRel + 1; //[-1;1]
    leftVelocityPureRel = 1;
  }
  
  
  // Serial.print("leftVelocityPureRel=");
  // Serial.println(leftVelocityPureRel);
  
  // Serial.print("rightVelocityPureRel=");
  // Serial.println(rightVelocityPureRel);
  
  //rising speed for low orders. 
  leftVelocityPureRel=smoothLow(leftVelocityPureRel);
  rightVelocityPureRel=smoothLow(rightVelocityPureRel);
  
  leftVelocityPure = speedOrderRel * leftVelocityPureRel * 255; //[-255;255]
  rightVelocityPure = speedOrderRel * rightVelocityPureRel * 255; //[-255;255]

  // Serial.print("              Velocity = (");
  // Serial.print(leftVelocityPure);
  // Serial.print(" , ");
  // Serial.print(rightVelocityPure);
  // Serial.println(")");

  //correct the direction
  if (courseOrderRel_prev == courseOrderRel && speedOrderRel_prev == speedOrderRel) {
    float stepLeft = encoderLeftLastStep();
    float stepRight = encoderRightLastStep();

    if (stepLeft != 0 && stepRight != 0) {
      courseDeviationReal = abs(stepLeft / stepRight); //(0;Inf)
      courseDeviationOrder = abs(leftVelocityPure_prev / rightVelocityPure_prev); //(0;Inf)
      courseCorrection = courseDeviationOrder / courseDeviationReal; //(0;Inf)

      Serial.print("courseDeviationReal= ");
      Serial.print(courseDeviationReal);
      Serial.print(", courseDeviationOrder= ");
      Serial.print(courseDeviationOrder);
      Serial.print(", courseCorrection= ");
      Serial.println(courseCorrection);
      leftCorrection = min(courseCorrection, 255 / leftVelocityPure); // v - [0; 255]
      rightCorrection = min(1 / courseCorrection, 255 / rightVelocityPure); // v - [0; 255]
      leftCorrection = max(leftCorrection, 0.6); // v - [0.6U; 255]
      rightCorrection = max(rightCorrection, 0.6); // v - [0.6U; 255]
    }
  }

  Serial.print("              Velocity = (");
  Serial.print(leftVelocityPure*leftCorrection);
  Serial.print(" , ");
  Serial.print(rightVelocityPure*rightCorrection);
  Serial.println(")");

  setLeftMotorOrder(leftVelocityPure * leftCorrection);
  setRightMotorOrder(rightVelocityPure * rightCorrection);

  courseOrderRel_prev = courseOrderRel;
  speedOrderRel_prev = speedOrderRel;
  leftVelocityPure_prev = leftVelocityPure;
  rightVelocityPure_prev = rightVelocityPure;
  return;
}

float smoothLow(float velocity) {
  float speedOrder;
  if (velocity >=0) {
	  speedOrder=sqrt(2*velocity - velocity*velocity);
  } else {
	 speedOrder=-sqrt(2*abs(velocity)- abs(velocity)*abs(velocity)); 
  }
  return speedOrder;
}	

void stopMotors() {
  moveSmart(0, 0);
}


bool isBumperPressed(int pinBumper) {
  return !digitalRead(pinBumper);
}

////////////////////////////////////
void setLeftMotorOrder(int velocity) {
  setMotorOrder(pinLeftMotorDirection, pinLeftMotorSpeed, velocity);
}
void setRightMotorOrder(int velocity) {
  setMotorOrder(pinRightMotorDirection, pinRightMotorSpeed, velocity);
}
void setMotorOrder(int pinDirection, int pinSpeed, int velocity) {
  if (velocity >= 0) {
    digitalWrite(pinDirection, HIGH);
  }
  else {
    digitalWrite(pinDirection, LOW);
  }
  setMotorSpeed(pinSpeed, abs(velocity));
}

void setMotorSpeed(int pinMotorSpeed, int motorSpeed) {
  analogWrite(pinMotorSpeed, motorSpeed);
}
////////////////////////////////////

void runMotorForward(int pinMotorDirection) {
  digitalWrite(pinMotorDirection, HIGH); //set direction forward
}

void runMotorBackward(int pinMotorDirection) {
  digitalWrite(pinMotorDirection, LOW); //set direction backward
}



//float motorCurrentNow=0;
//const float motorCurrentOnWork=0.21;
//const float motorCurrentOnStuck=0.21;
//const float motorCurrentZeroValue = 0.5; //volt

//motorCurrentNow=getMotorCurrent();

//float  getMotorCurrent(){
//  int mVperAmp = 185; // use 100 for 20A Module and 66 for 30A Module
//  float ACSoffset = 2717.689;
//
//  float motorCurrentSum=0;
//  // Amps = ((Voltage - ACSoffset) / mVperAmp);
//
//  float valueRaw = analogRead(pinMotorCurrent);
//  if ( abs(valueRaw - 512) > 300) {
//    Serial.print("Motor current sensor error. Power off.");
//    Sleep();
//  }
//
//  //Serial.print("ASC712 pure value: ");
//  //Serial.print(valueRaw);
//
//  float value = valueRaw*5000/arefCoeff/1023;
//  //Serial.print("ASC712 value, mV: ");
//  //Serial.print(value, 3);
//
//  float valueCurrent = ((value - ACSoffset) / mVperAmp);
//  //Serial.print(", ASC712 current, mA: ");
//  //Serial.print(valueCurrent, 3);
//
//  //float temp;
//  //temp = speedChosen;
//  //Serial.print(", speedChosen: ");
//  //Serial.print(temp);
//
//  float currentNormFactor=max((140.0 / 255.0), (speedChosen/255.0));
//  float currentReferenceValue=currentNormFactor*currentBatteryVoltage/okBatteryValue;
//
//  float currentNormalized=valueCurrent/currentReferenceValue;
//  //float valueCurrentNorm = valueCurrent*(analogRead(pinRightMotorSpeed)+analogRead(pinLeftMotorSpeed))/1023/2;
//
//	//Serial.print(", currentNormFactor: ");
//	//Serial.print(currentNormFactor, 3);
//	//Serial.print(", currentReferenceValue: ");
//	//Serial.print(currentReferenceValue, 3);
//	//Serial.print(", currentRelative: ");
//	//Serial.println(currentNormalized, 3);
//
//
//  for (int index = 2; index <= motorCurrentSmoothingAmount; index++) {
//    motorCurrentArray[index-1] = motorCurrentArray[index];
//  }
//  motorCurrentArray[motorCurrentSmoothingAmount] = currentNormalized;
//
//  for (int index = 1; index <= motorCurrentSmoothingAmount; index++) {
//    motorCurrentSum = motorCurrentSum+motorCurrentArray[index];
//  }
//  motorCurrentAverage=motorCurrentSum/motorCurrentSmoothingAmount;
//  Serial.print("Motor average current: ");
//  Serial.println(motorCurrentAverage, 2);
//
//  return motorCurrentAverage;
//}



