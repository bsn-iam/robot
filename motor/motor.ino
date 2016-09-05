//#include <math.h>
//#include <music.ino>
const int Baud = 9600; //UART port speed
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
const int stepDurationOrder = 333; // ms for step
const int timeBeforeBlowing = 1000;
const int timeBackwardMoving = 400;
const int timeRotatingMoving = 500;

float voltageAverage = 0;
float rightSonarDistance = 0;
float leftSonarDistance = 0;
float currentBatteryVoltage = 0;
bool wasCollision=0;

const int encoderSmoothingAmount = 5;
int encoderLeftArray[encoderSmoothingAmount + 1];
int encoderRightArray[encoderSmoothingAmount + 1];

const int voltageSmoothingAmount = 10;
float voltageArray[voltageSmoothingAmount + 1];    // the readings from the analog input

float motorCurrentAverage = 0;
const int motorCurrentSmoothingAmount = 7;
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
const int pinLeftIR = 99;
//const int pinRightIR = 99;

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

const int turnRightTimeoutDefault = 400;
int turnRightTimeout = turnRightTimeoutDefault;
const int turnLeftTimeoutDefault = 300;
int turnLeftTimeout = turnLeftTimeoutDefault;

int stuckState = 0;
//int stuckedMovement=0;
int countDownWhileMovingToRight;
int countDownWhileMovingToLeft;

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
#define LowBattery 3
int mode = Spiral;


char *songs[]={
	"Indiana:d=4,o=5,b=250:e,8p,8f,8g,8p,1c6,8p.,d,8p,8e,1f,p.,g,8p,8a,8b,8p,1f6,p,a,8p,8b,2c6,2d6,2e6,e,8p,8f,8g,8p,1c6,p,d6,8p,8e6,1f.6,g,8p,8g,e.6,8p,d6,8p,8g,e.6,8p,d6,8p,8g,f.6,8p,e6,8p,8d6,2c6",
	"Entertainer:d=4,o=5,b=140:8d,8d#,8e,c6,8e,c6,8e,2c.6,8c6,8d6,8d#6,8e6,8c6,8d6,e6,8b,d6,2c6,p,8d,8d#,8e,c6,8e,c6,8e,2c.6,8p,8a,8g,8f#,8a,8c6,e6,8d6,8c6,8a,2d6",
	"Bond:d=4,o=5,b=80:32p,16c#6,32d#6,32d#6,16d#6,8d#6,16c#6,16c#6,16c#6,16c#6,32e6,32e6,16e6,8e6,16d#6,16d#6,16d#6,16c#6,32d#6,32d#6,16d#6,8d#6,16c#6,16c#6,16c#6,16c#6,32e6,32e6,16e6,8e6,16d#6,16d6,16c#6,16c#7,c.7,16g#6,16f#6,g#.6",
	"StarWars:d=4,o=5,b=45:32p,32f#,32f#,32f#,8b.,8f#.6,32e6,32d#6,32c#6,8b.6,16f#.6,32e6,32d#6,32c#6,8b.6,16f#.6,32e6,32d#6,32e6,8c#.6,32f#,32f#,32f#,8b.,8f#.6,32e6,32d#6,32c#6,8b.6,16f#.6,32e6,32d#6,32c#6,8b.6,16f#.6,32e6,32d#6,32e6,8c#6",
	"GoodBad:d=4,o=5,b=56:32p,32a#,32d#6,32a#,32d#6,8a#.,16f#.,16g#.,d#,32a#,32d#6,32a#,32d#6,8a#.,16f#.,16g#.,c#6,32a#,32d#6,32a#,32d#6,8a#.,16f#.,32f.,32d#.,c#,32a#,32d#6,32a#,32d#6,8a#.,16g#.,d#",
	"MissionImp:d=16,o=6,b=95:32d,32d#,32d,32d#,32d,32d#,32d,32d#,32d,32d,32d#,32e,32f,32f#,32g,g,8p,g,8p,a#,p,c7,p,g,8p,g,8p,f,p,f#,p,g,8p,g,8p,a#,p,c7,p,g,8p,g,8p,f,p,f#,p,a#,g,2d,32p,a#,g,2c#,32p,a#,g,2c,a#5,8c,2p,32p,a#5,g5,2f#,32p,a#5,g5,2f,32p,a#5,g5,2e,d#,8d",
	//"The Simpsons:d=4,o=5,b=160:c.6,e6,f#6,8a6,g.6,e6,c6,8a,8f#,8f#,8f#,2g,8p,8p,8f#,8f#,8f#,8g,a#.,8c6,8c6,8c6,c6",
	//"Looney:d=4,o=5,b=140:32p,c6,8f6,8e6,8d6,8c6,a.,8c6,8f6,8e6,8d6,8d#6,e.6,8e6,8e6,8c6,8d6,8c6,8e6,8c6,8d6,8a,8c6,8g,8a#,8a,8f",
	//"MASH:d=8,o=5,b=140:4a,4g,f#,g,p,f#,p,g,p,f#,p,2e.,p,f#,e,4f#,e,f#,p,e,p,4d.,p,f#,4e,d,e,p,d,p,e,p,d,p,2c#.,p,d,c#,4d,c#,d,p,e,p,4f#,p,a,p,4b,a,b,p,a,p,b,p,2a.,4p,a,b,a,4b,a,b,p,2a.,a,4f#,a,b,p,d6,p,4e.6,d6,b,p,a,p,2b",
	//"TopGun:d=4,o=4,b=31:32p,16c#,16g#,16g#,32f#,32f,32f#,32f,16d#,16d#,32c#,32d#,16f,32d#,32f,16f#,32f,32c#,16f,d#,16c#,16g#,16g#,32f#,32f,32f#,32f,16d#,16d#,32c#,32d#,16f,32d#,32f,16f#,32f,32c#,g#",
	//"A-Team:d=8,o=5,b=125:4d#6,a#,2d#6,16p,g#,4a#,4d#.,p,16g,16a#,d#6,a#,f6,2d#6,16p,c#.6,16c6,16a#,g#.,2a#",
	//"Flinstones:d=4,o=5,b=40:32p,16f6,16a#,16a#6,32g6,16f6,16a#.,16f6,32d#6,32d6,32d6,32d#6,32f6,16a#,16c6,d6,16f6,16a#.,16a#6,32g6,16f6,16a#.,32f6,32f6,32d#6,32d6,32d6,32d#6,32f6,16a#,16c6,a#,16a6,16d.6,16a#6,32a6,32a6,32g6,32f#6,32a6,8g6,16g6,16c.6,32a6,32a6,32g6,32g6,32f6,32e6,32g6,8f6,16f6,16a#.,16a#6,32g6,16f6,16a#.,16f6,32d#6,32d6,32d6,32d#6,32f6,16a#,16c.6,32d6,32d#6,32f6,16a#,16c.6,32d6,32d#6,32f6,16a#6,16c7,8a#.6",
	//"Jeopardy:d=4,o=6,b=125:c,f,c,f5,c,f,2c,c,f,c,f,a.,8g,8f,8e,8d,8c#,c,f,c,f5,c,f,2c,f.,8d,c,a#5,a5,g5,f5,p,d#,g#,d#,g#5,d#,g#,2d#,d#,g#,d#,g#,c.7,8a#,8g#,8g,8f,8e,d#,g#,d#,g#5,d#,g#,2d#,g#.,8f,d#,c#,c,p,a#5,p,g#.5,d#,g#",
	//"MahnaMahna:d=16,o=6,b=125:c#,c.,b5,8a#.5,8f.,4g#,a#,g.,4d#,8p,c#,c.,b5,8a#.5,8f.,g#.,8a#.,4g,8p,c#,c.,b5,8a#.5,8f.,4g#,f,g.,8d#.,f,g.,8d#.,f,8g,8d#.,f,8g,d#,8c,a#5,8d#.,8d#.,4d#,8d#.",
	//"KnightRider:d=4,o=5,b=125:16e,16p,16f,16e,16e,16p,16e,16e,16f,16e,16e,16e,16d#,16e,16e,16e,16e,16p,16f,16e,16e,16p,16f,16e,16f,16e,16e,16e,16d#,16e,16e,16e,16d,16p,16e,16d,16d,16p,16e,16d,16e,16d,16d,16d,16c,16d,16d,16d,16d,16p,16e,16d,16d,16p,16e,16d,16e,16d,16d,16d,16c,16d,16d,16d"
};

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
  pinMode(pinLeftIR, INPUT);
  //pinMode(pinRightIR, INPUT);

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
  Serial.begin(Baud);
  Serial1.begin(Baud);
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

  turnRightTimeout = turnRightTimeoutDefault / stepDurationOrder;
  if (turnRightTimeout == 0)  turnRightTimeout = 1;
  turnLeftTimeout = turnLeftTimeoutDefault / stepDurationOrder;
  if (turnLeftTimeout == 0) turnLeftTimeout = 1;

 randomSeed(analogRead(0));
 int songNumber=random (0,6);
 //play_rtttl(songs[songNumber]); //uncomment to play a music

	
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
  setBlowerState(currentBatteryVoltage);
  //printTimeLabel(2);

  Serial1ActionSelect ();

  if (mode == Auto || mode == Spiral) {
    checkForStuck();
    if (!stuckState) {
      processMovement();
    }
  }

  // checkIR();
	

  Serial.println(" ");
  encoderLeftPosition_prev = encoderLeftPosition;
  encoderRightPosition_prev = encoderRightPosition;

  stepDurationReal = millis() - loopStartTime;
  if (stepDurationReal < stepDurationOrder) {
    delay(stepDurationOrder - stepDurationReal);
  }
  else {
    Serial.print("Warning! Step duration is greater than required.");
  }
}

void setMode (){
	if (mode == Spiral && wasCollision){
		mode=Auto;
	}
return;	
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
	int ss;
	ss=0;
	
	ss++; backwardToRelease(ss); 
    ss++; rotateToRelease(ss);
	if ( stuckState == (ss+1) ) {
      if ( !encodersStuck(speedChosen)) {
        Serial.println("Released. Continue the movement.");
        stuckState = 0;
		return;
      } 
    }
	ss++; waitToCheckRelease(ss);
	ss++; rotateToRelease(ss);
	ss++; backwardToRelease(ss); 
	ss++; rotateToRelease(ss);

	if ( stuckState == (ss+1) ) {
	  if ( !encodersStuck(speedChosen)) {
		Serial.println("Released. Continue the movement.");
		stuckState = 0;
	  } else {
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

	Serial.print ("encoderLeftPosition = ");
	Serial.print  (encoderLeftPosition);
	Serial.print (",  encoderRightPosition = ");
	Serial.println (encoderRightPosition);

	
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

int encoderLeftLastStep () {
  return encoderLeftPosition_prev - encoderLeftPosition;
}

int encoderRightLastStep () {
  return encoderRightPosition_prev - encoderRightPosition;
}


int countSleep = 0;
int amountStepstoWait = 2;

void waitToCheckRelease(int releaseLevel) {
  if (stuckState == releaseLevel) {
    if (countSleep == 0) {
      countSleep = amountStepstoWait;
      processMovement();
      Serial.print(countSleep);
      Serial.println(". Attempt to release. Sleeping start.");
    }
    if (countSleep <= amountStepstoWait ) {
      countSleep--;
      processMovement();
      Serial.print(countSleep);
      Serial.println(". Attempt to release. Sleeping.");
    }
    if (countSleep <= 0) {
      countSleep = 0;
      stuckState = releaseLevel + 1;
      Serial.print(countSleep);
      Serial.println(". Attempt to release. Sleeping finished.");
    }
  }
  return;
}

unsigned long timeBackwardStart = 0;
void backwardToRelease(int releaseLevel) {
	Serial.println(releaseLevel);
  if (stuckState == releaseLevel) {
    tone(pinBuzzer, 1600, 20);
    if (timeBackwardStart == 0) {
      timeBackwardStart = millis();
      moveSmart(0, -HalfSpeed / 255);
      //moveBackward(HalfSpeed);
      Serial.println("Attempt to release. Backward.");
    }
    if (isInTheRange(timeBackwardMoving, 1, millis() - timeBackwardStart) ) {
      //moveBackward(FullSpeed);
      moveSmart(0, -1);
      Serial.println("Attempt to release. Backward.");
    }
    if (millis() - timeBackwardStart > timeBackwardMoving) {
      timeBackwardStart = 0;
      stuckState = releaseLevel + 1;
      Serial.println("Attempt to release. Backward finished.");
    }
  }
  return;
}

unsigned long timeRotatingStart = 0;
void rotateToRelease(int releaseLevel) {
  if (stuckState == releaseLevel) {
    tone(pinBuzzer, 1900, 20);
    if (timeRotatingStart == 0) {
      timeRotatingStart = millis();
      //moveRight(FullSpeed);
      moveSmart(rotateRight, FullSpeed / 255);
      Serial.println("Attempt to release. Rotating.");
    }
    if (isInTheRange(timeRotatingMoving, 1, millis() - timeRotatingStart) ) {
      //moveRight(HalfSpeed);
      moveSmart(rotateRight, HalfSpeed / 255);
      Serial.println("Attempt to release. Rotating.");
    }
    if (millis() - timeRotatingStart > timeRotatingMoving) {
      timeRotatingStart = 0;
      stuckState = releaseLevel + 1;
      Serial.println("Attempt to release. Rotating finished.");
    }
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

//void checkIR() {
//  int LeftIRResult=analogRead(pinLeftIR);
//  //int RightIRResult=digitalRead(pinRightIR);
//  Serial.print("Left IR: ");
//  Serial.println(LeftIRResult);
//  //Serial.print(", Right IR: ");
//  //Serial.println(RightIRResult);
//  if (!LeftIRResult){
//    tone(pinBuzzer, 2100, 5);
//  }
//  //if (!RightIRResult){
//  //  tone(pinBuzzer, 1500, 5);
//  //}
//  return;
//}


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
  tone(pinBuzzer, 1700, 3000);
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

void getStartSound() {
  const int DelaySound = 100;
  tone(pinBuzzer, 1915);
  delay(DelaySound);
  tone(pinBuzzer, 1700);
  delay(DelaySound);
  tone(pinBuzzer, 1519);
  delay(DelaySound);
  tone(pinBuzzer, 1432);
  delay(DelaySound);
  tone(pinBuzzer, 1275);
  delay(DelaySound);
  tone(pinBuzzer, 1136);
  delay(DelaySound);
  tone(pinBuzzer, 1014);
  delay(DelaySound);
  noTone(pinBuzzer);
}

const float spiralStep=0.005;
const float spiralDirectionLimit=0.2;

float increment=spiralStep;
float spiralDirectionOrder=spiralDirectionLimit;

float getDirection() {
	if (mode == Spiral) {
		if (spiralDirectionOrder >= abs(spiralDirectionLimit)){
			increment=-abs(spiralStep);
		}
		if (spiralDirectionOrder < -abs(spiralDirectionLimit)){
			//increment=abs(spiralStep);
			mode=Auto;
		}
		spiralDirectionOrder=spiralDirectionOrder+increment;
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
    //return HalfSpeed;
  }
  if (leftSonarDistance == 0) {
    Serial.println("Warning! Left sonar is not available!");
    tone(pinBuzzer, 900, 10);
    //return HalfSpeed;
  }

  if ( countDownWhileMovingToRight > 0 || countDownWhileMovingToLeft > 0 ) {
    return FullSpeed;
  }
  if (isInTheRange(rightSonarDistance, 1, 20) || isInTheRange(leftSonarDistance, 1, 20)) {
    return QuarterSpeed;
  }
  if (isInTheRange(rightSonarDistance, 20, 50) || isInTheRange(leftSonarDistance, 20, 50)) {
    return HalfSpeed;
  }
  if (isInTheRange(rightSonarDistance, 50, 9999) || isInTheRange(leftSonarDistance, 50, 9999)) {
    return FullSpeed;
  }
  return FullSpeed;
}

//isInTheRange(rightSonarDistance, 50, 9999)
boolean isInTheRange(float parameter, float minimum, float maximum) {
  if (parameter >= minimum && parameter <= maximum) {
    return true;
  }
  return false;
}

// start movement if wheel is not moving backward
void processMovement() {
  float speedChosenRel;
  if (isLeftSideCollision() && countDownWhileMovingToLeft <= 0 )
    countDownWhileMovingToRight = turnRightTimeout;
  if (isRightSideCollision() && countDownWhileMovingToRight <= 0 )
    countDownWhileMovingToLeft = turnLeftTimeout;

  speedChosen = getSpeed();
  // Serial.print("speedChosen=");
  // Serial.println(speedChosen);
  speedChosenRel = speedChosen / 255.0;
  if (countDownWhileMovingToRight <= 0 && countDownWhileMovingToLeft <= 0 ) {
	  float direction=getDirection();
    moveSmart(direction, speedChosenRel);
    //moveForward(speedChosen);
  }
  if (countDownWhileMovingToRight > 0 && countDownWhileMovingToLeft <= 0 ) {
    Serial.print(countDownWhileMovingToRight);
    Serial.println(". Right side backward");
    countDownWhileMovingToRight--;
    moveSmart(rotateRight, speedChosenRel);
    //moveRight(speedChosen);
  }

  if (countDownWhileMovingToLeft > 0 && countDownWhileMovingToRight <= 0 ) {
    Serial.print(countDownWhileMovingToLeft);
    Serial.println(". Left side backward");
    countDownWhileMovingToLeft--;
    moveSmart(rotateLeft, speedChosenRel);
    //moveLeft(speedChosen);
  }
}

boolean isRightSideCollision () {
  rightSonarDistance = checkTheDistance (pinRightSonarTrig, pinRightSonarEcho, "Right");
  if (checkTheCollision (rightSonarDistance, "Right") || isBumperPressed(pinRightBumper)) {
    Serial.println("Right side collision.");
	wasCollision=1;
    return true;
  }
  return false;
}
boolean isLeftSideCollision () {
  leftSonarDistance = checkTheDistance (pinLeftSonarTrig, pinLeftSonarEcho, "Left");
  if (checkTheCollision (leftSonarDistance, "Left") || isBumperPressed(pinLeftBumper)) {
    Serial.println("Left side collision.");
	wasCollision=1;
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
  int pulseDuration;
  pulseDuration = millis() - startTime;
  cm = duration / 58;
  return cm;
}

bool checkTheCollision (float distance, char descr[ ]) {
  if (distance <= distanceInCollision  && distance > 0 )
  {
    //Serial.print(descr);
    //Serial.print(" sonar: collision!   ");
    //Serial.print(distance);
    //Serial.println(" cm");

    return true;
  }

  //Serial.print(descr);
  //Serial.print(" sonar: distance: ");
  //Serial.print(distance);
  //Serial.println(" cm");
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

bool Serial1ActionSelect ()
{
  String input = readSerial1 ();
  if ( mode == Joystick )
  {
    if (input == "u")
    {
      moveSmart(0, 1);
      //moveForward(FullSpeed);
      tone(pinBuzzer, 1700, 5);
      return true;
    }
    if ( input == "d")
    {
      moveSmart(0, -HalfSpeed / 255);
      //moveBackward(HalfSpeed);
      tone(pinBuzzer, 1700, 5);
      return true;
    }
    if (input == "l")
    {
      moveSmart(-1, QuarterSpeed / 255);
      //moveLeft(QuarterSpeed);
      tone(pinBuzzer, 1700, 5);
      return true;
    }
    if ( input == "r")
    {
      moveSmart(1, QuarterSpeed / 255);
      //moveRight(QuarterSpeed);
      tone(pinBuzzer, 1700, 5);
      return true;
    }
  }
  if ( input == "m")
  {
    Serial.println("Manual mode");
    tone(pinBuzzer, 1700, 5);
    mode = Joystick;
    delay(1000);
    return true;
  }
  if ( input == "a")
  {
    Serial.println("Auto mode");
    tone(pinBuzzer, 1700, 5);
    delay(1000);
    mode = Auto;
    return true;
  }

  if ( input == "x")
  {
    Serial.println("Stop all");
    tone(pinBuzzer, 1700, 5);
    delay(1000);
    mode = None;
    return true;
  }

  if ( input == "")
  {
    if ( mode == Joystick )
    {
      Serial.println("Stop");
      stopMotors();
    }
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
  float courseDeviationOrder=1;
  float courseCorrection=1;
	float leftCorrection=1;
	float rightCorrection=1;

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
  leftVelocityPure = speedOrderRel * leftVelocityPureRel * 255; //[-255;255]
  rightVelocityPure = speedOrderRel * rightVelocityPureRel * 255; //[-255;255]

  // Serial.print("              Velocity = (");
  // Serial.print(leftVelocityPure);
  // Serial.print(" , ");
  // Serial.print(rightVelocityPure);
  // Serial.println(")");
  
  if (courseOrderRel_prev == courseOrderRel && speedOrderRel_prev == speedOrderRel) {
    float stepLeft = encoderLeftLastStep();
    float stepRight = encoderRightLastStep();

    if (stepLeft != 0 && stepRight != 0) {
      courseDeviationReal = abs(stepLeft / stepRight); //(0;Inf)
	  courseDeviationOrder= abs(leftVelocityPure_prev / rightVelocityPure_prev); //(0;Inf)
	  courseCorrection=courseDeviationOrder/courseDeviationReal; //(0;Inf)
	  
      Serial.print("courseDeviationReal= ");
      Serial.print(courseDeviationReal);
      Serial.print(", courseDeviationOrder= ");
      Serial.print(courseDeviationOrder);
      Serial.print(", courseCorrection= ");
      Serial.println(courseCorrection);
	  leftCorrection=min(courseCorrection, 255/leftVelocityPure); // v - [0; 255]
	  rightCorrection=min(1/courseCorrection, 255/rightVelocityPure); // v - [0; 255]
	  leftCorrection=max(leftCorrection, 0.7); // v - [0.7U; 255]
	  rightCorrection=max(rightCorrection, 0.7);// v - [0.7U; 255]
    }
  }

  // Serial.print("              Velocity = (");
  // Serial.print(leftVelocityPure*leftCorrection);
  // Serial.print(" , ");
  // Serial.print(rightVelocityPure*rightCorrection);
  // Serial.println(")");

  setLeftMotorOrder(leftVelocityPure*leftCorrection);
  setRightMotorOrder(rightVelocityPure*rightCorrection);
  
  courseOrderRel_prev=courseOrderRel;
  speedOrderRel_prev=speedOrderRel;
  leftVelocityPure_prev=leftVelocityPure;
  rightVelocityPure_prev=rightVelocityPure;
  return;
}


//void moveForward(int velocity)
//{
//  setMotorsSpeed(velocity);
//  runLeftMotorForward();
//  runRightMotorForward();
//}
//void moveBackward(int velocity)
//{
//  setMotorsSpeed(velocity);
//  runLeftMotorBackward();
//  runRightMotorBackward();
//}
//void moveRight(int velocity)
//{
//  setMotorsSpeed(velocity);
//  runLeftMotorForward();
//  runRightMotorBackward();
//}
//void moveLeft(int velocity)
//{
//  setMotorsSpeed(velocity);
//  runLeftMotorBackward();
//  runRightMotorForward();
//}

//void setMotorsSpeed(int velocity) {
//  setMotorSpeed(pinRightMotorSpeed, velocity);
//  setMotorSpeed(pinLeftMotorSpeed, velocity);
//}

void stopMotors() {
  moveSmart(0, 0);
  //setMotorSpeed(pinRightMotorSpeed, 0);
  //setMotorSpeed(pinLeftMotorSpeed, 0);
}

boolean isBumperPressed(int pinBumper) {
  return !digitalRead(pinBumper);
}

//void runRightMotorForward() {
//  runMotorForward(pinRightMotorDirection);
//}
//
//void runLeftMotorForward() {
//  runMotorForward(pinLeftMotorDirection);
//}
//
//void runRightMotorBackward() {
//  runMotorBackward(pinRightMotorDirection);
//}
//
//void runLeftMotorBackward() {
//  runMotorBackward(pinLeftMotorDirection);
//}


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


