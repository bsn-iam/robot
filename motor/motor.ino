

const int Serial1Baud = 9600; //UART port speed
const int SerialBaud = 19200; //UART port speed
unsigned long loopStartTime;
unsigned long timeLabelCurrent = 0;
const float voltageDividerCoeff = 4.02;
const float minBatteryValue = 10.0; //volt
const float lowBatteryValue = 10.5; //volt
const float blowingStartBatteryValue = 10.5; //volt
const float blowingStopBatteryValue = 10.0; //volt
const float maxBatteryValue = 14; //volt
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
const int distanceInCollision = 6; //cm

const int pinLed = 13;
//const int pinRX0 = 19;
const int pinVoltage = 15;
const int pinMotorCurrent = 9;
const int pinMotorOrder = 17;
const int pinOwnPowerSwitch = 22;
const int pinBlower = 14;
const int pinSpinning = 26;
const int pinBuzzer = 24;

// Board	int.0	int.1	int.2	int.3	int.4	int.5
// Mega2560	2		3		21		20		19		18
const int interruptNumberIR = 5;
const int pinIRcenter = 19; // IRheadPin 10 //corresponds the interrupt number!
const int pinIRleft = 32;  // IRheadPin 11
const int pinIRright = 36;  // IRheadPin 12

const int pinBaseActivation = 46;  // ISwitch on the base bumpers
const int pinBaseRespond = 48;  // Appears when we have 12v from base


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
bool isIRcenter=false;
bool isIRleft=false;
bool isIRright=false;
const int lookingTimeMax=2500; //ms to turn around

#define FullSpeed 255
#define HalfSpeed 200
#define QuarterSpeed 190
#define MinSpeed 140

#define rotateLeft -1
#define stright 0
#define rotateRight 1

#define None 0
#define Joystick 1
#define Auto 2
#define Spiral 3
#define Wall 4
#define Searching 5
//int mode = Spiral;
int mode = Searching;

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

  // pinMode(pinRX0, INPUT);
  // digitalWrite(pinRX0, HIGH);

  pinMode(pinOwnPowerSwitch, OUTPUT);
  pinMode(pinSpinning, OUTPUT);
  pinMode(pinBlower, OUTPUT);
  digitalWrite(pinBlower, HIGH);

  pinMode(pinVoltage, INPUT);
  digitalWrite(pinVoltage, HIGH);
  
  pinMode(pinIRcenter, INPUT);
  digitalWrite(pinIRcenter, LOW);  
  pinMode(pinIRleft, INPUT);
  digitalWrite(pinIRleft, LOW);
  pinMode(pinIRright, INPUT);
  digitalWrite(pinIRright, LOW);

  pinMode(pinBaseRespond, INPUT);
  digitalWrite(pinBaseRespond, LOW);
  pinMode(pinBaseActivation, OUTPUT);
  
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
  attachInterrupt (interruptNumberIR, IRmonitorCenter, CHANGE);

  for (int index = 1; index <= encoderSmoothingAmount; index++) {
    encoderLeftArray[index] = 10;
    encoderRightArray[index] = 10;
  }

  for (int index = 1; index <= voltageSmoothingAmount; index++)
    voltageArray[index] = okBatteryValue;

  for (int index = 1; index <= motorCurrentSmoothingAmount; index++)
    motorCurrentArray[index] = 0;

  getStartSound();
  delay(1000);
}

//Main loop
void loop() {
  loopStartTime = millis();
  // timeLabelCurrent = loopStartTime;
  //printTimeLabel(1);

  currentBatteryVoltage = getBatteryVoltage();
  checkBattery(currentBatteryVoltage);
  setMode();
  speedChosen = getSpeed();

  BluetoothCheck();

  setSpinBlowState(currentBatteryVoltage, mode);
  
  if (mode == Auto || mode == Spiral) {
    processMovement();
  }
  
  IRsearchHandler();
  
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
  // if (mode == Spiral && wasCollision) {
    // mode = Auto;
  // }
  if ( doIseeBase() ) {
	mode = Searching;
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



void IRmonitorCenter () {
  if (mode == Searching) {
	  stopMotors();
  }
  return;
}

void startBaseConnectors(){
	digitalWrite(pinBaseActivation, HIGH);
return;
}

bool IsConnected(){
	bool var=digitalRead(pinBaseRespond);
return var;
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
        startForceMovement(stright, -1, timeBackwardMoving); break;	//backward
      case 2:
        startForceMovement(rotateLeft, 1, timeRotatingMoving); break;	//left
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
        startForceMovement(stright, 1, 600); break;	//forward
      case 4:
        startForceMovement(rotateLeft, 1, timeRotatingMoving); break;	//left
      case 5:
        startForceMovement(stright, -1, timeBackwardMoving); break;	//backward
      case 6:
        startForceMovement(rotateLeft, 1, timeRotatingMoving); break;	//left
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

bool encodersStuck(float speedOrder) {

  float encoderLeftAverage;
  float encoderLeftSum;
  float encoderRightAverage;
  float encoderRightSum;

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


//|AtTheEnd(i);
void incrementAtTheEnd(int &i) {
  if (isForceMoveLastStep) {
    i++;
  }
  return;
}


int lookingStepCount=0;

void IRsearchHandler () {
  if (mode==Searching) {
    stopBlowing();
    stopSpinning(); 
	startBaseConnectors();    

	if (IsConnected()){
		tone(pinBuzzer, 2000, 10);
		Serial.println("Waitnig to confirm the connection.");
		stopMotors();
		delay(1000);
		if (IsConnected()){
			Serial.println("Connected to Base. Sleep.");
			goToSleep();
		} else {
			tone(pinBuzzer, 1500, 10);
		}
	} else {
		processSearch();
	}

  }
return;	
}

void processSearch() {
    if ( doIseeBase () ) lookingStepCount=0;
    
    if (isLeftSideCollision() || isRightSideCollision()) {
      processMovement();
    } else {
      checkForStuck();
      if (!stuckState) {	
        runTobase();
      }
    }
return;	
}

void runTobase () {
  if (isIRcenter) {
    moveSmart(stright, FullSpeed/255.0); 
    Serial.println("IRhead. I see it!");
    if (isIRleft && !isIRright) { // Center and left
      moveSmart(-0.4, FullSpeed/255.0); 
    }
    if (isIRright && !isIRleft) { // Center and right
      moveSmart(0.4, FullSpeed/255.0); 
    }
  
  } else {
    if (isIRleft && !isIRright) { // I see left
      moveSmart(rotateLeft, QuarterSpeed/255.0); 
      Serial.println("IRhead. I see left.");
    }
    
    if (isIRright && !isIRleft) { //I see right
      moveSmart(rotateRight, QuarterSpeed/255.0); 
      Serial.println("IRhead. I see right.");
    }
    
    if (isIRright && isIRleft) { //I see right, left, but not centre
      stopMotors();
      Serial.println("IRhead. I see sides.");
    }
  }
  if (!isIRright && !isIRleft && !isIRcenter)	{ //don't see at all
    int timeWithoutContact=lookingStepCount*stepDurationOrder;
    lookingStepCount++;
    //lookingTimeMax
    if (isInTheRange(timeWithoutContact, 1, 500)) {
      stopMotors();
      Serial.println("IRhead. Stopping.");
    }
    if (isInTheRange(timeWithoutContact, 500, 3000)) {
      moveSmart(rotateRight, QuarterSpeed/255.0); 
      Serial.println("IRhead. Rotating right.");
    }
    if (isInTheRange(timeWithoutContact, 3000, 6000)) {
      moveSmart(rotateLeft, QuarterSpeed/255.0); 
      Serial.println("IRhead. Rotating left.");
    }
    if (timeWithoutContact > 6000) {
      processMovement();
      //lookingStepCount=0;
      //mode=Auto;
      //Serial.println("IRhead. Exit.");
    }
  }
  return;
}

void getIRstate () {
	isIRcenter=digitalRead(pinIRcenter);
	isIRleft=digitalRead(pinIRleft);
	isIRright=digitalRead(pinIRright);
  Serial.print("("); Serial.print(isIRleft);
  Serial.print(", "); Serial.print(isIRcenter);
  Serial.print(", "); Serial.print(isIRright); Serial.println(")"); 
}

bool doIseeBase () {
	getIRstate();
	if (isIRcenter || isIRleft || isIRright) {
		return true;
	}
return false;	
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
    Serial.println("Battery pin is not connected. Go to sleep.");
    tone(pinBuzzer, 2700, 50);
    goToSleep();
  }
  if (OutNow == 1023) {
    Serial.println("Warning! Battery voltage is greater then allowed!");
    tone(pinBuzzer, 2700, 5);
    //goToSleep();
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
  Serial.print("Current voltage: ");
  Serial.println(VoltageNow, 2);

  return voltageAverage;
}


void setSpinBlowState(float currentVoltage, int localMode) {
  if (localMode == Auto || localMode == Spiral) {
    if ( currentVoltage > blowingStartBatteryValue && !stuckState ) {
      StartBlowing();
      startSpinning();
    }
  }
    if ( currentVoltage < blowingStopBatteryValue || stuckState ) {
      stopBlowing();
      stopSpinning();
    }
  return;
}

void checkBattery(float currentVoltage) {
  if (currentVoltage < lowBatteryValue) {
    Serial.print("WARNING! Low battery voltage - ");
    Serial.println(currentVoltage);
    mode=Searching;
    tone(pinBuzzer, 1700, 5);
  }
  if (currentVoltage < minBatteryValue) {
    Serial.print("ALARM! Go to sleep. Minimal battery voltage - ");
    Serial.println(currentBatteryVoltage);
    tone(pinBuzzer, 2700, 50);
    goToSleep();
  }
  if (currentVoltage > maxBatteryValue) {
    Serial.print("ALARM! Go to sleep. Maximal battery voltage - ");
    Serial.println(currentBatteryVoltage);
    //tone(pinBuzzer, 2000, 500);
    //goToSleep();
  }
  return;
}

void lockPower() {
  digitalWrite(pinOwnPowerSwitch, 255);
  Serial.println("Started");
  return;
}


void goToSleep() {
  stopBlowing();
  stopSpinning();
  stopMotors();
  playRandomSong(); //uncomment to play a music
  //tone(pinBuzzer, 1700, 3000);
  digitalWrite(pinOwnPowerSwitch, 0);
  delay(5000);
  digitalWrite(pinOwnPowerSwitch, 1);
  return;
}

void StartBlowing() {
static unsigned long timeBlowingStart = 0;
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
void stopBlowing() {
  digitalWrite(pinBlower, HIGH);
  return;
}

void changeBlowing() {
  digitalWrite(pinBlower, !digitalRead(pinBlower));
  return;
}

void startSpinning() {
  digitalWrite(pinSpinning, HIGH);
  return;
}

void stopSpinning() {
  digitalWrite(pinSpinning, LOW);
  return;
}

void changeSpinning() {
  digitalWrite(pinSpinning, !digitalRead(pinSpinning));
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
const float spiralDirectionLimit = 0.4;

float getDirection() {
  static float spiralDirectionOrder = spiralDirectionLimit;
  static float increment = spiralStep;
  if (mode == Spiral) {
    if (spiralDirectionOrder >= abs(spiralDirectionLimit)) {
      increment = -abs(spiralStep);
    }
    if (spiralDirectionOrder < -abs(spiralDirectionLimit)) {
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
    //tone(pinBuzzer, 900, 10);
  }
  if (leftSonarDistance == 0) {
    Serial.println("Warning! Left sonar is not available!");
    //tone(pinBuzzer, 900, 10);
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
  checkForStuck();
  if (!stuckState) {	
    if (isLeftSideCollision()) {
      startForceMovement(rotateRight, FullSpeed/255.0, turnRightTimeout);
      Serial.println("Right side backward");
      return;
    }

    if (isRightSideCollision()) {
      startForceMovement(rotateLeft, FullSpeed/255.0, turnLeftTimeout);
      Serial.println("Left side backward");
      return;
    }

    if (!isForceMovementActive()) {
      moveSmart(getDirection(), speedChosen / 255.0);
    }
  }
  return;
}

bool isRightSideCollision () {
  static float rightSonarDistance_prev;
  rightSonarDistance_prev=rightSonarDistance;
  rightSonarDistance = checkTheDistance (pinRightSonarTrig, pinRightSonarEcho, "Right");
  if (checkTheCollision (rightSonarDistance, rightSonarDistance_prev, "Right") || isBumperPressed(pinRightBumper)) {
    Serial.println("Right side collision.");
    wasCollision = 1;
    return true;
  }
  return false;
}


bool isLeftSideCollision () {
  static float leftSonarDistance_prev;
  leftSonarDistance_prev=leftSonarDistance;
  leftSonarDistance = checkTheDistance (pinLeftSonarTrig, pinLeftSonarEcho, "Left");
  if (checkTheCollision (leftSonarDistance, leftSonarDistance_prev, "Left") || isBumperPressed(pinLeftBumper)) {
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

  duration = pulseIn(pinSonarEcho, HIGH, 50000);
  cm = duration / 58;
  return cm;
}

bool checkTheCollision (float distance, float distance_prev, char descr[ ]) {
  bool sonarFreeze=false;
  if (distance == distance_prev && distance != 0) {
    //tone(pinBuzzer, 2500, 20);
    Serial.print(descr);
    Serial.print(" sonar is dead. Result is ");
    Serial.println(distance);
    sonarFreeze=true;
  }
  if (distance <= distanceInCollision  && distance > 0 && !sonarFreeze)
  {
    return true;
  }
  return false;
}


char readSerial1 ()
{
  char inChar;
  inChar = ' ';
  String inputString = "";
  inputString.reserve(200);
  while (Serial1.available() > 0) {
    inChar = (char)Serial1.read();
    inputString = "";
    inputString += inChar;
  }
  char output=inputString[0];
  return output;
}

void BluetoothCheck ()
{
  char input= readSerial1 ();
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
  switch ( input ) {
  case 'm':
    mode = Joystick;
    Serial.println("Manual mode");
    tone(pinBuzzer, 1700, 5); delay(1000); break;
  case 'a':
    mode = Auto;
    Serial.println("Auto mode");
    tone(pinBuzzer, 1700, 5); delay(1000); break;
  case 'w':
    mode = Wall;
    Serial.println("Wall mode");
    tone(pinBuzzer, 1700, 5); delay(1000); break;
  case 's':
    mode = Spiral;
    Serial.println("Spiral mode");
    tone(pinBuzzer, 1700, 5); delay(1000); break;
  case 'x':
    mode = None;
    Serial.println("Stop all"); 
	tone(pinBuzzer, 1700, 5); delay(1000); break;
  case 'b':
    changeBlowing();
    Serial.println("Blowing");
    tone(pinBuzzer, 1700, 5); delay(1000); break;
  case 't': //turn
    changeSpinning();
    Serial.println("Spin");
    tone(pinBuzzer, 1700, 5); delay(1000); break;
  }
  return;
}

//moveSmart(0,1)- forward full speed
//moveSmart(0,-1)- backward full speed
//moveSmart(-1,1)- full left full speed
//moveSmart(0.5,1)- half right full speed

//courseOrderRel - [-1;1], speedOrderRel - [-1;1]

void moveSmart(float courseOrderRel, float speedOrderRel)
{
  static float courseOrderRel_prev;
  static float speedOrderRel_prev;
  static int leftVelocityPure_prev;
  static int rightVelocityPure_prev;
  
  float leftVelocityPureRel;
  float rightVelocityPureRel;
  int leftVelocityPure;
  int rightVelocityPure;

  float courseDeviationReal = 1;
  float courseDeviationOrder = 1;
  float courseCorrection = 1;
  float leftCorrection = 1;
  float rightCorrection = 1;

  haltIfElevated(courseOrderRel, speedOrderRel);
  
  if ( abs(courseOrderRel)>1 || abs(speedOrderRel)>1 ) {
	  Serial.println("ERROR! Incorrect velocity order!");
	  goToSleep();
  }
  
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
  leftVelocityPureRel=smoothLow(leftVelocityPureRel); //[-1;1]
  rightVelocityPureRel=smoothLow(rightVelocityPureRel); //[-1;1]
  
  leftVelocityPure = speedOrderRel * leftVelocityPureRel * 255; //[-255;255]
  rightVelocityPure = speedOrderRel * rightVelocityPureRel * 255; //[-255;255]

  // Serial.print("              Velocity = (");
  // Serial.print(leftVelocityPure);
  // Serial.print(" , ");
  // Serial.print(rightVelocityPure);
  // Serial.println(")");


  // Serial.print("              Velocity = (");
  // Serial.print(leftVelocityPure);
	
  //correct the direction
  if (courseOrderRel_prev == courseOrderRel && speedOrderRel_prev == speedOrderRel) {
    float stepLeft = encoderLeftLastStep();
    float stepRight = encoderRightLastStep();

    if (stepLeft != 0 && stepRight != 0) {
      courseDeviationReal = abs(stepLeft / stepRight); //(0;Inf)
      courseDeviationOrder = abs(leftVelocityPure_prev / rightVelocityPure_prev); //(0;Inf)
      courseCorrection = courseDeviationOrder / courseDeviationReal; //(0;Inf)

      // Serial.print("courseDeviationReal= ");
      // Serial.print(courseDeviationReal);
      // Serial.print(", courseDeviationOrder= ");
      // Serial.print(courseDeviationOrder);
      // Serial.print(", courseCorrection= ");
      // Serial.println(courseCorrection);
      leftCorrection = min(courseCorrection, 255 / leftVelocityPure); // v - [0; 255]
      rightCorrection = min(1 / courseCorrection, 255 / rightVelocityPure); // v - [0; 255]
      leftCorrection = max(leftCorrection, 0.7); // v - [0.7U; 255]
      rightCorrection = max(rightCorrection, 0.7); // v - [0.7U; 255]
    }
  }

  // Serial.print("              Velocity = (");
  // Serial.print(leftVelocityPure*leftCorrection);
  // Serial.print(" , ");
  // Serial.print(rightVelocityPure*rightCorrection);
  // Serial.println(")");

  setLeftMotorOrder(leftVelocityPure * leftCorrection);
  setRightMotorOrder(rightVelocityPure * rightCorrection);

  courseOrderRel_prev = courseOrderRel;
  speedOrderRel_prev = speedOrderRel;
  leftVelocityPure_prev = leftVelocityPure;
  rightVelocityPure_prev = rightVelocityPure;
  return;
}

int noChangeStepCount=0;
int maximumElevatedTime=10000;

void haltIfElevated (float speed, float course) {
	checkForElevation (speed, course);
	int timeFromLastChange=noChangeStepCount*stepDurationOrder;
	// Serial.print("Time since last course change - ");
	Serial.println(timeFromLastChange);
	if ( timeFromLastChange > maximumElevatedTime ) {
		Serial.println("Elevated state detected. Go to sleep.");
		goToSleep();
		noChangeStepCount=0;
	}
	return;
}

void checkForElevation (float speed, float course) {
  static float speed_prev;
  static float course_prev;
	if (motorPower && (mode==Auto || mode==Spiral)) {
	  if (course_prev == course && speed_prev == speed) {
		  noChangeStepCount++;
	  } else {
		noChangeStepCount=0;
	  }
	} else {
	  noChangeStepCount=0;
	}
	course_prev = course;
	speed_prev == speed;
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



