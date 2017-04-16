

const int feederAmount=5;
const int pinValves[feederAmount] = {2, 3, 5, 7, 9}; //digital out
const int pinOrders[feederAmount]= {5, 6, 7, 8, 9}; //analog in 


const int pinManualRun = 2;  //digital in
const int pinConsumptionPower = 11;  //digital OUT
const int pinConsumption = 12;  //digital in
const int pinPump = 13;   //digital out
const int pinBuzzer = 11;   //digital out


float volumes[5];
float deadVolumeOrder=20; //mL
const int SerialBaud = 19200; //UART port speed

unsigned long const stepTime=1000*60*60*24;

unsigned long currentStartTime;
unsigned long lastFinishTime=0;
int currentDuration;
int internalDelay=100; //ms
int flowAbsentDelay=2*1000;

float currentFlow;
float currentVolume;
float maximumVolume=200; //mL
float pumpTableFlow=0.03; //mL/ms
float minPossibleFlow=0.001; //mL/ms
//int maximumDuration=30000; //ms till the stop

void initPins() {
	Serial.begin(SerialBaud);
	pinMode(pinPump, OUTPUT);
	pinMode(pinBuzzer, OUTPUT);

	pinMode(pinManualRun, INPUT);
	digitalWrite(pinManualRun, LOW);
	
	pinMode(pinConsumptionPower, INPUT);
	digitalWrite(pinConsumptionPower, LOW);

	pinMode(pinConsumption, INPUT);
	digitalWrite(pinConsumption, LOW);
  
	for (int index = 0; index <= feederAmount-1; index++) {
		pinMode(pinValves[index], OUTPUT);
		pinMode(pinOrders[index], INPUT);
	}
return;
}

void setup() {
	initPins();
	startTheSensor();
	getStartSound();
	volumes[0]=200;
	volumes[1]=200;
	volumes[2]=0;
	volumes[3]=0;
	volumes[4]=0;
}

void loop() {
	while (millis()-lastFinishTime > stepTime){
		checkManualRun();
		delay(1000);
	}
	runFullCycle();
	lastFinishTime=millis();
	delay(5000);
}

void runFullCycle(){
//	timer0_millis=0;
	for (int index = 0; index <= feederAmount-1; index++) {
		FeedTheLine(index);
	}
return;	
}

void checkManualRun(){
	if (digitalRead(pinManualRun)){
		runFullCycle();
	}
return;	
}

float getOrder(int index){
	float rawVolume=analogRead(pinOrders[index]); //0-5v
	volumes[index]=rawVolume/5.0*maximumVolume;
return volumes[index];	
}

float GetFlow() {
	uint32_t varPulse; 
	float flow;
	varPulse=pulseIn(pinConsumption, HIGH, 200000);

	if(varPulse){flow=1000000/(15*varPulse);}
    else        {flow=0;}  //L/min  
	flow=flow*1000/60*1000; //mL/ms
	//Serial.println((String) "Current flow [" + flow + "] mL/ms].Pulse duration-"+varPulse);
return flow; //mL/ms
}


void FeedTheLine(int index){
	int counter=0;
	//float requestedVolume=getOrder(index); //mL
	float requestedVolume=volumes[index]; //mL
	int requestedDuration=requestedVolume/pumpTableFlow; //ms

	if (requestedVolume > deadVolumeOrder){
		Serial.println((String) "Feeder [" + index + "] has order ["+requestedVolume+"].");

		openTheValve(index);
		currentStartTime=millis();
		startThePump();
		calculateCurrents();
		while (currentVolume < requestedVolume && 
				currentDuration < requestedDuration){
			counter+=1;
			delay(internalDelay); //ms
			calculateCurrents();
			//checkTheFlow(counter, currentFlow);
		}
	
		stopThePump();
		closeTheValve(index);
		Serial.println((String) "Pump [" + index + "] stopped with volume [" + currentVolume + "/" + requestedVolume +
		"] and duration ["+currentDuration+ + "/" + requestedDuration +"].");
	}
	
return;	
}



void checkTheFlow(int count, float flow){
	float stepsMaxWithNoFlow=flowAbsentDelay/internalDelay;
	if (count > stepsMaxWithNoFlow) {
		if (flow < minPossibleFlow){
			//stopThePump();
			tone(pinBuzzer, 1900, 10);
		}
	}
	
	
return;	
}

void calculateCurrents(){
	currentFlow=GetFlow();
	currentDuration=millis()-currentStartTime; //ms
	currentVolume=currentFlow*currentDuration; // mL=(mL/ms) * ms
return;	
}

void startTheSensor() {
	digitalWrite(pinConsumptionPower, HIGH);
}

void stopTheSensor() {
	digitalWrite(pinConsumptionPower, LOW);
}

void startThePump() {
	digitalWrite(pinPump, HIGH);
}
void stopThePump() {
	digitalWrite(pinPump, LOW);
}
void openTheValve(int index) {
	Serial.println((String) index+ "-valve opened. Pin-"+pinValves[index]);
	digitalWrite(pinValves[index], HIGH);
}

void closeTheValve(int index) {
	digitalWrite(pinValves[index], LOW);
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
