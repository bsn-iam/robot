

const int feederAmount=5;
const int pinValves[feederAmount] = {1, 2, 3, 4, 5}; //digital out
const int pinOrders[feederAmount]= {5, 6, 7, 8, 9}; //analog in 


const int pinManualRun = 15;  //digital in
const int pinConsumption = 15;  //digital in
const int pinPump = 15;   //digital out
const int pinBuzzer = 15;   //digital out


int volumes[5];
float deadOrder=0.15; //0-5v
const int SerialBaud = 19200; //UART port speed

unsigned long const stepTime=1000*60*60*24;

unsigned long currentStartTime;
unsigned long lastFinishTime=0;
int currentDuration;
int internalDelay=100; //ms
int flowAbsentDelay=2*1000;

float currentFlow;
float currentVolume;
int maximumVolume=200; //mL
float pumpTableFlow=0.03; //mL/ms
float minPossibleFlow=0.001; //mL/ms
//int maximumDuration=30000; //ms till the stop

void initPins() {
	Serial.begin(SerialBaud);
	pinMode(pinPump, OUTPUT);
	pinMode(pinBuzzer, OUTPUT);

	pinMode(pinManualRun, INPUT);
	digitalWrite(pinManualRun, LOW);

	pinMode(pinConsumption, INPUT);
  
	for (int index = 0; index <= feederAmount-1; index++) {
		pinMode(pinValves[index], OUTPUT);
		pinMode(pinOrders[index], INPUT);
	}
return;
}

void setup() {
	initPins();
	getStartSound();
}

void loop() {
	while (millis()-lastFinishTime < stepTime){
		checkManualRun();
		delay(1000);
	}
	runFullCycle();
	lastFinishTime=millis();
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

float GetFlow() {
	uint32_t varPulse; 
	float flow;
	varPulse=pulseIn(pinConsumption, HIGH, 200000);

	if(varPulse){flow=1000000/(15*varPulse);}
    else        {flow=0;}  //L/min  
	flow=flow*1000/60*1000; //mL/ms
	Serial.println((String) "Current flow [" + flow + "] mL/ms].");
return flow; //mL/ms
}


void FeedTheLine(int index){
	int counter=0;
	int rawVolume=analogRead(pinOrders[index]); //0-5v
	//volumes[index]=rawVolume; //0-5
	float requestedVolume=rawVolume/5*maximumVolume; //mL
	int requestedDuration=requestedVolume/pumpTableFlow; //ms
	Serial.println((String) "Feeder [" + index + "] has order ["+volumes[index]+"].");

	if (rawVolume > deadOrder){

		openTheValve(index);
		currentStartTime=millis();
		startThePump();
		calculateCurrents();
		while (currentVolume < requestedVolume && 
				currentDuration < requestedDuration){
			counter+=1;
			delay(internalDelay); //ms
			calculateCurrents();
			checkTheFlow(counter, currentFlow);
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

void startThePump() {
	digitalWrite(pinPump, HIGH);
}

void stopThePump() {
	digitalWrite(pinPump, LOW);
}
void openTheValve(int index) {
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
