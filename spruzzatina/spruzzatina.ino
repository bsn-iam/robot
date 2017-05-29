

const int feederAmount=5;
const int pinValves[feederAmount] = {6, 7, 8, 4, 2}; //digital out
const int pinOrders[feederAmount]= {1, 2, 3, 4, 5}; //analog in 


const int pinManualRun = 9;  //analog in
const int pinConsumptionPower = 12;  //digital OUT
const uint8_t pinConsumption = 3;  //digital in //Interrupt
const int pinPump = 13;   //digital out
const int pinBuzzer = 11;   //digital out


float volumes[5];
float deadVolumeOrder=0.001; //
const int SerialBaud = 19200; //UART port speed

unsigned long lastStepTime;
unsigned long lastFinishTime=0;
unsigned long int currentDuration;
int internalDelay=100; //ms

unsigned long hour=3600UL*1000UL; //ms
unsigned long workFrequency=hour*48UL; //ms


int flowAbsentDelay=2*1000;
volatile uint16_t flowCount  = 0;  // Определяем переменную для подсчёта количества импульсов поступивших от датчика

float currentVolume;
float maximumVolume=100; //mL
float pumpTableFlow=0.03/6; //mL/ms
unsigned long lastFlowCallTime=0;
unsigned long flowStartTime=0;
float Kcorr=9;
bool isAlarmState=false;
//int maximumDuration=30000; //ms till the stop

void initPins() {
	Serial.begin(SerialBaud);
	pinMode(pinPump, OUTPUT);
	pinMode(pinBuzzer, OUTPUT);

	pinMode(pinManualRun, INPUT);
	digitalWrite(pinManualRun, LOW);
	
	pinMode(pinConsumptionPower, OUTPUT);
	//digitalWrite(pinConsumptionPower, LOW);

	pinMode(pinConsumption, INPUT);
  
	for (int index = 0; index <= feederAmount-1; index++) {
		pinMode(pinValves[index], OUTPUT);
		pinMode(pinOrders[index], INPUT);
		//digitalWrite(pinOrders[index], LOW);
	}
return;
}


void setup() {
	initPins();
	StartTheSensor();
	GetStartSound();
	volumes[0]=0;
	volumes[1]=0;
	volumes[2]=0;
	volumes[3]=0;
	volumes[4]=0;
	
	uint8_t interruptNumber = digitalPinToInterrupt(pinConsumption);              // Определяем № прерывания который использует вывод pinConsumption
    attachInterrupt(interruptNumber, IncrementFlow, RISING);           // Назначаем функцию funCountInt как обработчик прерываний interruptNumber при каждом выполнении условия RISING - переход от 0 к 1
    if(interruptNumber<0){Serial.print("Wrong Sensor number.");} 
	
	// digitalWrite(pinOrderPower, HIGH);
	lastFinishTime=0;
	Serial.println("Started.");
}

void loop() {
	if (workFrequency < 715827136){
		if (millis()-lastFinishTime > workFrequency){
			runFullCycle();
			Serial.println((String) "Flow counter= [" + flowCount + "].");
			lastFinishTime=millis();
		} else {
			checkManualRun();	
			delay(500);	
		}
		if (millis()<lastFinishTime) {
			lastFinishTime=millis();
		}
	} else {
		Serial.println((String) "Wrong division!");
		tone(pinBuzzer, 2900, 500);
	}
	
	if (isAlarmState) tone(pinBuzzer, 2900, 20);
}

void IncrementFlow(){flowCount++;}     

void runFullCycle(){
//	timer0_millis=0;
	tone(pinBuzzer, 1900, 500);
	Serial.println((String) "Started with time "+millis());
	for (int index = 0; index <= feederAmount-1; index++) {
		FeedTheLine(index);
	}
	Serial.println("");
return;	
}

void checkManualRun(){
	if (digitalRead(pinManualRun)){
		Serial.println("Manual run started.");
		tone(pinBuzzer, 2900, 1000);
		delay(2000);
		runFullCycle();
	}
return;	
}

float getOrder(int index){
	float rawVolume=analogRead(pinOrders[index])*5.0/1023.0; //0-5v
	float orderVoltage=5-rawVolume; //
	Serial.println((String) index +". Order voltage [" + rawVolume + "]");
	volumes[index]=orderVoltage/5.0*maximumVolume;
return volumes[index];	
}

float GetFlow() {
	float  varResult = 0; 
	if (lastFlowCallTime>millis()) lastFlowCallTime=millis(); 		// переполнение переменной 
	
	if (millis() < lastFlowCallTime+internalDelay*10) {    		// если с момента последнего запроса прошло меньше 10 шагов..
        varResult=flowCount/7.5*(millis()-lastFlowCallTime)/1000;   // Рассчитываем скорость потока воды: Q = F/7,5 л/мин
        flowCount=0; lastFlowCallTime=millis();             
    }
	//if (varResult!=0) Serial.println((String) "Flow speed = "+varResult+" L/min");

	float flow=varResult/60*Kcorr; //mL/ms
return flow; //mL/ms
}

// Pulse based calculation

//float GetFlow() {
//	uint32_t varPulse; 
//	float flow;
//	varPulse=pulseIn(pinConsumption, HIGH, 200000);
//
//	if(varPulse){flow=1000000/(15*varPulse);}
//    else        {flow=0;}  //L/min  
//	flow=flow*1000/60*1000; //mL/ms
//	//Serial.println((String) "Current flow [" + flow + "] mL/ms].Pulse duration-"+varPulse);
//return flow; //mL/ms
//}


void FeedTheLine(int index){
	unsigned long counter=0;
	float requestedVolume=getOrder(index); //mL
	float requestedDuration=requestedVolume/pumpTableFlow; //ms

	if (requestedVolume > deadVolumeOrder*maximumVolume){
		Serial.println((String) "Feeder [" + index + "] has order ["+requestedVolume+"].");

		OpenTheValve(index);
		StartThePump();

		flowCount=0;
		currentVolume=0;
		lastStepTime=millis();
		flowStartTime=millis();
		currentDuration=millis()-flowStartTime; //ms
		lastFlowCallTime=millis();

		while (currentVolume < requestedVolume && 
			currentDuration < requestedDuration){
				
			Serial.println((String) "Duration [" + currentDuration + "]/["+requestedDuration+"], volume [" + currentVolume + "]/["+requestedVolume+"].");
			counter+=1;
			delay(internalDelay); //ms
			currentDuration=millis()-flowStartTime; //ms
			currentVolume=currentVolume+GetVolumeDelta(lastStepTime);
			CheckTheFlow(counter, requestedDuration, currentVolume, requestedVolume);
			lastStepTime=millis();
		}
	
		StopThePump();
		CloseTheValve(index);
		Serial.println((String) "Pump [" + index + "] stopped with volume [" + currentVolume + "/" + requestedVolume +
		"] and duration ["+currentDuration+ + "/" + requestedDuration +"].");
	}
return;	
}


void CheckTheFlow(int count, float feedTime, float volume, float feedVolume){
	float stepsMaxWithNoFlow=flowAbsentDelay/internalDelay;
	if (count > stepsMaxWithNoFlow) {
		float spentTime=count*internalDelay;
		float expectedVolume=feedVolume*spentTime/feedTime;
		count=0;
		Serial.println((String) "Volume [" + volume + "], min limit ["+0.5*expectedVolume+"].");

		if (volume < 0.5*expectedVolume){
			StopThePump();
			isAlarmState=true;
			Serial.println((String) "Current volume  [" + volume + "], while expected ["+expectedVolume+"].");
			tone(pinBuzzer, 1400, 1000);
		} else {
			isAlarmState=false;
		}
	}
return;	
}

float GetVolumeDelta(unsigned long lastCallTime){
	float currentVolumeDelta=GetFlow()*(millis()-lastCallTime); // mL=(mL/ms) * ms
return currentVolumeDelta;	
}

void StartTheSensor() {
	digitalWrite(pinConsumptionPower, HIGH);
}

void StopTheSensor() {
	digitalWrite(pinConsumptionPower, LOW);
}

void StartThePump() {
	digitalWrite(pinPump, HIGH);
}
void StopThePump() {
	digitalWrite(pinPump, LOW);
}
void OpenTheValve(int index) {
	pinMode(pinValves[index], OUTPUT);
	Serial.println((String) index+ "-valve opened. Pin-"+pinValves[index]);
	digitalWrite(pinValves[index], HIGH);
}

void CloseTheValve(int index) {
	digitalWrite(pinValves[index], LOW);
	pinMode(pinValves[index], INPUT);
}

void GetStartSound() {
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
