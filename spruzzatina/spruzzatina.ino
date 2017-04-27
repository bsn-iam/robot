

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

unsigned long const stepTime=1000*60*60*24;

unsigned long currentStartTime;
unsigned long lastFinishTime=0;
int currentDuration;
int internalDelay=100; //ms
int workFrequency=60*60*1000; //s
int flowAbsentDelay=2*1000;
volatile uint16_t flowCount  = 0;  // Определяем переменную для подсчёта количества импульсов поступивших от датчика
         uint32_t flowTime   = 0;  // Определяем переменную для хранения времени последнего расчёта

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
	//startTheSensor();
	getStartSound();
	volumes[0]=0;
	volumes[1]=0;
	volumes[2]=0;
	volumes[3]=0;
	volumes[4]=0;
	
	uint8_t interruptNumber = digitalPinToInterrupt(pinConsumption);              // Определяем № прерывания который использует вывод pinConsumption
    attachInterrupt(interruptNumber, IncrementFlow, RISING);           // Назначаем функцию funCountInt как обработчик прерываний interruptNumber при каждом выполнении условия RISING - переход от 0 к 1
    if(interruptNumber<0){Serial.print("Wrong Sensor number.");} 
	
	// digitalWrite(pinOrderPower, HIGH);
	Serial.println("Started.");
}

void loop() {
	if (millis()-lastFinishTime > workFrequency){
		runFullCycle();
		Serial.println((String) "Flow counter= [" + flowCount + "].");
		lastFinishTime=millis();
	} else {
		checkManualRun();	
		delay(500);	
	}
	if (millis()<lastFinishTime)
		lastFinishTime=millis();
}

void IncrementFlow(){flowCount++;}     

void runFullCycle(){
//	timer0_millis=0;
	tone(pinBuzzer, 1900, 100);
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
		tone(pinBuzzer, 2900, 100);
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
    if((flowTime+1000)<millis() || flowTime>millis()){           // Если c момента последнего расчёта прошла 1 секунда, или произошло переполнение millis то ...
        varResult=flowCount/7.5;                                // Рассчитываем скорость потока воды: Q = F/7,5 л/мин
        flowCount=0; flowTime=millis();                          // Сбрасываем счётчик и сохраняем время расчёта
    }                                                          // (количество импульсов от датчика flowCount равно частоте в Гц, так как расчёт происходит 1 раз в секунду)
    if (varResult!=0) Serial.println((String) "Flow speed = "+varResult+" L/min"); // Выводим скорость потока воды, показания которой будут меняться 1 раз в секунду

	float flow=varResult*1000/60*1000; //mL/ms
return flow; //mL/ms
}

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
	int counter=0;
	float requestedVolume=getOrder(index); //mL
	//float requestedVolume=volumes[index]; //mL
	int requestedDuration=requestedVolume/pumpTableFlow; //ms

	if (requestedVolume > deadVolumeOrder*maximumVolume){
		Serial.println((String) "Feeder [" + index + "] has order ["+requestedVolume+"].");

		openTheValve(index);
		currentStartTime=millis();
		startThePump();
		calculateCurrents();
		// while (currentVolume < requestedVolume && 
		while (currentDuration < requestedDuration){
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
	pinMode(pinValves[index], OUTPUT);
	Serial.println((String) index+ "-valve opened. Pin-"+pinValves[index]);
	digitalWrite(pinValves[index], HIGH);
}

void closeTheValve(int index) {
	digitalWrite(pinValves[index], LOW);
	pinMode(pinValves[index], INPUT);
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
