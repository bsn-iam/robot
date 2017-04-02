#include <IRremote.h> //An IR LED must be connected to Arduino PWM pin 3.
#include <Servo.h>

const int pinLed = 13;
const int pinServo = 6;
const int pinDetector = 2;
const int pinPowerFeed = 8;
const int SerialBaud = 19200; //UART port speed

unsigned long connectorStartTime;
int rised=180;
int connected=0;
int position=connected;

Servo connector;
IRsend irsend;

void initPins() {
    Serial.begin(SerialBaud);
	pinMode(pinPowerFeed, OUTPUT);
	
	pinMode(pinDetector, INPUT);
	digitalWrite(pinDetector, LOW);
}

void setup() {
	initPins();
	connector.attach(pinServo); delay(500);
	connectorStartTime = millis();
	moveConnector(connected);
}

void loop() {
	digitalWrite(pinLed, HIGH);
	irsend.sendSony(0xa90, 12);
	delay(30);
	digitalWrite(pinLed, LOW);
	delay(20);
  
	runConnection();
}

void runConnection () {
	if (isContact()) {
		lowerConnector ();
		delay(200);
	}
	if (!isContact()) {
		riseConnector ();
		delay(200);
	}
}

bool isContact(){
	bool value=digitalRead(pinDetector);
	if (value){
		Serial.println((String) "Contact!-["+value+"]");
		return true;
	}
return false;	
}

void riseConnector () {
	Serial.println("Rise command.");
	if (position == connected) {
		position=rised;
		moveConnector(position);
		Serial.println(position);
	}
}

void lowerConnector () {
	Serial.println("Lower command.");
    if (position == rised) {
		position=connected;
		moveConnector(position);
		Serial.println(position);
    }
}

void moveConnector (int deg) {
	Serial.println((String) "Moving to "+deg);
  connector.write(deg);
  connectorStartTime = millis();	
  //connector.detach();
}

