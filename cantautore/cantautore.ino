
const int pinRelay = 6;
const int pinLed = LED_BUILTIN;
const int pinLightSensor = 5;
const int SerialBaud = 19200; //UART port speed
const float lightOnVoltageLimit = 1.0;
int internalDelay=100; //ms

void initPins() {
	Serial.begin(SerialBaud);
	
	pinMode(pinRelay, OUTPUT);
	pinMode(pinLed, OUTPUT);

	pinMode(pinLightSensor, INPUT);
	digitalWrite(pinLightSensor, LOW);

return;
}


void setup() {
	initPins();
	Serial.println("Started.");
}

void loop() {
	if (isLightOn()) {
		Serial.println("Relay on ");
		digitalWrite(pinRelay, HIGH);
		digitalWrite(pinLed, HIGH);
	} else {
		Serial.println("Relay off");
		digitalWrite(pinRelay, LOW);
		digitalWrite(pinLed, LOW);
	}
	delay(internalDelay);

}


bool isLightOn (){
	float rawLightSensorVoltage=analogRead(pinLightSensor)*5.0/1023.0; //0-5v
	Serial.println(rawLightSensorVoltage);
	if (rawLightSensorVoltage > lightOnVoltageLimit) 
		return true;
	return false;
}
