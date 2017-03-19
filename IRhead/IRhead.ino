#include <IRremote.h>
#include <IRremoteInt.h>

// To support more than 5 receivers, remember to change the define
// IR_PARAMS_MAX in IRremoteInt.h as well.
#define IRamount 3
	IRrecv *irrecvs[IRamount];
	decode_results results;

const int SerialBaud = 19200; //UART port speed

const int pinLED = 13;
const int pinIRpower = 2;

int pinIRinput[IRamount];
int pinIRoutput[IRamount];

void initPins() {
  pinMode(pinIRpower, OUTPUT);
  digitalWrite(pinIRpower, HIGH);
	pinIRinput[0]=3; // pinIRcenter
	pinIRinput[1]=4; // pinIRleft
	pinIRinput[2]=5; // pinIRright
	pinIRoutput[0]=10; // pinIRcenter
	pinIRoutput[1]=11; // pinIRleft
	pinIRoutput[2]=12; // pinIRright

	for (int i = 0; i < IRamount; i++) {
		pinMode(pinIRoutput[i], OUTPUT);
		digitalWrite(pinIRoutput[i], LOW);
	}  
}

void setup() {
  Serial.begin(SerialBaud);
  Serial.println("Started..");
  initPins();

  for (int i = 0; i < IRamount; i++) {
	irrecvs[i] = new IRrecv(pinIRinput[i]);
	irrecvs[i]->enableIRIn();
  }  
}

void loop() {
  readIR();
  delay (100);
}


void readIR() {
  for (int i = 0; i < IRamount; i++)
  {
    if (irrecvs[i]->decode(&results))
    {
      irrecvs[i]->resume();
      //if (results.value == 0xa90 ) { }
      digitalWrite(pinLED, HIGH);
      digitalWrite(pinIRoutput[i], HIGH);
    } else {
      digitalWrite(pinLED, LOW);
      digitalWrite(pinIRoutput[i], LOW);
    }
  }
  
  for (int i = 0; i < IRamount; i++)
  {
    Serial.print("  "); Serial.print(pinIRoutput[i]); Serial.print(":"); Serial.print(digitalRead( pinIRoutput[i]) );
  }
  Serial.println("");  
  return;
}

