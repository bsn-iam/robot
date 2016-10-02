#include <IRremote.h>
#include <IRremoteInt.h>

// To support more than 5 receivers, remember to change the define
// IR_PARAMS_MAX in IRremoteInt.h as well.
#define IRamount 3
	IRrecv *irrecvs[IRamount];
	decode_results results;

const int SerialBaud = 19200; //UART port speed

const int pinLED = 13;

int pinIRinput[IRamount];
int pinIRoutput[IRamount];

void initPins() {
	pinIRinput[0]=3;
	pinIRinput[1]=4;
	pinIRinput[2]=5;
	pinIRoutput[0]=10;
	pinIRoutput[1]=11;
	pinIRoutput[2]=12;

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
  // delay (20);
}


void readIR() {
  for (int i = 0; i < IRamount; i++)
  {
    if (irrecvs[i]->decode(&results))
    {
		if (results.value == 0xa90 ) {
			Serial.print(i); Serial.print(". "); Serial.println(results.value, HEX);
			digitalWrite(pinLED, HIGH);
			digitalWrite(pinIRoutput[i], HIGH);
		} else {
			digitalWrite(pinIRoutput[i], LOW);
		}
      //IRinfo(&results);
      irrecvs[i]->resume();
    } else {
	  digitalWrite(pinLED, LOW);
	}
  }
  return;
}


// void IRinfo(decode_results *results) {
  // // Dumps out the decode_results structure.
  // // Call this after IRrecv::decode()
  // int count = results->rawlen;
  // if (results->decode_type == UNKNOWN) {
    // Serial.print("Unknown encoding: ");
  // }
  // else if (results->decode_type == NEC) {
    // Serial.print("Decoded NEC: ");
  // }
  // else if (results->decode_type == SONY) {
    // Serial.print("Decoded SONY: ");
  // }
  // else if (results->decode_type == RC5) {
    // Serial.print("Decoded RC5: ");
  // }
  // else if (results->decode_type == RC6) {
    // Serial.print("Decoded RC6: ");
  // }
  // else if (results->decode_type == PANASONIC) {
    // Serial.print("Decoded PANASONIC - Address: ");
    // Serial.print(results->address, HEX);
    // Serial.print(" Value: ");
  // }
  // else if (results->decode_type == LG) {
    // Serial.print("Decoded LG: ");
  // }
  // else if (results->decode_type == JVC) {
    // Serial.print("Decoded JVC: ");
  // }
  // else if (results->decode_type == AIWA_RC_T501) {
    // Serial.print("Decoded AIWA RC T501: ");
  // }
  // else if (results->decode_type == WHYNTER) {
    // Serial.print("Decoded Whynter: ");
  // }
  // Serial.print(results->value, HEX);
  // Serial.print(" (");
  // Serial.print(results->bits, DEC);
  // Serial.println(" bits)");
  // Serial.print("Raw (");
  // Serial.print(count, DEC);
  // Serial.print("): ");

  // Serial.println();
// }
