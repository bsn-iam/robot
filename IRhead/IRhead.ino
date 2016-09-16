#include <IRremote.h>
#include <IRremoteInt.h>

// To support more than 5 receivers, remember to change the define
// IR_PARAMS_MAX in IRremoteInt.h as well.
#define IRamount 3
  IRrecv *irrecvs[IRamount];
  decode_results results;

const int pinIR1 = 2;
const int pinIR2 = 3;
const int pinIR3 = 4;
// const int pinIR4 = 5;
// const int pinIR5 = 6;

void setup() {
	
  irrecvs[0] = new IRrecv(pinIR1); // Receiver #0: pin 2
  irrecvs[1] = new IRrecv(pinIR2); // Receiver #1: pin 3
  irrecvs[2] = new IRrecv(pinIR3); // Receiver #2: pin 4
  // irrecvs[3] = new IRrecv(pinIR4); // Receiver #3: pin 5
  // irrecvs[4] = new IRrecv(pinIR5); // Receiver #4: pin 6

  for (int i = 0; i < IRamount; i++)
    irrecvs[i]->enableIRIn();

}

void loop() {
  readIR();
}


void readIR() {
  for (int i = 0; i < IRamount; i++)
  {
    if (irrecvs[i]->decode(&results))
    {
		if(results.value == 0xE0E0E01F || results.value == 0x8F7708F ) {
		// if(1 ) {
			//tone(pinBuzzer, 1900, 10);
		  Serial.print("Receiver #");
		  Serial.print(i);
		  Serial.print(":");
		  Serial.println(results.value, HEX);
		  //Serial.println(results.value, DEC);
		}
      //IRinfo(&results);
      irrecvs[i]->resume();
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
