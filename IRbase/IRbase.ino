
#include <IRremote.h> //An IR LED must be connected to Arduino PWM pin 3.
#include <LiquidCrystal.h>

  // The circuit:
 // 1* LCD RS pin to digital pin
 // 2* LCD Enable pin to digital pin
 // 3* LCD D4 pin to digital pin
 // 4* LCD D5 pin to digital pin
 // 5* LCD D6 pin to digital pin
 // 6* LCD D7 pin to digital pin
 // * LCD R/W pin to ground
 // * LCD VSS pin to ground
 // * LCD VCC pin to 5V
 // * 10K resistor:
 // * ends to +5V and ground
 // * wiper to LCD VO pin (pin 3)
// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(41, 42, 43, 44, 45, 46);

const int pinLed = 13;
//const int pinIR = 9;
 

IRsend irsend;

void setup()
{
	lcdWelcomePrint ();
}

void loop() {
	digitalWrite(pinLed, HIGH);
	irsend.sendSony(0xa90, 12);
	delay(30);
	digitalWrite(pinLed, LOW);
	delay(20);
	
	// for (int i = 0; i < 3; i++) {
		// irsend.sendSony(0xa90, 12);
	// }
//	delay(1000); //5 second delay between each signal burst
}



void lcdWelcomePrint () {
  // set the cursor to column 0, line 1
  // (note: line 1 is the second row, since counting begins with 0):
  lcd.setCursor(0, 1);
  
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  
  // Print a message to the LCD.
  lcd.print("hello, world!");
	
}
