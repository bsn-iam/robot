#include <SoftwareSerial.h>
#include <Wire.h>

// I2C OLED
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"
#define I2C_ADDRESS 0x3C
SSD1306AsciiWire oled;

// CO2 sensor:
SoftwareSerial SensorSerial(8,9); // RX,TX
byte cmdRead[9] = {0xFF,0x01,0x86,0x00,0x00,0x00,0x00,0x00,0x79}; 
byte cmdRange5000[9] = {0xFF,0x01,0x99,0x00,0x00,0x00,0x13,0x88,0xCB}; 

byte cmdAbcOn[9] = {0xFF,0x01,0x79,0xA0,0x00,0x00,0x00,0x00,0xE6}; 
byte cmdAbcOff[9] = {0xFF,0x01,0x79,0x00,0x00,0x00,0x00,0x00,0x86}; 
byte cmdReset[9] = {0xFF,0x01,0x8d,0x00,0x00,0x00,0x00,0x00,0x72}; 

// "\xFF\x01\x87\x00\x00\x00\x00\x00\x78"); ZERO POINT CALIBRATION
// "\xFF\x01\x79\x00\x00\x00\x00\x00\x86"); ABC logic off
// "\xFF\x01\x79\xA0\x00\x00\x00\x00\xE6"); ABC logic on
//
//calibrate = "\xFF\x01\x87\x00\x00\x00\x00\x00\x78"
//abc_on    = "\xFF\x01\x79\xA0\x00\x00\x00\x00\xE6"
//abc_off   = "\xFF\x01\x79\x00\x00\x00\x00\x00\x86"
//read      = "\xFF\x01\x86\x00\x00\x00\x00\x00\x79"


// mhzCmdCalibrateZero[]        = {0xFF,0x01,0x87,0x00,0x00,0x00,0x00,0x00,0x78};
// mhzCmdABCEnable[]            = {0xFF,0x01,0x79,0xA0,0x00,0x00,0x00,0x00,0xE6};
// mhzCmdABCDisable[]           = {0xFF,0x01,0x79,0x00,0x00,0x00,0x00,0x00,0x86};
// mhzCmdReset[]                = {0xFF,0x01,0x8d,0x00,0x00,0x00,0x00,0x00,0x72};
// mhzCmdMeasurementRange1000[] = {0xFF,0x01,0x99,0x00,0x00,0x00,0x03,0xE8,0x7B};
// mhzCmdMeasurementRange2000[] = {0xFF,0x01,0x99,0x00,0x00,0x00,0x07,0xD0,0x8F};
// mhzCmdMeasurementRange3000[] = {0xFF,0x01,0x99,0x00,0x00,0x00,0x0B,0xB8,0xA3};
// mhzCmdMeasurementRange5000[] = {0xFF,0x01,0x99,0x00,0x00,0x00,0x13,0x88,0xCB};
unsigned char response[9];

unsigned long th, tl, ppm = 0, ppm2 = 0, ppm3 = 0;

//DHT22 sensor
#include <DHT.h>
#include <DHT_U.h>
const int pinDHT = 14;
const int pinDHTpwm = 10;
DHT dht(pinDHT, DHT22);
const int stepTime = 10; //sec
int time = 0;

void setup() {
  // Serial
  delay (500);
  Serial.begin(9600);
  SensorSerial.begin(9600);
    
  delay (500);
  pinMode(pinDHTpwm, INPUT);


  // OLED
  Wire.begin();         
  oled.begin(&Adafruit128x32, I2C_ADDRESS);
  oled.set400kHz();  
  oled.setFont(ZevvPeep8x16);  
  delay (100);
  oled.clear();  
  oled.println("Initializing...");
  delay (5000);
  

  //DHT

  dht.begin();
}

long t = 0;

void loop() 
{
	//mh-z19b
	String CRCmessage = GetCOdata();

	GetPpmPwm();

	//DHT
	String DHTmessage = GetDHTdata();
	
	oled.clear();  
	oled.println(CRCmessage);
	oled.println(DHTmessage);
	
	Serial.print(CRCmessage);
	Serial.print(DHTmessage);
	
	delay(1000 * stepTime);
	t += stepTime;
	
	//if (t == 30){
	//	MHZ19Reset();
	//	delay(1000 * stepTime);
	//	t += stepTime;
	//}
}

String GetCOdata(){
	String message = "";
	
	SensorSerial.write(cmdRead, 9);
	memset(response, 0, 9);
	SensorSerial.readBytes(response, 9);
	
	//unsigned char r[9];
	//r = response;
	
    Serial.println("Response is " +String(response[0])+"-"+String(response[1])+"-"+String(response[2])+"-"+String(response[3])+"-"+String(response[4])+"-"+String(response[5])+"-"+String(response[6])+"-"+String(response[7])+"-"+String(response[8]));


	int i;
	byte crc = 0;
	for (i = 1; i < 8; i++) crc+=response[i];
	crc = 255 - crc;
	crc++;
	
  if ( !(response[0] == 0xFF && response[1] == 0x86 && response[8] == crc) ) {
    Serial.println("CRC error: " + String(crc) + " / "+ String(response[8]));
    //message = "Sensor CRC error";
    message = "No power...";
  } else {
    unsigned int responseHigh = (unsigned int) response[2];
    unsigned int responseLow = (unsigned int) response[3];
    unsigned int ppm = (256 * responseHigh) + responseLow;
	//ppm = 0.4 * ppm; // https://mysku.ru/blog/aliexpress/59397.html 
    //Serial.print(String(t)); Serial.print(","); Serial.print(ppm); Serial.println(";");
	
	unsigned long temp = response[4] - 40;
	Serial.println("CO temp: " + String(temp) + " deg");
	
    if (ppm <= 300 || ppm > 4900) {
      message = "CO2: " + String(ppm) + "(wrong)";
    } else {
      message = "CO2:" + String(ppm); 
      if (ppm < 450) {   
        message += "(fine)";
      }
      else if (ppm < 600) {   
        message += "(good)";
      }
      else if (ppm < 1000) {   
        message += "(accept)";
      }
      else if (ppm < 2500) {   
        message += "(bad)";
      }
	  else {   
        message += "(alarm)";
      }
    }
  }
	return message;
}


String GetDHTdata(){
	int h = dht.readHumidity();
	int t = dht.readTemperature();
	String message = "";
	if (isnan(h) || isnan(t)) {
		message = "Wrong DHT data";
	} else {
		message = "H: "+String(h)+"%  "+"T:"+String(t)+" C ";
	}
	return message;
}

void GetPpmPwm(){
    do {
    th = pulseIn(pinDHTpwm, HIGH, 1004000) / 1000;
    tl = 1004 - th;
    ppm2 =  2000 * (th-2)/(th+tl-4); // расчёт для диапазона от 0 до 2000ppm 
    ppm3 =  5000 * (th-2)/(th+tl-4); // расчёт для диапазона от 0 до 5000ppm 
	} while (th == 0);

  Serial.print(th);
  Serial.println(" <- Milliseconds PWM is HIGH");
  Serial.print(ppm2);
  Serial.println(" <- ppm2 (PWM) with 2000ppm as limit");
  Serial.print(ppm3);
  Serial.println(" <- ppm3 (PWM) with 5000ppm as limit");
}


void AbcOff(){
	unsigned char r[9];
	Serial.println("ABC off command... ");
	SensorSerial.write(cmdAbcOff, 9);
	memset(r, 0, 9);
	Serial.println("Response is " +String(r[0])+"-"+String(r[1])+"-"+String(r[2])+"-"+String(r[3])+"-"+String(r[4])+"-"+String(r[5])+"-"+String(r[6])+"-"+String(r[7])+"-"+String(r[8]));
}

void MHZ19Reset(){
	unsigned char r[9];
	Serial.println("MHZ19Reset command... ");
	SensorSerial.write(cmdReset, 9);
	memset(r, 0, 9);
	Serial.println("Response is " +String(r[0])+"-"+String(r[1])+"-"+String(r[2])+"-"+String(r[3])+"-"+String(r[4])+"-"+String(r[5])+"-"+String(r[6])+"-"+String(r[7])+"-"+String(r[8]));
}

void AbcOn(){
	unsigned char r[9];
	Serial.println("ABC on command... ");
	SensorSerial.write(cmdAbcOn, 9);
	memset(r, 0, 9);
	Serial.println("Response is " +String(r[0])+"-"+String(r[1])+"-"+String(r[2])+"-"+String(r[3])+"-"+String(r[4])+"-"+String(r[5])+"-"+String(r[6])+"-"+String(r[7])+"-"+String(r[8]));
}



