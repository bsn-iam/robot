#include <SoftwareSerial.h>
#include <Wire.h>

// I2C OLED
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"
#define I2C_ADDRESS 0x3C
SSD1306AsciiWire oled;

// CO2 sensor:
SoftwareSerial SensorSerial(8,9); // RX,TX
byte cmd[9] = {0xFF,0x01,0x86,0x00,0x00,0x00,0x00,0x00,0x79}; 
byte cmdRange[9] = {0xFF,0x01,0x99,0x00,0x00,0x00,0x13,0x88,0xCB}; 
unsigned char response[9];

unsigned long th, tl, ppm = 0, ppm2 = 0, ppm3 = 0;

//DHT22 sensor
#include <DHT.h>
#include <DHT_U.h>
const int pinDHT = 14;
const int pinDHTpwm = 10;
DHT dht(pinDHT, DHT22);

void setup() {
  // Serial
  delay (5000);
  Serial.begin(9600);
  SensorSerial.begin(9600);
    
  delay (5000);
  pinMode(pinDHTpwm, INPUT);
  
  SensorSerial.write(cmdRange, 9);
  memset(response, 0, 9);

  // OLED
  Wire.begin();         
  oled.begin(&Adafruit128x32, I2C_ADDRESS);
  oled.set400kHz();  
  oled.setFont(ZevvPeep8x16);  
  delay (1000);
  oled.clear();  
  oled.println("Initializing...");
  delay (5000);
  
  //DHT

  dht.begin();
}

long t = 0;

void loop() 
{
  SensorSerial.write(cmd, 9);
  memset(response, 0, 9);
  
  SensorSerial.readBytes(response, 9);
  int i;
  byte crc = 0;
  for (i = 1; i < 8; i++) crc+=response[i];
  crc = 255 - crc;
  crc++;

  
  String CRCmessage = "";
  
  if ( !(response[0] == 0xFF && response[1] == 0x86 && response[8] == crc) ) {
    Serial.println("CRC error: " + String(crc) + " / "+ String(response[8]));
    //CRCmessage = "Sensor CRC error";
    CRCmessage = "CRC heating...";
  } else {
    unsigned int responseHigh = (unsigned int) response[2];
    unsigned int responseLow = (unsigned int) response[3];
    unsigned int ppm = (256 * responseHigh) + responseLow;
	ppm = 0.4 * ppm; // https://mysku.ru/blog/aliexpress/59397.html 
    //Serial.print(String(t)); Serial.print(","); Serial.print(ppm); Serial.println(";");
    if (ppm <= 300 || ppm > 4900) {
      CRCmessage = "CO2: " + String(ppm) + "(wrong)";
    } else {
      CRCmessage = "CO2:" + String(ppm); 
      if (ppm < 450) {   
        CRCmessage += "(fine)";
      }
      else if (ppm < 600) {   
        CRCmessage += "(good)";
      }
      else if (ppm < 1000) {   
        CRCmessage += "(accept)";
      }
      else if (ppm < 2500) {   
        CRCmessage += "(bad)";
      }
	  else {   
        CRCmessage += "(alarm)";
      }
    }
  }

  
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
  
  
  
  
  
  
  
  //DHT
  int h = dht.readHumidity();
  int t = dht.readTemperature();
  
  String DHTmessage = "";
  
  if (isnan(h) || isnan(t)) {
	DHTmessage = "Wrong DHT data";
  } else {
    DHTmessage = "H: "+String(h)+"%  "+"T:"+String(t)+" C ";
  }
  
  oled.clear();  
  oled.println(CRCmessage);
  oled.println(DHTmessage);
  
  Serial.print(CRCmessage);
  Serial.print(DHTmessage);
  

  
  
  delay(10000);
  t += 10;
}