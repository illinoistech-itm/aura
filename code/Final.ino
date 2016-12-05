/*********************************************************************
This is an example for our nRF8001 Bluetooth Low Energy Breakout

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/1697

Adafruit invests time and resources providing this open source code, 
please support Adafruit and open-source hardware by purchasing 
products from Adafruit!

Written by Kevin Townsend/KTOWN  for Adafruit Industries.
MIT license, check LICENSE for more information
All text above, and the splash screen below must be included in any redistribution
*********************************************************************/

// This version uses the internal data queing so you can treat it like Serial (kinda)!

#include <SPI.h>
#include "Adafruit_BLE_UART.h"
#include <Wire.h>
#include "Adafruit_TCS34725.h"

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

// Connect CLK/MISO/MOSI to hardware SPI
// e.g. On UNO & compatible: CLK = 13, MISO = 12, MOSI = 11
#define ADAFRUITBLE_REQ 10
#define ADAFRUITBLE_RDY 2     // This should be an interrupt pin, on Uno thats #2 or #3
#define ADAFRUITBLE_RST 8

int redPin = 6;
int greenPin = 3;
int bluePin = 5;

int rgbLED = 4;

Adafruit_BLE_UART BTLEserial = Adafruit_BLE_UART(ADAFRUITBLE_REQ, ADAFRUITBLE_RDY, ADAFRUITBLE_RST);

String RGB = "";
String Hex = "";
char val = 'x' ;

/**************************************************************************/
/*!
    Configure the Arduino and start advertising with the radio
*/
/**************************************************************************/
void setup(void)
{ 
  Serial.begin(9600);

  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT); 
  pinMode(rgbLED, OUTPUT);
  digitalWrite(rgbLED, LOW);

  if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1); // halt!
  }
  BTLEserial.setDeviceName("AURA"); /* 7 characters max! */
  BTLEserial.begin();
}

/**************************************************************************/
/*!
    Constantly checks for new events on the nRF8001
*/
/**************************************************************************/
aci_evt_opcode_t laststatus = ACI_EVT_DISCONNECTED;

void loop()
{
  delay(50); // slow down execution to 20 times per second
  
  // Tell the nRF8001 to do whatever it should be working on.
  BTLEserial.pollACI();

  // Ask what is our current status
  aci_evt_opcode_t status = BTLEserial.getState();
  // If the status changed....
  if (status != laststatus) {
    // print it out!
    if (status == ACI_EVT_DEVICE_STARTED) {
        Serial.println(F("* Advertising started"));
    }
    if (status == ACI_EVT_CONNECTED) {
        Serial.println(F("* Connected!"));
    }
    if (status == ACI_EVT_DISCONNECTED) {
        Serial.println(F("* Disconnected or advertising timed out"));
    }
    // OK set the last status change to this one
    laststatus = status;
  }

  if (status == ACI_EVT_CONNECTED) {
    // Lets see if there's any data for us!
    // OK while we still have something to read, get a character and print it out
    while (BTLEserial.available()) {
      val = BTLEserial.read();
      if (val == 'a'){
        uint16_t clear, red, green, blue;
        digitalWrite(rgbLED, HIGH);
        delay(60);  // takes 50ms to read 
        
        tcs.getRawData(&red, &green, &blue, &clear);
        
        Serial.print("\tR:\t"); Serial.print(red);
        Serial.print("\tG:\t"); Serial.print(green);
        Serial.print("\tB:\t"); Serial.print(blue);

        digitalWrite(rgbLED, LOW);

      
        // Figure out some basic hex code for visualization
        uint32_t sum = clear;
        float r, g, b;
        r = red; r /= sum;
        g = green; g /= sum;
        b = blue; b /= sum;
        r *= 255; g *= 255; b *= 255;
        Serial.print("\t");
        //change float values to integers
        int rI = (int) r;
        int gI = (int) g;
        int bI = (int) b;
        //turn on led withhighest value
        int max=rI;
        if(rI<gI){
          max = gI;
          }
        if(bI>max){
          max=bI;
          }
        if(max==rI){
          analogWrite(redPin, 255);
        analogWrite(greenPin, 0);
        analogWrite(bluePin, 0);  
          }
        if(max==gI){
          analogWrite(redPin, 0);
        analogWrite(greenPin, 255);
        analogWrite(bluePin, 0);  
          }
        if(max==bI){
          analogWrite(redPin, 0);
        analogWrite(greenPin, 0);
        analogWrite(bluePin, 255);  
          }
        
       
        //change int to string to send data
        String rS = String(rI);
        String gS = String(gI);
        String bS = String(bI);
        sendData("R: ");
        sendData(rS);
        sendData(", G: ");
        sendData(gS);
        sendData(", B: ");
        sendData(bS);
        //send data as Hex value
        String rHex = String(rI, HEX);
        String gHex = String(gI, HEX);
        String bHex = String(bI, HEX);
        Hex = rHex + gHex + bHex;
        Serial.println(Hex);
        Serial.print((int)r, HEX); Serial.print((int)g, HEX); Serial.print((int)b, HEX);
        Serial.println();
        sendData(" Hex: ");
        sendData(Hex);
      }  
    }
  }
}

void sendData(String Send){
  uint8_t sendbuffer[20];
  Send.getBytes(sendbuffer, 20);
  char sendbuffersize = min(20, Send.length());
  // write the data
  BTLEserial.write(sendbuffer, sendbuffersize);
}


