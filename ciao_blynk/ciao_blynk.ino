#include <Ciao.h>
#include <CiaoData.h>

/**************************************************************
 * Blynk is a platform with iOS and Android apps to control
 * Arduino, Raspberry Pi and the likes over the Internet.
 * You can easily build graphic interfaces for all your
 * projects by simply dragging and dropping widgets.
 *
 *   Downloads, docs, tutorials: http://www.blynk.cc
 *   Blynk community:            http://community.blynk.cc
 *   Social networks:            http://www.fb.com/blynkapp
 *                               http://twitter.com/blynk_app
 *
 * This example code is in public domain. This example is based
 * on ESP8266_http_API, it is only modified to run on UNO WiFi 
 * boards from arduino.org using the Ciao library.
 *
 **************************************************************
 * Project setup in the Blynk app:
 *  Two Value Display widget on V2 and V0
 *
 **************************************************************/
#include <Wire.h>
#define CONNECTOR     "rest" // define connector type for the Ciao Library
#define SERVER_ADDR   "46.101.143.225" // blynk-clud.com IP address
// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
char auth[] = "bddbe40a24c549b596fe9c941c684d18";
void setup() {
  Serial.begin(9600);
  delay(10);
  Ciao.begin();
  Ciao.println("WiFi connected");
}
void loop() {
  long value = millis();
  // Send value to the cloud with Ciao.write()
  // similar to Blynk.virtualWrite()
  Serial.print("Sending value: ");
  Serial.println(value);
  Ciao.print("Sending on V2: ");
  Ciao.println(value);
  String wcommand = String("/") + auth + "/update/V2?value=" + value;
  //Ciao.println(wcommand);
  Ciao.write(CONNECTOR, SERVER_ADDR, wcommand);
  // Read the value back
  // similar to Blynk.syncVirtual()
  Serial.print("Reading value: ");
  String rcommand = String("/") + auth + "/get/V2";
  CiaoData rdata = Ciao.read(CONNECTOR, SERVER_ADDR, rcommand);
  int rlenght = String(rdata.get(2)).length();
  String rvalue = String(rdata.get(2)).substring(2,rlenght-2);
  Serial.println(rvalue);
  Ciao.println("Reading V2: "+ rvalue);
  //delay(10);
  // Write the V2 value on pin D2 virtual and harware are now synced
  analogWrite(2,rvalue.toInt());
  //Serial.println(rvalue.toInt());
  // write again the value on pin V0 to experience the delay (more than 15 seconds)
  String command = String("/") + auth + "/update/V0?value=" + rvalue.toInt();
  //Ciao.println(command);
  Ciao.write(CONNECTOR, SERVER_ADDR, command);
  Ciao.print("Sent to V0: ");
  Ciao.println(rvalue.toInt());
  // For more HTTP API, see http://docs.blynkapi.apiary.io
  // Wait
  delay(50);
}
