
/*
  HecaWorld unLTD codes for IOT Autonomous Garden Kit. (c) 2017
*/

#include <Nextion.h>
#include <math.h>
float pHThreshold; //Holds the value of the initial Threshold
float TempThreshold;
float LuxThreshold;
float FinalpHThreshold; //1a2
float TempsensorValue;
float LuxsensorValue;
float farh;
float Rsensor; //Resistance of sensor in K
static float pHValue, voltage; //review and determine usage.
String ssid;
String pass;


#define pHSensorPin A0            //pH meter Analog output to Arduino Analog Input 0
#define Offset 0.00               //deviation compensate
#define LED 6                     //Use this Pin to drive the Pump.LuxsensorValue
#define TempsensorPin A1          //Temp meter Analog output to Arduino Analog Input 1
#define LuxsensorPin A2           //Lux meter Analog output to Arduino Analog Input 2
#define samplingInterval 20
#define printInterval 1000
#define ArrayLenth  40            //times of collection
int pHArray[ArrayLenth];          //Store the average value of the sensor feedback
int pHArrayIndex = 0;
const int B = 4275;               // B value of the thermistor
const int R0 = 100000;            // R0 = 100k
int Luxsensor;

SoftwareSerial nextion(10, 11);// Nextion TX to pin 2 and RX to pin 3 of Arduino
Nextion myNextion(nextion, 9600); //create a Nextion object named myNextion using the nextion serial port @ 9600bps


void setup(void)
{
  //FinalpHThreshold = 7.0;
  pHThreshold = 7.0;
  LuxThreshold = 100;
  TempThreshold = 69;
  pinMode(LED, OUTPUT);
  digitalWrite(6, LOW);
  Serial.begin(9600);
  myNextion.init(); // send the initialization commands for Page 0
}



void loop(void)
{
  ssidsetup();

}
/*if (message != "") {
  Serial.println(message);
  }*/

void ssidsetup() {
  String message = myNextion.listen(); //check for message
  if (message == "65 7 1 0 ffff ffff ffff") {
    myNextion.sendCommand("page WiFi_SSID");
    Serial.println("Setup your WiFi SSID");

    while (message =="65 7 1 0 ffff ffff ffff") {
      //Serial.println(message);
      if (message == "65 8 1 0 ffff ffff ffff") {
        myNextion.sendCommand("get SSIDinput");
        //String message = myNextion.listen();

        if (message.startsWith("70 ")) {
          message.replace("70 ", "");
          ssid = message;
          Serial.println("SSID: = ");
          Serial.println(ssid);
        }
      }

    }
  }
}



/*




  //WiFi Connection
  void WiFiConnection() {
  String message = myNextion.listen(); //check for message

  // - message for WiFi connect page from Nextion") //consider using a message from an actual button rather than a sendme command. in this case, use the message from the okay button from the previous page. This will be from the action to opened that WiFi Connect page for SSID input.
  if (message == "65 4 3 0 ffff ffff ffff")
  {
    SSIDConnect();
  }
  }

  //use the Arduino to send command to load the WiFi Page
  //use the Arduino to listen for the 'print' response from the screen
  //use the !=, && and || commands to exclude none applicable messages
  //see example below
  void SSIDConnect() {
  //include command to use arduino to send command to load the SSID input page here

  String message = myNextion.listen(); //check for message
  //  - (note it states if message is not equal to' i.e message from any other button on that page order than the okay button which should 'print' the message in the text field to Arduino, if no message is expected from this page, it could be ignored and the no-null message still allowed to ensure it captures any message from that page not excluded by the 'not equal to message.
  while (message != "")
  {
    if (message.startsWith("70 ")) {
      message.replace("70 ", "");
      ssid = message;
      Serial.println("SSID: = ");
      Serial.println(ssid);
      //load the page for the password connect command before going to the next line of code
      //delay(100); //give a little delay to allow the page to load properly.
      PWConnect();
    }
  }
  }

  //use Arduino to load the Network password page
  //repeat as above for the SSID
  //see example below

  void PWConnect() {
  //include command to use arduino to send command to load the network password input page here
  String message = myNextion.listen(); //check for message
  // - message from any other button on that page order than the okay button which should 'print' the message in the text field to Arduino
   //while (message =="")
   while (message != "")
  {
    if (message.startsWith("70 ")) {
      message.replace("70 ", "");
      ssid = message;
      Serial.println("Password: = ");
      Serial.println(pass);
      //load the page for the password connect command before going to the next line of code
      //delay(100); //give a little delay to allow the page to load properly.
      wificonnect();
    }
  }
  }

  void wificonnect(){
  //include the rest of the codes for the WiFi connection here. The above codes...
  //...will request the ssid and password from the user which will then be used
  //...in this section
  }*/

