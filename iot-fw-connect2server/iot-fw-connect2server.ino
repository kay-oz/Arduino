
/*
  HecaWorld unLTD in collaboration with IoT- Gardens, Inc.
  Arduino code for Track My Tree Autonomous Cultivation Kit.
*/
#include <Bridge.h>
#include <Blynk.h>
#include <BlynkSimpleYun.h>
#include <SPI.h>
#include <DHT.h>
#include <Nextion.h>
float pHThreshold; //Holds the value of the initial Threshold
float FinalpHThreshold;
float LuxThreshold;
float TempThreshold;
float LuxsensorValue;
float TempsensorValue;
float farh;
float h;
float t;
float Rsensor; //Resistance of sensor in K
static float pHValue, voltage; //review and determine usage.

#define BLYNK_PRINT Serial
#define DHTPIN 3                  // What digital pin we're connected to
#define pHSensorPin A1            //pH meter Analog output to Arduino Analog Input 0
#define Offset 0.00               //deviation compensate
#define LED 6                     //Use this Pin to drive the Pump.
#define TempsensorPin A0          //Temp meter Analog output to Arduino Analog Input 1
#define LuxsensorPin A2           //Lux meter Analog output to Arduino Analog Input 2
#define samplingInterval 20
#define printInterval 1000
#define ArrayLenth  40            //times of collection
#define DHTTYPE DHT11             // DHT 11


// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
char auth[] = "INSERT YOUR BLYNK AUTH TOKEN";

// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "AP_NAME";
char pass[] = "PASSWORD";

int pHArray[ArrayLenth];          //Store the average value of the sensor feedback
int pHArrayIndex = 0;
const int B = 4275;               // B value of the thermistor
const int R0 = 100000;            // R0 = 100k
int Luxsensor;

SoftwareSerial nextion(10, 11);// Nextion TX11 to pin 2 and RX 11 to pin 3 of Arduino
Nextion myNextion(nextion, 9600); //create a Nextion object named myNextion using the nextion serial port @ 9600bps

DHT dht(DHTPIN, DHTTYPE);


void sendSensor()
{
  float h = dht.readHumidity();
  float t = dht.readTemperature(); // or dht.readTemperature(true) for Fahrenheit

  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }
  // You can send any value at any time.
  // Please don't send more that 10 values per second.
 }


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
  Blynk.begin(auth, "iot-gardens.blynk.cc");
 }



void loop(void)
{
  publishResults();
  SetpHThreshold();
  TempSensor();
  LuxReading();
  pHMeasurementValue();
  Blynk.run();
  //timer.run();
  //WiFiConnection();
}



void pHMeasurementValue() {
  static unsigned long samplingTime = millis();
  static unsigned long printTime = millis();
  //static float pHValue, voltage;
  if (millis() - samplingTime > samplingInterval)
  {
    pHArray[pHArrayIndex++] = analogRead(pHSensorPin);
    if (pHArrayIndex == ArrayLenth)pHArrayIndex = 0;
    voltage = avergearray(pHArray, ArrayLenth) * 5.0 / 1024;
    pHValue = 3.5 * voltage + Offset;
    samplingTime = millis();
  }
  if (millis() - printTime > printInterval)  //Every 800 milliseconds, print a numerical, convert the state of the LED indicator
  {

    printTime = millis();
  }
}
double avergearray(int* arr, int number) {
  int i;
  int max, min;
  double avg;
  long amount = 0;
  if (number <= 0) {
    //Serial.println("Error number for the array to avraging!/n");
    return 0;
  }
  if (number < 5) { //less than 5, calculated directly statistics
    for (i = 0; i < number; i++) {
      amount += arr[i];
    }
    avg = amount / number;
    return avg;
  } else {
    if (arr[0] < arr[1]) {
      min = arr[0]; max = arr[1];
    }
    else {
      min = arr[1]; max = arr[0];
    }
    for (i = 2; i < number; i++) {
      if (arr[i] < min) {
        amount += min;      //arr<min
        min = arr[i];
      } else {
        if (arr[i] > max) {
          amount += max;  //arr>max
          max = arr[i];
        } else {
          amount += arr[i]; //min<=arr<=max
        }
      }//if
    }//for
    avg = (double)amount / (number - 2);
  }//if
  return avg;
}




//Set pH Threshold.

void SetpHThreshold()
{
  String message = myNextion.listen(); //check for message

  if ((message == "65 6 1 0 ffff ffff ffff") && pHThreshold <= 10)
  {
    
      //Serial.println("pH Up Button Pressed");
      pHThreshold = pHThreshold + 0.1; //this increments the Threshold by 0.1
      //Serial.println (pHThreshold); //this prints the Threshold value
      myNextion.setComponentText("pHThresh", String(pHThreshold));

      //digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
    }
    else if ((message == "65 6 2 0 ffff ffff ffff") && pHThreshold >= 0.1)
    {
      //Serial.println("pH Down Button Pressed");
      pHThreshold = pHThreshold - 0.1; //this decrements the Threshold by 0.1
      //Serial.println (pHThreshold); //this prints the Threshold value
      myNextion.setComponentText("pHThresh", String(pHThreshold));
      //Serial.println(pHValue);
    
  }
  if ((message == "65 6 5 0 ffff ffff ffff")||(message == "65 6 5 1 ffff ffff ffff")||(message == "6"))//on page load or press/release event, do the below event.
  {
    //Serial.println ("pH page loaded - page 3"); //this prints the Threshold value
    myNextion.setComponentText("pHThresh", String(FinalpHThreshold));

  }
  if ((message == "65 6 3 1 ffff ffff ffff")||(message == "65 6 3 0 ffff ffff ffff"))
  {
    FinalpHThreshold = pHThreshold;
    //Serial.println(FinalpHThreshold);

  }
  if ((pHValue) > FinalpHThreshold) // if the measured pH value falls below exceeds your set pH Threshold, the Pump is turned on.
  {
    digitalWrite(LED, HIGH); // turn on the led
  }
  else // if that doesn't happen, then turn the led off
  {
    digitalWrite (LED, LOW);
  }
}




//Temperature Sensor Reading

void TempSensor() {
 /* int a = analogRead(TempsensorPin);
  float R = 1023.0/((float)a)-1.0;
  R = 100000.0*R;;*/

  TempsensorValue= dht.readTemperature(); //convert to temperature via datasheet ;
  farh = dht.readTemperature(true);
  h = dht.readHumidity();

}


void LuxReading() {
  LuxsensorValue = analogRead(LuxsensorPin);
  Luxsensor = (float)(1023 - LuxsensorValue) * 10 / LuxsensorValue;
  //Serial.println(Luxsensor);
  //Serial.println(LuxsensorValue);
}


void publishResults() {
  String message = myNextion.listen(); //check for message

  //Publish results/Display items on the home page

  if (message == "") {   //if there is no message, publish results on the home page
    myNextion.setComponentText("Lux", String(LuxsensorValue));
    myNextion.setComponentText("Luxb", String(LuxsensorValue));
    myNextion.setComponentText("pH", String(pHValue));
    myNextion.setComponentText("pHb", String(pHValue));
    myNextion.setComponentText("Temp", String(farh));
    myNextion.setComponentText("Tempb", String(TempsensorValue));
    myNextion.setComponentText("Unit", String("F"));
    myNextion.setComponentText("Unitb", String("C"));
    delay(250);
    Blynk.virtualWrite(V2,h);
    Blynk.virtualWrite(V1,farh);
    Blynk.virtualWrite(V3,pHValue);
    Blynk.virtualWrite(V4,LuxsensorValue);
  }
}




/*

//WiFi Connection
void WiFiConnection() {
  String message = myNextion.listen(); //check for message

  // - message for WiFi connect page from Nextion") //consider using a message from an actual button rather than a sendme command. in this case, use the message from the okay button from the previous page. This will be from the action to opened that WiFi Connect page for SSID input.
  if (message == "65 7 1 0 ffff ffff ffff")
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
  //  - (note is states if message is not equal to' i.e message from any other button on that page order than the okay button which should 'print' the message in the text field to Arduino, if no message is expected from this page, it could be ignored and the no-null message still allowed to ensure it captures any message from that page not excluded by the 'not equal to message.
  if (message != "65 7 1 0 ffff ffff ffff" && message != "65 4 3 0 ffff ffff ffff" && message != "65 7 3 0 ffff ffff ffff" && message != "7" && message != "") {
    ssid = message;
    Serial.println("SSID: = ");
    Serial.println(ssid);
    //load the page for the password connect command before going to the next line of code
    //delay(100); //give a little delay to allow the page to load properly.
    PWConnect();
  }
}

//use Arduino to load the Network password page
//repeat as above for the SSID
//see example below

void PWConnect() {
  //include command to use arduino to send command to load the network password input page here
  String message = myNextion.listen(); //check for message
  // - message from any other button on that page order than the okay button which should 'print' the message in the text field to Arduino
  if (message != "65 8 1 0 ffff ffff ffff" && message != "65 8 4 0 ffff ffff ffff" && message != "65 8 3 0 ffff ffff ffff" && message != "8" && message != "")
  {
    pass = message;
    Serial.println("Network Password: = ");
    Serial.println(pass);
    //WiFiCompleteConnect(); //Use this loop to enter the rest of the codes to complete the WiFi connection.
  }
}
*/
