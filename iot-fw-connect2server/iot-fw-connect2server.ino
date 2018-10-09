
/*
  IoT- Gardens, Inc.
  Arduino code for Track Your Tree Autonomous Cultivation Kit.
*/
#include <Bridge.h>
#include <Blynk.h>
#include <BlynkSimpleYun.h>
#include <SPI.h>
#include <DHT.h>
#include <Nextion.h>
#include <OneWire.h>
#include <DFRobot_EC.h>
#include <EEPROM.h>

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
DFRobot_EC ec;

#define EC_PIN A4

#define StartConvert 0
#define ReadTemperature 1
#define BLYNK_PRINT Serial
#define pHSensorPin A1            //pH meter Analog output to Arduino Analog Input 0
#define Offset 0.00               //deviation compensate
#define LED 6                     //Use this Pin to drive the Pump.
#define TempsensorPin A0          //Temp meter Analog output to Arduino Analog Input 1
#define LuxsensorPin A2           //Lux meter Analog output to Arduino Analog Input 2
#define samplingInterval 20
#define printInterval 700
#define tempSampleInterval 850
#define AnalogSampleInterval 25
#define ArrayLenth  40            //times of collection
#define DHTTYPE DHT11             // DHT 11


const byte numReadings = 20;                             //the number of sample times
byte ECsensorPin = A4;                              //EC Meter analog output,pin on analog 4
byte DS18B20_Pin = 2;                               //DS18B20 signal, pin on digital 2
//unsigned int AnalogSampleInterval=25,printInterval=700,tempSampleInterval=850;  //analog sample interval;serial print interval;temperature sample interval
unsigned int readings[numReadings];                         // the readings from the analog input
byte index = 0;                                         // the index of the current reading
unsigned long AnalogValueTotal = 0;                               // the running total
unsigned int AnalogAverage = 0,averageVoltage=0;                       // the average
unsigned long AnalogSampleTime,printTime,tempSampleTime;
float temperature,ECcurrent;
float ecValue = 25;

//Temperature chip i/o
OneWire ds(DS18B20_Pin);  // on digital pin 

// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
char auth[] = "69f2123ee5164cfe9665752a2bbe87e1";

// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "Monitored WiFi";
char pass[] = "Kmav1067$$";

int pHArray[ArrayLenth];          //Store the average value of the sensor feedback
int pHArrayIndex = 0;
const int B = 4275;               // B value of the thermistor
const int R0 = 100000;            // R0 = 100k
int Luxsensor;

SoftwareSerial nextion(10, 11);// Nextion TX11 to pin 2 and RX 11 to pin 3 of Arduino
Nextion myNextion(nextion, 9600); //create a Nextion object named myNextion using the nextion serial port @ 9600bps



void setup(void)
{
  //FinalpHThreshold = 7.0;
  ec.begin();
  pHThreshold = 7.0;
  LuxThreshold = 100;
  TempThreshold = 69;
  Serial.begin(9600);
  for (byte thisReading = 0; thisReading < numReadings; thisReading++)
    readings[thisReading] = 0;
  TempProcess(StartConvert);   //let the DS18B20 start the convert
  AnalogSampleTime=millis();
  printTime=millis();
  tempSampleTime=millis();
  pinMode(LED, OUTPUT);
  digitalWrite(6, LOW);
  myNextion.init(); // send the initialization commands for Page 0
  Blynk.begin(auth);
 }



void loop(void)
{
  publishResults();
  //SetpHThreshold();
  //TempSensor();
  LuxReading();
  pHMeasurementValue();
  Blynk.run();
  ECReading();
  //timer.run();
  //WiFiConnection();
}

void ECReading() {
  /*
   Every once in a while,sample the analog value and calculate the average.
  */
 /* if(millis()-AnalogSampleTime>=AnalogSampleInterval)  
  {
    AnalogSampleTime=millis();
     // subtract the last reading:
    AnalogValueTotal = AnalogValueTotal - readings[index];
    // read from the sensor:
    readings[index] = analogRead(ECsensorPin);
    // add the reading to the total:
    AnalogValueTotal = AnalogValueTotal + readings[index];
    // advance to the next position in the array:
    index = index + 1;
    // if we're at the end of the array...
    if (index >= numReadings)
    // ...wrap around to the beginning:
    index = 0;
    // calculate the average:
    AnalogAverage = AnalogValueTotal / numReadings;
  }
  /*
   Every once in a while,MCU read the temperature from the DS18B20 and then let the DS18B20 start the convert.
   Attention:The interval between start the convert and read the temperature should be greater than 750 millisecond,or the temperature is not accurate!
  */
   if(millis()-tempSampleTime>=tempSampleInterval) 
  {
    tempSampleTime=millis();
    temperature = TempProcess(ReadTemperature)*1.8+32;  // read the current temperature from the  DS18B20
    TempProcess(StartConvert);                   //after the reading,start the convert for next reading
    ecValue =  ec.readEC(voltage,temperature);

  }
  
   /*
   Every once in a while,print the information on the serial monitor.
  */
  if(millis()-printTime>=printInterval)
  {
    printTime=millis();
    averageVoltage=AnalogAverage*(float)5000/1024;
    //Serial.print("Analog value:");
    //Serial.print(AnalogAverage);   //analog average,from 0 to 1023
    //Serial.print("    Voltage:");
    //Serial.print(averageVoltage);  //millivolt average,from 0mv to 4995mV
    //Serial.print("mV    ");
    //Serial.print("temp:");
    //Serial.print(temperature);    //current temperature
    //Serial.print("^C     EC:");
    
    float TempCoefficient=temperature*1.8+32;    //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.0185*(fTP-25.0));
    float CoefficientVolatge=(float)averageVoltage/TempCoefficient;   
    if(CoefficientVolatge<150)//Serial.println("No solution!");   //25^C 1413us/cm<-->about 216mv  if the voltage(compensate)<150,that is <1ms/cm,out of the range
    //else if(CoefficientVolatge>3300)//Serial.println("Out of the range!");  //>20ms/cm,out of the range
    //else
    { 
      if(CoefficientVolatge<=448)ecValue=6.84*CoefficientVolatge+64.32;   //1ms/cm<EC<=3ms/cm
      else if(CoefficientVolatge<=1457)ecValue=6.98*CoefficientVolatge+127;  //3ms/cm<EC<=10ms/cm
      else ecValue=5.3*CoefficientVolatge+2278;                           //10ms/cm<EC<20ms/cm
      //ecValue/=1000;    //convert us/cm to ms/cm
      //Serial.print(ecValue,2);  //two decimal
      //Serial.println("ms/cm");
    }
  }

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



/*
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
*/


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
    myNextion.setComponentText("Temp", String(temperature));
    myNextion.setComponentText("Tempb", String(ecValue,2));
    myNextion.setComponentText("Unit", String("F "));
    myNextion.setComponentText("Unitb", String("EC"));
    delay(250);
    Blynk.virtualWrite(V1,ecValue,4);
    Blynk.virtualWrite(V3,pHValue);
    Blynk.virtualWrite(V4,LuxsensorValue);
    Blynk.virtualWrite(V5,temperature);
  }
}

/*
ch=0,let the DS18B20 start the convert;ch=1,MCU read the current temperature from the DS18B20.
*/
float TempProcess(bool ch)
{
  //returns the temperature from one DS18B20 in DEG Celsius
  static byte data[12];
  static byte addr[8];
  static float TemperatureSum;
  if(!ch){
          if ( !ds.search(addr)) {
              //Serial.println("no more sensors on chain, reset search!");
              ds.reset_search();
              return 0;
          }      
          if ( OneWire::crc8( addr, 7) != addr[7]) {
              //Serial.println("CRC is not valid!");
              return 0;
          }        
          if ( addr[0] != 0x10 && addr[0] != 0x28) {
              //Serial.print("Device is not recognized!");
              return 0;
          }      
          ds.reset();
          ds.select(addr);
          ds.write(0x44,1); // start conversion, with parasite power on at the end
  }
  else{  
          byte present = ds.reset();
          ds.select(addr);    
          ds.write(0xBE); // Read Scratchpad            
          for (int i = 0; i < 9; i++) { // we need 9 bytes
            data[i] = ds.read();
          }         
          ds.reset_search();           
          byte MSB = data[1];
          byte LSB = data[0];        
          float tempRead = ((MSB << 8) | LSB); //using two's compliment
          TemperatureSum = tempRead / 16;
    }
          return TemperatureSum;  
}

