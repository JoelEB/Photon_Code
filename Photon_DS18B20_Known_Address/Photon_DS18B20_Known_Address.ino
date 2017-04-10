/******************************************************************************

*******************************************************************************/
#include "OneWire.h"
#include "spark-dallas-temperature.h"
//OneWire and DallasTemperature libraries are needed for DS18B20 Temp sensor

#define ONE_WIRE_BUS D4
#define TEMPERATURE_PRECISION 11
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

//Run I2C Scanner to get address of DS18B20(s)
//(found in the Firmware folder in the Photon Weather Shield Repo)
/***********REPLACE THIS ADDRESS WITH YOUR ADDRESS*************/
DeviceAddress tempSensor =
{0x28, 0xC3, 0xD1, 0x5F, 0x07, 0x00, 0x00, 0x73};//Waterproof temp sensor address
/***********REPLACE THIS ADDRESS WITH YOUR ADDRESS*************/

double InTempC = 0;//original temperature in C from DS18B20
float tempf = 0;//converted temperature in F from DS18B20

void update18B20Temp(DeviceAddress deviceAddress, double &tempC);//predeclare to compile

//---------------------------------------------------------------
void setup()
{
    // DS18B20 initialization
    sensors.begin();
    sensors.setResolution(tempSensor, TEMPERATURE_PRECISION);
    getTemp();
    Serial.begin(9600);   // open serial over USB
    getTemp();//throw away values
    // Make sure your Serial Terminal app is closed before powering your device
    // Now open your Serial Terminal, and hit any key to continue!
    Serial.println("\n\nPress any key to begin");
    //This line pauses the Serial port until a key is pressed
    while(!Serial.available()) Spark.process();

}
//---------------------------------------------------------------
void loop()
{

      getTemp();//Read the DS18B20 waterproof temp sensor
      Serial.print("Temp:");
      Serial.print(tempf);
      Serial.println("F");
      delay(2000);
}
//---------------------------------------------------------------
void update18B20Temp(DeviceAddress deviceAddress, double &tempC)
{
  tempC = sensors.getTempC(deviceAddress);
}
//---------------------------------------------------------------
void getTemp()
{
    //get temp from DS18B20
    sensors.requestTemperatures();
    update18B20Temp(tempSensor, InTempC);
    //Every so often there is an error that throws a -127.00, this compensates
    if(InTempC < -100)
      tempf = tempf;//push last value so data isn't out of scope
    else
      tempf = (InTempC * 9)/5 + 32;//else grab the newest, good data
}
