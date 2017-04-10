/*
This program is designed to help identifiy more than one
DS18B20 Dallas Temperature sensor on a particular bus. Attach
all sensors in parallel on the same pin (don't forget the 4.7Kohm
resistor). Change the ONE_WIRE_BUS variable to match the pin used.
Change the resolution if you choose.

Open a serial port @ 9600. Press any key to start the scan.
Code will scan for x number of devices and print the results
for each device attahced including a copy/pastable verision of
each devices address for use in later code.

The temps from each device are then printed in loop. This will
allow you to change the temp of one sensor, see which device temp
changes in the temrinal, and mark that device accordingly.

Test2: This also allows you to stick all sensors into a glass of water
or other known temp substance and see the accuracy of all sensors. 
*/
//#pragma SPARK_NO_PREPROCESSOR

#include "OneWire.h"
#include "spark-dallas-temperature.h"

// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS D4
#define TEMPERATURE_PRECISION 12

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

int numberOfDevices; // Number of temperature devices found

DeviceAddress tempDeviceAddress; // We'll use this variable to store a found device address

// function to print a device address
void printAddress(DeviceAddress deviceAddress)
{
//tempSensor1 = { 0x28, 0x98, 0x78, 0x5F, 0x07, 0x00, 0x00, 0x65 };
byte i;
Serial.print("0x");
Serial.print(deviceAddress[0],HEX);
for( i = 1; i < 8; i++)
  {

    Serial.print(", 0x");
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i],HEX);
  }

}

void setup(void)
{
  // start serial port
  Serial.begin(9600);
  //Serial.println("Dallas Temperature IC Control Library Demo");
  Serial.println("Press any key to begin");
  //This line pauses the Serial port until a key is pressed
  while(!Serial.available()) Spark.process();
  // Start up the library
  sensors.begin();
  delay(200);

  // Grab a count of devices on the wire
  //sensors.begin();
  numberOfDevices = sensors.getDeviceCount();
  delay(20);
  // locate devices on the bus
  Serial.print("Locating devices...");

  Serial.print("Found ");
  Serial.print(numberOfDevices, DEC);
  Serial.println(" devices.");
  delay(500);

  // report parasite power requirements
  Serial.print("Parasite power is: ");
  if (sensors.isParasitePowerMode()) Serial.println("ON");
  else Serial.println("OFF");

  // Loop through each device, print out address
  for(int i=0;i<numberOfDevices; i++)
  {
    // Search the wire for address
  if(sensors.getAddress(tempDeviceAddress, i))
	{
		Serial.print("Found device ");
		Serial.print(i, DEC);
		Serial.print(" with address: ");
		printAddress(tempDeviceAddress);
		Serial.println();

		Serial.print("Setting Resolution to ");
		Serial.println(TEMPERATURE_PRECISION, DEC);

		// set the resolution to TEMPERATURE_PRECISION bit (Each Dallas/Maxim device is capable of several different resolutions)
		sensors.setResolution(tempDeviceAddress, TEMPERATURE_PRECISION);

		 Serial.print("Reading Resolution: ");
		Serial.print(sensors.getResolution(tempDeviceAddress), DEC);
		Serial.println();
	}else{
		Serial.print("Found ghost device at ");
		Serial.print(i, DEC);
		Serial.print(" but could not detect address.");
    delay(500);
	}
  }

}

// function to print the temperature for a device
void printTemperature(DeviceAddress deviceAddress)
{
  // method 1 - slower
  //Serial.print("Temp C: ");
  //Serial.print(sensors.getTempC(deviceAddress));
  //Serial.print(" Temp F: ");
  //Serial.print(sensors.getTempF(deviceAddress)); // Makes a second call to getTempC and then converts to Fahrenheit

  // method 2 - faster
  float tempC = sensors.getTempC(deviceAddress);
  //Serial.print("Temp C: ");
  //Serial.print(tempC);
  Serial.print(DallasTemperature::toFahrenheit(tempC)); // Converts tempC to Fahrenheit
  Serial.println("F");
}

void loop(void)
{
  // call sensors.requestTemperatures() to issue a global temperature
  // request to all devices on the bus
  Serial.print("Requesting temperatures...");
  sensors.requestTemperatures(); // Send the command to get temperatures
  Serial.println("DONE");
  delay(100);

  // Loop through each device, print out temperature data
  for(int i=0;i<numberOfDevices; i++)
  {
    // Search the wire for address
    if(sensors.getAddress(tempDeviceAddress, i))
	{
		// Output the device ID
		Serial.print("Device: ");
		Serial.print(i,DEC);
    Serial.print(" ");
		// It responds almost immediately. Let's print out the data
		printTemperature(tempDeviceAddress); // Use a simple function to print out the data
	}
	//else ghost device! Check your power requirements and cabling

  }

  delay(2000);
}
