/*
This program is designed to help identifiy more than one
DS18B20 Dallas Temperature sensor on a particular bus. Attach
all sensors in parallel on the same pin (don't forget the 4.7Kohm
resistor). Change the ONE_WIRE_BUS variable to match the pin used.


*/
#pragma SPARK_NO_PREPROCESSOR

#include "OneWire.h"
#include "spark-dallas-temperature.h"
#include "SparkFunMicroOLED.h"
#include "bitmaps.h"

// Data wire is plugged into pin D4
#define ONE_WIRE_BUS D4
#define TEMPERATURE_PRECISION 12

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

double tempC = 0.0;
double tempF = 0.0;
//New variable added to check the Temp before assigning a new value to tempC
double tempCheck = 0.0;

unsigned long lastUpdate = 0;
bool scanFlag = 0;

//////////////////////////////////
// MicroOLED Object Declaration //
//////////////////////////////////
// Declare a MicroOLED object. If no parameters are supplied, default pins are
// used, which will work for the Photon Micro OLED Shield (RST=D7, DC=D6, CS=A2)
MicroOLED oled;
//MicroOLED oled(MODE_I2C, D7, 0);    // Example I2C declaration RST=D7, DC=LOW (0)

DeviceAddress tempSensor1, tempSensor2;
//tempSensor1 = { 0x28, 0x98, 0x78, 0x5F, 0x07, 0x00, 0x00, 0x65 };
//tempSensor1 = { 0x28, 0x98, 0x78, 0x5F, 0x07, 0x00, 0x00, 0x65 };
//-----------------------------------------------------------
void printMenu(void)
{
  Serial.println("\n\nOpenponics Config");
  Serial.println("===========================");
  Serial.println("s   Scan for One Wire Devices");
  Serial.println("c   One Wire device count");
  Serial.println("a   Set Device addresses by index");
  Serial.println("g   Get Device addresses by index");
  Serial.println("r   Set Resolution");
  Serial.println("i   Get All Device Info");
  Serial.println("Send me a command letter.");
}
//-----------------------------------------------------------
void getTempByIndex()
{
  sensors.requestTemperatures();
  //This next line assigns the current temp to the 'Check variable'
  tempCheck = sensors.getTempCByIndex(0);
  //This statement checks to see if the temp is less than -50 deg C - I selected -50C as the sensor for me isn't going to get that cold
  //You can select a different minimum temp, I avoided setting it to -127 as I wasn't sure if there were rounding or decimal issues which I didn't bother checking for
  if ( tempCheck < -50 )
  {
      //The below statement leaves tempC as is if the sensor reports < -50C
      tempC = tempC;
      delay(1000);
  }
  else
  {
    tempC = tempCheck;
    tempF = tempC * 9.0 / 5.0 + 32.0;
  }
}
//-----------------------------------------------------------
void i2cScan()
{
 scanFlag = 0;
 unsigned long now = millis();

    if((now - lastUpdate) > 3000)
    {
        lastUpdate = now;
        byte i;
        byte present = 0;
        byte addr[8];

      if ( !oneWire.search(addr)) {
        Serial.println("No more addresses.");
        Serial.println();
        oneWire.reset_search();
        scanFlag = 1;
        return;
      }
      // the first ROM byte indicates which chip
      switch (addr[0]) {
        case 0x10:
          Serial.println("Chip = DS18S20");  // or old DS1820
          break;
        case 0x28:
          Serial.println("Chip = DS18B20");
          break;
        case 0x22:
          Serial.println("Chip = DS1822");
          break;
        default:
          Serial.println("Device is not a DS18x20 family device.");
          return;
      }


      Serial.print("ROM = ");
      Serial.print("0x");
        Serial.print(addr[0],HEX);
      for( i = 1; i < 8; i++) {
        Serial.print(", 0x");
        Serial.print(addr[i],HEX);
      }

      if (OneWire::crc8(addr, 7) != addr[7]) {
          Serial.println("CRC is not valid!");
          return;
      }


    Serial.println();
      oneWire.reset();

    }
}
//-----------------------------------------------------------
void deviceCount()
{
  // locate devices on the bus
  Serial.print("Locating devices...");
  Serial.print("Found ");
  Serial.print(sensors.getDeviceCount(), DEC);
  Serial.println(" devices.");
}
//-------------------------------------------
// function to print a device address
void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    // zero pad the address if necessary
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}
//-------------------------------------------
void setDeviceAddressByIndex()
{
  if (!sensors.getAddress(tempSensor1, 0)) Serial.println("Unable to find address for Device 0");
  if (!sensors.getAddress(tempSensor2, 1)) Serial.println("Unable to find address for Device 1");

  // show the addresses we found on the bus
  Serial.print("Device 0 Address: ");
  printAddress(tempSensor1);
  Serial.println();

  Serial.print("Device 1 Address: ");
  printAddress(tempSensor2);
  Serial.println();
}
//-------------------------------------------
void getDeviceAddressByIndex()
{
  int x = sensors.getDeviceCount();
  for(i=0; i <= x; i++)
  {
    // show the addresses we found on the bus
    Serial.print("Device ");
    Serial.print(i, DEC);
    Serial.prin("Address: ");
    printAddress();
    Serial.println();

    Serial.print("Device 1 Address: ");
    printAddress(tempSensor2);
    Serial.println();
  }
}
//-------------------------------------------
// function to print the temperature for a device
void printTemperature(DeviceAddress deviceAddress)
{
  float tempC = sensors.getTempC(deviceAddress);
  Serial.print("Temp C: ");
  Serial.print(tempC);
  Serial.print(" Temp F: ");
  Serial.print(DallasTemperature::toFahrenheit(tempC));
}

// function to print a device's resolution
void printResolution(DeviceAddress deviceAddress)
{
  Serial.print("Resolution: ");
  Serial.print(sensors.getResolution(deviceAddress));
  Serial.println();
}

// main function to print information about a device
void printData(DeviceAddress deviceAddress)
{
  Serial.print("Device Address: ");
  printAddress(deviceAddress);
  Serial.print(" ");
  printTemperature(deviceAddress);
  Serial.println();
}

void setResolution()
{
  sensors.setResolution(tempSensor1, TEMPERATURE_PRECISION);
  sensors.setResolution(tempSensor2, TEMPERATURE_PRECISION);

  Serial.print("Device 0 Resolution: ");
  Serial.print(sensors.getResolution(tempSensor1), DEC);
  Serial.println();

  Serial.print("Device 1 Resolution: ");
  Serial.print(sensors.getResolution(tempSensor2), DEC);
  Serial.println();
}

void getAllDeviceInfo()
{
  // call sensors.requestTemperatures() to issue a global temperature
  // request to all devices on the bus
  Serial.print("Requesting temperatures...");
  sensors.requestTemperatures();
  Serial.println("DONE");

  // print the device information
  printData(tempSensor1);
  printData(tempSensor2);
}

//-----------------------------------------------------------
void checkMenuInput()
{
  if (Serial.available())
  {
    char ch = Serial.read();
    if (ch == 's' || ch == 'S')
    {
      do{
      i2cScan();
      }while(scanFlag == 0);
    }
    else if (ch == 't' || ch == 'T')
    {
      getTemp();
      printInfo();
    }
    else if (ch == 'c' || ch == 'C')
    {
      sensors.begin();
      deviceCount();
    }
    else if (ch == 'a' || ch == 'A')
    {
      setDeviceAddressByIndex();
    }
    else if (ch == 'r' || ch == 'R')
    {
      setResolution();
    }
    else if (ch == 'i' || ch == 'I')
    {
      getAllDeviceInfo();
    }
  }
}

//-----------------------------------------------------------
void setup()
{
    sensors.begin();//initialize the DS18B20 temp sensors
    Serial.begin(9600);

    oled.begin();
    oled.clear(PAGE);  // clear(PAGE) will clear the Photon's display buffer.
    oled.clear(ALL);  // clear(ALL) will clear out the OLED's graphic memory.
    oled.drawBitmap(logo);
    oled.display();
    delay(3000);
    getTemp();//throw-away value
    printMenu();
}
//-----------------------------------------------------------
void loop()
{
  //getTemp();
  //printInfo();
  checkMenuInput();
  //delay(20);

}




/*
void setup(void)
{
  // assign address manually.  the addresses below will beed to be changed
  // to valid device addresses on your bus.  device address can be retrieved
  // by using either oneWire.search(deviceAddress) or individually via
  // sensors.getAddress(deviceAddress, index)
  //tempSensor1 = { 0x28, 0x1D, 0x39, 0x31, 0x2, 0x0, 0x0, 0xF0 };
  //tempSensor2   = { 0x28, 0x3F, 0x1C, 0x31, 0x2, 0x0, 0x0, 0x2 };

  // search for devices on the bus and assign based on an index.  ideally,
  // you would do this to initially discover addresses on the bus and then
  // use those addresses and manually assign them (see above) once you know
  // the devices on your bus (and assuming they don't change).
  //
  // method 1: by index
  if (!sensors.getAddress(tempSensor1, 0)) Serial.println("Unable to find address for Device 0");
  if (!sensors.getAddress(tempSensor2, 1)) Serial.println("Unable to find address for Device 1");

  // method 2: search()
  // search() looks for the next device. Returns 1 if a new address has been
  // returned. A zero might mean that the bus is shorted, there are no devices,
  // or you have already retrieved all of them.  It might be a good idea to
  // check the CRC to make sure you didn't get garbage.  The order is
  // deterministic. You will always get the same devices in the same order
  //
  // Must be called before search()
  //oneWire.reset_search();
  // assigns the first address found to tempSensor1
  //if (!oneWire.search(tempSensor1)) Serial.println("Unable to find address for tempSensor1");
  // assigns the seconds address found to tempSensor2
  //if (!oneWire.search(tempSensor2)) Serial.println("Unable to find address for tempSensor2");

  // show the addresses we found on the bus
  Serial.print("Device 0 Address: ");
  printAddress(tempSensor1);
  Serial.println();

  Serial.print("Device 1 Address: ");
  printAddress(tempSensor2);
  Serial.println();

  // set the resolution to 9 bit
  sensors.setResolution(tempSensor1, TEMPERATURE_PRECISION);
  sensors.setResolution(tempSensor2, TEMPERATURE_PRECISION);

  Serial.print("Device 0 Resolution: ");
  Serial.print(sensors.getResolution(tempSensor1), DEC);
  Serial.println();

  Serial.print("Device 1 Resolution: ");
  Serial.print(sensors.getResolution(tempSensor2), DEC);
  Serial.println();
}


*/
