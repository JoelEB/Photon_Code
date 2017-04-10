#pragma SPARK_NO_PREPROCESSOR

#include "OneWire.h"
#include "spark-dallas-temperature.h"
#include "SparkFunMicroOLED.h"
#include "bitmaps.h"

#define phStampAddress 99               //default I2C ID number for EZO pH Circuit.

// DS18B20 Temperature Sensor bus info
#define ONE_WIRE_BUS D4
#define TEMPERATURE_PRECISION 12

// MicroOLED Object Declaration
// Declare a MicroOLED object. If no parameters are supplied, default pins are
// used, which will work for the Photon Micro OLED Shield (RST=D7, DC=D6, CS=A2)
MicroOLED oled;
//MicroOLED oled(MODE_I2C, D7, 0);    // Example I2C declaration RST=D7, DC=LOW (0)

//FuelGauge fuel;//declare instance of Fuel Gage built into Electron

DeviceAddress tempSensor0 = {0x28, 0x98, 0x78, 0x5F, 0x07, 0x00, 0x00, 0x65};
DeviceAddress tempSensor1 = {0x28, 0x21, 0x17, 0x60, 0x07, 0x00, 0x00, 0x58};
DeviceAddress tempSensor2 = {0x28, 0xC3, 0xD1, 0x5F, 0x07, 0x00, 0x00, 0x73};

char ph_data[20];//we make a 20 byte character array to hold incoming data from the pH stamp.
float ph=0;//used to store the results of converting the char to float for later use.

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

int numberOfDevices; // Number of temperature devices found

DeviceAddress tempDeviceAddress; // We'll use this variable to store a found device address
//used to find all DS18B20 devices


//------------------------------------------------------------------------------
// function to print a device address
void printAddress(DeviceAddress deviceAddress)
{
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
//------------------------------------------------------------------------------
void getAllTempAddresses(void)
{
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

		/*Serial.print("Setting Resolution to ");
		Serial.println(TEMPERATURE_PRECISION, DEC);

		// set the resolution to TEMPERATURE_PRECISION bit (Each Dallas/Maxim device is capable of several different resolutions)
		sensors.setResolution(tempDeviceAddress, TEMPERATURE_PRECISION);

		Serial.print("Reading Resolution: ");
		Serial.print(sensors.getResolution(tempDeviceAddress), DEC);
		Serial.println();
    */
	}else{
		Serial.print("Found ghost device at ");
		Serial.print(i, DEC);
		Serial.println(" but could not detect address.");
    delay(500);
	}
  }

}
//------------------------------------------------------------------------------
void setAllDeviceResolution(void)
{
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

  // Loop through each device, print out address
  for(int i=0;i<numberOfDevices; i++)
  {
		Serial.print("Setting Resolution to ");
		Serial.println(TEMPERATURE_PRECISION, DEC);

		// set the resolution to TEMPERATURE_PRECISION bit (Each Dallas/Maxim device is capable of several different resolutions)
		sensors.setResolution(tempDeviceAddress, TEMPERATURE_PRECISION);

		Serial.print("Reading Resolution: ");
		Serial.print(sensors.getResolution(tempDeviceAddress), DEC);
		Serial.println();

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
//------------------------------------------------------------------------------
void printInfo()
{
  Serial.print("TempF: ");
  Serial.print(tempF);
  Serial.print(", pH: ");
  Serial.print(ph_data);
  Serial.println();

  oled.clear(PAGE);
  /*Electron Only
  oled.setCursor(0,0);
  oled.print("Bat%:");
  oled.print(fuel.getSoC());

  oled.setCursor(0,10);
  oled.print("volts:");
  oled.print(fuel.getVCell());
  */

  oled.setCursor(0,30);
  oled.print("pH:");
  oled.print(ph_data);

  oled.setCursor(0,40);
  oled.print("TempF:");
  oled.print(tempF);

  oled.display();
}
//-----------------------------------------------------------
void getPh()
{
  Serial1.flush();

  Serial1.print("R\r");
  delay(10);
  if(Serial1.available() > 0)
  {
    Serial1.readBytesUntil(13,ph_data,20);
    ph=atof(ph_data);
  }
}
//-----------------------------------------------------------
void responseCodes()
{
  //disable response codes
  Serial1.print("RESPONSE,0\r");
  delay(1000);

}
//---------------------------------------------------------------
void calibratePHwithTemp()
{
  //getTemp();
  //Serial1.print(waterTempF); //calibrate with current temp
  //Serial1.write(0x0D);//carriage return
  //delay(10);
}
//---------------------------------------------------------------
void calibratePHwithSolutions()
{
  //getTemp();
  //Serial1.print(waterTempF); //calibrate with current temp
  //Serial1.write(0x0D);//carriage return
  //delay(10);
}
//------------------------------------------------------------------------------
void continuousMode()
{
  //this function enables or disables continuous reading mode on
  //the pH Stamp EZO
  Serial1.print("C\r");//Enables/Disables continuous read mode
  delay(10);

  //For older versions of the stamp use the following:
  //Serial1.print("E\r");//uncomment to stop continuous read mode
  //delay(10);
  //Serial1.print("C\r");//uncomment to start continuous read mode
  //delay(10);

}
//------------------------------------------------------------------------------
void stampLEDs()
{
  //this function enables or disables the pH Stamp EZO LEDs
  Serial1.print("L\r");//Enables/Disables continuous read mode
  delay(10);

  //For older versions of the stamp use the following:
  //Serial1.write("L1\r");//Uncomment to turn LEDs ON
  //delay(10);
  //Serial1.write("L0\r");//Uncomment to turn LEDs OFF
  //delay(10);
}
//------------------------------------------------------------------------------
void getTempByIndex(int index)
{
  sensors.requestTemperatures();
  //This next line assigns the current temp to the 'Check variable'
  tempCheck = sensors.getTempCByIndex(index);
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
//------------------------------------------------------------------------------
void getTempByAddress(DeviceAddress deviceAddress)
{
  sensors.requestTemperatures();

  tempCheck = sensors.getTempC(deviceAddress);
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
//------------------------------------------------------------------------------
void printAllTemperatures()
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
    Serial.println();
	}
	//else ghost device! Check your power requirements and cabling

  }

  delay(2000);
}
//------------------------------------------------------------------------------
void switchPHtoSerial()
{
  Wire.begin();                 //enable I2C port.
  delay(1000);

  Wire.beginTransmission(phStampAddress); //call the circuit by its ID number.
  Wire.write("SERIAL,38400");        //transmit the Serial command as a string
  Wire.endTransmission();
  delay(10);

  Wire.end();

  //LED on Stamp should turn Green
}
//------------------------------------------------------------------------------
void switchPHtoI2C()
{
  Serial1.print("I2C,99\r");
  delay(100);

  //LED on Stamp should turn Blue
}
//------------------------------------------------------------------------------
void calibratePHMid()
{
  Serial1.print("Cal,mid,7.00\r");
  delay(100);
  //The LED will turn Cyan
}
//------------------------------------------------------------------------------
void calibratePHLow()
{
  Serial1.print("Cal,mid,4.00\r");
  delay(100);
  //The LED will turn Cyan
}
//------------------------------------------------------------------------------
void calibratePHHigh()
{
  Serial1.print("Cal,mid,10.00\r");
  delay(100);
  //The LED will turn Cyan
}
//------------------------------------------------------------------------------
void queryCalibration()
{
  char cal_data[20];
  Serial1.flush();

  Serial1.print("Cal,?\r");
  delay(10);
  if(Serial1.available() > 0)
  {
    Serial1.readBytesUntil(13,cal_data,20);
    for(int i=0;i<7;i++)
    {
      Serial.print(cal_data[i]);
    }
    Serial.println();
  }
}
//------------------------------------------------------------------------------
void printMenu(void)
{
  Serial.println("\r\nPress any key to begin");
  //This line pauses the Serial port until a key is pressed
  while(!Serial.available()) Spark.process();

  Serial.println("\r\n    Openponics Config");
  Serial.println("===========================");
  Serial.println("s   Print All DS18B20 Addresses");
  Serial.println("t   Print All DS18B20 Temperatures");
  Serial.println("c   Print DS18B20 Device Count");
  Serial.println("p   Print pH");
  Serial.println("0   Print Device0 Temperature");
  Serial.println("r   Set Resolution for All DS18B20 Devices");
  Serial.println("q   Query Calibration State");
  Serial.println("1   Midpoint Calibration");
  Serial.println("2   Lowpoint Calibration");
  Serial.println("3   Highpoint Calibration");
  Serial.println("\r\nSend me a command letter.");
}
//-----------------------------------------------------------
void checkMenuInput()
{
  if (Serial.available())
  {
    char ch = Serial.read();
    if (ch == 's' || ch == 'S')
    {
      //do{
      getAllTempAddresses();
      //}while(scanFlag == 0);
    }
    else if (ch == 't' || ch == 'T')
    {
      printAllTemperatures();
      //printInfo();
    }
    else if (ch == 'c' || ch == 'C')
    {
      //sensors.begin();
      deviceCount();
    }
    else if (ch == 'p' || ch == 'P')
    {
      getPh();
      printInfo();
    }
    else if (ch == '0')
    {
      getTempByIndex(0);
      printInfo();
    }
    else if (ch == 'q' || ch == 'Q')
    {
      queryCalibration();
    }
    else if (ch == '1')
    {
      calibratePHMid();
    }
    else if (ch == '2')
    {
      calibratePHLow();
    }
    else if (ch == '3')
    {
      calibratePHHigh();
    }
    else if (ch == 'r' || ch == 'R')
    {
      setAllDeviceResolution();
    }
  }
}
//---------------------------------------------------------------
void setup()
{
    sensors.begin();
    Serial.begin(115200);

    Serial1.begin(38400);//Enable serial port for PH Stamp

    oled.begin();
    oled.clear(PAGE);  // clear(PAGE) will clear the Arduino's display buffer.
    oled.clear(ALL);  // clear(ALL) will clear out the OLED's graphic memory.
    oled.drawBitmap(logo);
    oled.display();
    delay(3000);

    //switchPHtoSerial();
    //switchPHtoI2C();
    //delay(10);
    //continuousMode();
    //responseCodes();
    deviceCount();
    for(int i=0;i<3;i++)
    {
      for(int j=0;j<sensors.getDeviceCount();j++)
      {
        getTempByIndex(j);
      }
      getPh();
      delay(100);
    }
    printMenu();
}
//------------------------------------------------------------------------------
void loop()
{
   checkMenuInput();
   //getTempByIndex(0);
   //getPh();
   //printInfo();
   //delay(1000);

}
