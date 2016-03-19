/*This program is for the pH & temp demo prototype
Currently using Photon3, which is configured for the
Openponics Network.
The pH Stamp is communicating over serial. The temp
from the DS18B20 is used to calibrate the pH reading.
Battery shield reports batt% and voltage.
OLED screen prints all four values and post status.
JSON Library sends data out for use with node-red
SparkFun Phant Library is used to send data to
data.sparkfun.com for storage and graphing:
https://data.sparkfun.com/streams/bGaw7Y4pgAHm4KQpE2O1

This code also handles the switch located on the prototype.
Switch and reset the device to disable Internet connectivity
for demoing in poor network scenarios.

########IMPORATANT: This uses an older version of the
OneWire and DallasTemp libraries. This no longer compiles
in the Build IDE due to those libraries being updated.
The new Dallas library finds sensors automatically.

Last updated 3/17/16

*/
#include "bitmaps.h"
#include "SparkFunMAX17043.h"
#include "DallasTemperature.h"
#include "SparkFunMicroOLED.h"
#include "OneWire.h"
#include "SparkJson.h"
#include "SparkFunPhant.h"

//////////////////////////
// MicroOLED Definition //
//////////////////////////
#define PIN_RESET D7  // Connect RST to pin 9 (req. for SPI and I2C)
#define PIN_DC    D3  // Connect DC to pin 8 (required for SPI)
#define PIN_CS    A2 // Connect CS to pin 10 (required for SPI)
// Also connect pin 13 to SCK and pin 11 to MOSI

#define internetInput D5
#define internetGND D6//for internet disabling switch

////////////PHANT STUFF//////////////////////////////////////////////////////////////////
const char SparkFunServer[] = "data.sparkfun.com";
const char publicKey[] = "bGaw7Y4pgAHm4KQpE2O1";
const char privateKey[] = "VpNzVJay1vUoj4X25PBY";
Phant phant(SparkFunServer, publicKey, privateKey);
/////////////////////////////////////////////////////////////////////////////////////////

//Create Instance of TCPCLient for sending the data to node-red Server
TCPClient client;
//Create the varible for the IP address commas are suppose to be there.
byte server[] = { 192, 168, 1, 141 };

//////////////////////////////////
// MicroOLED Object Declaration //
//////////////////////////////////
// Declare a MicroOLED object. The parameters include:
// 1 - Mode: Should be either MODE_SPI or MODE_I2C
// 2 - Reset pin: Any digital pin
// 3 - D/C pin: Any digital pin (SPI mode only)
// 4 - CS pin: Any digital pin (SPI mode only, 10 recommended)
MicroOLED oled(MODE_SPI, PIN_RESET, PIN_DC, PIN_CS);
//MicroOLED uOLED(MODE_I2C, PIN_RESET); // Example I2C declaration

double voltage = 0; // Variable to keep track of LiPo voltage
double soc = 0; // Variable to keep track of LiPo state-of-charge (SOC)
//bool alert; // Variable to keep track of whether alert has been triggered

// DS18B20 Thermometer Stuff
#define ONE_WIRE_BUS D4
#define TEMPERATURE_PRECISION 12
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

//Run I2C Scanner to get address of DS18B20(s)
DeviceAddress inWaterThermometer = { 0x28, 0xD4, 0x53, 0x60, 0x06, 0x00, 0x00, 0x9C };

double InTempC = -1;
float waterTempF = 0;

int count = 10000;

char ph_data[20];//we make a 20 byte character array to hold incoming data from the pH stamp.

float ph=0;//used to store the results of converting the char to float for later use.

void update18B20Temp(DeviceAddress deviceAddress, double &tempC);

void setup()
{
  //Setup for switch to turn Internet off
  pinMode(internetInput, INPUT_PULLUP);
  pinMode(internetGND, OUTPUT);
  digitalWrite(internetGND, LOW);

  if(digitalRead(internetInput) == LOW)
  {
    Spark.disconnect();
    delay(2000);
    WiFi.off();
    RGB.control(true);
    RGB.color(0, 0, 0);
  }
  else
  {
    WiFi.on();
    delay(2000);
    Spark.connect();
    RGB.control(false);
  }

  client.connect(server, 2525);

  // DS18B20 initialization
  sensors.begin();
  sensors.setResolution(inWaterThermometer, TEMPERATURE_PRECISION);

  Serial.begin(9600);
  Serial1.begin(38400);//ph Stamp serial

  oled.begin();
  	// Set up the MAX17043 LiPo fuel gauge:
  lipo.begin(); // Initialize the MAX17043 LiPo fuel gauge

	// Quick start restarts the MAX17043 in hopes of getting a more accurate
	// guess for the SOC.
  lipo.quickStart();

	// We can set an interrupt to alert when the battery SoC gets too low.
	// We can alert at anywhere between 1% - 32%:
	//lipo.setThreshold(20); // Set alert threshold to 20%.

  calibrate();//throw away start up values
  getPh();
  delay(500);
  calibrate();//throw away start up values
  getPh();

  oled.clear(PAGE);
  oled.clear(ALL);
  oled.drawBitmap(logo);
  oled.display();
  delay(3000);

    //Serial1.write("L1\r");//Uncomment to turn LEDs ON
    //delay(10);
    //Serial1.write("L0\r");//Uncomment to turn LEDs OFF
    //delay(10);
    //Serial1.print("E\r");//uncomment to stop continuous read mode
    //delay(10);
    //Serial1.print("C\r");//uncomment to start continuous read mode
    //delay(10);
}

void loop()
{
    calibrate();
    getPh();
    getLipo();
    printInfo();
    sendInfo();

    count++;
    if(count>=375)
    //375 = post every 15 mins
    //1500 should post once and hour
    {
      postToPhant();
      count = 0;
    }

    delay(2000);
}

//---------------------------------------------------------------
void printInfo()
{

      //Serial.print("pH: ");
      //Serial.println(ph);

      oled.clear(PAGE);
      oled.setCursor(0,0);
      oled.print("Bat%:");
      oled.print(soc);

      oled.setCursor(0,10);
      oled.print("volts:");
      oled.print(voltage);

      oled.setCursor(0,30);
      oled.print("pH:");
      oled.print(ph);

      oled.setCursor(0,40);
      oled.print("TempF:");
      oled.print(waterTempF);

      oled.display();
}
//---------------------------------------------------------------
void getPh()
{
  Serial1.print("R\r");
  delay(10);
  if(Serial1.available() > 0)
  {
    Serial1.readBytesUntil(13,ph_data,20);
    ph=atof(ph_data);
  }
}
//---------------------------------------------------------------
void getTemp()
{
    //get temp from DS18B20
    sensors.requestTemperatures();
    update18B20Temp(inWaterThermometer, InTempC);

    //Every so often there is an error that throws a -127.00, this compensates
    if(InTempC < -100)
      waterTempF = waterTempF;//push last value so data isn't out of scope
    else
      waterTempF = (InTempC * 9)/5 + 32;

}
//---------------------------------------------------------------
void update18B20Temp(DeviceAddress deviceAddress, double &tempC)
{
  tempC = sensors.getTempC(deviceAddress);
}
//---------------------------------------------------------------
void calibrate()
{
  getTemp();
  Serial1.print(waterTempF); //calibrate with current temp
  Serial1.write(0x0D);//carriage return
  delay(10);
}

void sendInfo()
{
//Create Json object
 //String idm = System.deviceID();
 client.connect(server, 2525);

 StaticJsonBuffer<1024> jsonBuffer;
 JsonObject& root = jsonBuffer.createObject();

 //Assign data to the JSON object
 //root["topic"] = idm;
 root["Name"] = "Sensor1";
 root["waterTempF"] = waterTempF;
 root["ph"] = ph;
 root["voltage"] = voltage;
 root["soc"] = soc;
//client.println(System.deviceID());
root.printTo(client);
client.println("/FF");
}
void getLipo()
{
	// lipo.getVoltage() returns a voltage value (e.g. 3.93)
	voltage = lipo.getVoltage();
	// lipo.getSOC() returns the estimated state of charge (e.g. 79%)
	soc = lipo.getSOC();
	// lipo.getAlert() returns a 0 or 1 (0=alert not triggered)
	//alert = lipo.getAlert();
}
//---------------------------------------------------------------
int postToPhant()
{
    phant.add("voltage", voltage);
    phant.add("soc", soc);
    phant.add("ph", ph);
    phant.add("watertempf", waterTempF);

    TCPClient client;
    char response[512];
    int i = 0;
    int retVal = 0;

    if (client.connect(SparkFunServer, 80))
    {
        Serial.println("Posting!");
        client.print(phant.post());
        delay(1000);
        while (client.available())
        {
            char c = client.read();
            //Serial.print(c);
            if (i < 512)
                response[i++] = c;
        }
        if (strstr(response, "200 OK"))
        {
            Serial.println("Post success!");
            oled.clear(PAGE);
            oled.setCursor(0,40);
            oled.print("PostGood");
            oled.display();
            retVal = 1;
        }
        else if (strstr(response, "400 Bad Request"))
        {
            Serial.println("Bad request");
            oled.clear(PAGE);
            oled.setCursor(0,40);
            oled.print("PostBad");
            oled.display();
            retVal = -1;
        }
        else
        {
            retVal = -2;
        }
    }
    else
    {
        Serial.println("connection failed");
        oled.clear(PAGE);
        oled.setCursor(0,40);
        oled.print("CnnctFail");
        oled.display();
        retVal = -3;
    }
    client.stop();
    return retVal;

}
