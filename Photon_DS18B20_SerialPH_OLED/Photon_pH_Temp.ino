#include "SparkFunMicroOLED.h"
#include "OneWire.h"
#include "DallasTemperature.h"
#include "bitmaps.h"
//////////////////////////
// MicroOLED Definition //
//////////////////////////
#define PIN_RESET D7  // Connect RST to pin 9 (req. for SPI and I2C)
#define PIN_DC    D3  // Connect DC to pin 8 (required for SPI)
#define PIN_CS    A2 // Connect CS to pin 10 (required for SPI)
// Also connect pin 13 to SCK and pin 11 to MOSI

#define internetInput D5
#define internetGND D6

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


// DS18B20 Thermometer Stuff
#define ONE_WIRE_BUS D0
#define TEMPERATURE_PRECISION 12
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

//Run I2C Scanner to get address of DS18B20(s)
DeviceAddress inWaterThermometer = { 0x28, 0xD4, 0x53, 0x60, 0x06, 0x00, 0x00, 0x9C };

double InTempC = -1;
float waterTempF = 0;


char ph_data[20];//we make a 20 byte character array to hold incoming data from the pH stamp.

float ph=0;//used to store the results of converting the char to float for later use.

void update18B20Temp(DeviceAddress deviceAddress, double &tempC);
void printInfo();
void getPh();
void getTemp();
void calibrate();


SYSTEM_MODE(SEMI_AUTOMATIC);//set to semi-auto to disable WiFi for demos

void setup()
{
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

  // DS18B20 initialization
  sensors.begin();
  sensors.setResolution(inWaterThermometer, TEMPERATURE_PRECISION);

  Serial.begin(9600);
  Serial1.begin(38400);

  oled.begin();
  // clear(ALL) will clear out the OLED's graphic memory.
  // clear(PAGE) will clear the Arduino's display buffer.
  oled.clear(PAGE);
  oled.clear(ALL);

  oled.drawBitmap(logo);
  oled.display();

  calibrate();//throw away values
  getPh();

  delay(2000);

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
    printInfo();
    delay(500);
}

//---------------------------------------------------------------
void printInfo()
{
    if(waterTempF > 90.00)
    {
      oled.clear(PAGE);
      oled.setCursor(0,0);
      oled.drawBitmap(bender);
      oled.display();
    }
    else
    {
      Serial.print("pH: ");
      Serial.println(ph);

      oled.clear(PAGE);
      oled.setCursor(0,30);
      oled.print("pH:");
      oled.print(ph);

      oled.setCursor(0,40);
      oled.print("TempF:");
      oled.print(waterTempF);

      oled.display();
    }
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
