#include "OneWire.h"
#include "spark-dallas-temperature.h"
#include "SparkFunMicroOLED.h"
#include "bitmaps.h"

double tempC = 0.0;
double tempF = 0.0;
//New variable added to check the Temp before assigning a new value to tempC
double tempCheck = 0.0;

int tempSensorPin = D4;

OneWire ds = OneWire(D4);  // on pin 4 (a 4.7K resistor is necessary)
unsigned long lastUpdate = 0;

bool scanFlag = 0;

OneWire oneWire(tempSensorPin);
DallasTemperature sensors(&oneWire);

//////////////////////////////////
// MicroOLED Object Declaration //
//////////////////////////////////
// Declare a MicroOLED object. If no parameters are supplied, default pins are
// used, which will work for the Photon Micro OLED Shield (RST=D7, DC=D6, CS=A2)
MicroOLED oled;
//MicroOLED oled(MODE_I2C, D7, 0);    // Example I2C declaration RST=D7, DC=LOW (0)

//FuelGauge fuel;//declare instance of Fuel Gage built into Electron

void setup()
{
    sensors.begin();
    Serial.begin(9600);

    oled.begin();
    oled.clear(PAGE);  // clear(PAGE) will clear the Arduino's display buffer.
    oled.clear(ALL);  // clear(ALL) will clear out the OLED's graphic memory.
    oled.drawBitmap(logo);
    oled.display();
    delay(3000);

    printMenu();
}

void loop()
{
  //getTemp();
  //printInfo();
  checkInput();
  //delay(20);

}

void printInfo()
{
  Serial.print("TempF: ");
  Serial.print(tempF);
  Serial.println();

  oled.clear(PAGE);
  oled.setCursor(0,0);
  oled.print("TempF:");
  oled.print(tempF);

  oled.display();
}

void getTemp()
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

      if ( !ds.search(addr)) {
        Serial.println("No more addresses.");
        Serial.println();
        ds.reset_search();
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
      ds.reset();

    }
}
//-----------------------------------------------------------
void printMenu()
{
  Serial.println("\n\nOpenponics Config");
  Serial.println("===========================");
  Serial.println("s   Scan for One Wire Devices");
  Serial.println("t   Print Temperature");
  Serial.println("Send me a command letter.");
}
//-----------------------------------------------------------
void checkInput()
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
    else if (ch == '+')
    {

    }
  }
}
