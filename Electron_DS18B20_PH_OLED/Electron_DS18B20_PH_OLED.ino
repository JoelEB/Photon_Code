#include "OneWire.h"
#include "spark-dallas-temperature.h"
#include "SparkFunMicroOLED.h"
#include "bitmaps.h"

#define address 99               //default I2C ID number for EZO pH Circuit.

double tempC = 0.0;
double tempF = 0.0;
//New variable added to check the Temp before assigning a new value to tempC
double tempCheck = 0.0;

int tempSensorPin = D4;

OneWire oneWire(tempSensorPin);
DallasTemperature sensors(&oneWire);

//////////////////////////////////
// MicroOLED Object Declaration //
//////////////////////////////////
// Declare a MicroOLED object. If no parameters are supplied, default pins are
// used, which will work for the Photon Micro OLED Shield (RST=D7, DC=D6, CS=A2)
MicroOLED oled;
//MicroOLED oled(MODE_I2C, D7, 0);    // Example I2C declaration RST=D7, DC=LOW (0)

FuelGauge fuel;//declare instance of Fuel Gage built into Electron

void setup()
{
    sensors.begin();
    Serial.begin(9600);

    oled.begin();
    oled.clear(PAGE);  // clear(PAGE) will clear the Arduino's display buffer.
    oled.clear(ALL);  // clear(ALL) will clear out the OLED's graphic memory.
    oled.drawBitmap(logo);
    oled.display();
    delay(4000);
}

void loop()
{
  getTemp();
  printInfo();
  delay(5000);

}

void printInfo()
{
  Serial.print("TempF: ");
  Serial.print(tempF);
  //Serial.print(", pH: ");
  //Serial.print(ph_data);
  Serial.println();

  oled.clear(PAGE);
  oled.setCursor(0,0);
  oled.print("Bat%:");
  oled.print(fuel.getSoC());

  oled.setCursor(0,10);
  oled.print("volts:");
  oled.print(fuel.getVCell());


  oled.setCursor(0,30);
  oled.print("pH:");
  oled.print("7");

  oled.setCursor(0,40);
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
