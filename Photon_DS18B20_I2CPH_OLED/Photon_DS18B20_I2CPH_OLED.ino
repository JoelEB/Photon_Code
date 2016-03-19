#include "OneWire.h"
#include "spark-dallas-temperature.h"
#include "SparkFunMicroOLED.h"
#include "bitmaps.h"

#define address 99               //default I2C ID number for EZO pH Circuit.

char computerdata[20];           //we make a 20 byte character array to hold incoming data from a pc/mac/other.
byte received_from_computer=0;   //we need to know how many characters have been received.
byte serial_event=0;             //a flag to signal when data has been recived from the pc/mac/other.
byte code=0;                     //used to hold the I2C response code.
char ph_data[20];                //we make a 20 byte character array to hold incoming data from the pH circuit.
byte in_char=0;                  //used as a 1 byte buffer to store in bound bytes from the pH Circuit.
byte i=0;                        //counter used for ph_data array.
int time_=1800;                   //used to change the delay needed depending on the command sent to the EZO Class pH Circuit.
float ph_float;                  //float var used to hold the float value of the pH.

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

//FuelGauge fuel;//declare instance of Fuel Gage built into Electron

void setup()
{
    sensors.begin();
    Serial.begin(9600);
    Wire.begin();                 //enable I2C port.

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
  getPh();
  printInfo();
  delay(2000);

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
  oled.print(ph_data);

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
void getPh()
{
  Wire.beginTransmission(address); //call the circuit by its ID number.
  Wire.write('r');        //transmit the command that was sent through the serial port.
  Wire.endTransmission();          //end the I2C data transmission.
  delay(1800);                    //wait the correct amount of time for the circuit to complete its instruction.
  Wire.requestFrom(address,20,1); //call the circuit and request 20 bytes (this may be more then we need).

  code=Wire.read();               //the first byte is the response code, we read this separately.

  while(Wire.available())
  {
   in_char = Wire.read();           //receive a byte.
   ph_data[i]= in_char;             //load this byte into our array.
   i+=1;                            //incur the counter for the array element.
    if(in_char==0)
    {                 //if we see that we have been sent a null command.
        i=0;                        //reset the counter i to 0.
        Wire.endTransmission();     //end the I2C data transmission.
        break;                      //exit the while loop.
    }
  }

}
