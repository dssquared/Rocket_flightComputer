/*
***  Flight computer with accelerometer interupt   ***
*/

#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <EEPROM.h>
#include <SPI.h>

Adafruit_BMP085 bmp;

#define MAIN_LEVEL 1000.0   //***SET BEFORE EACH PROGRAM*** AGL in feet you want main to deploy
#define DELAY_PERIOD 15000  //***SET BEFORE EACH PROGRAM*** delay from ignition to drogue deploy in miliseconds
#define DROGUE 5            //output pin for ejection charge
#define MAIN 6              //output pin for ejection charge
#define THRESH_ACT 0x24
#define ACT_INACT_CTL 0x27
#define POWER_CTL 0x2D
#define INT_ENABLE 0x2E
#define INT_MAP 0x2F
#define INT_SOURCE 0x30
#define DATA_FORMAT 0x31
#define DATAX0 0x32
#define DATAX1 0x33
#define DATAY0 0x34
#define DATAY1 0x35
#define DATAZ0 0x36
#define DATAZ1 0x37
#define LEDPIN 3    //indicator LED
#define CS 10       //chip select pin for SPI

float baroPres = 101480;   //***SET BEFORE EACH PROGRAM**** current barometric pressue in Pascals(ie 1015 millibars = 101500 pascals)
float gndLevelOne;
float gndLevelTwo;
float gndLevelThree;
float gndLevelAvg;
float mainDeployAlt;
int altDrogue, altMain, altDrogueAGL, altMainAGL;
uint8_t i = 0;
uint8_t launch = 0;
uint8_t drogueDeployed = 0;
uint8_t mainDeployed = 0;
unsigned char values[10];

void setup()
{
  pinMode(CS, OUTPUT);
  pinMode(DROGUE, OUTPUT);
  pinMode(MAIN, OUTPUT);
  pinMode(LEDPIN, OUTPUT);
  
  SPI.begin();
  SPI.setDataMode(SPI_MODE3);
  //Serial.begin(9600);
  digitalWrite(CS, HIGH);
  /*
  Serial.println("Initializing Altimeter.");
  if (!bmp.begin())
  {
    Serial.println("Could not find BMP085 sensor, check wiring!!");
    while (1) {}
  }  
  Serial.println("Altimeter ready.");
  delay(1000);
  
  Serial.println("Calculating ground level\n");
  */
  gndLevelOne = (bmp.readAltitude(baroPres) * 3.28);
  delay(500);
  gndLevelTwo = (bmp.readAltitude(baroPres) * 3.28);
  delay(500);
  gndLevelThree = (bmp.readAltitude(baroPres) * 3.28);
  delay(500);
  for (i = 0; i <= 5; i++)
  {
    gndLevelOne = (gndLevelOne + (bmp.readAltitude(baroPres) * 3.28)) /2;
    delay(500);
    gndLevelTwo = (gndLevelTwo + (bmp.readAltitude(baroPres) * 3.28)) /2;
    delay(500);
    gndLevelThree = (gndLevelThree + (bmp.readAltitude(baroPres) * 3.28)) /2;
    delay(500);
  }
  
  gndLevelAvg = (gndLevelOne + gndLevelTwo + gndLevelThree) / 3;
  /*
  Serial.print("Ground level = ");
  Serial.print(gndLevelAvg);
  Serial.print(" feet");
  Serial.println("\n\n");
  */
  mainDeployAlt = gndLevelAvg + MAIN_LEVEL;
  /*
  Serial.print("Main set to deploy at ");
  Serial.print(mainDeployAlt);
  Serial.print(" feet, ");
  Serial.print(MAIN_LEVEL);
  Serial.print(" feet AGL.");
  Serial.println("\n\n");
  */
  
  //Serial.println("Clearing EEPROM");
  for (i = 0; i < 1024; i++)
  {
    EEPROM.write(i, 0);
  }
  //Serial.println("EEPROM cleared");
  //delay(500);
  //Serial.print("Initializing Accelerometer\n");
  writeRegister(INT_MAP, 0xEF);
  writeRegister(ACT_INACT_CTL, 0x40);  //0x40 dc coupled with x-axis only participating in activity
  writeRegister(THRESH_ACT, 0x7F);   //0x3F =3.94gs  0x7f =7.94gs
  writeRegister(DATA_FORMAT, 0x01);  //0x01 = +/-4gs  0x02 = +/-8gs  0x03 = +/-16gs
  writeRegister(INT_ENABLE, 0x90);
  writeRegister(POWER_CTL, 0x08);
  readRegister(INT_SOURCE, 1, values);
  //Serial.print("Accelerometer Ready!!\n");
  //delay(500);
  attachInterrupt(0, takeOff, RISING);
  digitalWrite(LEDPIN, HIGH);
  //Serial.println("All systems GO!! Stand by for liftoff!!\n");
}  // end setup()

void loop()
{	
  if((!drogueDeployed) && (launch))
  {
    digitalWrite(LEDPIN, LOW);
    //Serial.print("Launch detected\n");
    detachInterrupt(0);
    delay(DELAY_PERIOD);
    digitalWrite(DROGUE, HIGH);
    delay(1000);
    digitalWrite(DROGUE, LOW);
	drogueDeployed = 1;
    altDrogue = (bmp.readAltitude(baroPres) * 3.28);
    //Serial.print("Drogue deployed at ");
    //Serial.print(altDrogue);
    //Serial.print(" feet\n");
    altDrogueAGL = altDrogue - gndLevelAvg;
    //Serial.print(altDrogueAGL);
    //Serial.print(" feet AGL\n\n");
    //  ??? put accelerometer to sleep ???
    EEPROM.write(0, highByte(altDrogueAGL));
    EEPROM.write(1, lowByte(altDrogueAGL));
  }
 
  if (((bmp.readAltitude(baroPres) * 3.28) <= mainDeployAlt)  && (!mainDeployed)  && (drogueDeployed))
  {
    digitalWrite(MAIN, HIGH);
    delay(1000);
    digitalWrite(MAIN, LOW);
	mainDeployed = 1;
    altMain = (bmp.readAltitude(baroPres) * 3.28);
    //Serial.print("Main deployed at ");
    //Serial.print(altMain);
    //Serial.println(" feet");
    altMainAGL = altMain - gndLevelAvg;
    //Serial.print(altMainAGL);
    //Serial.print(" feet AGL");
    EEPROM.write(2, highByte(altMainAGL));
    EEPROM.write(3, lowByte(altMainAGL));
  }
}  // end loop()

void writeRegister(char registerAddress, char value)
{
  digitalWrite(CS, LOW);
  SPI.transfer(registerAddress);
  SPI.transfer(value);
  digitalWrite(CS, HIGH);
}

void readRegister(char registerAddress, int numBytes, unsigned char *values)
{
  char address = 0x80 | registerAddress;
  if(numBytes > 1)address = address | 0x40;
  
  digitalWrite(CS, LOW);
  SPI.transfer(address);
  for (int i = 0; i < numBytes; i++)
  {
    values[i] = SPI.transfer(0x00);
  }
  digitalWrite(CS, HIGH);
}

void takeOff(void)
{
  readRegister(INT_SOURCE, 1, values);
  launch = 1;
}
