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

#define N_MORSE  (sizeof(morsetab)/sizeof(morsetab[0]))
#define SPEED  (13)
#define txpin    (13)
#define DOTLEN  (1200/SPEED)
#define DASHLEN  (3*(1200/SPEED))


float baroPres = 101625;   //***SET BEFORE EACH PROGRAM**** current barometric pressue in Pascals(ie 1015 millibars = 101500 pascals)
float gndLevelOne;
float gndLevelTwo;
float gndLevelThree;
float gndLevelAvg;
float mainDeployAlt;
int altDrogue, altMain, altDrogueAGL, altMainAGL;
uint8_t launch = 0;
uint8_t drogueDeployed = 0;
uint8_t mainDeployed = 0;
uint8_t n = 0;  // var for end of flight counter to test if watchdog kicks in, we don't want the watchdog to reset cpu
unsigned char values[10];

struct t_mtab { char c, pat; } ;

struct t_mtab morsetab[] = {
	{'+', 42},
	{'-', 97},
	{'=', 49},
	{'.', 106},
	{',', 115},
	{'?', 76},
	{'/', 41},
	{'A', 6},
	{'B', 17},
	{'C', 21},
	{'D', 9},
	{'E', 2},
	{'F', 20},
	{'G', 11},
	{'H', 16},
	{'I', 4},
	{'J', 30},
	{'K', 13},
	{'L', 18},
	{'M', 7},
	{'N', 5},
	{'O', 15},
	{'P', 22},
	{'Q', 27},
	{'R', 10},
	{'S', 8},
	{'T', 3},
	{'U', 12},
	{'V', 24},
	{'W', 14},
	{'X', 25},
	{'Y', 29},
	{'Z', 19},
	{'1', 62},
	{'2', 60},
	{'3', 56},
	{'4', 48},
	{'5', 32},
	{'6', 33},
	{'7', 35},
	{'8', 39},
	{'9', 47},
	{'0', 63}
} ;

void setup()
{
  pinMode(CS, OUTPUT);
  pinMode(DROGUE, OUTPUT);
  pinMode(MAIN, OUTPUT);
  pinMode(LEDPIN, OUTPUT);
  //pinMode(txpin, OUTPUT);       // output for 433mhz radio signal
  
  SPI.begin();
  SPI.setDataMode(SPI_MODE3);
  Serial.begin(9600);
  digitalWrite(CS, HIGH);
  
  Serial.println("Initializing Altimeter.");
  if (!bmp.begin())
  {
    Serial.println("Could not find BMP085 sensor, check wiring!!");
    while (1) {}
  }  
  Serial.println("Altimeter ready.");
  delay(1000);
  
  Serial.println("Calculating ground level\n");
  
  gndLevelOne = currentAlt();
  delay(500);
  gndLevelTwo = currentAlt();
  delay(500);
  gndLevelThree = currentAlt();
  delay(500);
  for (int i = 0; i <= 5; i++)
  {
    gndLevelOne = (gndLevelOne + currentAlt()) /2;
    delay(500);
    gndLevelTwo = (gndLevelTwo + currentAlt()) /2;
    delay(500);
    gndLevelThree = (gndLevelThree + currentAlt()) /2;
    delay(500);
  }
  
  gndLevelAvg = (gndLevelOne + gndLevelTwo + gndLevelThree) / 3;
  
  Serial.print("GL 1 = ");
  Serial.println(gndLevelOne);
  Serial.print("GL 2 = ");
  Serial.println(gndLevelTwo);
  Serial.print("GL 3 = ");
  Serial.println(gndLevelThree);
  
  Serial.print("Ground level = ");
  Serial.print(gndLevelAvg);
  Serial.print(" feet");
  Serial.println("\n\n");
  
  mainDeployAlt = gndLevelAvg + MAIN_LEVEL;
  
  Serial.print("Main set to deploy at ");
  Serial.print(mainDeployAlt);
  Serial.print(" feet, ");
  Serial.print(MAIN_LEVEL);
  Serial.print(" feet AGL.");
  Serial.println("\n\n");
  
  /*
  Serial.println("Clearing EEPROM");
  for (int j = 0; j < EEPROM.length(); j++)
  {
	Serial.println("before eeprom write...");
    EEPROM.write(j, 0);
	Serial.println("After eeprom write: ");
	Serial.print("j is: ");
	Serial.println(j);
  }
  Serial.println("EEPROM cleared");
  */
  delay(500);
  Serial.print("Initializing Accelerometer\n");
  writeRegister(INT_MAP, 0xEF);
  writeRegister(ACT_INACT_CTL, 0x40);  //0x40 dc coupled with x-axis only participating in activity
  writeRegister(THRESH_ACT, 0x7F);   //0x3F =3.94gs  0x7f =7.94gs
  writeRegister(DATA_FORMAT, 0x01);  //0x01 = +/-4gs  0x02 = +/-8gs  0x03 = +/-16gs
  writeRegister(INT_ENABLE, 0x90);
  writeRegister(POWER_CTL, 0x08);
  readRegister(INT_SOURCE, 1, values);
  Serial.print("Accelerometer Ready!!\n");
  delay(500);
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;

  OCR1A = 31250;            // compare match register 16MHz/256/2Hz
  TCCR1B |= (1 << WGM12);   // CTC mode
  TCCR1B |= (1 << CS12);    // 256 prescaler
  TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
  interrupts();             // enable all interrupts
  attachInterrupt(0, takeOff, RISING);
  digitalWrite(LEDPIN, HIGH);
  Serial.println("All systems GO!! Stand by for liftoff!!\n");
}  // end setup()

void loop()
{	
  if((!drogueDeployed) && (launch))
  {
	TIMSK1 = 0;                // disable timer 1 interrupts
	detachInterrupt(0);        // disable interrupts from accelerometer
    digitalWrite(LEDPIN, LOW);
    Serial.print("Launch detected\n");
    detachInterrupt(0);
    delay(DELAY_PERIOD);
    digitalWrite(DROGUE, HIGH);
    delay(1000);
    digitalWrite(DROGUE, LOW);
	drogueDeployed = 1;
    altDrogue = currentAlt();
    Serial.print("Drogue deployed at ");
    Serial.print(altDrogue);
    Serial.print(" feet\n");
    altDrogueAGL = altDrogue - gndLevelAvg;
    Serial.print(altDrogueAGL);
    Serial.print(" feet AGL\n\n");
    //  ??? put accelerometer to sleep ???
    EEPROM.write(0, highByte(altDrogueAGL));
    EEPROM.write(1, lowByte(altDrogueAGL));
  }
 
  if ((currentAlt() <= mainDeployAlt) && (!mainDeployed) && (drogueDeployed))
  {
    digitalWrite(MAIN, HIGH);
    delay(1000);
    digitalWrite(MAIN, LOW);
	mainDeployed = 1;
    altMain = currentAlt();
    Serial.print("Main deployed at ");
    Serial.print(altMain);
    Serial.println(" feet");
    altMainAGL = altMain - gndLevelAvg;
    Serial.print(altMainAGL);
    Serial.println(" feet AGL");
    EEPROM.write(2, highByte(altMainAGL));
    EEPROM.write(3, lowByte(altMainAGL));
  }
  
  while(mainDeployed){
	  Serial.println("beep...");
	  delay(10000);
	  n++;
	  Serial.println(n);
	  /*
	  sendmsg("KG7CCY") ;
	  delay(2500) ;
	  */
  }
}  // end loop()

ISR(TIMER1_COMPA_vect)          // timer compare interrupt service routine
{
	digitalWrite(LEDPIN, digitalRead(LEDPIN) ^ 1);   // toggle LED pin
}

float currentAlt()
{
	return (bmp.readAltitude(baroPres) * 3.28);
}
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

void dash()
{
	digitalWrite(txpin, HIGH);
	delay(DASHLEN);
	digitalWrite(txpin, LOW);
	delay(DOTLEN);
}

void dit()
{
	digitalWrite(txpin, HIGH);
	delay(DOTLEN);
	digitalWrite(txpin, LOW);
	delay(DOTLEN);
}


void send(char c)
{
	int i ;
	if (c == ' ') {
		delay(7*DOTLEN) ;
		return ;
	}
	for (i=0; i<N_MORSE; i++) {
		if (morsetab[i].c == c) {
			unsigned char p = morsetab[i].pat ;

			while (p != 1) {
				if (p & 1)
				dash() ;
				else
				dit() ;
				p = p / 2 ;
			}
			delay(2*DOTLEN) ;
		}
	}
}


void sendmsg(char *str)
{
	while (*str)
	send(*str++) ;
}
