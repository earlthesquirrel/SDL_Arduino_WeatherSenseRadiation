// SDL_Arduino_WeatherSenseRadiation
// SwitchDoc Labs Radiation Detector

#define TXDEBUG
//#undef TXDEBUG
#include <JeeLib.h>

#include "MemoryFree.h"
#include "DFRobot_Geiger.h"
#include <EEPROM.h>

// WeatherSenseProtocol of 8 is SolarMAX LiPo   BatV < 7V
// WeatherSenseProtocol of 10 is SolarMAX LeadAcid   BatV > 7V LoRa version
// WeatherSenseProtocol of 11 is SolarMAX4 LeadAcid BatV > 7V
// WeatherSenseProtocol of 15 is WeatherSense AQI 433MHz
// WeatherSenseProtocol of 16 is WeatherSense ThunderBoard 433MHz
// WeatherSenseProtocol of 19 is WeatherSense Radiation
#define WEATHERSENSEPROTOCOL 19

// Software version
#define SOFTWAREVERSION 5

// unique ID of this WeatherSenseRadiation system - change if you have multiple WeatherSenseRadiation systems
#define WEATHERSENESTHBID 1
// Which WeatherSense TRadiation Protocol Version
#define WEATHERSENSEPROTOCOLVERSION 1

#define LED 13

// Device ID is changed if you have more than one WeatherSense Radiation Monitor in the area
// Number of milliseconds between wake up  30 seconds.   - if you move this over 60000 ms,
// you will need to add the watchdog in the sleep loop - see SDL_Arduino_WeatherSenseAQI ResetWatchDog
// Every 120 wakeups send packet (~60 minutes) keepalivemessage
#define SLEEPCYCLE 30000
#define WAKEUPS 5

#include "Crc16.h"
//Crc 16 library (XModem)
Crc16 crc;

ISR(WDT_vect) {
  Sleepy::watchdogEvent();
}

#include <RH_ASK.h>
#include <avr/sleep.h>
#include <avr/power.h>

// Other Pins
#define WATCHDOG_1 5
#define TXPIN 8
#define RXPIN 9
#define INT1_PIN 2  //Arduino pin connected to the INT1 pin of the D7S sensor

// Radiation Outputs
volatile long CPM = 0;
volatile long nSVh = 0;
volatile float uSVh = 0.0;

// Radio Head driver set up -- for 433Mhz communication.
RH_ASK driver(2000, RXPIN, TXPIN);

unsigned long MessageCount = 0;

#include "avr/pgmspace.h"
#include <Time.h>
#include <TimeLib.h>
#include <Wire.h>

typedef enum {

  NO_INTERRUPT,
  IGNORE_INTERRUPT,
  SLEEP_INTERRUPT,
  ALARM_INTERRUPT,
  REBOOT
} wakestate;


byte byteBuffer[100];  // contains string to be sent to RX unit

// State Variables
long TimeStamp;

// State Status
byte AuxA;
byte SoftwareVersion;

// AuxA has state information
// coded in the byte
// 0000DCBA

// A = 1, Radiation Present, 0 not present
// B = 0, IN3221 (Solar) Not Present
// C = 0, Was for low battery
// D = 1, I'm alive message

wakestate wakeState;  // who woke us up?

long nextSleepLength;

int convert4ByteLongVariables(int bufferCount, long myVariable) {

  int i;
  union {
    long a;
    unsigned char bytes[4];
  } thing;
  thing.a = myVariable;

  for (i = 0; i < 4; i++) {
    byteBuffer[bufferCount] = thing.bytes[i];
    bufferCount++;
  }
  return bufferCount;
}

int convert4ByteFloatVariables(int bufferCount, float myVariable) {
  int i;

  union {
    float a;
    unsigned char bytes[4];
  } thing;
  thing.a = myVariable;

  for (i = 0; i < 4; i++) {
    byteBuffer[bufferCount] = thing.bytes[i];
    bufferCount++;
  }

  return bufferCount;
}


int convert2ByteVariables(int bufferCount, int myVariable)
 {
  union {
    int a;
    unsigned char bytes[2];
  } thing;

  thing.a = myVariable;

  byteBuffer[bufferCount] = thing.bytes[0];
  bufferCount++;
  byteBuffer[bufferCount] = thing.bytes[1];
  bufferCount++;

  return bufferCount;
}


int convert1ByteVariables(int bufferCount, int myVariable) 
{
  byteBuffer[bufferCount] = (byte)myVariable;
  bufferCount++;
  return bufferCount;
}


int checkSum(int bufferCount) 
{
  unsigned short checksumValue;
  // calculate checksum
  checksumValue = crc.XModemCrc(byteBuffer, 0, 59);
//#if defined(TXDEBUG)
//  Serial.print(F("crc = 0x"));
//  Serial.println(checksumValue, HEX);
//#endif

  byteBuffer[bufferCount] = checksumValue >> 8;
  bufferCount++;
  byteBuffer[bufferCount] = checksumValue & 0xFF;
  bufferCount++;

  return bufferCount;
}


int buildProtocolMessage()
{
  int bufferCount = 0;
  bufferCount = convert4ByteLongVariables(bufferCount, MessageCount);

  byteBuffer[bufferCount++] = WEATHERSENESTHBID;  // WeatherSenseRadiation unique ID
  byteBuffer[bufferCount++] = WEATHERSENSEPROTOCOL;  // Type of WeatherSense System
  byteBuffer[bufferCount++] = WEATHERSENSEPROTOCOLVERSION;  // WeatherSense Radiation protocol version

  bufferCount = convert4ByteLongVariables(bufferCount, CPM);
  bufferCount = convert4ByteLongVariables(bufferCount, nSVh);
  bufferCount = convert4ByteFloatVariables(bufferCount, uSVh);

  // Since we're running on wall power, we no longer need to provide Solar and Battery data.  
  // So we'll send 0's
  bufferCount = convert4ByteFloatVariables(bufferCount, 0);  
  bufferCount = convert4ByteFloatVariables(bufferCount, 0);
  bufferCount = convert4ByteFloatVariables(bufferCount, 0);
  bufferCount = convert4ByteFloatVariables(bufferCount, 0);
  bufferCount = convert4ByteFloatVariables(bufferCount, 0);
  bufferCount = convert4ByteFloatVariables(bufferCount, 0);

  byteBuffer[bufferCount++] = AuxA;  // Aux
  byteBuffer[bufferCount++] = SOFTWAREVERSION;

  return bufferCount;
}


void return2Digits(char returnString[], char *buffer2, int digits) 
{
  if (digits < 10)
    sprintf(returnString, "0%i", digits);
  else
    sprintf(returnString, "%i", digits);

  strcpy(returnString, buffer2);
}


void ResetWatchdog() 
{
  digitalWrite(WATCHDOG_1, LOW);
  delay(200);
  digitalWrite(WATCHDOG_1, HIGH);
}

DFRobot_Geiger geiger(INT1_PIN);
int state = 0;
unsigned long wakeCount;


void sendMessage() 
{
#if defined(TXDEBUG)
  Serial.println(F("###############"));
  Serial.print(F(" MessageCount="));
  Serial.println(MessageCount);
  Serial.print(F(" STATUS - WeatherSenseProtocol:"));
  Serial.println(WEATHERSENSEPROTOCOL);
  Serial.print(F(" WakeCount="));
  Serial.println(wakeCount);
  Serial.print(F(" CPM Count: "));
  Serial.println(CPM);
  Serial.print(F(" nSVh Value: "));
  Serial.println(nSVh);
  Serial.print(F(" uSVh Value: "));
  Serial.println(uSVh);
  Serial.print(F(" Currentmillis() = "));
  Serial.println(millis());
  Serial.print(F("  AuxA State:"));
  Serial.print(AuxA);
  Serial.print(F(" "));
  Serial.println(AuxA, HEX);

  Serial.println(F("###############"));
#endif
  // write out the current protocol to message and send.
  Serial.println(F("----------Sending packets----------"));
  int bufferLength = buildProtocolMessage();

  // Send a message
  Serial.print(F("bufferlength="));
  Serial.println(bufferLength);
  driver.send(byteBuffer, bufferLength);

  if (!driver.waitPacketSent(6000))
  {
    Serial.println(F("Timeout on transmission"));
    // re-initialize board
    if (!driver.init()) {
      Serial.println(F("init failed"));
      while (1)
        ;
    }
    Serial.println(F("----------Board Reinitialized----------"));
  }

  Serial.println(F("----------After Sending packet----------"));

  for (int i = 0; i < bufferLength; i++) 
  {
    Serial.print(" ");
    if (byteBuffer[i] < 16) {
      Serial.print(F("0"));
    }
    Serial.print(byteBuffer[i], HEX);  //  write buffer to hardware serial port
  }

  Serial.println();
  Serial.println(F("----------After Wait Sending packet----------"));
  delay(100);
  digitalWrite(LED, HIGH);
  delay(100);
  digitalWrite(LED, LOW);
  MessageCount++;
  // set to MessageCount
  EEPROM.put(0, MessageCount);
}



void setup()
{
  Serial.begin(115200);  // TXDEBUGging only
  // Pat the WatchDog
  ResetWatchdog();
  wakeCount = 0;

  AuxA = 0x00;

  Serial.println(F(">>>>>>>>>><<<<<<<<<"));
  Serial.println(F("WeatherSense Radiation"));
  Serial.println(F(">>>>>>>>>><<<<<<<<<"));
  Serial.print(F("Software Version:"));
  Serial.println(SOFTWAREVERSION);
  Serial.print(F("Unit ID:"));
  Serial.println(WEATHERSENESTHBID);

  if (!driver.init())
  {
    Serial.println(F("init failed"));
    while (1)
      ;
  }

  Serial.print("max message length=");
  Serial.println(driver.maxMessageLength());

  // read the values of messageCount from EEPROM
  unsigned long tempLong;

  if (tempLong == 0xFFFFF)  // uninitialized
  {
    // set to messageID
    EEPROM.put(0, MessageCount);
  } 
  else 
  {
    EEPROM.get(0, MessageCount);
    // read the message value
  }

  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);
  delay(1000);
  digitalWrite(LED, LOW);
  delay(1000);
  digitalWrite(LED, HIGH);
  delay(1000);
  digitalWrite(LED, LOW);
  delay(1000);

  // setup initial values of variables

  wakeState = REBOOT;
  nextSleepLength = SLEEPCYCLE;
  TimeStamp = 0;

  pinMode(WATCHDOG_1, OUTPUT);
  digitalWrite(WATCHDOG_1, HIGH);
  Wire.begin();

  // Set State Variable to indicate Geiger Counter present.
  // Haven't found code that could be used to actually check this,
  // DFRobot nor SDL code has a real check.
  AuxA = AuxA | 0X01;
  
  //--- INTERRUPT SETTINGS ---
  pinMode(INT1_PIN, INPUT);
  digitalWrite(INT1_PIN, HIGH);

  geiger.start();

  Serial.println(" ---- 30 second warmup ----");
  // delay 30 seconds after startup
  delay(10000);
  // Pat the WatchDog
  ResetWatchdog();
  delay(10000);
  // Pat the WatchDog
  ResetWatchdog();
  delay(10000);
  // Pat the WatchDog
  ResetWatchdog();

  Serial.println(F("\nReady for Radiation!"));
}


void loop() 
{

  if ((wakeState == SLEEP_INTERRUPT) || (wakeState == REBOOT))
  {
    wakeState = NO_INTERRUPT;
    TimeStamp = millis();

    bool readyToTransmit = false;

    // check if it is time to send message  - 30 seconds pre check

    if (((wakeCount % WAKEUPS) == 0))
    {
      // Pat the WatchDog
      ResetWatchdog();
      readyToTransmit = true;

      Serial.println(F(" ---- 3 second warmup ----"));
      delay(3000);
      Serial.println(F("Starting Geiger Read"));

      CPM = geiger.getCPM();
      nSVh = geiger.getnSvh();
      uSVh = geiger.getuSvh();

    }

    Serial.println();

    if (readyToTransmit)  // ready to transmit
    {
        // Now send the message
        Serial.println(F(">>>>>>>>>>>>>>>Transmitting radiation message<<<<<<<<<<<<"));
        sendMessage();
        Serial.println(F("----------Packet Sent.  Sleeping Now----------"));
    } 
  }

  // Pat the WatchDog
  ResetWatchdog();

  if (wakeState != REBOOT)
    wakeState = SLEEP_INTERRUPT;

  long timeBefore;
  long timeAfter;
  timeBefore = millis();
  delay(100);

  //Sleepy::loseSomeTime(nextSleepLength);
  for (long i = 0; i < nextSleepLength / 16; ++i) {
    Sleepy::loseSomeTime(16);
  }

  wakeState = SLEEP_INTERRUPT;
  wakeCount++;

#if defined(TXDEBUG)
  Serial.println(F("Awake now: "));
#endif
  timeAfter = millis();
#if defined(TXDEBUG)
  Serial.print(F("Current Sensor UpTime: "));
#endif
  long time;
  time = millis();
#if defined(TXDEBUG)
  //prints time since program started
  Serial.println(time / 1000.0);
  Serial.print(F("Free RAM : "));
  Serial.println(FreeRam());
#endif
  // Pat the WatchDog
  ResetWatchdog();
}

/** Returns the number of bytes currently free in RAM. */
static int FreeRam(void) {
  extern int  __bss_end;
  extern int* __brkval;
  int free_memory;
  if (reinterpret_cast<int>(__brkval) == 0) {
    // if no heap use from end of bss section
    free_memory = reinterpret_cast<int>(&free_memory)
                  - reinterpret_cast<int>(&__bss_end);
  } else {
    // use from top of stack to heap
    free_memory = reinterpret_cast<int>(&free_memory)
                  - reinterpret_cast<int>(__brkval);
  }
  return free_memory;
}
