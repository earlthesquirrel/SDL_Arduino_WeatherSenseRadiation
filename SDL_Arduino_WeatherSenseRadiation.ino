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
// WeatherSenseProtocol of 16 is WeatherSense ThunderBoard 433MHz
// WeatherSenseProtocol of 17 is Generic data
// WeatherSenseProtocol of 18 is WeatherSense AfterShock
// WeatherSenseProtocol of 19 is WeatherSense Radiation

#define WEATHERSENSEPROTOCOL 19

#define LED 13
// Software version
#define SOFTWAREVERSION 5

// unique ID of this WeatherSenseRadiation system - change if you have multiple WeatherSenseRadiation systems
#define WEATHERSENESTHBID 1
// Which WeatherSense TRadiation Protocol Version
#define WEATHERSENSEPROTOCOLVERSION 1

// Radiation


// Device ID is changed if you have more than one WeatherSense Radiation Monitor in the area

// Number of milliseconds between wake up  30 seconds.   - if you move this over 60000 ms, you will need to add the watchdog in the sleep loop - see SDL_Arduino_WeatherSenseAQI ResetWatchDog
// Every 120 wakeups send packet (~60 minutes) keepalivemessage
#define SLEEPCYCLE 30000
#define WAKEUPS 10


#include "Crc16.h"

//Crc 16 library (XModem)
Crc16 crc;

ISR(WDT_vect) {
  Sleepy::watchdogEvent();
}

#include <RH_ASK.h>

#include <avr/sleep.h>
#include <avr/power.h>
#include "SDL_Arduino_INA3221.h"


SDL_Arduino_INA3221 INA3221;



// the three channels of the INA3221 named for INA3221 Solar Power Controller channels (www.switchdoc.com)
#define LIPO_BATTERY_CHANNEL 1
#define SOLAR_CELL_CHANNEL 2
#define OUTPUT_CHANNEL 3


// Other Pins
#define WATCHDOG_1 5

#define TXPIN 8
#define RXPIN 9

#define INT1_PIN 2 //Arduino pin connected to the INT1 pin of the D7S sensor


// Number of milliseconds between data ou


volatile long CPM  = 0;
volatile long nSVh = 0;
volatile float uSVh = 0.0;



RH_ASK driver(2000, RXPIN, TXPIN);

unsigned long MessageCount = 0;

#include "avr/pgmspace.h"
#include <Time.h>
#include <TimeLib.h>


#include <Wire.h>

typedef enum  {

  NO_INTERRUPT,
  IGNORE_INTERRUPT,
  SLEEP_INTERRUPT,
  ALARM_INTERRUPT,
  REBOOT
} wakestate;


// Device Present State Variables

bool INA3221_Present;

bool Radiation_Present;

byte byteBuffer[100]; // contains string to be sent to RX unit

// State Variables

long TimeStamp;



// State Status

float BatteryVoltage;
float BatteryCurrent;
float LoadVoltage;
float LoadCurrent;
float SolarPanelVoltage;
float SolarPanelCurrent;
byte AuxA;
byte SoftwareVersion;

// AuxA has state information
// coded in the byte
// 0000DCBA


// A = 1, Radiation Present, 0 not present
// B = 1, IN3221 (Solar) Present, 0 not present
// C = 1, Low battery, Chip shut off (too many false alarms in low voltage mode)
// D = 1, I'm alive message





wakestate wakeState;  // who woke us up?


long nextSleepLength;





int convert4ByteLongVariables(int bufferCount, long myVariable)
{

  int i;

  union {
    long a;
    unsigned char bytes[4];
  } thing;
  thing.a = myVariable;

  for (i = 0; i < 4; i++)
  {
    byteBuffer[bufferCount] = thing.bytes[i];
    bufferCount++;
  }
  return bufferCount;

}

int convert4ByteFloatVariables(int bufferCount, float myVariable)
{
  int i;

  union {
    float a;
    unsigned char bytes[4];
  } thing;
  thing.a = myVariable;

  for (i = 0; i < 4; i++)
  {
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


  byteBuffer[bufferCount] = (byte) myVariable;
  bufferCount++;
  return bufferCount;

}

int checkSum(int bufferCount)
{
  unsigned short checksumValue;
  // calculate checksum
  checksumValue = crc.XModemCrc(byteBuffer, 0, 59);
#if defined(TXDEBUG)
  Serial.print(F("crc = 0x"));
  Serial.println(checksumValue, HEX);
#endif

  byteBuffer[bufferCount] = checksumValue >> 8;
  bufferCount++;
  byteBuffer[bufferCount] = checksumValue & 0xFF;
  bufferCount++;

  return bufferCount;
}




// variables






int buildProtocolMessage()
{

  int bufferCount;


  bufferCount = 0;

  bufferCount = convert4ByteLongVariables(bufferCount, MessageCount);


  byteBuffer[bufferCount] = WEATHERSENESTHBID; // WeatherSenseRadiation unique ID
  bufferCount++;
  byteBuffer[bufferCount] = WEATHERSENSEPROTOCOL; // Type of WeatherSense System
  bufferCount++;
  byteBuffer[bufferCount] = WEATHERSENSEPROTOCOLVERSION; // WeatherSense Radiation protocol version
  bufferCount++;



  bufferCount = convert4ByteLongVariables(bufferCount, CPM);

  bufferCount = convert4ByteLongVariables(bufferCount, nSVh);

  bufferCount = convert4ByteFloatVariables(bufferCount, uSVh);



  bufferCount = convert4ByteFloatVariables(bufferCount, LoadVoltage);  // Solar Data
  bufferCount = convert4ByteFloatVariables(bufferCount, BatteryVoltage);
  bufferCount = convert4ByteFloatVariables(bufferCount, BatteryCurrent);
  bufferCount = convert4ByteFloatVariables(bufferCount, LoadCurrent);
  bufferCount = convert4ByteFloatVariables(bufferCount, SolarPanelVoltage);
  bufferCount = convert4ByteFloatVariables(bufferCount, SolarPanelCurrent);



  byteBuffer[bufferCount] = AuxA;  // Aux
  bufferCount++;
  byteBuffer[bufferCount] =  SOFTWAREVERSION;
  bufferCount++;



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


  //Serial.println(F("Watchdog1 Reset - Patted the Dog"));


}

//
//
//



DFRobot_Geiger  geiger(INT1_PIN);




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
  Serial.print(F(" WakeState="));
  Serial.println(wakeState);
  Serial.print(F(" wakeCount="));
  Serial.println(wakeCount);


  Serial.print(F("CPM Count: "));
  Serial.println(CPM);





  Serial.print(F(" Battery Voltage:  ")); Serial.print(BatteryVoltage); Serial.println(F(" V"));
  Serial.print(F(" Battery Current:       ")); Serial.print(BatteryCurrent); Serial.println(F(" mA"));
  Serial.print(F(" Solar Panel Voltage:   ")); Serial.print(SolarPanelVoltage); Serial.println(F(" V"));
  Serial.print(F(" Solar Current:  ")); Serial.print(SolarPanelCurrent); Serial.println(F(" mA"));
  Serial.print(F(" Load Voltage:  ")); Serial.print(LoadVoltage); Serial.println(F(" V"));
  Serial.print(F(" Load Current:       ")); Serial.print(LoadCurrent); Serial.println(" mA");
  Serial.print(F(" Currentmillis() = "));
  Serial.println(millis());

  Serial.print(F("  AuxA State:"));
  Serial.print(AuxA);
  Serial.print(F(" "));
  Serial.println(AuxA, HEX);

  Serial.println(F("###############"));
#endif
  // write out the current protocol to message and send.
  int bufferLength;


  Serial.println(F("----------Sending packets----------"));
  bufferLength = buildProtocolMessage();

  // Send a message



  Serial.print(F("bufferlength="));
  Serial.println(bufferLength);



  driver.send(byteBuffer, bufferLength);

  if (!driver.waitPacketSent(6000))
  {
    //Serial.println(F("Timeout on transmission"));
    // re-initialize board
    if (!driver.init())
    {
      //Serial.println(F("init failed"));
      while (1);
    }
    //Serial.println(F("----------Board Reinitialized----------"));
  }




  Serial.println(F("----------After Sending packet----------"));

  for (int i = 0; i < bufferLength; i++) {
    Serial.print(" ");
    if (byteBuffer[i] < 16)
    {
      Serial.print(F("0"));
    }
    Serial.print(byteBuffer[i], HEX);           //  write buffer to hardware serial port
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



  Serial.begin(115200);    // TXDEBUGging only
  // Pat the WatchDog
  ResetWatchdog();
  wakeCount = 0;

  AuxA = 0x00;

  Serial.println();
  Serial.println();
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
    while (1);
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


  BatteryVoltage = 0.0;
  BatteryCurrent = 0.0;
  LoadCurrent = 0.0;
  SolarPanelVoltage = 0.0;
  SolarPanelCurrent = 0.0;







  pinMode(WATCHDOG_1, OUTPUT);
  digitalWrite(WATCHDOG_1, HIGH);


  Wire.begin();




  // test for INA3221_Present
  INA3221_Present = false;



  int MIDNumber;
  INA3221.wireReadRegister(0xFE, &MIDNumber);
  Serial.print(F("Manuf ID:   0x"));
  Serial.print(MIDNumber, HEX);
  Serial.println();

  if (MIDNumber != 0x5449)
  {
    INA3221_Present = false;
    Serial.println(F("INA3221 Not Present"));
  }
  else
  {
    INA3221_Present = true;
    Serial.println("SunAirPlus3 Found");

    // State Variable
    AuxA = AuxA | 0X02;
  }

  int error;

  Radiation_Present = true;
  error = 0;

  if (error == 0)
  {
    Serial.println("Radiation device found");
    Radiation_Present = true;
    // State Variable
    AuxA = AuxA | 0X01;
  }
  else if (error == 4)
  {
    Serial.println("Radiation device Not Found");
    Radiation_Present = false;
  }
  //start D7S connection



  //--- INTERRUPT SETTINGS ---


  pinMode(INT1_PIN, INPUT);
  digitalWrite(INT1_PIN, HIGH);

  //EIFR |= bit(INTF1); // clear INT1 interrupt flag




  //attachInterrupt(digitalPinToInterrupt(INT2_PIN), &RadiationHandler, CHANGE);
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




  // Only send if source is SLEEP_INTERRUPT
#if defined(TXDEBUG)
  Serial.print(F("wakeState="));
  Serial.println(wakeState);
#endif


  if ((wakeState == SLEEP_INTERRUPT) || (wakeState == REBOOT))
  {

    wakeState = NO_INTERRUPT;






    TimeStamp = millis();

    // if INA3221 present, read charge data

    if (INA3221_Present)
    {


      BatteryVoltage = INA3221.getBusVoltage_V(LIPO_BATTERY_CHANNEL);
      BatteryCurrent = INA3221.getCurrent_mA(LIPO_BATTERY_CHANNEL);

      SolarPanelVoltage = INA3221.getBusVoltage_V(SOLAR_CELL_CHANNEL);
      SolarPanelCurrent = -INA3221.getCurrent_mA(SOLAR_CELL_CHANNEL);


      LoadVoltage = INA3221.getBusVoltage_V(OUTPUT_CHANNEL);
      LoadCurrent = INA3221.getCurrent_mA(OUTPUT_CHANNEL) * 0.75;


    }


    if (BatteryVoltage < 2.80)
      AuxA = AuxA | 0x04;
    else
      AuxA = AuxA & 0xFB;





    bool readyToTransmit = false;



    // check if it is time to send message (every 10 minutes or on interrupt) - 30 seconds pre check

    if (((wakeCount % WAKEUPS) == 0))
    {

      // Pat the WatchDog
      ResetWatchdog();
      readyToTransmit = true;

      Serial.println(" ---- 3 second warmup ----");


      //EIFR |= bit(INTF1); // clear INT1 interrupt flag
      //attachInterrupt(digitalPinToInterrupt(INT2_PIN), &RadiationHandler, CHANGE);

      // doing CPM count
      //detachInterrupt(digitalPinToInterrupt(INT2_PIN));

      delay(3000);

      Serial.println("Starting Geiger Read");


      CPM = geiger.getCPM();

      nSVh = geiger.getnSvh();

      uSVh = geiger.getuSvh();

      Serial.print("CPM Count = ");
      Serial.println(CPM);



    }
    Serial.println();

    if (readyToTransmit)  // ready to transmit
    {
      // transmit


      if ((AuxA & 0x04)  == false)   // If the battery vboltage is less than 2.80V, then Radiation is flaky
      {
        // Now send the message


        Serial.println(F(">>>>>>>>>>>>>>>Transmitting radiation message<<<<<<<<<<<<"));
        sendMessage();

        Serial.println(F("----------Packet Sent.  Sleeping Now----------"));


      }
      else
      {
        Serial.println(F("Battery Voltage Low"));
        Serial.println(F("AuxA = "));
        Serial.println(AuxA, HEX);
      }

    }

    /*
      else
      {
      // send the I'm Alive Message

      AuxA = AuxA | 0x08;  // bit on

      Serial.println(F(">>>>>>>>>>>>>>>Transmitting Alive message<<<<<<<<<<<<"));

      AuxA = AuxA & 0xF7;  // bit off


      }

    */
  }



  // Pat the WatchDog
  ResetWatchdog();

  if (wakeState != REBOOT)
    wakeState = SLEEP_INTERRUPT;
  long timeBefore;
  long timeAfter;
  timeBefore = millis();
#if defined(TXDEBUG)
  Serial.print(F("timeBeforeSleep="));
  Serial.println(timeBefore);
#endif
  delay(100);


  //Sleepy::loseSomeTime(nextSleepLength);
  for (long i = 0; i < nextSleepLength / 16; ++i)
  {
    Sleepy::loseSomeTime(16);




  }



  wakeState = SLEEP_INTERRUPT;

  wakeCount++;

#if defined(TXDEBUG)
  Serial.print(F("Awake now: "));
#endif
  timeAfter = millis();
#if defined(TXDEBUG)
  Serial.print(F("timeAfterSleep="));
  Serial.println(timeAfter);

  Serial.print(F("SleepTime = "));
  Serial.println(timeAfter - timeBefore);

  Serial.print(F("Millis Time: "));
#endif
  long time;
  time = millis();
#if defined(TXDEBUG)
  //prints time since program started
  Serial.println(time / 1000.0);
  Serial.print(F("2wakeState="));
  Serial.println(wakeState);
#endif




                                                                                                                                                                                                                                                                        
  // Pat the WatchDog
  ResetWatchdog();



}
