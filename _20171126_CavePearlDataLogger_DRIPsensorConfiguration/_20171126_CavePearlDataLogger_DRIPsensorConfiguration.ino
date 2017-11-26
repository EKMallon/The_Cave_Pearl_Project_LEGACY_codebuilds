/*       Cave Pearl Project Data Logger code      DOI: 10.13140/RG.2.2.12299.18725
 *       written and developed by Edward Mallon  https://edwardmallon.wordpress.com/
 *       
 *       This script incorporates the work of many other people for the libraries and some sensor functions. I have done my best 
 *       to cite these sources directly in the comments with each section, and I make no claims whatsoever on those sections of code.
 *       That being said, the integration of these components into a functional datalogging codebase is something that I have spent 
 *       many months of hard work testing and refining, especially with regard to generic data buffering/saving functions which can 
 *       gracefully adapat to whatever sensor variables you feed into them. Please credit the Cave Pearl Project if you 
 *       use this software as the basis of for your own datalogger or other kind of project. 
 *
 *       This particular version is configured as an ADXL345 DRIP SENSOR, and assumes the following hardware
 *       Main MCU board : Sparkfun Pro Mini -or- Rocket scream ultra -or- Any other generic 3.3v ProMini Clone
 *       with Voltage divider:  RAW-10MΩ-A0-(3.3MΩ+0.1 µF) - GND  for the function that tracks batteryVoltage
 *       Ds3231 RTC module with AT24c32eeprom, Adxl345 I2C accelerometer, 3 color LED on pins 4-6 with at least a 10k limit resistor on the common ground
 *       A raspberry pi micro SD card adapter connected to SPI pins 10-13, with 50k pullup on the adapters unused connections
 *       (see  https://edwardmallon.wordpress.com/2015/10/24/diy-arduino-logger-build-instructions-part-1/ for wiring connections) 
 *       
 *       The drip sensor uses the use the RTC temperature register
 *       but I have left in script support for the DS18B20 & MCP9808 so you can see how ifdef/endif statements  add or remove 
 *       those sensors in the code, you can add your own sensors to the logger by following the same pattern
 *       After adding libraries for a new sensor you will still have to manually edit functions in the main loop
 *       by adding str.print(YourVariableName); to the centra loop functions which concatenate those readings into EEPROMBuffer (separated by commas!)
 *       Once your sensor reading has been transfered to the EEPROMBuffer char array, the rest of the data handling should happen automatically
 *       You will also have to manually change the file.pring statements in writeHeaderInfo(); so that the column headers match your new data.
 *
 *       I use the RocketScream sleep library <LowPower.h> to enable short processor sleeps at run-time while waiting for sensor readings
 *       but the power-down state between sensor readings uses RTC hardware interrupts to wake from sleep [see: https://www.gammon.com.au/forum/?id=11497 ]
 *       
 */

#include <Wire.h>       // I2C lib needs 128 bytes of ram for Serial buffer
//#include <RTClib.h>   // Date, Time and Alarm functions by https://github.com/MrAlvin/RTClib which is a fork of https://github.com/jcw/rtclib
#include <avr/power.h>  // I am using this for the ADC peripheral shutdown
#include <avr/sleep.h>  // used in sleepNwait4D3Interrupt & sleepNwait4RTC
#include <PString.h>    // from  http://arduiniana.org/
#include <SPI.h>
#include <SdFat.h>      // needs 512 byte ram buffer! see https://github.com/greiman/SdFat
SdFat sd; /*Create the objects to talk to the SD card*/
SdFile file;
const uint8_t chipSelect = 10; //sd card chip select
#include <LowPower.h>   // https://github.com/rocketscream/Low-Power and https://github.com/rocketscream/Low-Power 
// To set Duration of Rocket Screams low power sleep modes: SLEEP_15Ms,SLEEP_30MS,SLEEP_60MS,SLEEP_120MS,SLEEP_250MS,SLEEP_500MS,SLEEP_1S,SLEEP_2S,SLEEP_4S,SLEEP_8S,
// eg: LowPower.powerDown(SLEEP_15Ms, ADC_OFF, BOD_OFF); use SLEEP_FOREVER with hardware interrupts

//==========================================
// CRITICAL DATA LOGGER OPERATING PARAMETERS:  
//==========================================
//#define ECHO_TO_SERIAL                     // prints all readings to screen for USB tethered debugging only! Adds about 1700 bytes to compiled size so make sure there is room left before turning this on!
// sleep time before RTC interrupt triggers the next sample cycle controlled by SampleIntervalMinutes OR SampleIntervalSeconds but NOT both!
#define SampleIntervalMinutes 15              // Allowed values: 1,2,3,5,10,15,20 or 30, -MUST be a number that divides equally into 60! Time aligns to hour rollover no matter when unit is started
#define SampleIntervalSeconds 0              // 1Hz is maximum possible rate of recording discrete time stamped records with this logger!: 
//Usually SampleIntervalSeconds is only used for debugging: otherwise leave it at default 0 value // SampleIntSeconds WILL BE IGNORED unless SampleIntervalMinutes = 0
//WARNING: IN ALL CASES the sample interval must be longer than the time needed to acquire new sensor readings AND flush all eeprom buffer data to the SD card
//this can easily exceed 1 second with slow sensors like the ds18b20 which takes 750ms for a 12-bit reading, SD writes usually take at least 400ms, and can be significantly longer with larger eerpoms
uint16_t SamplesPerCycle = 64;              // AUTOADJUSTS later based on the eeprom being used as a buffer & number of pages buffered
// main loop takes account of 0-(#-1):   for (int bufferCycle = 0; bufferCycle < SamplesPerCycle; bufferCycle++) - counts from 0 to (SamplesPerCycle-1) so you can use full size numbers
// Max Samples per buffering Cycle = # of 32byte pages eeprom can hold / PagesBuffered2Eeprom
// = 64 SamplesPerCycle with two page-writes buffered to the 4k eeprom on the RTC board at 0x57
// AT24C256 32K eeprom @ 0x50: holds 512 2page records / 336 3 page records  // 256 4 page records // 200 5 page records // 168 6page records // 128 for 8 page (42 sensors)
uint16_t recordsSaved = 0;                  // # records have been written to each file so far
uint16_t newFileInterval = 1440;            // #of records saved in the log before a new file is created //usually 1440 (ie: 2 weeks worth of data @96 samples/day)
//If the sampleinterval is 15min and newFileInterval is 2880, then 96samples/day * 30days/month = 30 days before new file is created
char FileName[12] = "data000.csv";         // the first data file in the naming series
char TempName[12]; // to swap file names for the buffer dump
//strings printed in the SD card data file headers:
const char unitDetails[] PROGMEM = "Logger#1234 Sleep current=X(mA), pwersave mode on, etc"; //Put your own data here such as specific build settings & info
const char codebuild[] PROGMEM = __FILE__;                                                   //contains the compiled filename & directory path
const char boilerplate1[] PROGMEM = "The Cave Pearl Project: An open source data logger for research";
const char boilerplate2[] PROGMEM = "Developed by Ed Mallon & Patricia Beddows: http://edwardmallon.wordpress.com/";
const char contact[] PROGMEM = "For details contact: email@email.address";                   //Put your own data here
uint8_t EEPROM_ADDRESS = 0x57;             //AUTOADJUSTS! 0x57 RTC eeprm is default, but on some boards its 0x56 & changes to 0x50 if 32k eeprom is on the bus
uint8_t PagesBuffered2Eeprom = 2;          //default value for 1st buffer dump on startup only! If you put the wrong number here it AUTOADJUSTS to match the ACTUAL pages buffered later
uint16_t CutoffVoltage = 3650;             //lowest permitable battery voltage  - set this to accomodate your regulators maximum dropout
#define filterSamples  7                   // used for ACC filtering- odd number, no smaller than 3 - works ok at 7-9 samples & works well @ 13 
                                           // WARNING: filterSamples determines the size of arrays that can use up alot of ram!

//sensor configuration: enable attached sensor(s) with define statements here
//===========================================================================
//enable #define TipBucket_RainGauge 1 -OR- #define AdxlDripSensor ---not both!
//#define TipBucket_RainGauge 1   // highly simmilar to drip sensors, since the signal is the same Interrrupt on Low - but debounce time is longer
#define AdxlDripSensor 0x53       // don't forget to SET SENSITIVITY!
//#define MCP9808_I2CADDR  0x18   // uncomment this define if an MCP9808 is connected to the bus, in addition to the other sensors
//#define TS_DS18B20 INSTALLED    // uncomment this define if an DS18B20 is connected

//GLOBAL variables
//================
//bool bitBuffer;         // for fuctions that return a bit state
byte bytebuffer1 = 0;     // for functions that return a byte
byte bytebuffer2 = 0;     // need a second one for 16-bit registers
int integerBuffer = 9999; // for temp-swapping ADC readings
byte keep_ADCSRA;         // Needed for PRR shutdown & restart of ADC
//byte keep_SPCR;         // Needed for PRR restarts of SPI
uint16_t adcReading = 0;  // a temporary storage variable
int freeMem = 9999;       // dynamic memory at run time is always less than the compile time RAM estimate!
//use this varaible to trackthe 'minimum' free dynamic memory with : integerBuffer=freeRam();if (integerBuffer<freeMem){freeMem=integerBuffer;}

// battery level tracking:
uint16_t batteryVoltage = 9999;     //the main battery voltage (via 1.1 internal band gap OR analog read)
#define batteryPin A0               //where you have connected the voltage divider tracking the main battery voltage
uint16_t postSDbatt = 9999;         //the post SD card writing main battery voltage, 9999 until first card write cycle
const float resistorFactor = 511.5; // = 1023.0 * (R2/(R1 + R2)); // 511.5 if both resistors equal // 255.75 Assumes (high side=10Meg ohm)/(low side=3.3Meg ohm)divider
const float referenceVolts = 3.30;  // You need to check this with a DVM for each board to get accurate battery readings.
uint16_t CoinCellV = 9999;          // On SOME loggers I have voltage divider on the RTC coin cell connected to line A1 - this reading occurs only once per day
uint16_t VccBGap = 0;               //tracks of the main regulators output voltage using the internal 1.1v bandgap trick
uint16_t postSDvcc = 9999;
uint16_t blueBatteryLevel;          //Standard SensorRead LED pip changes from Green -> Blue -> Red based on these levels to tell you when battery is low
uint16_t redBatteryLevel;           //to warn you when the batteries are running low

// for time tracking in μS:         //note: Micros overflows will mess up the duration calculation every 71 minutes of cumulative cpu uptime
unsigned long startMicros;          //ulongs can store up to 4294 seconds=71 minutes max using micros();!
unsigned long sampleIntervalTime = 0;
unsigned long dailyCPUuptime = 0;   //perhaps convert this in milliseconds to save space?
unsigned long time4int1events = 0;
unsigned long time2writeSDdata = 0;

// variables for Temperature Sensors:
int TEMP_Raw = 0;                  //ints hold -32,768 to 32,767
int rtc_TEMP_Raw = 0; 
float TEMP_degC = 0.0;             //don't print floats - it eats alot of memory
uint16_t wholeTemp = 0;
uint16_t fracTemp = 0;
int MaxTemp = 0;
int MinTemp = 9999;

#define INT1_pin 3                   // The arduino pin D3 connects interrupt INT01 to LOW alarm inputs from ADXL345 AND Tipping buckets
uint16_t INT1_eventCounter = 0;      //can store up to 4,294,967,295 events allowing very long sample intervals (or fast sensors)
uint32_t dayTotalINT1count = 0;      //can store up to 4,294,967,295 events allowing very long sample intervals (or fast sensors)
volatile boolean INT1_Flag = false;  // changed to true by the ISR INT1pinD3_triggered() to count number of Interrupts coming in on D3 pin
                                     // "int" stores values -32768 to + 32767 and "unsigned int" 0 to +65535, so 65536 would require a "long"
                                     // uint16_t only allows a max of 72 counts per second if you are binning to 15 minute sample intervals

//Eeprom Buffer variables:
#define EEpromPageSize 32            // 32 bytes is page size for the AT24C32 & AT24C256 - can't move more than 30 bytes with wire.h anyway
uint16_t CurrentEEpromMemAddress = 0;// set to zero at the start of each cycle - must increment by steps of 32
uint16_t RolloverEEpMemAddress = 0;  // only used on midnight rollover
char EEPROMBuffer[30]; //size=(PageSize - 2) This char array recieves a string of ascii from the pstring function 
boolean printStatusLogHeaders = false; // flag that triggers the status logg header printing at startup
boolean midnightRollover = true;     // flag that triggers the saving ONCE per day data
                                     // I2C Bus address of AT24C32 EEPROM on RTC board with pins pulled high is usually 0x57 but sometimes 0x56
                                     // The AT25C32 (0x57) is internally organized into (32,768 bits)=4096 bytes - beyond 4096 bytes/characters it rewrites over top of the data
                                     // (128 x 32byte writes fill entire 4096 byte block) so MAX 64 with 2 pgwrites/cycle but max 42 if you go to 3 pgwrites/cycle
                                     // the separate AT24C256 has its address pins pulled low, so it is on the bus at 0x50...usually...

//DS3231 RTC variables
//====================
#define RTCPOWER_PIN 7               // Assumes you are powering your DS3231 RTC from this pin. Even if you are not, it is harmless to leave this in unless you have something else connected to digital pin 7. 
////When the arduino is awake, power the rtc from this pin (70uA), when arduino sleeps pin set low & rtc runs on battery at <3uA
#define COINCELLREAD 1               //if the unit has the capablity to read the RTC coin cell via a divider, enable this define
#define RTCcoinPin A6                //if you have the middle of that voltage divider on analog input A6
#define DS3231_ADDRESS     0x68      //=104 dec
#define DS3231_STATUS_REG  0x0F
#define DS3231_CONTROL_REG 0x0E
#define DS3231_TMP_UP_REG  0x11

//TimeStamps created with sprintf(CycleTimeStamp, "%04d/%02d/%02d %02d:%02d",t_year,t_month,t_day,t_hour,t_minute);
char CycleTimeStamp[] = "0000/00/00,00:00"; //16 characters without seconds!
uint8_t t_second;   //= sec;  used in RTC_DS3231_getTime()
uint8_t t_minute;   //= min;
uint8_t t_hour;     //= hour;
uint8_t t_day;      //= day;
uint8_t t_month;    //= month;
uint16_t t_year;    //= year //yOff = raw year; //need to add 2000
static uint8_t bcd2bin (uint8_t val) {
  return val - 6 * (val >> 4);
}
static uint8_t bin2bcd (uint8_t val) {
  return val + 6 * (val / 10);
}
uint32_t eTime; // mofified: eTime = unixtime/(long)(SampleIntervalMinutes*60);
uint8_t Alarmday = 1;
uint8_t Alarmhour = 1;
uint8_t Alarmminute = 1;
uint8_t Alarmsecond = 1; 
uint8_t rtc_INT0_Pin = 2; //DS3231's SQW output is connected to pin D2 on the arduino
volatile boolean rtc_INT0_Flag = false;  //volatile because it's changed in an ISR

// common variables for accelerometer readings
#if defined(LSM303_ADDRESS_ACCEL) || defined(AdxlDripSensor)  //drip sensors read x,y,z position once per day too
int rawACCx [filterSamples];   // raw sensor values for x // this is alot of memory to eat up
int rawACCy [filterSamples];   // raw sensor values for y
int rawACCz [filterSamples];   // raw sensor values for z
int smoothACCx = 0;            // smoothed x data
int smoothACCy = 0;            // smoothed y data
int smoothACCz = 0;            // smoothed z data
byte sixByteArray[6] ;         //6 byte buffer for reading 16-bit x,y,z output registers
#endif

//ADXL345 I2C ACCelerometer used as a drip counter
//================================================
#ifdef AdxlDripSensor
#define adxlTapSensitivity 7  // 4 is the highest reliable but might have to reduce this - some adxl's are twitchier than others
#define ADXL345_INT_SOURCE 0x30  //defined here so it can be used in INT1 loop
uint8_t ADXL345_ADDRESS = 0x53;
uint8_t adxl345lnterruptSource;  // need to read the int register to clear it
#endif

//TEMPERATURE Sensors enable if installed
//=======================================
//I've included some sample code here just to show you how a new sensor could be added to the logger

#ifdef TS_DS18B20               // variables for DS18B20 temperature sensor only included if #define TS_DS18B20 exists
#include <OneWire.h>            // from  http://www.pjrc.com/teensy/td_libs_OneWire.html
const uint8_t DS18B20_PIN = 8;  // DS18B20 Data line - assumes you have nothing else on pin 8!
OneWire ds(DS18B20_PIN);        // Also see Dallas Temperature library by Miles Burton: http://milesburton.com/Dallas_Temperature_Control_Library
byte addr[8];
#endif

// if there is only one indicator led on your logger then set these defines all to that same pin number
#define RED_PIN 4
#define GREEN_PIN 5
#define BLUE_PIN 6
uint8_t SensorReadLED = GREEN_PIN;  //sets the color of the LED that flashes when a sensor reading is being taken

//======================================================================================================================
//  *  *   *   *   *   *   SETUP   *   *   *   *   *
//======================================================================================================================
// note: errors in setup call error() routine that halts the system! In the Main Loop, only low-power errors stop the logger.

void setup () {

  keep_ADCSRA = ADCSRA; // used for PRR shutdown & wakeup of ADC
  //ADMUX = bit (REFS0) | (0 & 0x07);  // set AVcc to 3.3v and select input A0 - engages Avcc without reading!

#ifdef RTCPOWER_PIN
  digitalWrite(RTCPOWER_PIN, OUTPUT);// IF you are pin-powering the chip:
  pinMode(RTCPOWER_PIN, HIGH); // driving this high supplies power to the RTC Vcc pin while arduino is awake
#endif

  //turn on internal pullups for three SPI lines to help some SD cards go to sleep faster
  pinMode(chipSelect, OUTPUT);
  digitalWrite(chipSelect, HIGH); //pullup the CS pin on the SD card (but only if you dont already have a hardware pullup on your module)
  pinMode(11, OUTPUT);
  digitalWrite(11, HIGH); //pullup the MOSI pin on the SD card
  pinMode(12, INPUT);
  digitalWrite(12, HIGH); //pullup the MISO pin on the SD card
  // NOTE: In Mode (0), the SPI interface holds the CLK line low when the bus is inactive, so DO NOT put a pullup on it.
  // NOTE: when the SPI interface is active, digitalWrite() cannot effect MISO,MOSI,CS or CLK

  pinMode(RED_PIN, OUTPUT);
  digitalWrite(RED_PIN, HIGH);     // used for error state, SD card writes & low voltage warning
  pinMode(GREEN_PIN, OUTPUT);
  digitalWrite(GREEN_PIN, LOW);  // indicates sensor reading
  pinMode(BLUE_PIN, OUTPUT);
  digitalWrite(BLUE_PIN, LOW);    // eeprom writes & early low voltage warning

  //#ifdef ECHO_TO_SERIAL
  Serial.begin(9600);  // Note that this takes over pins D0 & D1  // Opening the serial monitor "resets" the Arduino
  //#endif

  Wire.begin();  // Note: this enables the internal 30-50k pull-up resistors on SDA & SCL by default
  //TWBR = 12;   // increase I2C bus speed to 200kHz with 8MHz or 400kHz mode with 16MHz CPU  see http://www.gammon.com.au/forum/?id=10896
  TWBR = 2;    // for 400 kHz @ 8MHz CPU speed ONLY! Bus may fail at this speed so comment this line out if you have sensor comm issues.
  RTC_DS3231_getTime();
  sprintf(CycleTimeStamp, "%04d/%02d/%02d %02d:%02d", t_year, t_month, t_day, t_hour, t_minute);
  RTC_DS3231_turnOffAlarms();          //stops RTC from holding the D2 interrupt line low if system reset just occured with RTC.turnOffAlarm(1);
  pinMode(rtc_INT0_Pin, INPUT_PULLUP); // pull up the RTC interrupt pin (though this is redundant with most RTC modules)
  pinMode(INT1_pin, INPUT_PULLUP);     // also pull up the hardware interrupt01 on D3 with internal 20K pullup resistors
  i2c_setRegisterBit(DS3231_ADDRESS, DS3231_STATUS_REG, 3, 0);    // disable the 32khz output  pg14-17 of datasheet  // This does not reduce the sleep current
//i2c_setRegisterBit(DS3231_ADDRESS, DS3231_CONTROL_REG, 6, 1); // Bit 6 (Battery-Backed Square-Wave Enable) of control register 0Eh, set this to 1 to force the wake-up alarms when running from the back up battery
//i2c_setRegisterBit(DS3231_ADDRESS,DS3231_STATUS_REG,4,1);     // see APPLICATION NOTE 3644 - only for the DS3234
//i2c_setRegisterBit(DS3231_ADDRESS,DS3231_STATUS_REG,5,1);     // setting bits 4&5 to 1, extends the time between RTC temp updates to 512seconds (from default 64s)

  // the following routine updates the clock to the code compile time if it is out of date
  // RTC_DS3231_set2compilerTime (F(__DATE__), F(__TIME__));
  // BUT that also makes it impossible to catch drift errors...

  // Keep this startup delay in place for system settling time at startup
  //=====================================================================
#ifdef ECHO_TO_SERIAL
  serial_boilerplate();
#endif

#ifndef ECHO_TO_SERIAL  // no need for the delay if you are on USB serial
  // 24 second delay in here so I have time to compile & upload if I just connected to UART
  // this delay also prevents loose connection power stutters from writing multiple headers on the sd card
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);//or delay(8000);
  digitalWrite(RED_PIN, LOW);
  digitalWrite(BLUE_PIN, HIGH);
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
  digitalWrite(BLUE_PIN, LOW);
  digitalWrite(GREEN_PIN, HIGH);
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
#endif

  // Check if a larger eeprom is availiable to use as a data buffer between SD writes
  // Wire.endTransmission returns zero if the bus address handshake was a success.
  Wire.beginTransmission (0x56); //some RTC modules have their 4k eeprom at this address & some have the eeprom at 0x57
  if (Wire.endTransmission () == 0) {
    EEPROM_ADDRESS = 0x56; // decimal 86
  }
  Wire.beginTransmission (0x57); //check if 4k eeprom is availiable
  if (Wire.endTransmission () == 0) {
    EEPROM_ADDRESS = 0x57; // decimal 87
  }
  Wire.beginTransmission (0x50); //if larger 32k eeprom is availiable on the system use that instead
  if (Wire.endTransmission () == 0) {
    EEPROM_ADDRESS = 0x50; // decimal 80
  }
  if (EEPROM_ADDRESS == 0x50) {
    SamplesPerCycle = 1024 / PagesBuffered2Eeprom;
  }
  else {
    SamplesPerCycle = 128 / PagesBuffered2Eeprom; // this updates again later in the code
  }

//Sensor initializations: depend on which #defines you enabled earlier
//====================================================================

#ifdef TS_DS18B20
  if ( !ds.search(addr))
  {
    Serial.println(F("ERROR: Did not find the DS18B20 Temp Sensor!"));
    return;
  }
  else
  {
    Serial.print(F("DS18B20 found @ ROM addr:"));
    for (uint8_t i = 0; i < 8; i++) {
      Serial.write(' ');
      Serial.print(addr[i], HEX);
    }
    Serial.println();
  }
#endif

#ifdef MCP9808_I2CADDR
  initMCP9808();
#endif

  //Accelerometer init
  //==================

#ifdef AdxlDripSensor
  initADXL345();
  readADXL345();
#endif

  //check main battery
  CutoffVoltage = 3650; //most of the 3.3v regulators go down to 3400 mV as their absolute minimum input - set this appropriately for the vreg you are using
  blueBatteryLevel = CutoffVoltage + 300; // SensorReadLED's led pip changes from Green -> Blue -> Red based on these numbers
  redBatteryLevel = CutoffVoltage + 100; // to warn you when the batteries are getting low
  #ifdef COINCELLREAD
  readCoinCell();    // usually this voltage divider is attached to pin A1
  #endif
  batteryVoltage = readBattery(); //check of the main battery voltage before SD card writing
  //Note readBattery checks against CutoffVoltage and will shut down if the voltage is too low for safe SD writing.
  VccBGap = getRailVoltage(); //takes 12 ms

  digitalWrite(GREEN_PIN, LOW);

//get the SD card ready
//=====================

#ifdef ECHO_TO_SERIAL
  Serial.print(F("SD card..."));
#endif
  digitalWrite(GREEN_PIN, LOW);
  digitalWrite(RED_PIN, HIGH);

  // Initialize SdFat or print a detailed error message and halt
  // change to SPI_FULL_SPEED for more performance.//change to SPI_HALF_SPEED for old cards -half speed like the native library. 
  if (!sd.begin(chipSelect, SPI_FULL_SPEED)) {
    Serial.println(F("Could NOT initialize!")); Serial.flush();
    error(); // not being able to save data is a terminal error in all cases
  }

#ifdef ECHO_TO_SERIAL
  Serial.print(F("initalized:"));
  Serial.println(CycleTimeStamp);
  Serial.print(F("Sample interval is: "));
  Serial.print(SampleIntervalMinutes);
  Serial.print(F(" min. "));
  Serial.print(SampleIntervalSeconds);
  Serial.println(F(" sec."));
  Serial.flush();
#endif

// Find the next availiable file name
//===================================
  // 2 GB or smaller cards should be formatted FAT16 - FAT16 has a limit of 512 entries in root
  if (!file.open(FileName, O_CREAT | O_EXCL | O_WRITE)) { // note that restarts often generate empty log files!
    for (int i = 1; i < 512; i++) {
      delay(5);
      snprintf(FileName, sizeof(FileName), "data%03d.csv", i);//concatenates the next number into the filename
      if (file.open(FileName, O_CREAT | O_EXCL | O_WRITE)) // O_CREAT = create file if not exist, O_EXCL = fail if file exists, O_WRITE - open for write
      {
        break; //if you can open a file with the new name, break out of the loop
      }
    }
  }
  writeHeaderInfo();
  file.close();

#ifdef ECHO_TO_SERIAL
  Serial.print(F("Current Filename:")); Serial.println(FileName); Serial.println();Serial.flush();
#endif

                         // On every BATTERY POWERED restart: flush any old data currently stored in the eeprom buffer
                         // to the SD to makesure no data gets overwritten - long sample intervals could mean days worth of data
                         // stored in the eeprom that you could loose when disconnecting the the main battery
                         // Some SD cards draw >50mA current! & the UART adapter may not supply enough power to drive it.
#ifndef ECHO_TO_SERIAL   //perhaps I need a better criterion here...dont want to loose data if I forget and leave echo to serial on?
  file.open("Ebuffer.csv", O_RDWR | O_CREAT | O_AT_END); // At every startup: flush the old eeprom memory buffer contents to a file called Ebuffer so no data is lost.
  writeHeaderInfo();     //every time a buffer dump happens, there will be a new header to separate the data
  file.close();
  // assign FileName to EPRbuffFileName before going to the generic "flush eeprom to SD" function
  memcpy(TempName, FileName, 12);      //store the existing log filename in TempName with memcpy(destination,source,size_t);
  //memcpy(FileName,EPRbuffFileName,12);
  strcpy(FileName, "Ebuffer.csv");     // see http://arduino.land/FAQ/content/6/30/en/how-to-copy-an-array.html
  flushEEpromBuffer();                 //same routine used in the main loop, so had to swap FileNames
  memcpy(FileName, TempName, 12);      //reload the standard data log name
  
  LowPower.powerDown(SLEEP_15MS, ADC_OFF, BOD_OFF); //give the sd some time for housekeeping.

  printStatusLogHeaders = true;
  file.open("StatusLog.csv", O_CREAT | O_WRITE);
  writeHeaderInfo();
  file.close();
  printStatusLogHeaders = false;
  
#endif

digitalWrite(RED_PIN, LOW);
LowPower.powerDown(SLEEP_60MS, ADC_OFF, BOD_OFF);// SD cards can draw power for up to 1sec after file close...

// On power-up, all pins are initialized to be a digital INPUT.
// pinMode() configures a pin for use as a digital input, not analog input.
// When analogRead() is called, it always reconfigures the Pin for “input”
// analogWrite() works on pins which support Pulse-Width Modulation (PWM), so it only makes sense to use it as an OUTPUT
/*
   Generally I set digital pins that I AM NOT USING to INPUT_PULLUP
   If you accidentally touch a hot wire to an output set low you've short-circuited your processor and fried it.
   If you accidentally touch a ground wire to an output set high you've short-circuited your processor and fried it.
   If you accidentally touch either wire to an input w/ pull-up you won't even notice.
*/

//A0 = always the main power supply voltage divider
//A1 = RTC coin cell voltage divider on some but not all units
  DIDR0 = 0x0F;             //  disables the digital inputs on analog 0..3 (analog 4&5 being used by I2C!)
  
#ifndef ECHO_TO_SERIAL
  pinMode(0, INPUT_PULLUP); //but don't do this if tethered to USB -then RX & TX are needed for serial coms
  pinMode(1, INPUT_PULLUP); //also dont't do this if you have a serial sensor like a GPS
#endif
  //2&3 are used as hardware interrupts,  4,5,6 used for RGB led
#ifndef RTCPOWER_PIN    //Only pullup D7 if it is NOT connected to your RTC'S VCC pin
  pinMode(7, INPUT_PULLUP); // on some builds I am using this pin to power the RTC
#endif

  pinMode(8, INPUT_PULLUP); //Pin 8 us often used to power my thermistor and/or as the one wire data line
  pinMode(9, INPUT_PULLUP); //pin 9 being used to drive the mosfet on builds that de-power the SD card
  //pins 10,11,12,13 are connected to the Sd card in SPI mode, and those pullups have already been set

  SensorReadLED = GREEN_PIN; // the "first" powersupply read is usually 50-100 mv low if divider the cap is not charged, and artificially sets the pip color to blue in read
  midnightRollover = true; //setting this forces a first record to be written in the StatusLog.csv file at startup.
  //  keep_SPCR = SPCR; //only needed if you decide to turn off the SPI peripheral later..
  dailyCPUuptime = 0; sampleIntervalTime = 0; time4int1events = 0; time2writeSDdata = 0;
  
//====================================================================================================
}  //   terminator for setup
//=====================================================================================================


// ========================================================================================================
//                         *  *  *  *  *  *  MAIN LOOP   *  *  *  *  *  *
//========================================================================================================
// sensor errors during main loop only call error routine & halt the system if ECHO_TO_SERIAL is defined (ie: we are in debug mode on a usb cable)
// that way if one sensor dies in the field you can still get data from the others. But SD data saving errors, and low battery voltage errors halt system in all cases

void loop ()
{

  if (recordsSaved >= newFileInterval) { // recordsSaved tracks how many lines have been written to the file

    sampleIntervalTime += (micros() - startMicros); startMicros = micros(); 

    digitalWrite(RED_PIN, HIGH); //I use red indicator for SD writing events
    createNewLogFile(); // create a new file this is the largest power using event!
    recordsSaved = 0;     // just made a new file so reset counter
    digitalWrite(RED_PIN, LOW);

    time2writeSDdata += (micros() - startMicros); startMicros = micros();
  }

  CurrentEEpromMemAddress = 0; //resets back to start of eeprom address space (redundant here, just making sure..)

  if (EEPROM_ADDRESS == 0x50) {  //PagesBuffered2Eeprom may change after a pass through the whole loop - so recalc SamplesPerCycle
    SamplesPerCycle = 1024 / PagesBuffered2Eeprom; //32K eeprom is 8x bigger than the eep on the RTC board
  }
  else {
    SamplesPerCycle = 128 / PagesBuffered2Eeprom; //the 4K eeprom on the RTC board
  }

//===========================================================================================
//this loop wraps the "cycle of samples" repeating until the eeprom memory is full:
//===========================================================================================

  for (int bufferCycle = 0; bufferCycle < SamplesPerCycle; bufferCycle++) {  //counts from 0 to (SamplesPerCycle-1)

    if (rtc_INT0_Flag) {  // this is redundant...just put it here to make sure the RTC alarm really is off
      RTC_DS3231_turnOffAlarms();
    }

    //digitalWrite(SensorReadLED, HIGH);  //is the Normal heartbeat pip
#if defined(AdxlDripSensor) || defined(TipBucket_RainGauge)
    digitalWrite(RED_PIN, HIGH);  //changed color on RTC initiated pip for event counters to distinguish from INT01 event pips
    LowPower.powerDown(SLEEP_15MS, ADC_OFF, BOD_OFF);  //A delay just to see the RTC triggered red sensor pip?
#else
    digitalWrite(SensorReadLED, HIGH); //Use this as the Normal heartbeat pip for flow sensors
#endif

    RTC_DS3231_getTime();  // Read new time & date values from the RTC // DateTime now = RTC.now(); with #include <RTClib.h>
    eTime = RTC_DS3231_unixtime(); eTime = eTime / (long)(SampleIntervalMinutes * 60);
    sprintf(CycleTimeStamp, "%04d/%02d/%02d %02d:%02d", t_year, t_month, t_day, t_hour, t_minute); // sprintf ref  http://www.perlmonks.org/?node_id=20519
    // alt: sprintf(CycleTimeStamp, "%04d/%02d/%02d %02d:%02d:%02d", now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second()); // Time read always occurs <1 sec after RTC interrupt, so seconds data was always "00" - so I dont record it

    //calculate the time for your next RTC alarm: (Note: roll-overs get caught later)
    Alarmhour = t_hour; // Alarmhour = now.hour(); //with #include <RTClib.h>
    Alarmminute = t_minute + SampleIntervalMinutes; //Alarmminute = now.minute()+SampleIntervalMinutes; //with #include <RTClib.h>
    // Make sure your sensor readings don't take longer than your sample interval or you pass your next alarm time & clock fails!
    Alarmday = t_day; //Alarmday = now.day(); //with #include <RTClib.h>
    Alarmsecond = t_second + SampleIntervalSeconds; //only used for special testing & debugging runs - usually gets ignored
    // Now we read the sensors that are attached:
    //-------------------------------------------

    //Read the temperature Sensor:

    TEMP_degC = RTC_DS3231_getTemp();  //RTC temp register always gets read as a backup
    wholeTemp = (int)TEMP_degC;        //split into 2 intergers so debugging echotoserial print funtions dont eat memory with floats
    fracTemp = int((TEMP_degC * 100) - abs(wholeTemp * 100)); // only convert two digits or gives 0.224 & x.238 errors
    TEMP_degC = TEMP_degC / 0.0625;    //converts the RTC float temperature into a 'RAW' 3 digit integer compatible with 12-bit temps sensors
    rtc_TEMP_Raw = (int)(TEMP_degC);   //save TEMP_Raw integer in the data file as it takes less space than the floats

TEMP_Raw = 0; //Note if TEMP_Raw remains at zero, then the RTC temperature register becomes the systems temp sensor

#ifdef TS_DS18B20                  //750ms for high bit depth reading
    TEMP_Raw = readDS18B20Temp();  // 1 second of sleep embedded here while waiting for it's high bit depth conversion!
#endif

#ifdef MCP9808_I2CADDR             //250ms delay for temp conversions
    TEMP_Raw = readMCP9808Temp();
#endif

    if (TEMP_Raw == 0 || TEMP_Raw == 1360 || TEMP_Raw == -1) {
      // If no other temperature sensor has been read, the RTC becomes the temperature sensor of last resort
      // 1360 and -1 are read error flags from the DS18b20, so are included in this failover event
      TEMP_Raw = rtc_TEMP_Raw;
    }
    else
    {
      //arduino introduces floating point calc errors here, so this is just for serial output!
      TEMP_degC = TEMP_Raw * 0.0625; //12 bit sensors use the same calculation, note I only save "raw" inteter temp data in logs, not floats!
      wholeTemp = (int)TEMP_degC;   //split into 2 intergers so print funtions dont eat memory with floats
      fracTemp = int((TEMP_degC * 1000) - abs(wholeTemp * 1000)); //abs handles negative temps

    }   // end of if (TEMP_Raw==0) else statment

    if (TEMP_Raw > MaxTemp) {
      MaxTemp = TEMP_Raw;
    }
    if (TEMP_Raw < MinTemp) {
      MinTemp = TEMP_Raw;
    }

// This is for serial output for debugging only  - comment out ECHO_TO_SERIAL skips this
//--------------------------------------------------------------------------------------------------
#ifdef ECHO_TO_SERIAL
    Serial.print(CycleTimeStamp);
    Serial.print(F(" bufferCycle: "));
    Serial.print(bufferCycle);
#if defined(AdxlDripSensor) || defined(TipBucket_RainGauge)  //If we are counting taps, the ACCelerometer is not making other readings, so we can sub in here.
    Serial.print(F(", Count:"));
    Serial.print(INT1_eventCounter);
#endif
    Serial.flush();
    postSDbatt = getRailVoltage();// there is no external battery voltage when you are thethered to USB
    Serial.print(F(", Rail(mV)= "));
    Serial.print(postSDbatt);
    Serial.print(F(", Coincell(mV)= "));
    Serial.println(CoinCellV);

#if defined(AdxlDripSensor)
    //readADXL345();
    Serial.print(F(", ACCx= "));
    Serial.print(smoothACCx);
    Serial.print(F(", ACCy= "));
    Serial.print(smoothACCy);
    Serial.print(F(", ACCz= "));
    Serial.println(smoothACCz);
#endif

    Serial.print(F("Temp C: "));
    Serial.print(wholeTemp);
    Serial.print(F("."));
    Serial.print(fracTemp); //avoid sending floats to serial print - it eats sram!
    Serial.print(F(", Free Ram: "));
    Serial.print(freeRam()); //only use this for debugging
    Serial.println(""); //terminate line
    Serial.flush();
#endif  //ENDIF FOR ECHO TO SERIAL

    //now turn off the LED indicators for the sensor reading phase
#if defined(AdxlDripSensor) || defined(TipBucket_RainGauge)
    digitalWrite(RED_PIN, LOW);  // RTC initiated pip for event counters
#else
    digitalWrite(SensorReadLED, LOW); //the Normal heartbeat pip for flow sensors
#endif

    //Construct first 30-byte char string from the sensor data 
    //--------------------------------------------------------
    PagesBuffered2Eeprom = 0; //do not zero this again before SamplesPerCycle is calculated!

    PString str(EEPROMBuffer, sizeof(EEPROMBuffer));
    str = ""; //30 character max payload:  wires buffer is 32 bytes, but 2 get used for bus&reg addresses
    str.print(CycleTimeStamp); str.print(F(","));   //17 / 16 characters without seconds plus comma
    str.print(batteryVoltage); str.print(F(","));         //5 char
    str.print(TEMP_Raw);  //"raw" 12-bit int. for most tempsensors, 4095 is largest value =4 charaters +1 for comma  //RTC is only 3 characters +1
    //str.print(rtc_TEMP_Raw);str.print(F(","));  //without a second temp sensor rtc_TEMP is TEMP_Raw
    str.print(F("      "));  //NO trailing comma on last data in string - this fills the end of the buffer with blank spaces - Pstring ignores any excess so will not generate an overflow error

    Write_i2c_eeprom_page(EEPROM_ADDRESS, CurrentEEpromMemAddress, EEPROMBuffer); // whole page is written at once here
    CurrentEEpromMemAddress += EEpromPageSize; PagesBuffered2Eeprom++;

    //Construct second char string of 30 bytes to complete the record
    //--------------------------------------------------------------------------------------------------
    str = ","; //each new string starts with a comma separator

#if defined(AdxlDripSensor) || defined(TipBucket_RainGauge)  //If we are counting drips, the other ACCelerometer readings will be zero, so we can sub in here.
    str.print(INT1_eventCounter); str.print(F(",")); // could add str.print(bufferCycle) for debugging
    dayTotalINT1count += INT1_eventCounter; INT1_eventCounter = 0; //once the data is in the string, you can reset the event counter to zero
    str.print(CoinCellV); str.print(F(","));  //5 characters with comma
    str.print(postSDbatt); str.print(F(",")); //5 char
    str.print(postSDvcc); str.print(F(",")); //5 char
#endif
    str.print(VccBGap);                       //4 char, last item gets no trailing comma!
    str.print(F("                         "));  //rest of string filled with spaces
    Write_i2c_eeprom_page(EEPROM_ADDRESS, CurrentEEpromMemAddress, EEPROMBuffer); // 28 bytes/page is max whole page is written at once here
    CurrentEEpromMemAddress += EEpromPageSize; PagesBuffered2Eeprom++;

    // Following the pattern above,you can fill and save as Eeprom Buffer strings as you need to store your sensor data, provided SamplesPerCycle gets updated accordingly.
    // Remember that the Maximum value of Samples per cycle is limited by buffer memory = # of 32byte pages your eeprom can hold / PagesBuffered2Eeprom (which is the number of pages you need per record)

    //Construct third char string of 30 bytes IF you need to
    //--------------------------------------------------------------------------------------------------

#if defined(HMC5883_ADDRESS) || defined(LSM303_ADDRESS_MAG)  // RH or compass data means we need to construct a third string
    str = ",";

#if defined(HMC5883_ADDRESS) || defined(LSM303_ADDRESS_MAG)  //variable names need updating here
    str.print(CompassX);  //max 18 characters from compass, usually less
    str.print(F(","));
    str.print(CompassY);
    str.print(F(","));
    str.print(CompassZ);
    //str.print(F(","));
#endif

    str.print(F("                     "));
    Write_i2c_eeprom_page(EEPROM_ADDRESS, CurrentEEpromMemAddress, EEPROMBuffer); // 28 bytes/page is max whole page is written at once here
    CurrentEEpromMemAddress += EEpromPageSize; PagesBuffered2Eeprom++;

#endif

    // Following the pattern above,you can create as many buffer strings as you need to store your sensor data in the eeprom, provided SamplesPerCycle is updated accordingly.
    // Maximum value of Samples per cycle is limited by buffer memory = # of 32byte pages your eeprom can hold / PagesBuffered2Eeprom (which is the number of pages you need per record)

    RolloverEEpMemAddress = CurrentEEpromMemAddress - EEpromPageSize; //this only gets used if we trigger the midnight rollover

    // If cycle is complete: FLUSH EEPROM BUFFER TO SD CARD
    //-----------------------------------------------------
    // if you are on the last cycle the eeprom is full - run a loop to flush buffered data to the sd card
    // all SD writing events check if battery voltage is above CutoffVoltage, so there is enough power to write data to the SD card safely

if (bufferCycle == (SamplesPerCycle - 1) && (batteryVoltage >= CutoffVoltage)) {
      sampleIntervalTime += (micros() - startMicros); startMicros = micros();
      flushEEpromBuffer();
      time2writeSDdata += (micros() - startMicros); startMicros = micros();
      //midnightRollover = true; //forces frequent status log writing for rapid testing only!
    }

if (midnightRollover) {
      sampleIntervalTime += (micros() - startMicros); startMicros = micros();
      oncePerDayEvents(); // saves output to the StatusLog.csv file
      midnightRollover = false; //reset the once per day event trigger
      time2writeSDdata += (micros() - startMicros); startMicros = micros();
    }


    // Check for RTC TIME ROLLOVERS: THEN SET the next RTC alarm and go back to sleep
    //============================================================================
if (SampleIntervalMinutes > 0) //then our alarm is in (SampleInterval) minutes
    {
      if (Alarmminute > 59) {  //error catch - if alarmminute=60 the interrupt never triggers due to rollover!
        Alarmminute = 0;
        Alarmhour = Alarmhour + 1;
        if (Alarmhour > 23) {
          Alarmhour = 0;
          midnightRollover = true;
        }
      }  //terminator for if (Alarmminute > 59) rollover catching

      RTC_DS3231_setAlarm1Simple(Alarmhour, Alarmminute);
    }  //terminator for if (SampleIntervalMinutes > 0)

else  //to get sub-minute alarms use the full setA1time function
    
    {  // for testing & debug I sometimes want the alarms more frequent than 1 per minute.
      if (Alarmsecond >59){
        Alarmsecond =0;
        Alarmminute = Alarmminute+1;  
        if (Alarmminute > 59) 
        {  //error catch - if alarmminute=60 the interrupt never triggers due to rollover!
          Alarmminute =0; 
          Alarmhour = Alarmhour+1; 
          if (Alarmhour > 23) { //uhoh a day rollover, but we dont know the month..so we dont know the next day number?
            Alarmhour =0; 
            midnightRollover = true;      
             // sleep for a total of 64 seconds (12 x 8s) to force day "rollover" while we are in this loop
             // this causes a small gap in the timing once per day, but I only use sub minute sampling for debugging anyway.
             for (int j = 0; j <12; j++){
             LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF); 
             }
          }
        }
      }
      
      //The sample interval must ALWAYS be greater than the time to acquire samples and flush eeprom buffer data to the SD PLUS ~1 second for SD write latency
      RTC_DS3231_setA1Time(Alarmday, Alarmhour, Alarmminute, Alarmsecond, 0b00001000, false, false, false);  
      //The variables ALRM1_SET bits and ALRM2_SET are 0b1000 and 0b111 respectively.
      //RTC_DS3231_setA1Time(byte A1Day, byte A1Hour, byte A1Minute, byte A1Second, byte AlarmBits, bool A1Dy, bool A1h12, bool A1PM)
    } // terminator for second else case of if (SampleIntervalMinutes > 0) 

    RTC_DS3231_turnOnAlarm(1);
    if (!RTC_DS3231_checkAlarmEnabled) {
      Serial.print(F("Unable to set next alarm!")); Serial.flush();
      error;
    }

#ifdef ECHO_TO_SERIAL
    Serial.print(F("A1 Set:"));
    Serial.print(t_hour, DEC);
    Serial.print(':');
    Serial.print(t_minute, DEC);
    Serial.print(':');
    Serial.print(t_second, DEC);
    Serial.print(F(" wake in:"));
    if (SampleIntervalMinutes == 0){
    Serial.print(SampleIntervalSeconds);
    Serial.println(F("s"));
    }
    else
    {
    Serial.print(SampleIntervalMinutes);
    Serial.println(F("m"));
    }
    Serial.println();Serial.flush();
#endif

    sampleIntervalTime += (micros() - startMicros); startMicros = micros();

    //=================================================================================================
    //Only hardware event counter configurations (like a tipping bucket rain gauge) use this sub-loop!
    //================================================================================================
    //This sub-loop increments "INT1_eventCounter" in sleepNwait4D3Interrupt()
    //until an RTC interrupt occurs on INT0, which breaks out of the sub-loop because rtc_INT0_Flag = true;

#if defined(AdxlDripSensor) || defined(TipBucket_RainGauge) //this D3 interrupt driven code is shared between Drip sensors & Tipping Bucket sensors

    while (rtc_INT0_Flag == false) { //this is the "sub-loop" that acts as an event counter

#ifdef AdxlDripSensor  // a tap just occured so need to read & clear the registers to turn off the interrupt alarm
      adxl345lnterruptSource = i2c_readRegisterByte(ADXL345_ADDRESS, ADXL345_INT_SOURCE); //read the ADXL345_INT_SOURCE register to clear it
      bytebuffer1 = i2c_readRegisterByte(ADXL345_ADDRESS, 0x2b);         //read the ADXL345_ACT_TAP_STAT register to clear it
#endif

#ifdef ECHO_TO_SERIAL
      if (adxl345lnterruptSource & 0b00010000) {
        Serial.print(F("Event Count:"));
        Serial.println(INT1_eventCounter);
      }
      //if(adxl345lnterruptSource & 0b01000000){Serial.println(F("*** Single Tap *** ")); }
      Serial.flush(); // when running without echo to serial, unit will be faster!
#endif

      // Pip indicator LED
      digitalWrite(SensorReadLED, HIGH);  // only need to pip the led for 15ms, no matter how long the rest of your delay is
      LowPower.powerDown(SLEEP_15MS, ADC_OFF, BOD_OFF);
      digitalWrite(SensorReadLED, LOW);

#ifdef TipBucket_RainGauge   //tipping buckets need more time for the reed switch to open and debounce
      LowPower.powerDown(SLEEP_120MS, ADC_OFF, BOD_OFF);
      LowPower.powerDown(SLEEP_30MS, ADC_OFF, BOD_OFF);//limits to 1000/165 = 5-6 bucket tips per second
      //Most Rain gauges listed reed switch closures of ~100ms, & bounce times of ~1ms
      //eg: TR-525M: 135 ms, Range 27"/hour(=only 2 tips/second!), TR4: 135 ms, Met1 380: +/-5% (30 to 120 mm/hr)
#endif

#ifdef AdxlDripSensor  // a delay so that drip strike-surface stops vibrating
      LowPower.powerDown(SLEEP_60MS, ADC_OFF, BOD_OFF); //limits to 1000/32 = 30 cycles per second: "ACTUAL" tested output goes down to 11 counts per sec with this delay
      // post flash delay lengthened from 30ms in 201612 due to some of the higher energy drips double counting!
      // using the I2c bus time of for the led flashes only works if you have a slow I2c bus!
      adxl345lnterruptSource = i2c_readRegisterByte(ADXL345_ADDRESS, ADXL345_INT_SOURCE); //read the interrupt register AGAIN to clear it in case an interupt was generated during the delay
      bytebuffer1 = i2c_readRegisterByte(ADXL345_ADDRESS, 0x2b); //read the ADXL345_ACT_TAP_STAT register AGAIN to clear it
#endif

      digitalWrite(INT1_pin, HIGH);      //set internal pull up the interrupt pin - not needed if you have hardware pullups but doesn't hurt anything
      digitalWrite(SensorReadLED, LOW);  //just makeing sure it is off

      time4int1events += (micros() - startMicros);
      sleepNwait4D3Interrupt();  //sleep & wait for RTC OR INT01 event
      startMicros = micros();

    }  //LOOP TERMINATOR for while(rtc_INT0_Flag == false) event counter!

    startMicros = micros(); //  duplicate here might be needed for loop breakout //repeated starts are not a problem, but missed starts are!
    if (rtc_INT0_Flag) {      //if you broke out of the while(rtc_INT0_Flag == false)loop, then your RTC interrupt fired.
      RTC_DS3231_turnOffAlarms();
    }

#else  //this is the RTC-only sleepNwait routine when no hardware/interrupt driven event-counter is connected

    digitalWrite(SensorReadLED, LOW);  //just makeing sure it is off
    sleepNwait4RTC();  //sleep and wait for RTC ONLY -  call is inside the main cycle counter loop */

    start = micros();
    if (rtc_INT0_Flag) {      //if you broke out of the while loop, then your RTC interrupt fired.
      RTC_DS3231_turnOffAlarms();      //and you need to clear that alarm
    }

#endif  //terminator for #if defined(AdxlDripSensor) || defined(TipBucket_RainGauge)

   //================================================================================================
   } //TERMINATOR for "cycle of samples" LOOP (# of sample cycles buffered in eeprom before sd save )
   //================================================================================================

//=================================================================================================================
} //TERMINATOR for the MAIN program LOOP - from here you go back to the start & take readings from all your sensors.
//========================================= END OF MAIN LOOP=======================================================


// ===================================================================
//              *  *   *   *  *  COMMON FUNCTIONS  *   *   *   *   *
// ===================================================================
//=================================================
//*** SLEEP & WAIT for INT1 event OR INT0 event ***
//=================================================
// Based on information from http://www.gammon.com.au/forum/?id=11497
// Responds to both hardware interrupt lines

#if defined(AdxlDripSensor) || defined(TipBucket_RainGauge)
//Note that this sleep enables sensor interrupts on D3, AND the RTC alarm interrupt on D2

void sleepNwait4D3Interrupt()    //the gammon version I'm using
{
#ifdef RTCPOWER_PIN  //if using pin power on RTC, now depower it:
  digitalWrite(RTCPOWER_PIN, LOW); // RTC's vcc line is jumpered to this pin - driving this LOW FORCES to the RTC to draw power from the coin cell during sleep
#endif

  ADCSRA = 0; //disable the ADC - worth 334 µA during sleep
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  noInterrupts ();          // make sure we don't get interrupted before we sleep
  attachInterrupt(0, rtcAlarmTrigger, LOW); //this is the main RTC interrupt - It breaks you out of the tap counter while loop
  attachInterrupt(1, INT1pinD3_triggered, LOW); //sensor alarm=low connected to pin D3
  EIFR = _BV (INTF0);  // clear flag for interrupt 0  see https://gammon.com.au/interrupts
  EIFR = _BV (INTF1);  // clear flag for interrupt 1
  sleep_enable();
  // or turn off brown-out enable http://www.gammon.com.au/forum/?id=11497 
  // BOD prevents EEPROM corruption by resetting the MCU if the voltage drops below the threshold. Since we have nothing in eeprom, we can turn it off
  MCUCR = bit (BODS) | bit (BODSE);  // BODS and BODSE to logical 1 // turn off brown-out enable select // re-enables on wake-up
  MCUCR = bit (BODS);      // this must be done within 4 clock cycles of above
  interrupts ();           // interrupts allowed now, next instruction WILL be executed
  sleep_mode();
  //HERE AFTER WAKING UP
  detachInterrupt(1); //eventCounter
  detachInterrupt(0); //if you broke out of the eventCounter loop, it was because your RTC interrupt fired.
  if (INT1_Flag && INT1_eventCounter < 429496729) { //INT1_Flag is set in the ISR
    INT1_eventCounter = INT1_eventCounter + 1;
    INT1_Flag = false;
  }

#ifdef RTCPOWER_PIN  // about to generate I2C traffic, so power the rtc from the pin its connected to
  pinMode (RTCPOWER_PIN, OUTPUT);
  digitalWrite(RTCPOWER_PIN, HIGH);
#endif

  // PRR settings to save runtime power - DO NOT mess with timer0!
  // power_twi_disable();  // Do not turn off twi
  power_timer1_disable();  // disables PWM 9 & 10  Servo library__ uses timer1
  power_timer2_disable();  // disables PWM 3 & 11
  //power_spi_disable();   // disable the clock to the SPI module - only if de-powering the SD cards!
  ADCSRA = 0;              // disable ADC by setting ADCSRA register to 0
  power_adc_disable();     // add this extra step to disable the ADC for run & idle modes only, not needed during sleep
  ACSR = 0b10000000;       // Disable the analog comparator by setting the ACD bit (bit 7) of the ACSR register to one.
#ifndef ECHO_TO_SERIAL   // dont disable the usart when you are in debug mode
  power_usart0_disable();
  //or
  //UCSR0B &= ~bit (RXEN0); // disable receiver
  //UCSR0B &= ~bit (TXEN0); // disable transmitter
#endif
}  //terminator for void sleepNwait4D3Interrupt()

//======================================================================================
#else  // a continuation of #if defined(AdxlDripSensor) || defined(TipBucket_RainGauge)
//======================================================================================
//======================================================
//   SLEEP and WAIT for RTC - with no INT1 event counter
//======================================================

void sleepNwait4RTC() {  //old method of sleeping (without using the RocketScreem lib)
  
#ifdef RTCPOWER_PIN  //if using pin power on RTC, now depower it:
  digitalWrite(RTCPOWER_PIN, LOW); //driving this LOW FORCES to the RTC to draw power from the coin cell during sleep
#endif

  ADCSRA = 0; //disable the ADC - worth 334 µA during sleep
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  noInterrupts (); // make sure we don't get interrupted before we sleep
  attachInterrupt(0, rtcAlarmTrigger, LOW); //RTC alarm-low connected to pin D2
  EIFR = bit (INTF0);  // clear flag for interrupt 0 (D2)  see https://gammon.com.au/interrupts
  sleep_enable();
  MCUCR = bit (BODS) | bit (BODSE);  // turn on brown-out enable select
  MCUCR = bit (BODS);        // this must be done within 4 clock cycles of above  //also see http://jeelabs.org/2010/09/01/sleep/
  interrupts ();           // interrupts allowed now, next instruction WILL be executed
  sleep_mode();
  //HERE AFTER WAKING UP
  detachInterrupt(0);

#ifdef RTCPOWER_PIN
  pinMode (RTCPOWER_PIN, OUTPUT);
  digitalWrite(RTCPOWER_PIN, HIGH); //about to generate I2C traffic, so power the rtc from the pin
#endif

  // PRR settings to save runtime power - DO NOT mess with timer0!
  // power_twi_disable();  // Do not turn off twi - RTC uses it!
  power_timer1_disable();  // Disables PWM 9 & 10  Servo library__ uses timer1
  power_timer2_disable();  // Disables PWM 3 & 11
  //power_spi_disable();   // Disable the clock to the SPI module - only if de-powering the SD cards!
  ADCSRA = 0;              // Disable ADC by setting ADCSRA register to 0
  power_adc_disable();     // Add this extra step to disable the ADC for run & idle modes only, not needed during sleep
  ACSR = 0b10000000;       // Disable the analog comparator by setting the ACD bit (bit 7) of the ACSR register to one.
#ifndef ECHO_TO_SERIAL     // Dont disable the usart when you are in debug mode
  power_usart0_disable();
  //or
  //UCSR0B &= ~bit (RXEN0); // disable receiver
  //UCSR0B &= ~bit (TXEN0); // disable transmitter
#endif
}  //terminator for sleepNwait4RTC
//======================================================================================
#endif  //terminator for #if defined(AdxlDripSensor) || defined(TipBucket_RainGauge)
//======================================================================================
//==========================
//  INT1 TRIGGERED ISR
//==========================
void INT1pinD3_triggered() {
  INT1_Flag = true;  //set a flag, & handle in mainloop();
}
//===========================
//  INT0 RTC TRIGGERED ISR
//===========================
void rtcAlarmTrigger() {  //set a flag, & handle in mainloop();
  rtc_INT0_Flag = true;
}

//===========================================================================================
// Once per Day events triggered by the midnight rollover
//===========================================================================================

void oncePerDayEvents() {
  dailyCPUuptime = sampleIntervalTime + time4int1events + time2writeSDdata; //a summary of all the timer variables
  integerBuffer = freeRam(); if (integerBuffer < freeMem) {freeMem = integerBuffer;}
#ifdef COINCELLREAD
  readCoinCell();  // Check RTC coin cell once per day 
  //readCoinCell must be in front of readBattery(); so analog pin cap does not stay connected to that divider
#endif
  batteryVoltage = readBattery();// this updates the battery voltage reading at least once per day
  VccBGap = getRailVoltage(); // our current rail voltage, as compared to the 1.1v internal bandgap
#ifdef AdxlDripSensor
  readADXL345(); //Read the accelerometer xyz once per day to make sure the sensor has not moved or fallen over
#endif

  file.open("StatusLog.csv", O_WRITE | O_APPEND);
  //  .write()  Writes binary data bytes to the port.
  //  .print()  Prints data to the port as human-readable ASCII text. 
  //   so Serial.write accepts single characters where Serial.print accepts strings

  //this loop retrieves the last record stored in the eeprom buffer
  //Because j is unsigned, it will always be greater than or equal to zero. 
  //When you decrement an unsigned variable that is equal to zero, it will wrap around to a very large number. - So only count down to 1 and use (j-1)
  for (int j = (PagesBuffered2Eeprom); j >= 1; j--) {
    Read_i2c_eeprom_page(EEPROM_ADDRESS, (RolloverEEpMemAddress - ((j - 1)*EEpromPageSize)), EEPROMBuffer, sizeof(EEPROMBuffer) );
    file.write(EEPROMBuffer, sizeof(EEPROMBuffer));
  }
  
  // cumulative daily sensor totals:
  file.print(F(",,"));

#if defined(AdxlDripSensor) || defined(TipBucket_RainGauge)
  file.print(dayTotalINT1count); file.print(F(","));
#endif

#ifdef AdxlDripSensor //read the axes once per day to make sure the unit has not fallen over
  file.print(smoothACCx); file.print(F(","));
  file.print(smoothACCy); file.print(F(","));
  file.print(smoothACCz); file.print(F(",,"));
#endif

  // Logger Status information:
  file.print(eTime); file.print(F(","));  // eTime is just a different way to store a time stamp
  file.print(freeMem); file.print(F(","));
  file.print(batteryVoltage - postSDbatt); file.print(F(","));
  file.print(VccBGap - postSDvcc); file.print(F(","));
  file.print(CoinCellV); file.print(F(","));
  file.print(dailyCPUuptime); file.print(F(","));
  file.print(sampleIntervalTime); file.print(F(","));
  file.print(time2writeSDdata); file.print(F(","));
  file.print(time4int1events); file.print(F(","));
  file.print(MaxTemp); file.print(F(","));
  file.print(MinTemp); file.print(F(","));
  file.print(PagesBuffered2Eeprom); file.print(F(","));
  file.print(SamplesPerCycle); file.print(F(","));
  file.println();//carridge return
  file.close(); //file close is big power user, so check the reg. vdrop under load:
  integerBuffer = readBattery(); if (integerBuffer < postSDbatt) {
    postSDbatt = integerBuffer;
  }
  integerBuffer = getRailVoltage(); if (integerBuffer < postSDvcc) {
    postSDvcc = integerBuffer; //this takes 10msec!
  }

  //reset variables for next day
  dayTotalINT1count = 0; dailyCPUuptime = 0; sampleIntervalTime = 0; time4int1events = 0; time2writeSDdata = 0;
  MaxTemp = 0; MinTemp = 9999;
//===================================================
}  //terminator for void oncePerDayEvents() function
//===================================================

// ==========================================================================================
// I2C SENSOR MEMORY REGISTER FUNCTIONS
//===========================================================================================

byte i2c_readRegisterByte(uint8_t deviceAddress, uint8_t registerAddress)
{
  byte registerData;
  Wire.beginTransmission(deviceAddress); //set destination target
  Wire.write(registerAddress);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)deviceAddress, (uint8_t)1);
  registerData = Wire.read();
  return registerData;
}

byte i2c_writeRegisterByte(uint8_t deviceAddress, uint8_t registerAddress, byte newRegisterByte)
{
  byte result;
  Wire.beginTransmission(deviceAddress);
  Wire.write(registerAddress);
  Wire.write(newRegisterByte);
  result = Wire.endTransmission();

  if (result > 0)   //error checking
  {
#ifdef ECHO_TO_SERIAL   //NOTE: only call halt on error if in debug mode!
    Serial.print(F("FAIL in I2C register write! Result code: "));
    Serial.println(result);
    error();
#endif
  }
  // NOTE: some sensors need settling time after a register change but MOST do not
  return result;
}

byte i2c_setRegisterBit(uint8_t deviceAddress, uint8_t registerAddress, uint8_t bitPosition, bool state) {
  byte registerByte;
  byte result;
  registerByte = i2c_readRegisterByte(deviceAddress, registerAddress);  //load the existing register contents
  if (state) {    // when state = 1
    registerByte |= (1 << bitPosition);  // bitPosition of registerByte now = 1
  }
  else {         // when state = 0
    registerByte &= ~(1 << bitPosition); // bitPosition of registerByte now = 0
  }
  result = i2c_writeRegisterByte(deviceAddress, registerAddress, registerByte);
  return result;    // result =0 if the writing the new data to the registry went ok
}

bool i2c_getRegisterBit(uint8_t deviceAddress, uint8_t registerAddress, uint8_t bitPosition) {
  byte registerByte;
  registerByte = i2c_readRegisterByte(deviceAddress, registerAddress);
  return ((registerByte >> bitPosition) & 0b00000001);
}

// Reads numberOfBytes starting from address register on device in to sixByteArray array  // this can also be used to read only one byte...
// Note: There is nothing in the I2C protocol to guarantee a sensor sends the number of bytes you asked for
// Arduino is clock master, and will simply recieve bytes of 0xFF even if the sensor does not send them!
void i2c_readToByteArray(uint8_t deviceAddress, uint8_t registerAddress, uint8_t numberOfBytes, byte arrayPointer[]) {
  Wire.beginTransmission(deviceAddress); // start transmission to deviceAddress
  Wire.write(registerAddress);             // sends address to read from
  Wire.endTransmission();         // end transmission
  //Wire.beginTransmission(deviceAddress); // start transmission to deviceAddress
  Wire.requestFrom(deviceAddress, numberOfBytes);    // request num bytes from deviceAddress

  int i = 0;
  while (Wire.available())        // device may send less than requested (abnormal)
  {
    arrayPointer[i] = Wire.read();    // receive a byte
    i++;
  }
  Wire.endTransmission();         // end transmission
}

void i2c_write16bitRegister(uint8_t deviceAddress, uint8_t registerAddress, uint16_t value) {
  Wire.beginTransmission(deviceAddress);
  Wire.write(registerAddress);  //register pointer
  Wire.write(highByte(value));  // sends MSB  //same as (x) >> (8)
  Wire.write(lowByte(value));   // sends LSB  //same as  (x) & (0xff)
  Wire.endTransmission();
}

uint16_t i2c_read16bitRegister(uint8_t deviceAddress, uint8_t registerAddress) { 
  //returns unsigned int! watch out for twos complement issues!
  uint16_t value_16bit;
  Wire.beginTransmission(deviceAddress);
  Wire.write(registerAddress);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)deviceAddress, (uint8_t)2);
  value_16bit = Wire.read();
  value_16bit <<= 8;
  value_16bit |= Wire.read();
  return value_16bit;
}
// for 24 bit values see https://forum.arduino.cc/index.php?topic=393956.0


//========================================================================================
// READ & WRITE data to the I2C EEprom (AT24C32 to AT24C256) all support 32byte pagewrites
//========================================================================================
// see http://www.hobbytronics.co.uk/eeprom-page-write for the details on these two functions
// The Arduino Wire library only has a 32 character buffer, so that is the maximun we can send.
// This buffer includes the two address bytes which limits our data payload to 30 bytes
// alternate libs at https://github.com/jlesech/Eeprom24C32_64/blob/master/Eeprom24C32_64.cpp  
// and https://github.com/mikaelpatel/Arduino-Storage/blob/master/src/Driver/AT24CXX.h

void Write_i2c_eeprom_page( uint8_t deviceAddress, uint16_t registerAddress_16bit, char* arrayPointer)
{
  uint8_t i = 0;
  Wire.beginTransmission(deviceAddress);
  Wire.write((byte)((registerAddress_16bit) >> 8));   // send the MSB of the address
  Wire.write((byte)((registerAddress_16bit) & 0xFF)); // send the LSB of the address
  do {
    Wire.write((byte) arrayPointer[i]);
    i++;
  }
  while (arrayPointer[i]);
  Wire.endTransmission();
  digitalWrite(BLUE_PIN, HIGH);  //just an indication of the eeprom saving
  //delay(6);  // data sheet says it takes 5ms for a page write of 32 bytes, whilst time for writing individual bytes = 3.5ms each byte
  LowPower.powerDown(SLEEP_15MS, ADC_OFF, BOD_ON); //sleeping for 15ms uses less power than delay(6);
  digitalWrite(BLUE_PIN, LOW);
}

void Read_i2c_eeprom_page( uint8_t deviceAddress, uint16_t registerAddress_16bit, char* arrayPointer, uint8_t numOfChars)
{
  uint8_t i = 0;
  Wire.beginTransmission(deviceAddress);
  Wire.write((byte)(registerAddress_16bit >> 8));   // send the MSB of the address
  Wire.write((byte)(registerAddress_16bit & 0xFF)); // send the LSB of the address
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)deviceAddress, (uint8_t)(numOfChars - 1));
  while (Wire.available()) {
    arrayPointer[i++] = Wire.read(); //added brackets
  }
}

//=================================================================================
//  Flush EEPROM buffered data to the SD card
//=================================================================================

void flushEEpromBuffer() {  //ADC reads integrated here to test main battery under load

  digitalWrite(RED_PIN, HIGH);

  batteryVoltage = readBattery();// Check psupply to make sure its ok to write to sd card - readBattery triggers shutdown if battery is low
  ADCSRA=keep_ADCSRA; // used for PRR shutdown & wakeup of ADC
  ADMUX = bit (REFS0) | (0 & 0x07);  // set AVcc to rail and select input port A0
  uint16_t lowestSDwriteBatt=analogRead(batteryPin); //throw away reading
  
  // Initialize SdFat or print error message and halt
  if (!sd.begin(chipSelect, SPI_FULL_SPEED)) {  // Use half speed like the native library for old cards // change to SPI_FULL_SPEED for more performance.
    Serial.println(F("Could NOT initialize SD Card")); Serial.flush();
    error();
  }

#ifdef ECHO_TO_SERIAL
  Serial.println(F("--Writing to SDcard --"));
  Serial.flush();
#endif

  file.open(FileName, O_WRITE | O_APPEND); // open the file for write at end like the Native SD library
  lowestSDwriteBatt=analogRead(batteryPin);
  CurrentEEpromMemAddress = 0; //reset the eeprom memory location counter back to the beginning of the stack

  for (int i = 0; i < SamplesPerCycle; i++) {  //loop to read from I2C eeprom and write to SD card

    for (int j = 0; j < PagesBuffered2Eeprom; j++) {
      digitalWrite(RED_PIN, HIGH);  //Let the user know SD card writing is in progress
      Read_i2c_eeprom_page(EEPROM_ADDRESS, CurrentEEpromMemAddress, EEPROMBuffer, sizeof(EEPROMBuffer) );  //there will be a few blank spaces
      CurrentEEpromMemAddress += EEpromPageSize;
      digitalWrite(RED_PIN, LOW);  // eeprom read takes about 5ms so this pulses the red LED
      file.write(EEPROMBuffer, sizeof(EEPROMBuffer));
    } // terminator: for (int j = 0; j < PagesBuffered2Eeprom; j++)   
    
    file.println();  //add a carridge return to the file after each new record is written
    recordsSaved++;  //keeps track of how ma
  }  // terminator:    for(int i = 0; i < SamplesPerCycle; i++) loop
  
  file.close();
  //use the SD event to keep track of system under load
  integerBuffer = analogRead(batteryPin);if(integerBuffer<lowestSDwriteBatt){lowestSDwriteBatt=integerBuffer;}
  ADCSRA = 0;  power_adc_disable();  // disable the ADC   //getRailVoltage also shuts off ADC at end of function
  integerBuffer = getRailVoltage(); if (integerBuffer < postSDvcc) {postSDvcc = integerBuffer;} 
  float volts=(lowestSDwriteBatt/resistorFactor)*referenceVolts;
  integerBuffer = (int)(volts * 1000.00);if (integerBuffer < postSDbatt){postSDbatt=integerBuffer;}
  
  digitalWrite(RED_PIN, LOW);
  CurrentEEpromMemAddress = 0; //now that we have copied the buffer data to SD, we can start filling it from the beginning

}  // terminator: END of function that flushes data buffered in eeprom to the SD card

//=============================================================================
//  SD card function : CREATE NEW LOGFILE
//=============================================================================

void createNewLogFile() {

  // Initialize SdFat or print a detailed error message and halt
  // Use half speed like the native library. //change to SPI_FULL_SPEED for more performance.//SPI_HALF_SPEED is for older cards
  if (!sd.begin(chipSelect, SPI_FULL_SPEED)) {
    Serial.println(F("Could NOT initialize SD Card")); Serial.flush();
    error();
  }

  // If we are creating another file after newFileInterval, then we must close any open files first.
  //if (file.isOpen()) {
  //  file.close();
  //}
  //delay(10);
  
// 2 GB or smaller cards should be formatted FAT16 which has a limit of 512 entries in root  // https://forum.arduino.cc/index.php?topic=301335.0
  for (int i = 0; i < 512; i++) {
    snprintf(FileName, sizeof(FileName), "data%03d.csv", i);
    // O_CREAT = create the file if it does not exist
    // O_EXCL = fail if the file exists, O_WRITE - open for write
    if (file.open(FileName, O_CREAT | O_EXCL | O_WRITE))
    {
      break; //if you can open a file with the new name, break out of the loop
    }
    //delay(2);
  }

  if (!file.isOpen()) {
    Serial.println(F("Unable to open SD file!"));
    error();  //lethal error in all cases
  }

#ifdef ECHO_TO_SERIAL
  Serial.print(F("Logging to: "));
  Serial.println(FileName);
#endif  // ECHO_TO_SERIAL

  if (!file.timestamp(T_CREATE, t_year, t_month, t_day, t_hour, t_minute, t_second)) {
#ifdef ECHO_TO_SERIAL
    Serial.println(F("Can't timestamp the new file"));
    error();
#endif
  }

  /*
    // set write/modification date time - optional!
    if (!file.timestamp(T_WRITE,t_year, t_month, t_day, t_hour, t_minute, t_second)) {
    now.minute(),now.second() )) {
    #ifdef ECHO_TO_SERIAL
      Serial.println(F("Can't write to the new file"));
      error();
    #endif
    }
    // set access date
    if (!file.timestamp(T_ACCESS,t_year, t_month, t_day, t_hour, t_minute, t_second)) {
    }
    #ifdef ECHO_TO_SERIAL
      Serial.println(F("Can't set new file access date"));
      error();
    #endif
    }
  */

  writeHeaderInfo();
  file.close();
  //the New file creation event is one of the biggest power loads on the system so I use it to track the battery under load
  integerBuffer = readBattery(); if (integerBuffer < postSDbatt) {postSDbatt = integerBuffer;}
  integerBuffer = getRailVoltage(); if (integerBuffer < postSDvcc) {postSDvcc = integerBuffer;} //this takes 10msec!
  integerBuffer = freeRam(); if (integerBuffer < freeMem) { freeMem = integerBuffer;} //checks freeram before exiting the function
#ifdef ECHO_TO_SERIAL
  Serial.println(F("New log file created on the SD card!"));
  Serial.flush();
#endif  // ECHO_TO_SERIAL

//=================================================================================  
}   //end of the create new log file routine
//=================================================================================

//=================================================================================
//  SD card function: Write HEADER INFO to file
//=================================================================================

void writeHeaderInfo()
{

  // HEADER INFO
  file.print(FileName); file.print(F(",Created:,")); file.println(CycleTimeStamp);
  file.println((__FlashStringHelper*)boilerplate1); //need the helper to print because data was stored in PROGMEM
  file.println((__FlashStringHelper*)boilerplate2);
  file.println((__FlashStringHelper*)contact);
  file.print(F("CodeBuild:,"));file.println((__FlashStringHelper*)codebuild); //for the entire path + filename
  file.print(F("UnitDetails:,")); file.println((__FlashStringHelper*)unitDetails);
  file.print(F("EEPROM@(dec):,")); file.print(EEPROM_ADDRESS);
  file.print(F(",Samp/Cycle:,")); file.print(SamplesPerCycle);
  file.print(F(",Interval(min):,")); file.print(SampleIntervalMinutes);
  file.print(F(",NewFile@:,")); file.print(newFileInterval); file.println(F(",records"));
  file.println();

  file.print(F("Sensors:,"));

//examples of how you could use #ifdef statements to modify the header data for different sensors:
#ifdef thermistor 
  //file.print(F("100K NTC thermistor,"));file.println(F("OS.Tpin:,"));file.print(togglePin);
  //file.println(F("Tres=SeriesRes/((32767(15bit)*(VccBGap/1100)/(RawADC))-1)"));
  //file.println(F("ThermistorTemp=(1/((ln(Tres/TnominalR)/TBeta)+(1.0/(TnominalTemp+273.15)))-273.15"));
#endif

#ifdef TS_DS18B20
  file.print(F("DS18B20,"));
#endif

#ifdef MCP9808_I2CADDR
  file.print(F("MCP9808,"));
#endif

#ifdef AdxlDripSensor
  file.print(F("ADXL345@,")); file.print(getAdxl_rate() / 2); file.print(F("Hz,")); file.print(F("Sens:")); file.print(adxlTapSensitivity);
#endif

  file.println();
  file.println(F("Etime=(CELL*interval in Seconds)/(86400)+ \"1/1/1970\"")); //note you can "escape" a special character by preceding it with a backslash
if (printStatusLogHeaders) {  
  file.println(F(",,,,,,,,,Daily Totals,,,,,StatusLog.csv Data")); // commas push this over to right
}// terminator for if (printStatusLogHeaders)

//this is STANDARD for ALL units : data from first 30 byte eeprom buffer page
file.print(F("TimeStamp,BatT(mV),RAWtemp(*.0625),"));

//data from secoind 30 byte eeprom buffer page
#if defined(AdxlDripSensor) || defined(TipBucket_RainGauge)
  file.print(F("Tip/Drips,Coin(2.3v min),postSDbatt,postSDvcc,Rail(mV),"));
#endif

//THIRD buffered data string headers get added here if needed

//system info and daily totals: Note this data is only written to "StatusLog.csv once on the midnight rollover
if (printStatusLogHeaders) {
#if defined(AdxlDripSensor) || defined(TipBucket_RainGauge)
  file.print(",DailyTotal-INT1events,");
#endif
#ifdef AdxlDripSensor
  file.print(F("AccX,"));
  file.print(F("AccY,"));
  file.print(F("AccZ,"));
#endif

  file.print(F(",Etime,MinFreeRam,SDbatt(mV)drop,SDrail(mV)drop,CoinCell(mV),DailyCPU runtime(us),SamplingTime(us),SDsaving(us),int01event handling(us),MaxTemp(raw),MinTemp,PagesBuffered/cycle,Samples/Cycle,"));
} // terminator for if (printStatusLogHeaders)

  file.println();//terminates main column header line
  
integerBuffer = freeRam(); if (integerBuffer < freeMem) { freeMem = integerBuffer;} //check dynamic memory left with all that string printing

//=================================================
} //    terminator for writeHeaderInfo function
//=================================================
// Arduino IDE saves the files as unicode. If you type the degree character and upload to the Arduino before saving, 
// you will get a different result to when you open that saved file and upload it
// ((char)230); gets around the unicode A-hat problem...sort of... 176 is the degree mark


//=================================================
//----------Voltage monitoring functions ----------
//=================================================

int getRailVoltage()    // from http://forum.arduino.cc/index.php/topic,38119.0.html
{
  power_adc_enable(); ADCSRA = keep_ADCSRA; //if you shut down the ADC with PRR
  int result; const long InternalReferenceVoltage = 1100L;

  for (int i = 0; i < 5; i++) { // have to loop 4 times before it yeilds consistent results - cap on aref needs to settle

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    // For mega boards
    // const long InternalReferenceVoltage = 1100L;  // Adjust this value to your boards specific internal BG voltage x1000
    // REFS1 REFS0          --> 0 1, AVcc internal ref.
    // MUX4 MUX3 MUX2 MUX1 MUX0  --> 11110 1.1V (VBG)
    ADMUX = (0 << REFS1) | (1 << REFS0) | (0 << ADLAR) | (1 << MUX4) | (1 << MUX3) | (1 << MUX2) | (1 << MUX1) | (0 << MUX0);
#else
    // For 168/328 boards
    // const long InternalReferenceVoltage = 1100L;
    // Adust this value to your boards specific internal BG voltage x1000
    // REFS1 REFS0          --> 0 1, AVcc internal ref.
    // MUX3 MUX2 MUX1 MUX0  --> 1110 1.1V (VBG)
    ADMUX = (0 << REFS1) | (1 << REFS0) | (0 << ADLAR) | (1 << MUX3) | (1 << MUX2) | (1 << MUX1) | (0 << MUX0);
#endif
    //delay(3);
    // Start a conversion
    ADCSRA |= _BV( ADSC );
    // Wait for it to complete
    while ( ( (ADCSRA & (1 << ADSC)) != 0 ) );
    // Scale the value
    result = (((InternalReferenceVoltage * 1023L) / ADC) + 5L); //scale Rail voltage in mV
    // note that you can tune the accuracy of this function by changing InternalReferenceVoltage to match your board
    // just tweak the constant till the reported rail voltage matches what you read with a DVM!
  }     // end of for (int i=0; i <= 3; i++) loop

  ADMUX = bit (REFS0) | (0 & 0x07);  // set AVcc to rail and select input port A0 + engage new Aref
  ADCSRA = 0;  power_adc_disable();  // disable the ADC

  return result;
//===================================
}  // terminator for getRailVoltage()
//===================================
// The spec sheet gives a nominal value of 1.1 volts, but states that it can vary from 1.0 to 1.2 volts.
// That means that any measurement with this method could be off by as much as 10%
// see http://provideyourown.com/2012/secret-arduino-voltmeter-measure-battery-voltage/
// about replacing the constant 1125300L with    scale_constant = internal1.1Ref * 1023 * 1000
// where internal 1.1Ref = 1.1 * [Vcc1 (per actual voltmeter) / postSDbatt (per this readVcc() function)]
// also https://code.google.com/p/tinkerit/wiki/SecretVoltmeter


int readBattery()       //Assumes analog read from (high side=10Meg ohm)/(low side=3.3Meg ohm) resistor divider
{
  power_adc_enable(); ADCSRA = keep_ADCSRA;        //turn on the ADC
  int result;
  ADMUX = bit (REFS0) | (0 & 0x07);                // set AVcc to rail and select channel A0
  result=analogRead(batteryPin);                   //always throw away the first reading!
  LowPower.powerDown(SLEEP_15MS, ADC_ON, BOD_ON);  //note the ADC is left for this cap stabilization period
  float medianVraw = median_of_3( analogRead(batteryPin), analogRead(batteryPin), analogRead(batteryPin));
  ADCSRA = 0;  power_adc_disable(); //disable the ADC after the reading
  float volts = (medianVraw / resistorFactor) * referenceVolts ; // calculate the ratio  Need to make sure reference volts is accurate by measurement on each unit!
  result = (int)(volts * 1000.00); // conv to millivolts

#ifdef ECHO_TO_SERIAL  //OOPS! you forgot there is no voltage on the batteryVoltage pin when you are connected to USB cable!
  result = result + 5000; //if you start seeing > 7V and there is no smoke rising form the board you know you forgot something...
#endif

  if (result < blueBatteryLevel) {
    SensorReadLED = BLUE_PIN; //if sensor pip is blue, batteries are getting low
  }
  if (result < redBatteryLevel) {
    SensorReadLED = RED_PIN; //If the sensor pip goes red, it is time to change the batteries.
  }

  if (result < CutoffVoltage) { //The pro mini board vregulators need 3.35 v minimum for stable output
    if (file.isOpen()) {
      file.close();
    }
#ifdef ECHO_TO_SERIAL
    Serial.print(result); Serial.println(F("Bat too low for SD card writing!")); Serial.flush();
#endif
    for (int k = 0; k < 100; k++) {           //a dramatic visual warning that you've triggered the low battery error
      digitalWrite(RED_PIN, HIGH); LowPower.powerDown(SLEEP_60MS, ADC_OFF, BOD_ON); //this is very handy when debugging!
      digitalWrite(RED_PIN, LOW); LowPower.powerDown(SLEEP_60MS, ADC_OFF, BOD_ON);
      digitalWrite(GREEN_PIN, HIGH); LowPower.powerDown(SLEEP_60MS, ADC_OFF, BOD_ON);
      digitalWrite(GREEN_PIN, LOW); LowPower.powerDown(SLEEP_60MS, ADC_OFF, BOD_ON);
      digitalWrite(BLUE_PIN, HIGH); LowPower.powerDown(SLEEP_60MS, ADC_OFF, BOD_ON);
      digitalWrite(BLUE_PIN, LOW); LowPower.powerDown(SLEEP_60MS, ADC_OFF, BOD_ON);
    }
    error();
  } // terminator for if (result < CutoffVoltage)

  return result;
}  // terminator for readBattery() function

#ifdef COINCELLREAD

int readCoinCell()  //A dedicated function to read the output of the v.divider on the rTC coincell connected to A1: with float calcs rather than median3 shown above
{
  power_adc_enable(); ADCSRA = keep_ADCSRA; // enable the ADC
  analogReference(DEFAULT);analogRead(RTCcoinPin); //throw away reading to set channel
  LowPower.powerDown(SLEEP_15MS, ADC_ON, BOD_ON);  //ADC on time to settle cap at Avcc
  float avrgVraw = analogRead(RTCcoinPin);
  avrgVraw = avrgVraw + analogRead(RTCcoinPin);
  avrgVraw = avrgVraw + analogRead(RTCcoinPin);
  avrgVraw = avrgVraw / 3; //avrg of readings
  CoinCellV = (int)((avrgVraw * (3.3 / 1023.0)) * 2000); //assumes 2x 4.7 Meg ohm divider
  ADCSRA = 0;  power_adc_disable();     // disable the ADC
}
#endif

//============
// BOILERPLATE
//============
void serial_boilerplate()
{
  Serial.flush();
  Serial.println((__FlashStringHelper*)boilerplate1); //need the helper to print because data is stored in PROGMEM
  Serial.println((__FlashStringHelper*)boilerplate2);
  Serial.println((__FlashStringHelper*)contact);
  Serial.print(F("CodeBuild:"));Serial.println((__FlashStringHelper*)codebuild); //for the entire path + filename
  Serial.println();Serial.flush();
}

//=======================================================================
// Check FREE RAM availiable
//=======================================================================

int freeRam ()     // from: http://learn.adafruit.com/memories-of-an-arduino/measuring-free-memory
{
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

//==============================================================================
// ERROR HANDLER "Houston we have a problem..."
//===============================================================================
// more advanced debugging: http://forum.arduino.cc/index.php?topic=203282.0

void error()
{
  if (file.isOpen()) {
    file.close();
  }
#ifdef ECHO_TO_SERIAL
  Serial.flush();
#endif
#ifdef RTCPOWER_PIN
  digitalWrite(RTCPOWER_PIN, LOW); // driving this LOW FORCES to the RTC to draw power from the coin cell
#endif
#if defined AdxlDripSensor
  i2c_setRegisterBit(ADXL345_ADDRESS, 0x2d, 3, 0); //bit three of POWER_CTL 0x2d puts ADXL345 in standby mode ~0.1 μA
#endif

  for (int CNTR = 0; CNTR < 120; CNTR++) { //spend some time flashing red indicator light on error before shutdown!
    digitalWrite(RED_PIN, HIGH);
    LowPower.powerDown(SLEEP_250MS, ADC_OFF, BOD_ON);
    digitalWrite(RED_PIN, LOW);
    LowPower.powerDown(SLEEP_250MS, ADC_OFF, BOD_ON);
  }
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_ON); //note that the BOD is on here
}


// ===========================================================================================================
// signal processing functions
// ===========================================================================================================
//============================================================================================================
// digitalSmooth FUNCTION for post processing sensor data (generally accelerometers) with outlier rejection
//============================================================================================================
// based on Paul Badger's  http://playground.arduino.cc/Main/DigitalSmooth
// "int *inputArray" passes an array to the function - the asterisk indicates the array name becomes a pointer

int digitalSmooth(int *inputArray) {  //number of elements in this array set by #define filterSamples at start
  int j, k, temp, top, bottom;
  long total;
  static int i;
  boolean done;

  done = 0;                // flag to know when we're done sorting
  while (done != 1) {      // simple swap sort, sorts numbers from lowest to highest
    done = 1;
    for (j = 0; j < (filterSamples - 1); j++) {
      if (inputArray[j] > inputArray[j + 1]) {    // numbers are out of order - swap
        temp = inputArray[j + 1];
        inputArray [j + 1] =  inputArray[j] ;
        inputArray [j] = temp;
        done = 0;
      }
    }
  }

#ifdef ECHO_TO_SERIAL
  Serial.println(F("Sorted Raw ACC readings:"));
  for (j = 0; j < (filterSamples); j++) {   // print the array for debugging
    Serial.print(inputArray[j]);
    Serial.print(F("   "));
  }
  Serial.println(); Serial.flush();
#endif

  // throw out top and bottom 15% of samples - limit to throw out at least one from top and bottom
  bottom = max(((filterSamples * 15)  / 100), 1);
  top = min((((filterSamples * 85) / 100) + 1  ), (filterSamples - 1)); // the + 1 is to make up for asymmetry caused by integer rounding
  k = 0;
  total = 0;
  for ( j = bottom; j < top; j++) {
    total += inputArray[j];  // total remaining indices
    k++;
    // Serial.print(sensSmoothArray[j]);Serial.print("   "); //more debugging output
  }
  //  Serial.println(); Serial.print("average = "); Serial.println(total/k); //more debugging
  return total / k;  // divide by number of samples

  integerBuffer = freeRam(); if (integerBuffer < freeMem) {
    freeMem = integerBuffer;
  }
} // teriminator for digitalSmooth


/* A Low pass filter works by blocking the higher frequency (in this case noise) and
   allowing the low frequency signal (in this case the underlying signal) to pass.
   This algorithm is a classified as a leaky integrator, where the latest reading have
   more weight than the older readings. The response of the filer can be adjusted by the SHIFT,
   to make it more responsive, then lower this value, to give better filtering and hence a
   slower response then increase this value. ONLY WORKS with positive INTs! As the bit shifting
   messes with the sign bits (see sign extension). 
*/
uint32_t lowPassFilter(uint32_t previousFilteredValue, uint32_t newValue, uint8_t shift) {
  previousFilteredValue += (newValue - (previousFilteredValue >> shift));
  return (previousFilteredValue >> shift); //shift1= new val/2, shift2= /4, shift3 =/16 I rarely go beyond shift4
} // teriminator for leaky integrator

/*
The median filter does better at getting rid of SINGLE SAMPLE NOISE SPIKES 
than any linear filter. (It is better than any low pass filter, moving average, 
weighted moving average, etc. IN TERMS OF ITS RESPONSE TIME and its ability 
to ignore such single-sample noise spike outliers. The median-of-3 requires very 
little CPU power, and is fast.  2014-03-25 by David Cary

// for continuous readings, drop oldest int value and shift in latest reading before calling this function:
oldest = recent;
recent = newest;
newest = analogRead(A0);
*/

int median_of_3( int a, int b, int c ){
    int the_max = max( max( a, b ), c );
    int the_min = min( min( a, b ), c );
    int the_median = the_max ^ the_min ^ a ^ b ^ c;
    //bitwise xor operator used here https://www.arduino.cc/reference/en/language/structure/bitwise-operators/bitwisexor/  
    return( the_median );
} // teriminator for median_of_3


// ===========================================================================================================
// ============================================================================================================
//   *  *   *  *  *  *  *  *  *  *  SENSOR SPECIFIC FUNCTIONS  *  *  *  *  *  *  *  *  *  *  *  *  *
// ============================================================================================================
// ===========================================================================================================
// ==================================================
// DS18B20  ONE WIRE TEMPERATURE
// ==================================================
// this returns the temperature from one DS18S20 using 12 bit conversion
// also see Dallas Temperature Control library by Miles Burton: http://milesburton.com/Dallas_Temperature_Control_Library

#if defined(TS_DS18B20)
int readDS18B20Temp()
{
  byte data[2]; //byte data[12]; there are more bytes of data to be read...
  ds.reset();
  ds.select(addr);
  ds.write(0x44); // start conversion, read temperature and store it in the scratchpad
  //The time needed between the CONVERT_T command and the READ_SCRATCHPAD command has to be at least
  //750 millisecs (can be shorter if using a D18B20 type with resolutions < 12 bits)
  //if you start getting "85" all the time you did not wait long enough
  // power saving during sleep from http://www.gammon.com.au/forum/?id=11497
  // no need to keep processor awake for that time:
  LowPower.powerDown(SLEEP_500MS, ADC_OFF, BOD_ON);
  LowPower.powerDown(SLEEP_250MS, ADC_OFF, BOD_ON);
  delay(3); //regulator stabilization after uC startup
  byte present = ds.reset();
  ds.select(addr);
  ds.write(0xBE); // Read Scratchpad
  for (int i = 0; i < 2; i++)
  {
    data[i] = ds.read();
  }
  byte MSB = data[1];
  byte LSB = data[0];
  int tempRaw = ((MSB << 8) | LSB); //using two's compliment //TEMP_degC = tempRaw / 16;
  return tempRaw;
}
#endif


// ========================================================================================
// I2C Accelerometer ADXL345    10 bit (+- 1023)
// ========================================================================================
// datasheet: http://www.analog.com/static/imported-files/data_sheets/ADXL345.pdf

#ifdef AdxlDripSensor   // if AdxlDripSensor is not defined at the beginning of the program, none of this gets compiled

/* ------- Register names ------- */
//#define ADXL345_DEVID 0x00
//#define ADXL345_RESERVED1 0x01
#define ADXL345_THRESH_TAP 0x1d
//#define ADXL345_OFSX 0x1e
//#define ADXL345_OFSY 0x1f
//#define ADXL345_OFSZ 0x20
#define ADXL345_DUR 0x21
#define ADXL345_LATENT 0x22
//#define ADXL345_WINDOW 0x23
#define ADXL345_THRESH_ACT 0x24
//#define ADXL345_THRESH_INAC 0x25
#define ADXL345_THRESH_INAC 0x25
#define ADXL345_TIME_INACT 0x26
#define ADXL345_ACT_INAC_CTL 0x27
//#define ADXL345_THRESH_FF 0x28
//#define ADXL345_TIME_FF 0x29
#define ADXL345_TAP_AXES 0x2a
#define ADXL345_ACT_TAP_STAT 0x2b
#define ADXL345_BW_RATE 0x2c  //d0-d3= output rate, d4=low power, The default value is 0x0A = 100 Hz output data rate p25
#define ADXL345_POWER_CTL 0x2d
#define ADXL345_INT_ENABLE 0x2e
#define ADXL345_INT_MAP 0x2f
#define ADXL345_DATA_FORMAT 0x31 //The default 4 interrupt pins is active high -changed to active low by setting the INT_INVERT bit in the  DATA_FORMAT (Address 0x31) register
#define ADXL345_DATAX0 0x32
//#define ADXL345_DATAX1 0x33
//#define ADXL345_DATAY0 0x34
//#define ADXL345_DATAY1 0x35
//#define ADXL345_DATAZ0 0x36
//#define ADXL345_DATAZ1 0x37
//#define ADXL345_FIFO_CTL 0x38
//#define ADXL345_FIFO_STATUS 0x39

#define ADXL345_BW_1600 0xF // 1111
#define ADXL345_BW_800  0xE // 1110
#define ADXL345_BW_400  0xD // 1101  
#define ADXL345_BW_200  0xC // 1100
#define ADXL345_BW_100  0xB // 1011  
#define ADXL345_BW_50   0xA // 1010 
#define ADXL345_BW_25   0x9 // 1001 
#define ADXL345_BW_12   0x8 // 1000 
#define ADXL345_BW_6    0x7 // 0111
#define ADXL345_BW_3    0x6 // 0110

/* Interrupt PINs */
#define ADXL345_INT1_PIN 0x00 //INT1: 0
#define ADXL345_INT2_PIN 0x01 // INT2: 1

/* Interrupt bit position for ADXL345_INT_ENABLE 0x2e*/
#define ADXL345_INT_DATA_READY_BIT 0x07
#define ADXL345_INT_SING_TAP_BIT 0x06
#define ADXL345_INT_DOUBLE_TAP_BIT 0x05
#define ADXL345_INT_ACTIVIT_BIT   0x04
#define ADXL345_INT_INACTIVITY_BIT 0x03
//#define ADXL345_INT_FREE_FALL_BIT  0x02
//#define ADXL345_INT_WATERMARK_BIT  0x01
//#define ADXL345_INT_OVERRUNY_BIT   0x00

#define bytes_to_read 6      // num of bytes we are going to read each time (two bytes for each axis)

//===========================================================
void initADXL345() {
//===========================================================
// 50HZ &powersave is optimal setting: saves about 0.017mA over always on @ 12hz and doesn’t miss drips

#ifdef ECHO_TO_SERIAL
  Serial.println(F("Init ADXL345..."));
#endif

  byte adxlRange;
  bool bitBuffer;
  //byte bytebuffer1;

#ifdef ECHO_TO_SERIAL
  Serial.print(F("Set Range to 2g..."));
  Serial.flush();
#endif

  //setRangeSetting(1); // Sets the range setting, possible values are: 2, 4, 8, 16
  adxlRange = 0b00000000; //2g = 0b00000000; 4g=0b00000001, 8g=0b00000010;16g=0b00000011; default=0
  bytebuffer1 = i2c_readRegisterByte(ADXL345_ADDRESS, ADXL345_DATA_FORMAT);
  adxlRange |= (bytebuffer1 & 0b11101100); // note: D4 is zero in datasheet
  i2c_writeRegisterByte(ADXL345_ADDRESS, ADXL345_DATA_FORMAT, adxlRange);
  // setFullResBit? //0=10bit mode, 1= output resolution increases with g range, gain is fixed to maintain a 4 mg/LSB scale factor
  // set the gain for each axis in Gs / count? since we did not set the full rez bit?
  // set the OFSX, OFSY and OFSZ bytes? // ignoring justify bit

  //set Interrupt Level Bit 0=active high (default)   //The default configuration of the interrupt pins is active high. This can be changed to active low by setting the INT_INVERT bit in the DATA_FORMAT (Address 0x31) register.
  i2c_setRegisterBit(ADXL345_ADDRESS, ADXL345_DATA_FORMAT, 5, 1); //1=interrupts are now active LOW
#ifdef ECHO_TO_SERIAL
  bitBuffer = i2c_getRegisterBit(ADXL345_ADDRESS, ADXL345_DATA_FORMAT, 5); // Gets the state of the INT_INVERT bit
  Serial.print(F("Intrupt Bit (1=low) set to:"));
  Serial.println(bitBuffer);
  Serial.flush();
#endif

  i2c_writeRegisterByte(ADXL345_ADDRESS, ADXL345_INT_ENABLE, 0b00000000); // this disables all interupts
  // When initially configuring the interrupt pins, it is recommended that the functions and interrupt mapping be done before enabling the interrupts.DDRESS, ADXL345_INT_ENABLE, 0b00000000); //
  // I am ignoring freefall setup here

  i2c_writeRegisterByte(ADXL345_ADDRESS, ADXL345_DUR, 60);
  // The DUR byte contains an unsigned time value representing the maximum time
  // that an event must be above THRESH_TAP threshold to qualify as a tap event
  // The scale factor is 625µs/LSB / A value of 0 disables the tap/double tap funcitons. Max value is 255.
  // set Tap Duration(60) to (15) works / 625μs per increment  (was 0x1F = 31) and that worked ok

  //set Tap Threshold betw 1 and 255 / NEVER SET to 0!
  i2c_writeRegisterByte(ADXL345_ADDRESS, ADXL345_THRESH_TAP , adxlTapSensitivity);
  // Sets the THRESH_TAP byte value /the scale factor is 62.5 mg/LSB
  // 62.5mg per increment / was able to set down to 60 dur &  2 sensitivity hard mounted inside the 4" caps

  //writeTo(ADXL345_TAP_AXES, 0b00000111);
  i2c_writeRegisterByte(ADXL345_ADDRESS, ADXL345_TAP_AXES, 0b00000111); //all 3 axis set to detect taps
  // or could do them one at a time
  //setRegisterBit(ADXL345_ADDRESS, ADXL345_TAP_AXES, 2, 1); // Tap Detection On X,    1,1=Y   0,1=Z

  // set Double Tap Latency
  i2c_writeRegisterByte(ADXL345_ADDRESS, ADXL345_LATENT, 0); // 0-255 max / scale factor is 1.25ms/LSB.
  // writing zero here disables the double tap & puts in single tap mode , typical value 200 for double taps /  1.25ms per increment
  // representing the wait time from the detection of a tap event to the start of the time window, during which a possible second tap can be detected.

  //inactivity config
  //set Inactivity Threshold // 62.5mg per LSB  // was 8?
  //i2c_writeRegisterByte(ADXL345_ADDRESS, ADXL345_THRESH_INAC, 8); // 0-255 max

  // Set the TIME_INACT register, amount of time that acceleration must be less thant the value in the THRESH_INACT register for inactivity to be declared.
  //i2c_writeRegisterByte(ADXL345_ADDRESS, ADXL345_TIME_INACT, 10); // 0-255 max 1 second increments

  // activity config
  // set Activity Threshold //max of 255  62.5mg/ LSB  setting this too sensitive generates bounce hits!
  i2c_writeRegisterByte(ADXL345_ADDRESS, ADXL345_THRESH_ACT, adxlTapSensitivity); // 0-255 max // factor is 62.5mg/LSB. Never set to 0!// threshold value for detecting activity.
  // The data format is unsigned, so the magnitude of the activity event is compared with the value is compared with the value in the THRESH_ACT register. The scale

  //the processor must respond to the activity and inactivity interrupts by reading the INT_SOURCE register (Address 0x30) and, therefore, clearing the interrupts.

  // set Activity Ac //turn on ac coupled mode - needed to prevent self triggering loop at high sensitivities.
  i2c_setRegisterBit(ADXL345_ADDRESS, ADXL345_ACT_INAC_CTL, 7, 1); // set which axes can trigger activity //using ac-coupled operation
  i2c_setRegisterBit(ADXL345_ADDRESS, ADXL345_ACT_INAC_CTL, 6, 1); // setActivityX
  i2c_setRegisterBit(ADXL345_ADDRESS, ADXL345_ACT_INAC_CTL, 5, 1); // setActivityY
  i2c_setRegisterBit(ADXL345_ADDRESS, ADXL345_ACT_INAC_CTL, 4, 1); // setActivityZ

  //inactivity config
  //i2c_setRegisterBit(ADXL345_ADDRESS,ADXL345_ACT_INAC_CTL,3, 1); //set Inactivity Ac//using ac-coupled operation
  //i2c_setRegisterBit(ADXL345_ADDRESS,ADXL345_ACT_INAC_CTL,2, 1);// setInActivityX
  //i2c_setRegisterBit(ADXL345_ADDRESS,ADXL345_ACT_INAC_CTL,1, 1);// setInActivityY
  //i2c_setRegisterBit(ADXL345_ADDRESS,ADXL345_ACT_INAC_CTL,0, 1);// setInActivityZ //could only use the z axis to set inactivity state?

  //could set the whole lot with:
  //i2c_writeRegisterByte(ADXL345_ADDRESS, ADXL345_ACT_INAC_CTL, 0b11111111); // enables inactivity and activity on x,z,y using ac-coupled operation
  //The interrupt functions are latched and cleared by either reading the data registers (Address 0x32 to Address 0x37) until the interrupt
  //condition is no longer valid for the data-related interrupts or by reading the INT_SOURCE register (Address 0x30) for the remaining interrupts

  // Now Map each interrupt to the right outgoing pin:
  i2c_setRegisterBit(ADXL345_ADDRESS, ADXL345_INT_MAP, ADXL345_INT_SING_TAP_BIT, ADXL345_INT1_PIN); // 6
  i2c_setRegisterBit(ADXL345_ADDRESS, ADXL345_INT_MAP, ADXL345_INT_ACTIVIT_BIT, ADXL345_INT1_PIN); // 4
  //i2c_setRegisterBit(ADXL345_ADDRESS, ADXL345_INT_MAP,ADXL345_INT_INACTIVITY_BIT,ADXL345_INT2_PIN); //not using inactivity yet
  //OR i2c_writeRegisterByte(ADXL345_ADDRESS, ADXL345_INT_MAP, 0b00000000); // which sends all interrupts to INT1

  // Now enable the interrupts  //i2c_setRegisterBit(ADXL345_ADDRESS,ADXL345_INT_ENABLE, interruptBit, state);
  i2c_setRegisterBit(ADXL345_ADDRESS, ADXL345_INT_ENABLE, ADXL345_INT_SING_TAP_BIT, 1);//enable single tap interrupts // 6
  i2c_setRegisterBit(ADXL345_ADDRESS, ADXL345_INT_ENABLE, ADXL345_INT_ACTIVIT_BIT, 1);//enable activity interrupts  // 4
  //i2c_setRegisterBit(ADXL345_ADDRESS, ADXL345_INT_ENABLE, ADXL345_INT_INACTIVITY_BIT, 1); //enable inactivity interrupts

  // then the power control bits
  // set the ADXL345 in measurment and sleep mode-save power but still detect activity
  // Link bit is set to 1 so inactivity and activity aren't concurrent
  // sets auto_sleep bit to 1 so it goes to sleep when inactivity is detected
  //i2c_writeRegisterByte(ADXL345_ADDRESS,ADXL345_POWER_CTL, 0b00111100);  // this sets Link, Auto_sleep, Measure,typically 23 µA so only 10uA better than power save
  i2c_writeRegisterByte(ADXL345_ADDRESS, ADXL345_POWER_CTL, 0b00001011); //my tests used full on measure mode no sleep

  // bandwidth
  i2c_writeRegisterByte(ADXL345_ADDRESS, ADXL345_BW_RATE, ADXL345_BW_50);  // ADXL345_BW_50 or ADXL345_BW_25 or ADXL345_BW_100
  //this is pretty low, might have to raise this later (50 seems to be about the best setting!)
  i2c_setRegisterBit(ADXL345_ADDRESS, ADXL345_BW_RATE, 4, 1); // set low power mode (draws 50 uA at 50HZ BW) -this is a high power draw test!
  // setting low power seems to make the logger miss a few drip registers at random! raise the BW to 50hz fixes this
  // power save only works up to BW settings of 12.5(34uA),25(40uA),50(45uA),10050uA),200(60uA) hz  don't use above that!
  // To enter low power mode, set the LOW_POWER bit (Bitposition 4) in the BW_RATE register (Address 0x2C)

#ifdef ECHO_TO_SERIAL
  Serial.print("Output(= 2xBW) set to: ");
  Serial.println(getAdxl_rate() / 2);
  Serial.print(F("ADXL TapSensing @: ")); Serial.println(adxlTapSensitivity);
  Serial.flush();
#endif

} //end of initADXL345 function
//===========================================================

//===========================================================
void readADXL345() {
//===========================================================
  for (int thisReading = 0; thisReading < filterSamples; thisReading++) { //fill the smoothing arrays
    i2c_readToByteArray(ADXL345_ADDRESS, ADXL345_DATAX0, bytes_to_read, sixByteArray); //read the acceleration data from the ADXL345
    // each axis reading comes in 10 bit resolution, ie 2 bytes.  Least Significat Byte first!!
    // thus we are converting both bytes in to one int
    rawACCx[thisReading] = (((int)sixByteArray[1]) << 8) | sixByteArray[0];
    rawACCy[thisReading] = (((int)sixByteArray[3]) << 8) | sixByteArray[2];
    rawACCz[thisReading] = (((int)sixByteArray[5]) << 8) | sixByteArray[4];
    LowPower.powerDown(SLEEP_120MS, ADC_OFF, BOD_OFF); //a VERY long delay between readings to deal with vortex shedding
  }
  // Now send those readings out to the digital smoothing function
  smoothACCx = digitalSmooth(rawACCx);
  smoothACCy = digitalSmooth(rawACCy);
  smoothACCz = digitalSmooth(rawACCz);
}  // end of void readADXL345()
//===========================================================

//double getAdxl_rate(){
int getAdxl_rate() {
  //i2c_readToByteArray(ADXL345_ADDRESS, ADXL345_BW_RATE, 1, &bytebuffer1);  // REPLACE THIS WITH I2C READREGBYTE
  bytebuffer1 = i2c_readRegisterByte(ADXL345_ADDRESS, ADXL345_BW_RATE);
  bytebuffer1 &= 0b00001111;
  //return (pow(2,((int) bytebuffer1)-6)) * 6.25);
  return int((pow(2, ((int) bytebuffer1) - 6)) * 6.25);
}//======================

//======================Other ADXL functions=============================

//adxl345lnterruptSource = getInterruptSource();
//byte _b;i2c_readToByteArray(ADXL345_ACT_TAP_STAT, 1, &_b); //read the ACT_TAP_STATUS register to clear it
//the processor must respond to the activity and inactivity interrupts by reading the INT_SOURCE register (Address 0x30) and, therefore, clearing the interrupts.
/*
  byte getInterruptSource() {
  byte bytebuffer1;
  i2c_readToByteArray(ADXL345_ADDRESS, ADXL345_INT_SOURCE, 1, &bytebuffer1);
  return bytebuffer1;
  }
*/
// Sets the TIME_INACT register, which contains an unsigned time value representing the
// amount of time that acceleration must be less thant the value in the THRESH_INACT
// register for inactivity to be declared. The scale factor is 1sec/LSB. The value must
// be between 0 and 255.
/*
  void setTimeInactivity(int timeInactivity) {
  timeInactivity = min(max(timeInactivity,0),255);
  byte bytebuffer1 = byte (timeInactivity);
  writeTo(ADXL345_ADDRESS,ADXL345_TIME_INACT, bytebuffer1);
  }

  //Gets the TIME_INACT register
  int getTimeInactivity() {
  byte bytebuffer1;
  i2c_readToByteArray(ADXL345_ADDRESS,ADXL345_TIME_INACT, 1, &bytebuffer1);
  return int (bytebuffer1);
  } */

// Sets the THRESH_INACT byte which holds the threshold value for detecting inactivity.
// The data format is unsigned, so the magnitude of the inactivity event is compared
// with the value is compared with the value in the THRESH_INACT register. The scale
// factor is 62.5mg/LSB. A value of 0 may result in undesirable behavior if the
// inactivity interrupt is enabled. The maximum value is 255.
/*
  void setInactivityThreshold(int inactivityThreshold) {
  inactivityThreshold = min(max(inactivityThreshold,0),255);
  byte bytebuffer1 = byte (inactivityThreshold);
  writeTo(ADXL345_ADDRESS,ADXL345_THRESH_INAC, bytebuffer1);
  }

  //Gets the THRESH_INACT byte
  int getInactivityThreshold() {
  byte bytebuffer1;
  i2c_readToByteArray(ADXL345_ADDRESS,ADXL345_THRESH_INAC, 1, &bytebuffer1);
  return int (bytebuffer1);
  } */

/* Gets the Latent value
  int getDoubleTapLatency() {
  byte bytebuffer1;
  i2c_readToByteArray(ADXL345_ADDRESS,ADXL345_LATENT, 1, &bytebuffer1);
  return int (bytebuffer1);
  } */

/*
  // Gets the DUR byte
  int getTapDuration() {
  byte bytebuffer1;
  i2c_readToByteArray(ADXL345_ADDRESS, ADXL345_DUR, 1, &bytebuffer1);
  return int (bytebuffer1);
  } */
// Sets the THRESH_TAP byte value
// it should be between 0 and 255
// the scale factor is 62.5 mg/LSB
// A value of 0 may result in undesirable behavior

// =========================================================================================
#endif   // terminator for #ifdef AdxlDripSensor that enables ADXL functions to be compiled
// =========================================================================================

// ===============================================================================
// MCP9808  Datasheet: http://ww1.microchip.com/downloads/en/DeviceDoc/25095A.pdf
// ===============================================================================
// for more info see https://github.com/adafruit/Adafruit_MCP9808_Library/blob/master/Adafruit_MCP9808.cpp

#ifdef MCP9808_I2CADDR

#define MCP9808_REG_CONFIG             0x01 //0b1
#define MCP9808_REG_MANUF_ID           0x06 //0b110
#define MCP9808_REG_DEVICE_ID          0x07 //0b111
#define MCP9808_REG_AMBIENT_TEMP       0x05 //0b101  The Temp register is read-only

void initMCP9808() {           //this is a bare minimum initialization!
// =================
  integerBuffer = i2c_read16bitRegister(MCP9808_I2CADDR, MCP9808_REG_MANUF_ID);
  if (integerBuffer != 0x0054) {
#ifdef ECHO_TO_SERIAL
    Serial.println(F("Couldn't find MCP9808!")); Serial.flush();
    error();
#endif
  }
  integerBuffer = i2c_read16bitRegister(MCP9808_I2CADDR, MCP9808_REG_DEVICE_ID);
  if (integerBuffer != 0x0400) {
#ifdef ECHO_TO_SERIAL
    Serial.println(F("Couldn't find MCP9808!")); Serial.flush();
    error();
#endif
  }
#ifdef ECHO_TO_SERIAL
  Serial.println(F("MCP9808 responding"));
  Serial.flush();
#endif

  //all defauts should be zero to launch continuous mode on this sensor
  // bits 11 = +0.0625°C (power-up default, tCONV = 250 ms typical)
  Wire.beginTransmission(MCP9808_I2CADDR);
  Wire.write(MCP9808_REG_CONFIG);
  Wire.write(0b00000000);
  Wire.write(0b00000000);
  Wire.endTransmission();

  return;
}

int readMCP9808Temp( void ) {  //Reads the 16-bit temp register
// ========================
  mcp9808wakeUp(); //tCONV = 250 ms typical so we had better give the sensor some time to wake up
  int rawMCP9808temp = 0;
  LowPower.powerDown(SLEEP_250MS, ADC_OFF, BOD_OFF);
  LowPower.powerDown(SLEEP_15MS, ADC_OFF, BOD_OFF); //a little extra time, to make sure we get new data
  Wire.beginTransmission(MCP9808_I2CADDR);// with mcp9808 bus address written in hex
  Wire.write(MCP9808_REG_AMBIENT_TEMP);// and the temperature output register
  Wire.endTransmission();
  Wire.requestFrom(MCP9808_I2CADDR, 2);
  bytebuffer1 = Wire.read(); // the MSB
  bytebuffer2 = Wire.read(); // the LSB 
  bytebuffer1 = bytebuffer1 & 0b00011111; //Mask away the three flag bits
  //easier to read when the mask is written in binary instead of hex
  // spacer comment for blog layout
  //now we use a mask in a slightly different way to check the value of the sign bit:
  if ((bytebuffer1 & 0b00010000) == 0b00010000) { //if signbit=1 then temp<0°C
    bytebuffer1 = bytebuffer1 & 0b00001111; //mask away the SIGN bit
    rawMCP9808temp = (((int)bytebuffer1) << 8) | bytebuffer2; // combine the MSB & LSB
    rawMCP9808temp -= 256; //convert to negative value: note suggested datasheet calculation has an error!
  }
  else// temp > 0°C  then the sign bit = 0  - so no need to mask it away
  {
    rawMCP9808temp = (((int)bytebuffer1) << 8) | bytebuffer2;
  }
  mcp9808shutdown();
  return rawMCP9808temp;  // temp is a 12-bit digital value +-4096 //TEMP_degC =TEMP_Raw*0.0625;
}

//bit 8 SHDN: Shutdown Mode bit, 0 = Continuous conversion (power-up default) 200 µA, 1 = Shutdown (Low-Power mode) 1 µA
void mcp9808shutdown() {  //this does not use the write 16 //read 16
// ========================
  Wire.beginTransmission(MCP9808_I2CADDR);
  Wire.write(MCP9808_REG_CONFIG);
  Wire.endTransmission();
  Wire.requestFrom(MCP9808_I2CADDR, 2);
  bytebuffer1 = Wire.read();  //upper 8 bits described in the data sheet as 15-8
  bytebuffer2 = Wire.read();  //lower 8 bits described as 7-0
  // in this case 'bit 8', is the zero position of the UPPER 8 bits of the register
  bytebuffer1 |= (1 << 0);  //x |= (1 << n); forces nth bit of x to be 1

  Wire.beginTransmission(MCP9808_I2CADDR);
  Wire.write(MCP9808_REG_CONFIG);
  Wire.write(bytebuffer1);   //altered
  Wire.write(bytebuffer2);   //unchanged
  Wire.endTransmission();
}

void mcp9808wakeUp() {   //set bit 8 to 0 = Continuous conversion (power-up default)
// ===================
  // power on defaults for all of config register are "0"
  // so you could just us the 4 lines in the init...
  Wire.beginTransmission(MCP9808_I2CADDR);
  Wire.write(MCP9808_REG_CONFIG);
  Wire.endTransmission();
  Wire.requestFrom(MCP9808_I2CADDR, 2);
  bytebuffer1 = Wire.read();  //upper MSB bits 16-8
  bytebuffer2 = Wire.read();  //then LSB bits 7-0

  // x &= ~(1 << n);      // forces nth bit of x to be 0.  all other bits left alone.
  bytebuffer1 &= ~(1 << 0); // bit position 8 = position zero of the upper byte

  Wire.beginTransmission(MCP9808_I2CADDR);
  Wire.write(MCP9808_REG_CONFIG);  //register pointer
  Wire.write(bytebuffer1);  //altered
  Wire.write(bytebuffer2);  //unchanged
  Wire.endTransmission();
}
#endif    // end of sensor specific functions for mcp9808
// ======================================================

// ============================================================================================================
// RTC functions
// ============================================================================================================
// some of these functions are from https://github.com/MrAlvin/RTClib 
// which is a fork of JeeLab's fantastic real time clock library for Arduino
// released to the public domain at:  https://github.com/jcw/rtclib 
//
// The only reason I extracted these functions to stand-alone versions was for the extra memory 
// It is quite common for multi-sensor logger configurations of the cave pearl logger to exceed the 30,720 bytes 
// availiable in program storage space on a 328P
// ============================================================================================================
// also see https://github.com/sleemanj/DS3231_Simple/blob/master/DS3231_Simple.cpp for an alternate library
// also see https://github.com/mizraith/RTClib
// or https://github.com/akafugu/ds_rtc_lib for more DS3231 specific libs
// http://www.rinkydinkelectronics.com/library.php
// https://github.com/MajicDesigns/MD_DS3231/blob/master/src/MD_DS3231.cpp
// https://github.com/JChristensen/DS3232RTC wrapper for the time.h lib
// alternate unixtime calculation: https://github.com/rodan/ds3231
// see http://tronixstuff.com/2014/12/01/tutorial-using-ds1307-and-ds3231-real-time-clock-modules-with-arduino/

// RTC_DS3231_turnOffAlarms    // from http://forum.arduino.cc/index.php?topic=109062.0
void RTC_DS3231_turnOffAlarms() {
  i2c_setRegisterBit(DS3231_ADDRESS, DS3231_STATUS_REG, 0, 0); //bitPosition 0 = Alarm 1 Flag (A1F)
  i2c_setRegisterBit(DS3231_ADDRESS, DS3231_STATUS_REG, 1, 0); //bitPosition 1 = Alarm 2 Flag (A2F) on DS3231 - pg17 of datasheet
  rtc_INT0_Flag = false; //clear the flag we use to indicate the RTC alarm occurred
}

float RTC_DS3231_getTemp()  // from http://forum.arduino.cc/index.php?topic=22301.0
{
  byte tMSB, tLSB;
  float temp3231;
  Wire.beginTransmission(DS3231_ADDRESS);
  Wire.write(DS3231_TMP_UP_REG); //temp registers (upper byte 0x11 & lower 0x12) get updated automatically every 64s
  Wire.endTransmission();
  Wire.requestFrom(DS3231_ADDRESS, 2);
  if (Wire.available()) {
    tMSB = Wire.read(); //2's complement int portion - If tMSB bit7 is a 1 then the temperature is negative
    tLSB = Wire.read(); //fraction portion
    temp3231 = ((((short)tMSB << 8) | (short)tLSB) >> 6) / 4.0; // Allows for readings below freezing - Thanks to Coding Badly
    //temp3231 = (temp3231 * 1.8) + 32.0; // to Convert Celcius to Fahrenheit
  }
  else {
    temp3231 = 0.0; //got no data from RTC
  }
  return temp3231;
}


void RTC_DS3231_getTime()
{
  Wire.beginTransmission(DS3231_ADDRESS);
  Wire.write(0);
  Wire.endTransmission();
  Wire.requestFrom(DS3231_ADDRESS, 7);
  t_second = bcd2bin(Wire.read() & 0x7F);
  t_minute = bcd2bin(Wire.read());
  t_hour = bcd2bin(Wire.read());
  Wire.read();
  t_day = bcd2bin(Wire.read());
  t_month = bcd2bin(Wire.read());
  t_year = bcd2bin(Wire.read()) + 2000;
  return;
}

void RTC_DS3231_setAlarm1Simple(byte hour, byte minute) {
  RTC_DS3231_setA1Time(0, hour, minute, 00, 0b00001000, false, false, false);
}

void RTC_DS3231_setA1Time(byte A1Day, byte A1Hour, byte A1Minute, byte A1Second, byte AlarmBits, bool A1Dy, bool A1h12, bool A1PM) {
  //  Sets the alarm-1 date and time on the DS3231, using A1* information
  byte temp_buffer;
  Wire.beginTransmission(DS3231_ADDRESS);
  Wire.write(0x07);    // A1 starts at 07h
  // Send A1 second and A1M1
  Wire.write(bin2bcd(A1Second) | ((AlarmBits & 0b00000001) << 7));
  // Send A1 Minute and A1M2
  Wire.write(bin2bcd(A1Minute) | ((AlarmBits & 0b00000010) << 6));
  // Figure out A1 hour
  if (A1h12) {
    // Start by converting existing time to h12 if it was given in 24h.
    if (A1Hour > 12) {
      // well, then, this obviously isn't a h12 time, is it?
      A1Hour = A1Hour - 12;
      A1PM = true;
    }
    if (A1PM) {
      // Afternoon
      // Convert the hour to BCD and add appropriate flags.
      temp_buffer = bin2bcd(A1Hour) | 0b01100000;
    } else {
      // Morning
      // Convert the hour to BCD and add appropriate flags.
      temp_buffer = bin2bcd(A1Hour) | 0b01000000;
    }
  } else {
    // Now for 24h
    temp_buffer = bin2bcd(A1Hour);
  }
  temp_buffer = temp_buffer | ((AlarmBits & 0b00000100) << 5);
  // A1 hour is figured out, send it
  Wire.write(temp_buffer);
  // Figure out A1 day/date and A1M4
  temp_buffer = ((AlarmBits & 0b00001000) << 4) | bin2bcd(A1Day);
  if (A1Dy) {
    // Set A1 Day/Date flag (Otherwise it's zero)
    temp_buffer = temp_buffer | 0b01000000;
  }
  Wire.write(temp_buffer);
  Wire.endTransmission();
}

void RTC_DS3231_getA1Time(byte& A1Day, byte& A1Hour, byte& A1Minute, byte& A1Second, byte& AlarmBits, bool& A1Dy, bool& A1h12, bool& A1PM) {
  byte temp_buffer;
  Wire.beginTransmission(DS3231_ADDRESS);
  Wire.write(0x07);
  Wire.endTransmission();
  Wire.requestFrom(DS3231_ADDRESS, 4);

  temp_buffer = Wire.read();   // Get A1M1 and A1 Seconds
  A1Second    = bcd2bin(temp_buffer & 0b01111111);
  // put A1M1 bit in position 0 of DS3231_AlarmBits.
  AlarmBits   = AlarmBits | (temp_buffer & 0b10000000) >> 7;

  temp_buffer     = Wire.read();   // Get A1M2 and A1 minutes
  A1Minute    = bcd2bin(temp_buffer & 0b01111111);
  // put A1M2 bit in position 1 of DS3231_AlarmBits.
  AlarmBits   = AlarmBits | (temp_buffer & 0b10000000) >> 6;

  temp_buffer = Wire.read();   // Get A1M3 and A1 Hour
  // put A1M3 bit in position 2 of DS3231_AlarmBits.
  AlarmBits   = AlarmBits | (temp_buffer & 0b10000000) >> 5;
  // determine A1 12/24 mode
  A1h12       = temp_buffer & 0b01000000;
  if (A1h12) {
    A1PM    = temp_buffer & 0b00100000;         // determine am/pm
    A1Hour  = bcd2bin(temp_buffer & 0b00011111);   // 12-hour
  } else {
    A1Hour  = bcd2bin(temp_buffer & 0b00111111);   // 24-hour
  }

  temp_buffer = Wire.read();   // Get A1M4 and A1 Day/Date
  // put A1M3 bit in position 3 of DS3231_AlarmBits.
  AlarmBits   = AlarmBits | (temp_buffer & 0b10000000) >> 4;
  // determine A1 day or date flag
  A1Dy        = (temp_buffer & 0b01000000) >> 6;
  if (A1Dy) {
    // alarm is by day of week, not date.
    A1Day   = bcd2bin(temp_buffer & 0b00001111);
  } else {
    // alarm is by date, not day of week.
    A1Day   = bcd2bin(temp_buffer & 0b00111111);
  }
}

void RTC_DS3231_turnOnAlarm(byte Alarm) {
  // turns on alarm number "Alarm". Defaults to 2 if Alarm is not 1.
  byte temp_buffer = RTC_DS3231_readControlByte(0);
  // modify control byte
  if (Alarm == 1) {
    temp_buffer = temp_buffer | 0b00000101;  //bitwise OR //either or both
  } else {
    temp_buffer = temp_buffer | 0b00000110;
  }
  RTC_DS3231_writeControlByte(temp_buffer, 0);
}
bool RTC_DS3231_checkAlarmEnabled(byte Alarm) {
  // Checks whether the given alarm is enabled.
  byte result = 0x0;
  byte temp_buffer = RTC_DS3231_readControlByte(0);
  if (Alarm == 1) {
    result = temp_buffer & 0b00000001;
  } else {
    result = temp_buffer & 0b00000010;
  }
  return result;
}
void RTC_DS3231_turnOffAlarm(byte Alarm) {
  // turns off alarm number "Alarm". Defaults to 2 if Alarm is not 1.
  // Leaves interrupt pin alone.
  byte temp_buffer = RTC_DS3231_readControlByte(0);
  // modify control byte
  if (Alarm == 1) {
    temp_buffer = temp_buffer & 0b11111110; //bitwise AND - zeros in mask become zeros in temp_buffer
  } else {
    temp_buffer = temp_buffer & 0b11111101;
  }
  RTC_DS3231_writeControlByte(temp_buffer, 0);
}

byte RTC_DS3231_readControlByte(bool which) {
  // Read selected control byte
  // first byte (0) is 0x0e, second (1) is 0x0f
  Wire.beginTransmission(DS3231_ADDRESS);
  if (which) {
    // second control byte
    Wire.write(0x0f);
  } else {
    // first control byte
    Wire.write(0x0e);
  }
  Wire.endTransmission();
  Wire.requestFrom(DS3231_ADDRESS, 1);
  return Wire.read();
}

void RTC_DS3231_writeControlByte(byte control, bool which) {
  // Write the selected control byte.
  // which=false -> 0x0e, true->0x0f.
  Wire.beginTransmission(DS3231_ADDRESS);
  if (which) {
    Wire.write(0x0f);
  } else {
    Wire.write(0x0e);
  }
  Wire.write(control);
  Wire.endTransmission();
}


//#ifdef CONFIG_UNIXTIME...
const uint8_t days_in_month [12] PROGMEM = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

//#define SECONDS_FROM_1970_TO_2000 946684800
uint32_t RTC_DS3231_unixtime() {
  uint32_t t;
  uint16_t days = date2days(t_year, t_month, t_day);
  t = time2long(days, t_hour, t_minute, t_second);
  t += 946684800;  // add # seconds from 1970 to 2000 which we took away with y -= 2000

  return t;
}

static long time2long(uint16_t days, uint8_t h, uint8_t m, uint8_t s) {
  return ((days * 24L + h) * 60 + m) * 60 + s;
}

// number of days since 2000/01/01, valid for 2001..2099
static uint16_t date2days(uint16_t y, uint8_t m, uint8_t d) {
  if (y >= 2000)
    y -= 2000;
  uint16_t days = d;
  for (uint8_t i = 1; i < m; ++i)
    days += pgm_read_byte(days_in_month + i - 1);
  if (m > 2 && (y % 4 == 0)) // modulo checks (if is LeapYear) add extra day
    ++days;
  return days + 365 * y + (y + 3) / 4 - 1;
}

// see http://tronixstuff.com/2014/12/01/tutorial-using-ds1307-and-ds3231-real-time-clock-modules-with-arduino/

// ------------------ other RTC functions ------------------------
// Other DS3231 Registers. Refer Sec 8.2 of datasheet
// https://github.com/bpg/DS3231/blob/master/DS3231.h
/*
  #define DS3231_SEC_REG        0x00
  #define DS3231_MIN_REG        0x01
  #define DS3231_HOUR_REG       0x02
  #define DS3231_WDAY_REG       0x03
  #define DS3231_MDAY_REG       0x04
  #define DS3231_MONTH_REG      0x05
  #define DS3231_YEAR_REG       0x06
  #define DS3231_AL1SEC_REG     0x07
  #define DS3231_AL1MIN_REG     0x08
  #define DS3231_AL1HOUR_REG    0x09
  #define DS3231_AL1WDAY_REG    0x0A
  #define DS3231_AL2MIN_REG     0x0B
  #define DS3231_AL2HOUR_REG    0x0C
  #define DS3231_AL2WDAY_REG    0x0D
  #define DS3231_AGING_OFFSET_REG     0x0F
  #define DS3231_TMP_LOW_REG          0x12
*/

// A constructor for using "the compiler's time":
// This version will save RAM by using PROGMEM to store it by using the F macro.
// call with RTC_DS3231_set2compilerTime (F(__DATE__), F(__TIME__));
/*
  void RTC_DS3231_set2compilerTime (const __FlashStringHelper* date, const __FlashStringHelper* time) {
    // sample input: date = "Dec 26 2009", time = "12:34:56"
    char buff[11];
    memcpy_P(buff, date, 11);
    t_year = conv2d(buff + 9);//yOff = conv2d(buff + 9);
    // Jan Feb Mar Apr May Jun Jul Aug Sep Oct Nov Dec
    switch (buff[0]) {
        case 'J': t_month = buff[1] == 'a' ? 1 : t_month = buff[2] == 'n' ? 6 : 7; break;
        case 'F': t_month = 2; break;
        case 'A': t_month = buff[2] == 'r' ? 4 : 8; break;
        case 'M': t_month = buff[2] == 'r' ? 3 : 5; break;
        case 'S': t_month = 9; break;
        case 'O': t_month = 10; break;
        case 'N': t_month = 11; break;
        case 'D': t_month = 12; break;
    }
   t_day = conv2d(buff + 4);
    memcpy_P(buff, time, 8);
   t_hour = conv2d(buff);
   t_minute = conv2d(buff + 3);
   t_second = conv2d(buff + 6);

   Wire.beginTransmission(DS3231_ADDRESS);
   Wire.write(0);
   Wire.write(bin2bcd(t_second));
   Wire.write(bin2bcd(t_minute));
   Wire.write(bin2bcd(t_hour));
   Wire.write(bin2bcd(0));
   Wire.write(bin2bcd(t_day));
   Wire.write(bin2bcd(t_month));
   Wire.write(bin2bcd(t_year - 2000));
   Wire.write(0);
   Wire.endTransmission();
  }
  static uint8_t conv2d(const char* p) {
    uint8_t v = 0;
    if ('0' <= *p && *p <= '9')
        v = *p - '0';
    return 10 * v + *++p - '0';
  }
*/
/*
  void RTC_DS3231_adjust(const DateTime& dt)
  {
   Wire.beginTransmission(DS3231_ADDRESS);
   Wire.write(0);
   Wire.write(bin2bcd(t_second));
   Wire.write(bin2bcd(t_minute));
   Wire.write(bin2bcd(t_hour));
   Wire.write(bin2bcd(0));
   Wire.write(bin2bcd(t_day));
   Wire.write(bin2bcd(t_month));
   Wire.write(bin2bcd(t_year - 2000));
   Wire.write(0);
    Wire.endTransmission();
  }
*/


//ISO 8601 Timestamps
//TIMESTAMP_TIME: sprintf(buffer, "%02d:%02d:%02d", hh, mm, ss);
//TIMESTAMP_DATE: sprintf(buffer, "%d-%02d-%02d", 2000+yOff, m, d)
//Full sprintf(buffer, "%d-%02d-%02dT%02d:%02d:%02d", 2000+yOff, m, d, hh, mm, ss);

//alarm 2 functions:
/*
  void RTC_DS3231_setAlarm2Simple(byte hour, byte minute) {
    RTC_DS3231_setA2Time(0, hour, minute, 0b00001000, false, false, false);
  }

  void RTC_DS3231_setA2Time(byte A2Day, byte A2Hour, byte A2Minute, byte AlarmBits, bool A2Dy, bool A2h12, bool A2PM) {
    //  Sets the alarm-2 date and time on the DS3231, using A2* information
    byte temp_buffer;
    Wire.beginTransmission(DS3231_ADDRESS);
   Wire._I2C_WRITE(0x0b);    // A1 starts at 0bh
    // Send A2 Minute and A2M2
   Wire._I2C_WRITE(bin2bcd(A2Minute) | ((AlarmBits & 0b00010000) << 3));
    // Figure out A2 hour
    if (A2h12) {
        // Start by converting existing time to h12 if it was given in 24h.
        if (A2Hour > 12) {
            // well, then, this obviously isn't a h12 time, is it?
            A2Hour = A2Hour - 12;
            A2PM = true;
        }
        if (A2PM) {
            // Afternoon
            // Convert the hour to BCD and add appropriate flags.
            temp_buffer = bin2bcd(A2Hour) | 0b01100000;
        } else {
            // Morning
            // Convert the hour to BCD and add appropriate flags.
            temp_buffer = bin2bcd(A2Hour) | 0b01000000;
        }
    } else {
        // Now for 24h
        temp_buffer = bin2bcd(A2Hour);
    }
    // add in A2M3 bit
    temp_buffer = temp_buffer | ((AlarmBits & 0b00100000)<<2);
    // A2 hour is figured out, send it
   Wire._I2C_WRITE(temp_buffer);
    // Figure out A2 day/date and A2M4
    temp_buffer = ((AlarmBits & 0b01000000)<<1) | bin2bcd(A2Day);
    if (A2Dy) {
        // Set A2 Day/Date flag (Otherwise it's zero)
        temp_buffer = temp_buffer | 0b01000000;
    }
   Wire._I2C_WRITE(temp_buffer);
    // All done!
    Wire.endTransmission();
  }

  void RTC_DS3231_getA2Time(byte& A2Day, byte& A2Hour, byte& A2Minute, byte& AlarmBits, bool& A2Dy, bool& A2h12, bool& A2PM) {
    byte temp_buffer;
    Wire.beginTransmission(DS3231_ADDRESS);
    Wire.write(0x0b);
    Wire.endTransmission();

    Wire.requestFrom(DS3231_ADDRESS, 3);
    temp_buffer = Wire.read();   // Get A2M2 and A2 Minutes
    A2Minute    = bcd2bin(temp_buffer & 0b01111111);
    // put A2M2 bit in position 4 of DS3231_AlarmBits.
    AlarmBits   = AlarmBits | (temp_buffer & 0b10000000)>>3;

    temp_buffer = Wire.read();   // Get A2M3 and A2 Hour
    // put A2M3 bit in position 5 of DS3231_AlarmBits.
    AlarmBits   = AlarmBits | (temp_buffer & 0b10000000)>>2;
    // determine A2 12/24 mode
    A2h12       = temp_buffer & 0b01000000;
    if (A2h12) {
        A2PM    = temp_buffer & 0b00100000;         // determine am/pm
        A2Hour  = bcd2bin(temp_buffer & 0b00011111);   // 12-hour
    } else {
        A2Hour  = bcd2bin(temp_buffer & 0b00111111);   // 24-hour
    }

    temp_buffer = Wire.read();   // Get A2M4 and A1 Day/Date
    // put A2M4 bit in position 6 of DS3231_AlarmBits.
    AlarmBits   = AlarmBits | (temp_buffer & 0b10000000)>>1;
    // determine A2 day or date flag
    A2Dy        = (temp_buffer & 0b01000000)>>6;
    if (A2Dy) {
        // alarm is by day of week, not date.
        A2Day   = bcd2bin(temp_buffer & 0b00001111);
    } else {
        // alarm is by date, not day of week.
        A2Day   = bcd2bin(temp_buffer & 0b00111111);
    }
  }
*/
//oscilator functions: (note I disabled the oscilator output in setup!)
/*
  bool RTC_DS3231_checkIfAlarm(byte Alarm) {
    // Checks whether alarm 1 or alarm 2 flag is on, returns T/F accordingly.
    // Turns flag off, also.
    // defaults to checking alarm 2, unless Alarm == 1.
    byte result;
    byte temp_buffer = readControlByte(1);
    if (Alarm == 1) {
        // Did alarm 1 go off?
        result = temp_buffer & 0b00000001;
        // clear flag
        temp_buffer = temp_buffer & 0b11111110;
    } else {
        // Did alarm 2 go off?
        result = temp_buffer & 0b00000010;
        // clear flag
        temp_buffer = temp_buffer & 0b11111101;
    }
    writeControlByte(temp_buffer, 1);
    return result;
  }

  void RTC_DS3231_enableOscillator(bool TF, bool battery, byte frequency) {
    // turns oscillator on or off. True is on, false is off.
    // if battery is true, turns on even for battery-only operation,
    // otherwise turns off if Vcc is off.
    // frequency must be 0, 1, 2, or 3.
    // 0 = 1 Hz
    // 1 = 1.024 kHz
    // 2 = 4.096 kHz
    // 3 = 8.192 kHz (Default if frequency byte is out of range)
    if (frequency > 3) frequency = 3;
    // read control byte in, but zero out current state of RS2 and RS1.
    byte temp_buffer = readControlByte(0) & 0b11100111;
    if (battery) {
        // turn on BBSQW flag
        temp_buffer = temp_buffer | 0b01000000;
    } else {
        // turn off BBSQW flag
        temp_buffer = temp_buffer & 0b10111111;
    }
    if (TF) {
        // set ~EOSC to 0 and INTCN to zero.
        temp_buffer = temp_buffer & 0b01111011;
    } else {
        // set ~EOSC to 1, leave INTCN as is.
        temp_buffer = temp_buffer | 0b10000000;
    }
    // shift frequency into bits 3 and 4 and set.
    frequency = frequency << 3;
    temp_buffer = temp_buffer | frequency;
    // And write the control bits
    writeControlByte(temp_buffer, 0);
  }

  void RTC_DS3231_enable32kHz(bool TF) {
    // turn 32kHz pin on or off
    byte temp_buffer = readControlByte(1);
    if (TF) {
        // turn on 32kHz pin
        temp_buffer = temp_buffer | 0b00001000;
    } else {
        // turn off 32kHz pin
        temp_buffer = temp_buffer & 0b11110111;
    }
    writeControlByte(temp_buffer, 1);
  }

  bool RTC_DS3231_oscillatorCheck() {
    // Returns false if the oscillator has been off for some reason.
    // If this is the case, the time is probably not correct.
    byte temp_buffer = readControlByte(1);
    bool result = true;
    if (temp_buffer & 0b10000000) {
        // Oscillator Stop Flag (OSF) is set, so return false.
        result = false;
    }
    return result;
  }
*/



