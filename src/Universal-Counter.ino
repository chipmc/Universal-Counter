/*
* Project Universal-Counter - converged software for Low Power and Solar - Optimized for counting
* Description: Cellular Connected Data Logger for Utility and Solar powered installations
* Author: Chip McClelland chip@mcclellands.org
* Date: 8-12-2018
*/

/*  Next level integration - pressure and PIR sensors will both be treated the same - one code base
    Both implementations will move over to the finite state machine approach
    Both implementations will observe the park open and closing hours.
    I will also increase the ability for the end-user to configure the sensor without reflashing
    I will add two new states: 1) Low Power mode - maintains functionality but conserves battery by
    enabling sleep  2) Low Battery Mode - reduced functionality to preserve battery charge
    The watchdog timer should be set with a period of over 1 hour for the lowest power useage

    The mode and states will be set and recoded in the FRAM::controlRegisterAddr so resets will not change the mode
    Control Register - bits 7-4, 3 - Verbose Mode, 2- Solar Power Mode, 1 - Open, 0 - Low Power Mode
*/

namespace FRAM {                                    // Moved to namespace instead of #define to limit scope
  enum Addresses {
    versionAddr           = 0x0,                    // Where we store the memory map version number
    resetCountAddr        = 0x1,                    // Sensitivity for Accelerometer sensors
    minTransitAddr        = 0x2,                    // Where we store debounce in dSec or 1/10s of a sec (ie 1.6sec is stored as 16)
    maxGroupAddr          = 0x3,                    // This is where we keep track of how often the Electron was reset
    timeZoneAddr          = 0x4,                    // Store the local time zone data
    openTimeAddr          = 0x5,                    // Hour for opening the park / store / etc - military time (e.g. 6 is 6am)
    closeTimeAddr         = 0x6,                    // Hour for closing of the park / store / etc - military time (e.g 23 is 11pm)
    controlRegisterAddr   = 0x7,                    // This is the control register for storing the current state
    currentHourlyCountAddr =0x8,                    // Current Hourly Count - 16 bits
    currentDailyCountAddr = 0xC,                    // Current Daily Count - 16 bits
    currentCountsTimeAddr = 0xE,                    // Time of last count - 32 bits
  };
};



// Finally, here are the variables I want to change often and pull them all together here
const int versionNumber = 9;                        // Increment this number each time the memory map is changed
const char releaseNumber[6] = "0.2";               // Displays the release on the menu

// Included Libraries
#include "Adafruit_FRAM_I2C.h"        // Library for FRAM functions
#include "FRAM-Library-Extensions.h"  // Extends the FRAM Library
#include "electrondoc.h"              // Documents pinout

// Prototypes and System Mode calls
SYSTEM_MODE(MANUAL);          // This will enable user code to start executing automatically.
SYSTEM_THREAD(ENABLED);               // Means my code will not be held up by Particle processes.
STARTUP(System.enableFeature(FEATURE_RESET_INFO));
FuelGauge batteryMonitor;             // Prototype for the fuel gauge (included in Particle core library)
PMIC power;                           //Initalize the PMIC class so you can call the Power Management functions below.

// State Maching Variables
enum State { INITIALIZATION_STATE, ERROR_STATE, IDLE_STATE, SLEEPING_STATE, NAPPING_STATE, LOW_BATTERY_STATE, REPORTING_STATE, RESP_WAIT_STATE };
char stateNames[8][14] = {"Initialize", "Error", "Idle", "Sleeping", "Napping", "Low Battery", "Reporting", "Response Wait" };
State state = INITIALIZATION_STATE;
State oldState = INITIALIZATION_STATE;

// Pin Constants
const int tmp36Pin =      A0;                     // Simple Analog temperature sensor
const int wakeUpPin =     A7;                     // This is the Particle Electron WKP pin
const int enablePin =     B4;                     // Hold low to power down the device - for GPS Equipped units
const int tmp36Shutdwn =  B5;                     // Can turn off the TMP-36 to save energy
const int intPinMirror =  C3;                     // So we can see how the microprocessor is interpreting the intPin
const int intPin =        D3;                     // PIR Sensor Interrupt pin
const int hardResetPin =  D4;                     // Power Cycles the Electron and the Carrier Board
const int userSwitch =    D5;                     // User switch with a pull-up resistor
const int donePin =       D6;                     // Pin the Electron uses to "pet" the watchdog
const int blueLED =       D7;                     // This LED is on the Electron itself

// Timing Variables
unsigned long stayAwake;                          // This is the time before sleeping - can be short or long
const int wakeBoundary = 1*3600 + 0*60 + 0;         // 1 hour 0 minutes 0 seconds
const unsigned long stayAwakeLong = 90000;        // In lowPowerMode, how long to stay awake every hour - long
const unsigned long webhookWait = 45000;          // How long will we wair for a WebHook response
const unsigned long resetWait = 30000;            // How long will we wait in ERROR_STATE until reset
const int publishFrequency = 1000;                // We can only publish once a second
unsigned long stayAwakeTimeStamp = 0;             // Timestamps for our timing variables
unsigned long webhookTimeStamp = 0;
unsigned long resetTimeStamp = 0;
unsigned long publishTimeStamp = 0;               // Keep track of when we publish a webhook
unsigned long lastPublish = 0;

// Program Variables
int temperatureF;                                   // Global variable so we can monitor via cloud variable
int resetCount;                                     // Counts the number of times the Electron has had a pin reset
bool awokeFromNap =false;                           // Flag that napping was interrupted by interrupt not time
bool ledState = LOW;                                // variable used to store the last LED status, to toggle the light
bool readyForBed = false;                           // Checks to see if steps for sleep have been completed
bool pettingEnabled = true;                         // Let's us pet the hardware watchdog
bool dataInFlight = false;                          // Tracks if we have sent data but not yet cleared it from counts until we get confirmation
byte controlRegisterValue;                               // Stores the control register values
bool lowPowerMode;                                  // Flag for Low Power Mode operations
bool solarPowerMode;                                // Changes the PMIC settings
bool verboseMode;                                   // Enables more active communications for configutation and setup
char SignalString[17];                              // Used to communicate Wireless RSSI and Description
const char* levels[6] = {"Poor", "Low", "Medium", "Good", "Very Good", "Great"};


// Time Related Variables
time_t t;                                           // Global time vairable
int openTime;                                       // Park Opening time - (24 hr format) sets waking
int closeTime;                                      // Park Closing time - (24 hr format) sets sleep
byte lastHour = 0;                                  // For recording the startup values
byte lastDate = 0;                                  // These values make sure we record events if time has lapsed
byte currentHourlyPeriod;                           // This is where we will know if the period changed
byte currentDailyPeriod;                            // We will keep daily counts as well as period counts

// Battery monitoring
int stateOfCharge = 0;                              // Stores battery charge level value
int lowBattLimit;                                   // Trigger for Low Batt State

// This section is where we will initialize sensor specific variables, libraries and function prototypes
// PIR Sensor variables
volatile bool sensorDetect = false;         // This is the flag that an interrupt is triggered
volatile unsigned long currentEvent = 0;    // Time for the current sensor event
volatile bool pinState = false;             // Pin change interrupt so need to capture state
unsigned long startDetect = 0;              // When was the sensor event start - intPin goes HIGH
unsigned long stopDetect = 0;               // When did sensor event stop - intPin goes LOW
int maxGroup=1;                             // What is the biggest group that will walk together - prevents large counts if someone stops in front of the sensor
int hourlyPersonCount = 0;                  // hourly counter
int hourlyPersonCountSent = 0;              // Person count in flight to Ubidots
int dailyPersonCount = 0;                   // daily counter
char transitTimeStr[8] = "NA";              // String to communicate transit time to app
char minTransitStr[8] = "NA";               // String to communicate transit time to app
float minTransit;                           // Ths is the minimum time needed to transit the field in sec to tenths

void setup()                                // Note: Disconnected Setup()
{
  pinMode(enablePin,OUTPUT);                // For GPS enabled units
  digitalWrite(enablePin,LOW);              // Turn off GPS to save battery
  pinMode(intPin,INPUT_PULLDOWN);           // PIR Sensor Interrupt pin
  pinMode(intPinMirror,OUTPUT);             // Mirrors the intPin
  pinMode(wakeUpPin,INPUT);                 // This pin is active HIGH
  pinMode(userSwitch,INPUT);                // Momentary contact button on board for direct user input
  pinMode(blueLED, OUTPUT);                 // declare the Blue LED Pin as an output
  pinMode(tmp36Shutdwn,OUTPUT);             // Supports shutting down the TMP-36 to save juice
  digitalWrite(tmp36Shutdwn, HIGH);         // Turns on the temp sensor
  pinMode(donePin,OUTPUT);                  // Allows us to pet the watchdog
  pinMode(hardResetPin,OUTPUT);             // For a hard reset active HIGH

  watchdogISR();                            // Pet the watchdog

  char responseTopic[125];
  String deviceID = System.deviceID();                                // Multiple Electrons share the same hook - keeps things straight
  deviceID.toCharArray(responseTopic,125);
  Particle.subscribe(responseTopic, UbidotsHandler, MY_DEVICES);      // Subscribe to the integration response event

  Particle.variable("HourlyCount", hourlyPersonCount);                // Define my Particle variables
  Particle.variable("DailyCount", dailyPersonCount);                  // Note: Don't have to be connected for any of this!!!
  Particle.variable("Last-Transit",transitTimeStr);
  Particle.variable("Min-Transit",minTransitStr);
  Particle.variable("Max-Group",maxGroup);
  Particle.variable("SignalString", SignalString);
  Particle.variable("ResetCount", resetCount);
  Particle.variable("Release",releaseNumber);
  Particle.variable("stateOfChg", stateOfCharge);
  Particle.variable("lowPowerMode",lowPowerMode);
  Particle.variable("OpenTime",openTime);
  Particle.variable("CloseTime",closeTime);

  Particle.function("Reset-FRAM", resetFRAM);                         // These functions allow you to configure and control the Electron
  Particle.function("Reset-Counts",resetCounts);
  Particle.function("Hard-Reset",hardResetNow);
  Particle.function("Set-Transit",setMinTransit);
  Particle.function("Set-Group",setMaxGroup);
  Particle.function("Send-Now",sendNow);
  Particle.function("LowPowerMode",setLowPowerMode);
  Particle.function("Solar-Mode",setSolarMode);
  Particle.function("Verbose-Mode",setVerboseMode);
  Particle.function("SetTimeZone",setTimeZone);
  Particle.function("SetOpenTime",setOpenTime);
  Particle.function("SetClose",setCloseTime);

  if (!fram.begin()) {                                                // You can stick the new i2c addr in here, e.g. begin(0x51);
    resetTimeStamp = millis();                                        // FRAM failed to initialize, reset
    state = ERROR_STATE;
  }
  else if (FRAMread8(FRAM::versionAddr) != versionNumber) {           // Check to see if the memory map in the sketch matches the data on the chip
    ResetFRAM();                                                      // Reset the FRAM to correct the issue
    if (FRAMread8(FRAM::versionAddr) != versionNumber) {
      resetTimeStamp = millis();
      state = ERROR_STATE;                                            // Resetting did not fix the issue
    }
    else {
      FRAMwrite8(FRAM::controlRegisterAddr,0);                        // Need to reset so not in low power or low battery mode
      FRAMwrite8(FRAM::openTimeAddr,0);                               // These set the defaults if the FRAM is erased
      FRAMwrite8(FRAM::closeTimeAddr,24);                             // This will ensure the device does not sleep
      FRAMwrite8(FRAM::minTransitAddr,20);                            // Min transit is stored in 10ths of a second
      FRAMwrite8(FRAM::maxGroupAddr,4);
    }
  }

  resetCount = FRAMread8(FRAM::resetCountAddr);                       // Retrive system recount data from FRAM
  if (System.resetReason() == RESET_REASON_PIN_RESET)                 // Check to see if we are starting from a pin reset
  {
    resetCount++;
    FRAMwrite8(FRAM::resetCountAddr,static_cast<uint8_t>(resetCount));// If so, store incremented number - watchdog must have done This
  }

  // Here we load the values from FRAM
  minTransit = float(FRAMread8(FRAM::minTransitAddr))/10.0;           // The time to cross the sensor's field of view in seconds (PIR only)
  snprintf(minTransitStr,sizeof(minTransitStr),"%2.1f sec",minTransit);
  maxGroup = FRAMread8(FRAM::maxGroupAddr);                           // How many people might walk together (PIR only)
  openTime = FRAMread8(FRAM::openTimeAddr);                           // Open time of the park (24 hour format)
  closeTime = FRAMread8(FRAM::closeTimeAddr);                         // Close time of the park (24 hr format)

  int8_t tempTimeZoneOffset = FRAMread8(FRAM::timeZoneAddr);          // Load Time zone data from FRAM
  if (tempTimeZoneOffset <= 12 && tempTimeZoneOffset >= -12)  Time.zone((float)tempTimeZoneOffset);  // Load Timezone from FRAM
  else Time.zone(-4);                                                 // Default is EST in case proper value not in FRAM

  // And set the flags from the control register
  controlRegisterValue = FRAMread8(FRAM::controlRegisterAddr);        // Read the Control Register for system modes so they stick even after reset
  lowPowerMode    = (0b00000001 & controlRegisterValue);              // Set the lowPowerMode
  solarPowerMode  = (0b00000100 & controlRegisterValue);              // Set the solarPowerMode
  verboseMode     = (0b00001000 & controlRegisterValue);              // Set the verboseMode

  PMICreset();                                                        // Executes commands that set up the PMIC for Solar charging - once we know the Solar Mode

  if ((0b00010000 & controlRegisterValue)) {
    Particle.connect();                                                // Connects and turns stuff on
    waitFor(Particle.connected, 20000);                               // Give us 20 seconds to connect
    Particle.process();
  }

  takeMeasurements();                                                 // For the benefit of monitoring the device

  currentHourlyPeriod = Time.hour();                                  // Sets the hour period for when the system starts
  currentDailyPeriod = Time.day();                                    // And the day
  time_t unixTime = FRAMread32(FRAM::currentCountsTimeAddr);          // Need to reload program control since reset
  lastHour = Time.hour(unixTime);                                     // Extract hour and day from last count
  lastDate = Time.day(unixTime);
  dailyPersonCount = FRAMread16(FRAM::currentDailyCountAddr);         // Load Daily Count from memory
  hourlyPersonCount = FRAMread16(FRAM::currentHourlyCountAddr);       // Load Hourly Count from memory


  while(pinReadFast(intPin) && millis() < resetWait);                 // Make sure that the int Pin is low before starting
  if (pinReadFast(intPin)) {                                          // Something is wrong - intPin should have cleared so will reset
    resetTimeStamp = millis();                                        // Send this to the ERROR_STATE
    state = ERROR_STATE;                                              // *** Need to improve this *** add logic to power cycle the sensor board
  }
  attachInterrupt(wakeUpPin, watchdogISR, RISING);                    // The watchdog timer will signal us and we have to respond
  attachInterrupt(intPin,sensorISR,CHANGE);                           // Will know when the PIR sensor is triggered

  if (!pinReadFast(userSwitch)) {                                     // Rescue mode to locally take lowPowerMode so you can connect to device
    openTime = 0;                                                     // Device may alos be sleeping due to time or TimeZone setting
    FRAMwrite8(FRAM::openTimeAddr,0);                                 // Reset open and close time values to ensure device is awake
    closeTime = 24;
    FRAMwrite8(FRAM::closeTimeAddr,24);
    controlRegisterValue = FRAMread8(FRAM::controlRegisterAddr);        // Get the control register (general approach)
    controlRegisterValue = (0b11111110 & controlRegisterValue);      // If so, turn off the lowPowerMode bit
    controlRegisterValue = (0b00010000 | controlRegisterValue);      // Turn on connected mode 1 = connected and 0 = disconnected
    lowPowerMode = false;
    Particle.connect();
    waitFor(Particle.connected,10000);
    Particle.process();
    digitalWrite(tmp36Shutdwn, HIGH);                                // Turns on the temp sensor
    FRAMwrite8(FRAM::controlRegisterAddr,controlRegisterValue);        // Write to the control register
    if (Particle.connected()) Particle.publish("Startup","Startup rescue - reset time and power");
  }

  if (state != ERROR_STATE) state = IDLE_STATE;                       // IDLE unless error from above code

  stayAwake = stayAwakeLong;
}

void loop()
{
  switch(state) {
  case IDLE_STATE:
    if (verboseMode && state != oldState) publishStateTransition();
    if(hourlyPersonCountSent) {                                         // Cleared here as there could be counts coming in while "in Flight"
      hourlyPersonCount -= hourlyPersonCountSent;                       // Confirmed that count was recevied - clearing
      FRAMwrite16(FRAM::currentHourlyCountAddr, static_cast<uint16_t>(hourlyPersonCount));  // Load Hourly Count to memory
      hourlyPersonCountSent = 0;            // Reset for next time
    }
    if (sensorDetect) recordCount();                                    // The ISR had raised the sensor flag
    if (lowPowerMode && (millis() > stayAwakeTimeStamp + stayAwake)) state = NAPPING_STATE;
    if (Time.hour() != currentHourlyPeriod) state = REPORTING_STATE;    // We want to report on the hour but not after bedtime
    if ((Time.hour() >= closeTime || Time.hour() < openTime)) state = SLEEPING_STATE;   // The park is closed, time to sleep
    if (stateOfCharge <= lowBattLimit) state = LOW_BATTERY_STATE;               // The battery is low - sleep
    break;

  case SLEEPING_STATE: {                                                // This state is triggered once the park closes and runs until it opens
    if (verboseMode && state != oldState) publishStateTransition();
    if (!readyForBed)                                                   // Only do these things once - at bedtime
    {
      detachInterrupt(intPin);                                          // Done sensing for the day
      if (hourlyPersonCount) {                                          // If this number is not zero then we need to send this last count
        state = REPORTING_STATE;
        break;
      }
      if ((0b00010000 & controlRegisterValue)) {
        controlRegisterValue = FRAMread8(FRAM::controlRegisterAddr);    // Get the control register (general approach)
        controlRegisterValue = (0b00000001 | controlRegisterValue);     // If so, flip the lowPowerMode bit
        controlRegisterValue = (0b11101111 & controlRegisterValue);     // Turn off connected mode 1 = connected and 0 = disconnected
        FRAMwrite8(FRAM::controlRegisterAddr,controlRegisterValue);     // Write to the control register
        lowPowerMode = true;
        Cellular.off();
        delay(1000);                                                    // Advice from Rickkas - https://community.particle.io/t/electron-sleep-problems-yet-again/42230/11
        digitalWrite(blueLED,LOW);                                      // Turn off the LED
        digitalWrite(tmp36Shutdwn, LOW);                                // Turns off the temp sensor
      }
      FRAMwrite16(FRAM::currentDailyCountAddr, 0);                      // Reset the counts in FRAM
      FRAMwrite8(FRAM::resetCountAddr, 0);
      FRAMwrite16(FRAM::currentHourlyCountAddr, 0);
      dailyPersonCount = resetCount = hourlyPersonCount = 0;            // All the counts have been reported so time to zero everything
      watchdogISR();                                                    // Pet the watchdog
      readyForBed = true;                                               // Set the flag for the night
    }
    int secondsToHour = (60*(60 - Time.minute()));                      // Time till the top of the hour
    System.sleep(SLEEP_MODE_SOFTPOWEROFF,secondsToHour);                // Very deep sleep till the next hour - then resets
    } break;

  case NAPPING_STATE: {  // This state puts the device in low power mode quickly
      if (verboseMode && state != oldState) publishStateTransition();
      if (pinState) break;                                              // Don't nap until we are done with event
      if ((0b00010000 & controlRegisterValue)) {
        Particle.disconnect();                                          // Otherwise Electron will attempt to reconnect on wake
        controlRegisterValue = FRAMread8(FRAM::controlRegisterAddr);    // Get the control register (general approach)
        controlRegisterValue = (0b11101111 & controlRegisterValue);     // Turn off connected mode 1 = connected and 0 = disconnected
        Cellular.off();
        delay(1000);                                                    // Bummer but only should happen once an hour
        FRAMwrite8(FRAM::controlRegisterAddr,controlRegisterValue);     // Write to the control register
      }
      watchdogISR();                                                    // Pet the watchdog
      stayAwake = minTransit*1000;                                      // Need to convert to millis
      int wakeInSeconds = constrain(wakeBoundary - Time.now() % wakeBoundary, 1, wakeBoundary);
      System.sleep(intPin, RISING, wakeInSeconds);                      // Sensor will wake us with an interrupt or timeout at the hour
      if (System.wokenUpByPin()) {
         awokeFromNap=true;                                             // Since millis() stops when sleeping - need this to debounce
         stayAwakeTimeStamp = millis();
      }
      state = IDLE_STATE;                                               // Back to the IDLE_STATE after a nap
    } break;

  case LOW_BATTERY_STATE: {                                             // Sleep state but leaves the fuel gauge on
      if (verboseMode && state != oldState) publishStateTransition();
      detachInterrupt(intPin);                                          // Done sensing for the day
      if ((0b00010000 & controlRegisterValue)) {
        controlRegisterValue = FRAMread8(FRAM::controlRegisterAddr);    // Get the control register (general approach)
        controlRegisterValue = (0b00000001 | controlRegisterValue);     // If so, flip the lowPowerMode bit
        controlRegisterValue = (0b11101111 & controlRegisterValue);     // Turn off connected mode 1 = connected and 0 = disconnected
        FRAMwrite8(FRAM::controlRegisterAddr,controlRegisterValue);     // Write to the control register
        lowPowerMode = true;
        Cellular.off();
        delay(1000);                                                    // Advice from Rickkas - https://community.particle.io/t/electron-sleep-problems-yet-again/42230/11
        digitalWrite(blueLED,LOW);                                      // Turn off the LED
        digitalWrite(tmp36Shutdwn, LOW);                                // Turns off the temp sensor
      }
      watchdogISR();                                                    // Pet the watchdog
      int secondsToHour = (60*(60 - Time.minute()));                    // Time till the top of the hour
      System.sleep(SLEEP_MODE_DEEP,secondsToHour);                      // Very deep sleep till the next hour - then resets
    } break;

  case REPORTING_STATE:                                                 // Reporting - hourly or on command
    if (verboseMode && state != oldState) publishStateTransition();
    watchdogISR();                                                      // Pet the watchdog once an hour
    pettingEnabled = false;                                             // Going to see the reporting process through before petting again
    if (!(0b00010000 & controlRegisterValue)) {                         // If in disconnected mode
      controlRegisterValue = FRAMread8(FRAM::controlRegisterAddr);      // Get the control register (general approach)
      controlRegisterValue = (0b00010000 | controlRegisterValue);       // Turn on connected mode 1 = connected and 0 = disconnected
      FRAMwrite8(FRAM::controlRegisterAddr,controlRegisterValue);       // Write to the control register
      Particle.connect();
      waitFor(Particle.connected,10000);
      Particle.process();
    }
    takeMeasurements();                                                 // Update Temp, Battery and Signal Strength values
    sendEvent();                                                        // Send data to Ubidots
    webhookTimeStamp = millis();
    state = RESP_WAIT_STATE;                                            // Wait for Response
    break;

  case RESP_WAIT_STATE:
    if (verboseMode && state != oldState) publishStateTransition();
    if (!dataInFlight)                                                  // Response received back to IDLE state
    {
      state = IDLE_STATE;
      pettingEnabled = true;                                            // Enable petting before going back into the loop
      stayAwakeTimeStamp = millis();
    }
    else if (millis() > webhookTimeStamp + webhookWait) {               // If it takes too long - will need to reset
      resetTimeStamp = millis();
      state = ERROR_STATE;                                              // Response timed out
    }
    break;

  case ERROR_STATE:                                          // To be enhanced - where we deal with errors
    if (verboseMode && state != oldState) publishStateTransition();
    if (millis() > resetTimeStamp + resetWait)
    {
      if (Particle.connected()) Particle.publish("State","ERROR_STATE - Resetting");  // Publish this regardless of Verbose Mode
      delay(2000);                                          // This makes sure it goes through before reset
      if (resetCount <= 3)  System.reset();                 // Today, only way out is reset
      else {
        FRAMwrite8(FRAM::resetCountAddr,0);                           // Time for a hard reset
        digitalWrite(hardResetPin,HIGH);                    // Zero the count so only every three
      }
    }
    break;
  }
  Particle.process();
}

void recordCount()                                          // Handles counting when the sensor triggers
{
  char data[256];                                           // Store the date in this character array - not global
  sensorDetect = false;                                     // Reset the flag
  if (pinState) {
    if ((millis() >= startDetect + minTransit*1000 || awokeFromNap)) {
    startDetect = currentEvent;
    }
  }
  else if (!pinState) {                                     // 1st attempt will fail if there are too many retriggers
    if (awokeFromNap) currentEvent+=minTransit*1000;
    stopDetect = currentEvent;
    float timeLapsed = (stopDetect - startDetect) / 1000.0;   // This is the time between this event and last
    if (minTransit == 0) minTransit = 1.0;                 // Cannot allow a divide by zero here
    int peopleInGroup = constrain(int(timeLapsed / minTransit),1,maxGroup);   // This will count multiples if longer signal
    hourlyPersonCount += peopleInGroup;
    dailyPersonCount += peopleInGroup;
    FRAMwrite16(FRAM::currentHourlyCountAddr, static_cast<uint16_t>(hourlyPersonCount));  // Load Hourly Count to memory
    FRAMwrite16(FRAM::currentDailyCountAddr, static_cast<uint16_t>(dailyPersonCount));   // Load Daily Count to memory
    if (verboseMode && !lowPowerMode) {                                          // Publish if we are in verbose mode
      snprintf(transitTimeStr, sizeof(transitTimeStr),"%2.1f sec",timeLapsed);
      snprintf(data, sizeof(data), "Group of %i so %i hourly and transit of %2.1f seconds", peopleInGroup,hourlyPersonCount,timeLapsed);
      if (Particle.connected()) Particle.publish("Count",data);                     // Don't meter this publish as it cold mess with timing
    }
  }
  awokeFromNap = false;                                    // Reset the flag
  if (!pinReadFast(userSwitch) && lowPowerMode) {          // A low value means someone is pushing this button take out of low power mode
    lowPowerMode = false;
    controlRegisterValue = FRAMread8(FRAM::controlRegisterAddr);        // Get the control register (general approach)
    controlRegisterValue = (0b11111110 & controlRegisterValue);      // If so, turn off the lowPowerMode bit
    controlRegisterValue = (0b00010000 | controlRegisterValue);      // Turn on connected mode 1 = connected and 0 = disconnected
    lowPowerMode = false;
    Particle.connect();
    waitFor(Particle.connected,10000);
    Particle.process();
    digitalWrite(tmp36Shutdwn, HIGH);                                // Turns on the temp sensor
    FRAMwrite8(FRAM::controlRegisterAddr,controlRegisterValue);        // Write to the control register
    if (Particle.connected()) Particle.publish("Mode","User Button in recordCount");
  }
}

void sendEvent()
{
  char data[256];                                                         // Store the date in this character array - not global
  snprintf(data, sizeof(data), "{\"hourly\":%i, \"daily\":%i,\"battery\":%i, \"temp\":%i, \"resets\":%i}",hourlyPersonCount, dailyPersonCount, stateOfCharge, temperatureF,resetCount);
  Particle.publish("Ubidots-Hook", data, PRIVATE);
  hourlyPersonCountSent = hourlyPersonCount;                              // This is the number that was sent to Ubidots - will be subtracted once we get confirmation
  currentHourlyPeriod = Time.hour();                                      // Change the time period
  dataInFlight = true;                                                    // set the data inflight flag
}

void UbidotsHandler(const char *event, const char *data)  // Looks at the response from Ubidots - Will reset Photon if no successful response
{
  // Response Template: "{{hourly.0.status_code}}" so, I should only get a 3 digit number back
  char dataCopy[strlen(data)+1];                        // data needs to be copied since if (Particle.connected()) Particle.publish() will clear it
  strncpy(dataCopy, data, sizeof(dataCopy));            // Copy - overflow safe
  if (!strlen(dataCopy)) return;                        // First check to see if there is any data
  int responseCode = atoi(dataCopy);                    // Response is only a single number thanks to Template
  if ((responseCode == 200) || (responseCode == 201))
  {
    if (verboseMode) {
      waitUntil(meterParticlePublish);
      if (Particle.connected()) Particle.publish("State","Response Received");
      lastPublish = millis();
    }
    dataInFlight = false;                                 // Data has been received
  }
  else {
    if (verboseMode) {
      waitUntil(meterParticlePublish);
      if (Particle.connected()) Particle.publish("Ubidots Hook", dataCopy);       // Publish the response code
      lastPublish = millis();
    }
  }
}

// These are the functions that are part of the takeMeasurements call

void takeMeasurements() {
  if (Cellular.ready()) getSignalStrength();                // Test signal strength if the cellular modem is on and ready
  getTemperature();                                         // Get Temperature at startup as well
  stateOfCharge = int(batteryMonitor.getSoC());             // Percentage of full charge
}

void getSignalStrength()
{
    CellularSignal sig = Cellular.RSSI();  // Prototype for Cellular Signal Montoring
    int rssi = sig.rssi;
    int strength = map(rssi, -131, -51, 0, 5);
    snprintf(SignalString,17, "%s: %d", levels[strength], rssi);
}

int getTemperature()
{
  int reading = analogRead(tmp36Pin);   //getting the voltage reading from the temperature sensor
  float voltage = reading * 3.3;        // converting that reading to voltage, for 3.3v arduino use 3.3
  voltage /= 4096.0;                    // Electron is different than the Arduino where there are only 1024 steps
  int temperatureC = int(((voltage - 0.5) * 100));  //converting from 10 mv per degree with 500 mV offset to degrees ((voltage - 500mV) times 100) - 5 degree calibration
  temperatureF = int((temperatureC * 9.0 / 5.0) + 32.0);  // now convert to Fahrenheit
  return temperatureF;
}

// Here are the various hardware and timer interrupt service routines

void sensorISR()
{
  sensorDetect = true;                              // sets the sensor flag for the main loop
  currentEvent = millis();                        // Time in time_t of the interrupt
  pinState = pinReadFast(intPin);                   // Capture the state of the pin
  if (pinState) pinSetFast(blueLED);
  else pinResetFast(blueLED);
}

void watchdogISR()
{
  if (pettingEnabled) {
    digitalWrite(donePin, HIGH);                      // Pet the watchdog
    digitalWrite(donePin, LOW);
  }
}

bool notConnected() {
  return !Particle.connected();                             // This is a requirement to use waitFor
}

// Power Management function

void PMICreset() {
  power.begin();                                            // Settings for Solar powered power management
  power.disableWatchdog();
  if (solarPowerMode) {
    lowBattLimit = 20;                                      // Trigger for Low Batt State
    power.setInputVoltageLimit(4840);                       // Set the lowest input voltage to 4.84 volts best setting for 6V solar panels
    power.setInputCurrentLimit(900);                        // default is 900mA
    power.setChargeCurrent(0,0,1,0,0,0);                    // default is 512mA matches my 3W panel
    power.setChargeVoltage(4208);                           // Allows us to charge cloe to 100% - battery can't go over 45 celcius
  }
  else  {
    lowBattLimit = 30;                                      // Trigger for Low Batt State
    power.setInputVoltageLimit(4208);                       // This is the default value for the Electron
    power.setInputCurrentLimit(1500);                       // default is 900mA this let's me charge faster
    power.setChargeCurrent(0,1,1,0,0,0);                    // default is 2048mA (011000) = 512mA+1024mA+512mA)
    power.setChargeVoltage(4112);                           // default is 4.112V termination voltage
  }
}

// These are the particle functions that allow you to configure and run the device
// They are intended to allow for customization and control during installations
// and to allow for management.

int resetFRAM(String command)   // Will reset the local counts
{
  if (command == "1")
  {
    ResetFRAM();
    return 1;
  }
  else return 0;
}

int resetCounts(String command)   // Resets the current hourly and daily counts
{
  if (command == "1")
  {
    FRAMwrite16(FRAM::currentDailyCountAddr, 0);   // Reset Daily Count in memory
    FRAMwrite16(FRAM::currentHourlyCountAddr, 0);  // Reset Hourly Count in memory
    FRAMwrite8(FRAM::resetCountAddr,0);          // If so, store incremented number - watchdog must have done This
    resetCount = 0;
    hourlyPersonCount = 0;                    // Reset count variables
    dailyPersonCount = 0;
    hourlyPersonCountSent = 0;                // In the off-chance there is data in flight
    dataInFlight = false;
    return 1;
  }
  else return 0;
}

int hardResetNow(String command)   // Will perform a hard reset on the Electron
{
  if (command == "1")
  {
    digitalWrite(hardResetPin,HIGH);          // This will cut all power to the Electron AND the carrir board
    return 1;                                 // Unfortunately, this will never be sent
  }
  else return 0;
}


int setMinTransit(String command)  // This is the amount of time in seconds we will wait before starting a new session
{
  char * pEND;
  char data[256];
  float tempMinTransit = strtof(command,&pEND);                       // Looks for the first integer and interprets it
  if ((tempMinTransit < 0.9) | (tempMinTransit > 25)) return 0;   // Make sure it falls in a valid range or send a "fail" result
  minTransit = tempMinTransit;                   // keepSession - The time to keep a session alive - in seconds to tenths
  int minTransitFRAM = constrain(int(minTransit*10),1,255); // Store as a byte in FRAM = 1.6 becomes 16
  FRAMwrite8(FRAM::minTransitAddr,static_cast<uint8_t>(minTransitFRAM));
  snprintf(minTransitStr,sizeof(minTransitStr),"%2.1f sec",minTransit);
  if (verboseMode) {
    waitUntil(meterParticlePublish);
    snprintf(data, sizeof(data), "Minimum Transit: %2.1f seconds",minTransit);
    if (Particle.connected()) Particle.publish("Variables",data);
    lastPublish = millis();
  }
  return 1;
}

int setMaxGroup(String command)  // This is the amount of time in seconds we will wait before starting a new session
{
  char * pEND;
  char data[256];
  int tempGroup = strtol(command,&pEND,10);                       // Looks for the first integer and interprets it
  if ((tempGroup < 0) | (tempGroup > 250)) return 0;   // Make sure it falls in a valid range or send a "fail" result
  FRAMwrite8(FRAM::maxGroupAddr,static_cast<uint8_t>(tempGroup));
  maxGroup = tempGroup;                   // keepSession - The time to keep a session alive - in seconds
  if (verboseMode) {
    waitUntil(meterParticlePublish);
    snprintf(data, sizeof(data), "Max Group Size: %i people",maxGroup);
    if (Particle.connected()) Particle.publish("Variables",data);
    lastPublish = millis();
  }
  return 1;
}

int sendNow(String command) // Function to force sending data in current hour
{
  if (command == "1")
  {
    state = REPORTING_STATE;
    return 1;
  }
  else return 0;
}

int setSolarMode(String command) // Function to force sending data in current hour
{
  if (command == "1")
  {
    solarPowerMode = true;
    FRAMread8(FRAM::controlRegisterAddr);
    controlRegisterValue = (0b00000100 | controlRegisterValue);          // Turn on solarPowerMode
    FRAMwrite8(FRAM::controlRegisterAddr,controlRegisterValue);               // Write it to the register
    PMICreset();                                               // Change the power management Settings
    if (verboseMode) {
      waitUntil(meterParticlePublish);
      if (Particle.connected()) Particle.publish("Mode","Set Solar Powered Mode");
      lastPublish = millis();
    }
    return 1;
  }
  else if (command == "0")
  {
    solarPowerMode = false;
    FRAMread8(FRAM::controlRegisterAddr);
    controlRegisterValue = (0b11111011 & controlRegisterValue);           // Turn off solarPowerMode
    FRAMwrite8(FRAM::controlRegisterAddr,controlRegisterValue);                // Write it to the register
    PMICreset();                                                // Change the power management settings
    if (verboseMode) {
      waitUntil(meterParticlePublish);
      if (Particle.connected()) Particle.publish("Mode","Cleared Solar Powered Mode");
      lastPublish = millis();
    }
    return 1;
  }
  else return 0;
}

int setVerboseMode(String command) // Function to force sending data in current hour
{
  if (command == "1")
  {
    verboseMode = true;
    FRAMread8(FRAM::controlRegisterAddr);
    controlRegisterValue = (0b00001000 | controlRegisterValue);                    // Turn on verboseMode
    FRAMwrite8(FRAM::controlRegisterAddr,controlRegisterValue);                        // Write it to the register
    if (verboseMode) {
      waitUntil(meterParticlePublish);
      if (Particle.connected()) Particle.publish("Mode","Set Verbose Mode");
      lastPublish = millis();
    }
    return 1;
  }
  else if (command == "0")
  {
    verboseMode = false;
    FRAMread8(FRAM::controlRegisterAddr);
    controlRegisterValue = (0b11110111 & controlRegisterValue);                    // Turn off verboseMode
    FRAMwrite8(FRAM::controlRegisterAddr,controlRegisterValue);                        // Write it to the register
    if (verboseMode) {
      waitUntil(meterParticlePublish);
      if (Particle.connected()) Particle.publish("Mode","Cleared Verbose Mode");
      lastPublish = millis();
    }
    return 1;
  }
  else return 0;
}

int setTimeZone(String command)
{
  char * pEND;
  int8_t tempTimeZoneOffset = strtol(command,&pEND,10);                       // Looks for the first integer and interprets it
  if ((tempTimeZoneOffset < -12) | (tempTimeZoneOffset > 12)) return 0;   // Make sure it falls in a valid range or send a "fail" result
  Time.zone((float)tempTimeZoneOffset);
  FRAMwrite8(FRAM::timeZoneAddr,tempTimeZoneOffset);                             // Store the new value in FRAMwrite8
  if (verboseMode) {
    char data[256];
    waitUntil(meterParticlePublish);
    t = Time.now();
    snprintf(data, sizeof(data), "Time zone offset %i",tempTimeZoneOffset);
    if (Particle.connected()) Particle.publish("Time",data);
    delay(1000);
    if (Particle.connected()) Particle.publish("Time",Time.timeStr(t));
    lastPublish = millis();
  }
  return 1;
}

int setOpenTime(String command)
{
  char * pEND;
  int tempTime = strtol(command,&pEND,10);                       // Looks for the first integer and interprets it
  if ((tempTime < 0) || (tempTime > 23)) return 0;   // Make sure it falls in a valid range or send a "fail" result
  openTime = tempTime;
  FRAMwrite8(FRAM::openTimeAddr,openTime);                             // Store the new value in FRAMwrite8
  if (verboseMode) {
    char data[256];
    waitUntil(meterParticlePublish);
    snprintf(data, sizeof(data), "Open time set to %i",openTime);
    if (Particle.connected()) Particle.publish("Time",data);
    lastPublish = millis();
  }
  return 1;
}

int setCloseTime(String command)
{
  char * pEND;
  int tempTime = strtol(command,&pEND,10);                       // Looks for the first integer and interprets it
  if ((tempTime < 18) || (tempTime > 24)) return 0;   // Make sure it falls in a valid range or send a "fail" result
  closeTime = tempTime;
  FRAMwrite8(FRAM::closeTimeAddr,closeTime);                             // Store the new value in FRAMwrite8
  if (verboseMode) {
    char data[256];
    waitUntil(meterParticlePublish);
    snprintf(data, sizeof(data), "Closing time set to %i",closeTime);
    if (Particle.connected()) Particle.publish("Time",data);
    lastPublish = millis();
  }
  return 1;
}


int setLowPowerMode(String command)                                   // This is where we can put the device into low power mode if needed
{
  if (command != "1" && command != "0") return 0;                     // Before we begin, let's make sure we have a valid input
  if (command == "1")                                                 // Command calls for setting lowPowerMode
  {
    if (verboseMode) {
      waitUntil(meterParticlePublish);
      if (Particle.connected()) Particle.publish("Mode","Low Power");
      lastPublish = millis();
    }
    controlRegisterValue = FRAMread8(FRAM::controlRegisterAddr);      // Get the control register (general approach)
    controlRegisterValue = (0b00000001 | controlRegisterValue);       // Flip the lowPowerMode bit
    FRAMwrite8(FRAM::controlRegisterAddr,controlRegisterValue);       // Write to the control register
    lowPowerMode = true;
  }
  else                                                                // Command calls for clearing lowPowerMode
  {
    if (verboseMode) {
      waitUntil(meterParticlePublish);
      if (Particle.connected()) Particle.publish("Mode","Normal Operations");
      lastPublish = millis();
    }
    controlRegisterValue = FRAMread8(FRAM::controlRegisterAddr);      // Get the control register (general approach)
    controlRegisterValue = (0b11111110 & controlRegisterValue);       // If so, turn off the lowPowerMode bit
    controlRegisterValue = (0b00010000 | controlRegisterValue);       // Turn on connected mode 1 = connected and 0 = disconnected
    FRAMwrite8(FRAM::controlRegisterAddr,controlRegisterValue);       // Write to the control register
    lowPowerMode = false;
    Particle.connect();
    waitFor(Particle.connected,10000);
    Particle.process();
  }
  return 1;
}

bool meterParticlePublish(void)
{
  if(millis() - lastPublish >= publishFrequency) return 1;
  else return 0;
}

void publishStateTransition(void)
{
  char stateTransitionString[40];
  snprintf(stateTransitionString, sizeof(stateTransitionString), "From %s to %s", stateNames[oldState],stateNames[state]);
  oldState = state;
  waitUntil(meterParticlePublish);
  if (Particle.connected()) Particle.publish("State Transition",stateTransitionString);
  lastPublish = millis();
}
