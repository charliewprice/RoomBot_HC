#include <Arduino.h>

/********************************************************************************************************************************************
 *
 * AgriMote (c)  2020, 2021, 2022, 2023 Charlie Price
 *
 * This uses OTAA (Over-the-air activation), where where a DevEUI and
 * application key is configured, which are used in an over-the-air
 * activation procedure where a DevAddr and session keys are
 * assigned/generated for use with all further communication.
 *
 *******************************************************************************************************************************************/

#include <Wire.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <RTCZero.h>
#include "ErriezSerialTerminal.h"
char newlineChar = '\n';
char delimiterChar = ' ';
SerialTerminal term(newlineChar, delimiterChar);

#include "HeatPadControl.h"
HeatPadControl heatPadController;

#include "Eeprom.h"
Eeprom eeprom;
#define _EEPROMUPDATE_MILLIS 	30000
boolean updateEeprom;

#define SerialTerminal Serial1
#define SerialDebug Serial

//#define WAIT_FOR_SERIALDEBUG

/*
 * Mote Record Structure
 */
#define _DEVICE_PROFILE 41
#include <DS18B20.h>
DS18B20 ds(A0);
struct moterecord {
  float temperature;
  float setpointtemperature;
  uint8_t  heaterpin;
  uint8_t  lastackreceived;
};
typedef struct moterecord Record;
Record moteRec;
char b[sizeof(moteRec)];

#define _PROCESS_TIME_MILLIS 1000
long upTimeSecs;
long lastProcessMillis;
long heatingTimeSecs;

struct ctl_parms {
	int16_t channel;
	int16_t zone;
	double setPoint;
	double idleband;
};

typedef struct ctl_parms Config;
Config cfg;

void bareMetal(byte* b) {
  SerialDebug.println("Bare Metal");
  Config cfg;

  cfg.channel = 63;
  cfg.zone = 100;
  cfg.setPoint = 100.0;
  cfg.idleband = 6.0;

  memcpy(b, &cfg, sizeof(cfg));
}

void initialize(byte* b) {
  SerialDebug.println("initializing...");
  memcpy(&cfg, b, sizeof(cfg));
}

float getTemperature() {
	float tempF;
	while (ds.selectNext()) {
	  tempF = ds.getTempF();
	}
	return tempF;
}

void cmdGetInfo() {
	 char durations[64];
	 long seconds;
	 double heat_pct;
     seconds = upTimeSecs;
     heat_pct=(100.0*heatingTimeSecs)/upTimeSecs;
     SerialTerminal.println(F("\n~~Heat Pad Status~~~~~~~~~~~~~~~~"));
     SerialTerminal.print("upTime   : ");
	 int hrs = seconds/3600;                                                        //Number of seconds in an hour
	 int mins = (seconds-hrs*3600)/60;                                     //Remove the number of hours and calculate the minutes.
	 int secs = seconds-hrs*3600-mins*60;                                //Remove the number of hours and minutes, leaving only seconds.
	 sprintf(durations, "%ih %im %is  [%i%%] ON", hrs, mins, secs, (int)heat_pct);
     SerialTerminal.println(durations);
     SerialTerminal.print("Current temperature "); SerialTerminal.print(heatPadController.getLastTemp()); SerialTerminal.println(" F");
     if (heatPadController.getStatus()== HEAT_ON)
       SerialTerminal.println("The heat pads are ON now.");
     else
       SerialTerminal.println("The heat pads are OFF now.");
     SerialTerminal.println("==================");
}


#define MAX_TEMP 120.0
#define MIN_TEMP 75.0
#define MIN_IDLEBAND 4.0
#define MAX_IDLEBAND 16.0

void cmdSet() {
  float setTemp = atof(term.getNext());
  float idleBand = atof(term.getNext());
  if ((setTemp>=MIN_TEMP)
		  && (setTemp<=MAX_TEMP)
		  && (idleBand>=MIN_IDLEBAND)
		  && (idleBand<=MAX_IDLEBAND)) {
    cfg.setPoint = setTemp;
    cfg.idleband = idleBand;
    heatPadController.set(cfg.setPoint, cfg.idleband);
    SerialTerminal.println("Temperature is set.");
  } else {
	SerialTerminal.println("Temperature could not be set.");
  }
}

void cmdHelp() {
	SerialTerminal.println(F("\n~~HeatBot Commands~~~~~~~~~~~~"));
	SerialTerminal.println(F("  help     Print usage info"));
	SerialTerminal.println(F("  set      <center> <idleband>"));
	SerialTerminal.println(F("  config   Show configuration"));
	SerialTerminal.println(F("  save     Save configuration"));
	SerialTerminal.println(F("  reset    Reset controller"));
	SerialTerminal.println(F("  info     Current status"));
	SerialTerminal.println("==================");
}

void cmdSave() {
	uint8_t size = sizeof(ctl_parms);
	byte b[size];
	memcpy(b, &cfg, size);
	eeprom.saveDeviceConfig(b, EEPROM_CONFIG_ADDRESS,size);
	SerialTerminal.println("Configuration saved.");
}

void cmdReboot() {
	SerialTerminal.println(F("Resetting the hardware..."));
	delay(500);
	NVIC_SystemReset();
}

void cmdShow() {
	float tempF = getTemperature();
	SerialTerminal.println(F("\n~~Heat Pad Configuration~~~~~~~~~"));
	SerialTerminal.print("Set point: "); SerialTerminal.print(cfg.setPoint); SerialTerminal.println(" F");
	SerialTerminal.print("Idleband : "); SerialTerminal.print(cfg.idleband);  SerialTerminal.println(" F");
	SerialTerminal.print("Channel  : "); SerialTerminal.println(cfg.channel);
	SerialTerminal.print("Zone     : "); SerialTerminal.println(cfg.zone);
    SerialTerminal.println("-----------------");
	SerialTerminal.print("On  below "); SerialTerminal.print(cfg.setPoint-cfg.idleband/2); SerialTerminal.println(" F");
	SerialTerminal.print("Off above "); SerialTerminal.print(cfg.setPoint+cfg.idleband/2); SerialTerminal.println(" F");
	SerialTerminal.print("Current T "); SerialTerminal.print(tempF); SerialTerminal.println(" F");
	SerialTerminal.println("==================");
}

#define _WATCHDOG_DONE 12
//#define BARE_METAL_BUILD
#include "project_config/lmic_project_config.h"
#define _FIXED_CHANNEL 10	// 904.3MHz is Ch10 in US915 Bullwinkle & Sherman NanoGateways only!
#define _ACK_PACKETS		// #16 low SNR/RSSI
#define _DONT_SLEEP
#define _DUMP_KEYS
#define _BLINK_MILLIS 2000
#define _SEND_MILLIS 60000
#define _EEPROM_IC2_ADDR 	0x50		//the I2C bus address for the EEPROM
#define _DEVEUI_ADDR 		0xF8		//the location in the EEPROM for the DEVEUI
//special pins for all motes
#define _BATTERY_PIN A7					// divider circuit internally wired here
#define _WATCHDOG_DONE 12				// pulsed once on each LoRa TX_COMPLETE event
#define _LOOP_PULSE_PIN 5				// pulsed once on each iteration of loop()
#define _BUZZER_PIN A3					// used for _LIQUID_LEVEL and _GEOLOCATION sensortypes only
#define _SEND_INTERVAL_PERIODS 12
const unsigned int TX_INTERVAL = _SEND_INTERVAL_PERIODS*5;         	// Schedule the interval(seconds) at which data is sent
static osjob_t sendjob;

//#14 reset the processor when frame count reaches a threshold
unsigned int frameCount;
//#define RESET_ON_FRAMECOUNT
#define _FRAMECOUNT_MAX 5000				// will reset when this count is reached
boolean ackRequested;
boolean ackReceived;


/*
 * LoRa Radio Pin Mappings
 */
const lmic_pinmap lmic_pins = {
  .nss = 8,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 4,
  .dio = {3,6},
}; //.dio = {3,6,11},

RTCZero rtc;

/* EEPROM I2C Device address calculated as follows:
 * 101 0[A2][A1][A0]
 *
 * DEVEUI the EUI-64 address is stored in the EEPROM at 0xF8
 * APPKEY is constructed from the APP_KEY_ROOT with the 3LSB replaced by the 3LSB of the DEVEUI
 *                                            s     e     n     s     e     i     0     0
 */
static const u1_t  	APPKEY_ROOT[16] 	= {0x73, 0x65, 0x6E, 0x73, 0x65, 0x69, 0x30, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static const u1_t 	PROGMEM APPEUI[8]  	= {0x22, 0xf6, 0xec, 0x6b, 0x9a, 0x9a, 0x62, 0xdf};

u1_t devEui[8];
u1_t appKey[16];

void i2c_eeprom_read_buffer( int deviceaddress, unsigned int eeaddress, byte *buffer, int length ) {
    Wire.beginTransmission(deviceaddress);
    Wire.write(eeaddress);
    Wire.endTransmission();
    Wire.requestFrom(deviceaddress,length);
    int c = 0;
    for ( c = 0; c < length; c++ )
       if (Wire.available())
         buffer[c] = Wire.read();
    Wire.endTransmission();
}

// These methods are called by the LMIC code after the JOIN
void os_getArtEui (u1_t* buf) {
  memcpy_P(buf, APPEUI, 8);
}

void os_getDevEui (u1_t* buf) {
  memcpy_P(buf, devEui, 8);
}

void os_getDevKey (u1_t* buf) {
  memcpy_P(buf, appKey, 16);
}

/*
 * do_send() is called periodically to send sensor data
 */
void do_send(osjob_t* j) {
	/*
	if ( (millis() - lastProcessMillis) > _PROCESS_TIME_MILLIS ) {
	  float tempF = getTemperature();
	  heatPadController.poll(tempF);
	  upTimeSecs +=  (millis() - lastProcessMillis)/1000;
	  if  (heatPadController.getStatus()== HEAT_ON)
	    heatingTimeSecs +=  (millis() - lastProcessMillis)/1000;
	  lastProcessMillis = millis();
	}
	*/

    if (LMIC.opmode & OP_TXRXPEND) {  // Check if there is a current TX/RX job running

    } else {
      while (ds.selectNext()) {
        SerialDebug.print("DS18B20 ");
        //SerialDebug.println(ds.getTempF());  //32.0 + 9.0 * (temperature/_BME680_AVERAGING_SAMPLES)/5.0;
        moteRec.temperature = ds.getTempF();
      }
      SerialDebug.print("T="); SerialDebug.println(moteRec.temperature);

      moteRec.heaterpin = heatPadController.getStatus();
      moteRec.setpointtemperature = heatPadController.getSetPoint();

      if (ackReceived)
        moteRec.lastackreceived = (uint8_t) 1;
      else
        moteRec.lastackreceived = (uint8_t) 0;

      memcpy(b, &moteRec, sizeof(moteRec));

      #ifdef _ACK_PACKETS
      if ( ((frameCount%5)==1) ||  ((frameCount%5)==3) ) {
        LMIC_setTxData2(1, (byte*) b, sizeof(moteRec), 1);	// queue the packet, ACK
        ackRequested = true;
        ackReceived = false;
      } else {
        LMIC_setTxData2(1, (byte*) b, sizeof(moteRec), 0);	// queue the packet, NO ACK
        ackRequested = false;
        ackReceived = false;
      }
      #else
      LMIC_setTxData2(1, (byte*) b, sizeof(moteRec), 0);	// queue the packet, no ACK
      ackRequested = false;
      ackReceived = false;
      #endif

      #ifdef _DEBUG_SERIAL
      const byte * p = (const byte*) &moteRec;
      for (uint8_t n=0; n<sizeof(moteRec); n++) {
        SerialDebug.print((byte) p[n], HEX); SerialDebug.print(" ");
      }

      if (ackRequested)
        SerialDebug.println(F("Confirmed packet queued for transmission."));
      else
        SerialDebug.println(F("Unconfirmed packet queued for transmission."));
      #endif
    }
}

void sitUbu() {
  #if not defined(_TEST_WATCHDOG)
  digitalWrite(_WATCHDOG_DONE, HIGH);				// rising edge pulse on DONE to keep the watchdog happy!
  delay(2);											// pet the dog before going to sleep.
  digitalWrite(_WATCHDOG_DONE, LOW);
  #endif
}

/* Issue #2
 * alarmMatch() is invoked when the rtcZero clock alarms.
 */
void alarmMatch()  {
  if (frameCount>=_FRAMECOUNT_MAX) {
#if defined(RESET_ON_FRAMECOUNT)
	NVIC_SystemReset();      // processor software reset, the scheduled packet will be delayed slightly.
#endif
  } else {
	//the following call attempts to avoid collisions with other motes by randomizing the send time
	os_setTimedCallback(&sendjob, os_getTime()+ms2osticks(random(2000)), do_send);
	frameCount++;
  }
}

void goToSleep() {
  rtc.setAlarmEpoch( rtc.getEpoch() + TX_INTERVAL);    // Sleep for a period of TX_INTERVAL using single shot alarm
  rtc.enableAlarm(rtc.MATCH_YYMMDDHHMMSS);
  rtc.attachInterrupt(alarmMatch);
  delay(10);
  /*
   * ZZZZZzzzzzzz (we're sleeping right here!)
   */
  #if not defined (_DONT_SLEEP)
    rtc.standbyMode();                                  // Enter sleep mode
  #endif
}

void onEvent (ev_t ev) {
  #ifdef _DEBUG_SERIAL
  SerialDebug.print(os_getTime());
  SerialDebug.print(": ["); SerialDebug.print(ev); SerialDebug.print("] ");
  #endif
  switch(ev) {
    case EV_SCAN_TIMEOUT:
      #ifdef _DEBUG_SERIAL
      SerialDebug.println(F("EV_SCAN_TIMEOUT"));
      #endif
      break;
    case EV_BEACON_FOUND:
      #ifdef _DEBUG_SERIAL
      SerialDebug.println(F("EV_BEACON_FOUND"));
      #endif
      break;
    case EV_BEACON_MISSED:
      #ifdef _DEBUG_SERIAL
      SerialDebug.println(F("EV_BEACON_MISSED"));
      #endif
      break;
    case EV_BEACON_TRACKED:
      #ifdef _DEBUG_SERIAL
      SerialDebug.println(F("EV_BEACON_TRACKED"));
      #endif
      break;
    case EV_JOINING:
      #ifdef _DEBUG_SERIAL
      SerialDebug.println(F("EV_JOINING"));
      #endif
      break;
    case EV_JOINED: {
      u4_t netid = 0;
      devaddr_t devaddr = 0;
      u1_t nwkKey[16];
      u1_t artKey[16];
      LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
      #ifdef _DEBUG_SERIAL
      SerialDebug.println(F("EV_JOINED"));
      SerialDebug.print("netid: ");
      SerialDebug.println(netid, DEC);
      SerialDebug.print("devaddr: "); // @suppress("Method cannot be resolved")
      SerialDebug.println(devaddr, HEX);
      SerialDebug.print("artKey: ");
      for (int i=0; i<sizeof(artKey); ++i) {
        if (i != 0)
          SerialDebug.print("-");
        SerialDebug.print(artKey[i], HEX);
      }
      SerialDebug.println("");
      SerialDebug.print("nwkKey: ");
      for (int i=0; i<sizeof(nwkKey); ++i) {
        if (i != 0)
          SerialDebug.print("-");
        SerialDebug.print(nwkKey[i], HEX);
      }
      SerialDebug.println("");
      #endif
      LMIC_setLinkCheckMode(0);
      break;
    }
    case EV_JOIN_FAILED:
      #ifdef _DEBUG_SERIAL
      SerialDebug.println(F("EV_JOIN_FAILED"));
      #endif
      goToSleep();
      break;
    case EV_REJOIN_FAILED:
      #ifdef _DEBUG_SERIAL
      SerialDebug.println(F("EV_REJOIN_FAILED"));
      #endif
      goToSleep();
      break;
    case EV_TXCOMPLETE:
      digitalWrite(LED_BUILTIN, LOW);     // LED Off
      #ifdef _DEBUG_SERIAL
      SerialDebug.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      #endif
      if ((ackRequested) && (LMIC.txrxFlags & TXRX_ACK)) {
        // if confirmation was requested and is received - cycle the WATCHDOG bit
        ackReceived = true;
        sitUbu();
        SerialDebug.println(F("Confirmed - Ok!"));
      }
      // Check if we have a downlink on either Rx1 or Rx2 windows
      if ((LMIC.txrxFlags & ( TXRX_DNW1 | TXRX_DNW2 )) != 0) {
      	if ((LMIC.txrxFlags & TXRX_DNW1) != 0)
       	  SerialDebug.print(F("Downlink on Rx1-"));
       	else
       	  SerialDebug.print(F("Downlink on Rx2-"));

       	if (LMIC.dataLen) {
       	  SerialDebug.print(LMIC.dataLen);
       	  SerialDebug.println(F(" bytes:"));
       	  // Receive bytes in LMIC.frame are the B64 decode of the
       	  // 'data' field sent by ChirpStack
       	  for (int i = 0; i < LMIC.dataLen; i++) {
       	     SerialDebug.print(" 0x"); SerialDebug.print(LMIC.frame[LMIC.dataBeg + i], HEX);
       	  }
       	  SerialDebug.println();
       	} else {
          SerialDebug.println();
       	}
      }


      #if not defined (_DONT_SLEEP)
      SerialDebug.flush();                                     // Ensure all debugging messages are sent before sleep
      USBDevice.detach();                                 // USB port consumes extra current
      #endif
      goToSleep();
      #if not defined (_DONT_SLEEP)
      USBDevice.init();                                   // Reinitialize USB for debugging
      USBDevice.attach();
      #endif
      break;
    case EV_LOST_TSYNC:
      SerialDebug.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      SerialDebug.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      SerialDebug.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      //no confirmation has been received for an extended perion
      SerialDebug.println(F("EV_LINK_DEAD"));
      goToSleep();
      break;
    case EV_LINK_ALIVE:
      SerialDebug.println(F("EV_LINK_ALIVE"));
      break;
    case EV_TXSTART:
      SerialDebug.println(F("EV_TXSTART"));
      //digitalWrite(LED_BUILTIN, LOW);
      break;
    default:
      SerialDebug.print(F("Unknown event: "));
      SerialDebug.println((unsigned) ev);
      break;
  }
}

void unknownCommand(const char *command) {
    SerialTerminal.print(F("Unknown command: "));
    SerialTerminal.println(command);
}

void setup() {
  // Resets occurring ~immediately after a successful SEND #10, is this the Brown Out Detector firing?
  // Configure the regulator to run in normal mode when in standby mode
  // Otherwise it defaults to low power mode and can only supply 50 uA
  //SYSCTRL->VREG.bit.RUNSTDBY = 1;

  pinMode(_WATCHDOG_DONE, OUTPUT);					 // Watchdog Done pin
  digitalWrite(_WATCHDOG_DONE, LOW);

  SerialDebug.begin(115200);

#if defined (WAIT_FOR_SERIALDEBUG)
  while(!SerialDebug) {

  }
#endif

  SerialDebug.println(F("Starting"));
  SerialDebug.println("Reading EEPROM...");
  Wire.begin();
  Wire.setClock(50000UL);			// lowering the clock frequency to see if reliability improves
  uint8_t size = sizeof(ctl_parms);
  byte b[size];
#if defined(BARE_METAL_BUILD)
  bareMetal(b);
  //SerialDebug.println("Bare metal is configured.");
  eeprom.saveDeviceConfig(b, EEPROM_CONFIG_ADDRESS, size);
  //SerialDebug.println("Bare metal config saved to EEPROM");
#endif
  eeprom.loadDeviceConfig(b, EEPROM_CONFIG_ADDRESS, size);
  initialize(b);
  heatPadController.set(cfg.setPoint, cfg.idleband);

  //get the DEVEUI from the EEPROM
  i2c_eeprom_read_buffer(_EEPROM_IC2_ADDR, _DEVEUI_ADDR, devEui, 8);
  memcpy_P(appKey, APPKEY_ROOT, 16);
  //modify the appKey
  for (uint8_t n=8; n<16; n++) {
    appKey[n] = devEui[n-8];
  }
  #if defined(_DUMP_KEYS)
  delay(1000);
  SerialDebug.println("EEPROM read complete.");
  char hexdigit[2];
  SerialDebug.print("coreId (DB): ");
  for(int n=0; n<8; n++) {
    sprintf(hexdigit,"%02x",devEui[n]);
    SerialDebug.print(hexdigit);
  }
  SerialDebug.print("\n\rdevEui (Chirpstack format): ");
  for(int n=7; n>=0; n--) {
    sprintf(hexdigit,"%02x",devEui[n]);
    SerialDebug.print(hexdigit);
  }
  SerialDebug.print("\n\rappKey: ");
  for(uint8_t n=0; n<16; n++) {
    sprintf(hexdigit,"%02x",appKey[n]);
    SerialDebug.print(hexdigit);
  }
  SerialDebug.println();
  #endif
  rtc.begin();                                        // Initialize RTC
  rtc.setEpoch(0);                                    // Use RTC as a second timer instead of calendar
  os_init();                                               // LMIC init
  LMIC_reset();                                            // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_setClockError(MAX_CLOCK_ERROR * 10 / 100);

  #if defined(_FIXED_CHANNEL)
  for(uint8_t c=0; c<72; c++) {
  	if (c!=_FIXED_CHANNEL)
      LMIC_disableChannel(c);
  	else
  	  LMIC_enableChannel(c);
  }
  #else
  LMIC_selectSubBand(1);
  #endif

  LMIC_setLinkCheckMode(0);
  LMIC_setDrTxpow(DR_SF7,14);

  #ifdef _DEBUG_SERIAL
  SerialDebug.println("Setup completed.");
  #endif

  randomSeed(analogRead(1));	//seed the random number generator
  frameCount=0;
  SerialTerminal.begin(9600);
  term.addCommand("help", cmdHelp);
  term.addCommand("set", cmdSet);
  term.addCommand("save", cmdSave);
  term.addCommand("show", cmdShow);
  term.addCommand("reset", cmdReboot);
  term.addCommand("info", cmdGetInfo);
  cmdHelp();
  do_send(&sendjob);
}

void loop() {
  os_runloop_once();
  term.readSerial();
  sitUbu();
  if ( (millis() - lastProcessMillis) > _PROCESS_TIME_MILLIS ) {
  	  float tempF = getTemperature();
  	  heatPadController.poll(tempF);
  	  upTimeSecs +=  (millis() - lastProcessMillis)/1000;
  	  if  (heatPadController.getStatus()== HEAT_ON)
  	    heatingTimeSecs +=  (millis() - lastProcessMillis)/1000;
  	  lastProcessMillis = millis();
   }
}
