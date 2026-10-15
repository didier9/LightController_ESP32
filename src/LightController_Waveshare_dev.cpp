/*
  LightController using Waveshare ESP32S3 6 channel relay board (optional 128x64 I2C OLED display)
  and RS485/SABus interface to remote switch modules.
  Didier Juges (c) 2025
  arduino-mqtt library by Joël Gähwiler:
  https://github.com/256dpi/arduino-mqtt
  NTPClient library for Arduino:
  https://github.com/arduino-libraries/NTPClient
  Preferences library by Espressif:
  https://docs.espressif.com/projects/arduino-esp32/en/latest/tutorials/preferences.html
  FastLED library for WS2812B (and other) RGB LEDS:
  https://github.com/FastLED/FastLED

  Use "ESP32S3 Dev Module"
  DO NOT USE "ESP32 xxx"
  Tool settings: CDC On Boot: enabled (necessary for serial debug)

  Check TEST_MODE in general.h 

  RGB LED:
    - blinks green: normal (WiFi and MQTT connected, RS485 bus active)
    - blinks red: indicates POB Fault. There may be more than one fault:
        - Blinks once: no WiFi
        - blinks twice: no MQTT
        - blinks 3 times: no RS485
  TXD LED: indicates Light Controller is querying remote switches
  RXD LED: indicates Light Controller is receiving data from switch(es)

  IMPORTANT for ESP32-S3: 
  1) to upload the sketch via USB, you need to press and hold BOOT, press and releasing RESET, then release BOOT to place the chip in bootloader mode
  2) to use Serial Monitor, enable Tools->USB CDC On Boot and pay attention that the IDE will switch to a different COM port when the chip boots.
  When USB CDC On Boot is enabled, you need to switch the COM port back to the original value after resetting the chip as above, before uploading your sketch again.
*/

// version history
// v001   HW724_SABus_OLED 20Dec24 128x64 OLED works 
// v002   24Dec24 SABus remote switch over UART2 (GPIO13, GPIO15) works
// v003   SABus Master on UART2 works with Remote Switch!
// v004   Reads switch activated on Remote Switch and changes output accordingly
//        Status LED on Remote Switch updated
// v001   Changed name to HW724_LightController
//        SABus, OLED, WiFi and generic MQTT work.
// v002   MQTT sends VersionMsg[] and OutputStatus
// v003   Removed interference between LIGHT outputs and OLED
// v004   NTP working 28Dec2024
// v005   Sending commands via mqtt works
//        Implemented offLineCounter to speedup scanning when not all SABus addresses are used
// v006   Cleaned up OLED display code
//        mqtt publish issue fixed, was wrong handling of mqttClient.loop() versus mqttClient.connected()
//        rs485 tx_enable works (not interrupt driven)
// v007   this build for the ESP32_Relay_X8 board (no OLED but it does not seem to interfere)
//        To flash ESP32_Relay_X8 board, press EN while holding IO0 down, then release IO0
//        Bit polarity in OutputStatus changed so that '1' means a switch is ON
//        Changed project name to ESP32_Relay_LightController
// v008   Can reprogram switches via mqtt command S<ndx>,<output>
//        Added blinkPobFault() driving builtin LED.
// v009   Save OutputStatus to EEPROM works
// v010   change channel assignments when receiving a channel change request from remote
// v011   implement master switch, channel 9
// v012   For Waveshare ESP32-S3-Relay-6CH - 
//        1) Use board "ESP32S3 Dev Module"
//        2) Replaced EEPROM with Preferences and LED_BUILTIN with FastLED WS2812B (RGB LED)
//        3) Adjusted pinout to match Waveshare
// v013   Blinks POBFault on RGB LED and buzzer
// v014   Removed hardware TX_ENABLE since Waveshare does not use it.
//        Removed OLED
//        Tweaked MQTT reconnect sequence, was very unstable
// v015   moved mqtt business to mqtt_if.cpp, imported from OTA_Blinky_Mqtt project
//        Released 1Mar2025
// v016   Add TEST_MODE
// v017   Add OTA 
//        "C<ndx>,<0|1>" now wants ndx in the range 1-NUM_OUTPUTSinstead of 0-(NUM_OUTPUTS-1)
//        fixed a few places where NUM_OUTPUTS changing from 8 to 6 was not properly accounted for
// v018   Added watchdog timer, cleaned up POBFault display on RGB LED
// v019   Added TEST_PIN switch on pin 8 to replace TEST_MODE, added MAC address to ID
// v020   For PlatformIO
// v021   Fixed MQTT reconnect blocking RS485 polling for minutes when the broker was
//        unreachable: mqttConnect() no longer loops with delay(1000) internally (single
//        bounded attempt via net.setTimeout(MQTT_CONNECT_TIMEOUT)), and mqttDo() now
//        throttles reconnect attempts to once per MQTT_RECONNECT_INTERVAL instead of
//        retrying on every loop() pass.
char VersionMsg[] = "LightController v021";

// MQTT commands accepted on MQTT_CMD_TOPIC ("lightcontroller/cmd", see mqtt_if.h)
// Payload is a command letter followed by optional comma-separated data. Handled in
// mqttProcessCommand() (mqtt_if.cpp). Replies/publishes go to the topic noted for each.
//
//  0                  Send controller ID -> mqtt_id_topic ("lightcontroller/id"): "<VersionMsg> <mac>"
//  1                  Send controller output status -> mqtt_status_topic: "on "/"off " per output
//  B0 | B1            Turn buzzer off/on
//  C<ndx>,<cmd>       Control channel <ndx> (0..NUM_OUTPUTS-1) on/off (<cmd> = 0|1)
//                     e.g. "C0,1" turns on output #1
//  D<n>               Set debug mode (<n> < NUM_DEBUG_MODES: 0=off, 1=RXDATA)
//  I<n>               Set mqtt status publish interval to <n> mS (<n> > 10000) and
//                     force an immediate status publish
//  R                  Query remote switch online status -> mqtt_rs485_status_topic:
//                     "<devNdx>:<onlineCount>, " for each switch currently online
//  R<ndx>,<cmd>       Control relay <ndx> (1..NUM_OUTPUTS) on/off (<cmd> = 0|1)
//                     e.g. "R1,1" turns on relay #1 (1-based, unlike "C")
//  S                  Send current switch-to-output assignments -> mqtt_setting_topic:
//                     comma-separated menu_val[] for all 2*NUM_REMOTE_DEVS switch inputs
//  S<ndx>,<val>       Assign switch <ndx> (0..2*NUM_REMOTE_DEVS-1) to output <val>
//                     (0 = unassigned, 1..NUM_OUTPUTS = output number); saved to Preferences
//  S<ndx>,?           Query current assignment for switch <ndx> -> mqtt_setting_topic:
//                     "S<ndx>,<val>"
//  *                  Factory reset: restore menu_val[] to defaults

#include "general.h"
#include <NTPClient.h>    // v3.2.1 https://github.com/arduino-libraries/NTPClient, this must be called before WiFi.h
#include <Wire.h>
#include <WiFi.h>         // v1.2.7 by Arduino, includes Networking library
#include <ESPmDNS.h>      // version 2.0.0 , part of WiFi_Kit_series library from Heltec https://github.com/HelTecAutomation/Heltec_ESP32
//#include <NetworkUdp.h>   // v1.0.0, part of Network library by Hristo Gochkov <hristo@espressif.com>
#include <ArduinoOTA.h>   // version 2.0.0 by Hristo Gochkov <hristo@espressif.com>
#include <MQTT.h>         // v2.5.2 by Joel Gaehwiler https://github.com/256dpi/arduino-mqtt (max packet size: 128 bytes)
#include <WiFiUdp.h>      // for NTP, part of WiFi library
#include <Preferences.h>  // v2.1.0 https://github.com/vshymanskyy/Preferences (vshymanskyy/Preferences@^2.2.2)
#include <FastLED.h>      // for WS2812B, v3.9.13 https://github.com/FastLED/FastLED
#include <HardwareSerial.h>

#include "general.h"
#include "mqtt_if.h"

/* ----- wifi access settings ----- */
const char* ssid     = "KO4BB-1";
const char* password = "All I want is good wireless.";

/* ----- OTA stuff ----- */
#define ARDUINO_OTA_HOSTNAME    "lightcontroller_ota" // will be preceeded by "test_" if the test switch is closed (TEST_MODE)
#define ARDUINO_OTA_PW          "ota1234"

/* ----- WiFi stuff ----- */
WiFiClient net;

/* ----- MQTT broker access settings ---- */
#define MQTT_HOST       IPAddress( 192, 168, 0, 92 )
#define MQTT_PORT       1883
#define MQTT_USER       "mqtt"
#define MQTT_PASSWORD   "123mqtt456"

/* ----- MQTT ----- */
MQTTClient mqttClient;

/* ----- NTP stuff ----- */
#define TIME_SERVER             "192.168.0.92"
#define TIME_OFFSET_SEC         0 // offset for time zone of time server. Set to 0 if using Home Assistant Chrony addon
#define TIME_UPDATE_INTERVAL    60000 // mS
WiFiUDP ntpUDP;
NTPClient timeClient( ntpUDP, TIME_SERVER, TIME_OFFSET_SEC, TIME_UPDATE_INTERVAL );
// for pool time server, use NTPClient timeClient( ntpUDP );
#define NTP_TIME_INTERVAL       60000 // print time every NTP_TIME_INTERVAL/1000 seconds
long lastNTPTime;
String ntpTime;
const int timeZone = -6;

/* ----- to save settings ----- */
Preferences preferences;

/* ----- POB Faults ----- */
#define POB_FAULT_NO_WIFI           1   // blink once
#define POB_FAULT_NO_MQTT           2   // blink twice
#define POB_FAULT_NO_RS485          4   // blink three times
#define BLINK_POB_INTERVAL_DEFAULT 200
int pobFault = 0;
long blinkPobLast;
int blinkPobInterval;

// the built-in RGB LED
#define NUM_LEDS          1     // How many leds in your strip?
#define LED_BRIGHTNESS    64
#define LED_INTERVAL      1000
#define BLINK_HAPPY_INTERVAL_DEFAULT 800

CRGB leds[NUM_LEDS];
long blinkHappyLast;
int blinkHappyInterval;

uint mac=0;

#define BUZZER_ON_DEFAULT false   // when true, buzzer is enabled
bool buzzerOn = BUZZER_ON_DEFAULT;

// the timer to reboot after a certain time with a faut
long rebootTimer;
#define REBOOT_TIMER_INTERVAL   125000 // reboot when POB Fault for 2 minutes 5 seconds

#define MODULE            "ESP32S3 Dev Module"

/* ----- proper use of millis() ----- */
/*  if( millis() - previousMillis >= interval ){
      // Update previousMillis to the current time for the next interval
      previousMillis = millis(); 
      // Time has elapsed, perform your action here
      // ...       
    }
*/

/* ----- prototypes ----- */
void otaInit( char *hostname );
void printMenuSettings();
void rs485Init();
void rs485Do(); 
void lightCntl_Init();
void blinkPobFault( void );
void blinkHappyLED( void );
void ProcessSabusRxChar( int tchar );
void ProcessSabusMsg( char *rxBuf );


/*---------------------------------------------------------------------------
* readMacAddress( )
*
* returns the last 16 bits of the MAC address as a uint
*---------------------------------------------------------------------------*/
uint readMacAddress( void ){
  uint8_t baseMac[6];
  //esp_err_t ret = esp_wifi_get_mac( WIFI_IF_STA, baseMac );
  WiFi.macAddress( baseMac);
  Serial.printf( "MAC: %02x:%02x:%02x:%02x:%02x:%02x\n",
                baseMac[0], baseMac[1], baseMac[2],
                baseMac[3], baseMac[4], baseMac[5] );
  return( ( uint )baseMac[4]*256 + baseMac[5] );
}// readMacAddress( )

/*---------------------------------------------------------------------------*/
/* setup()
*----------------------------------------------------------------------------*/
void setup(){
  char buf[80];
  char hostname[40];

	// join i2c bus (address optional for master). Used by OLED
	Wire.begin( SDA, SCL );
	// initialize both serial ports (Serial used for debugging, SABus used for RS485 communication with remote switches)
  Serial.begin( 115200 );   // USB
  Serial.println( "\n" );
  delay( 2000 );

  rs485Init();

  pinMode( BUZZER_OUT_PIN, OUTPUT );
  digitalWrite( BUZZER_OUT_PIN, 0 );

  lightCntl_Init();

  // check that preferences have been inited
  // RW-mode (second parameter has to be false).
  preferences.begin( "lightctrl", false );
  uchar d=0;
  if( preferences.isKey( "lightctrl" ))
      d = preferences.getUChar( "id", 0 );
  if( d == EEPROM_INIT_VAL ){
    // Preferences have been initted, restore settings
    preferences.getBytes( "menu_val", (byte *)(&menu_val), sizeof( menu_val ));
    buzzerOn = (bool)preferences.getUChar( "buzzerOn", 0 ); 
  }else{
    // Preferences not initted on this board, set sensible settings for a new board
    preferences.putUChar( "id", EEPROM_INIT_VAL );
    for( int i = 0; i < NUM_REMOTE_DEVS; i++ ){
      menu_val[i] = menu_val_default[i];
    }
    preferences.putUChar( "buzzerOn", (uchar)buzzerOn );
    SettingsSave();    
    Serial.println( "Preferences Inited" );
  }

  WiFi.begin( ssid, password );
  mac = readMacAddress();
  Serial.printf( "%s %0X\n", VersionMsg, mac );

  Serial.print( "WiFi.begin: " );
  delay( 50 );
  pobFault |= POB_FAULT_NO_WIFI;
  pobFault |= POB_FAULT_NO_MQTT;
  for( int i=0; i < 20; i++ ){
    if( WiFi.status() == WL_CONNECTED ){
      pobFault &= ~POB_FAULT_NO_WIFI;
      Serial.println( "\nWiFi.status() = WL_CONNECTED" );
      net.setTimeout( MQTT_CONNECT_TIMEOUT ); // bound how long a single MQTT (re)connect attempt can block loop()/RS485 polling
      mqttClient.begin( MQTT_HOST, MQTT_PORT, net);
      if( mqttInit())
        pobFault &= ~POB_FAULT_NO_MQTT; // clear the pobFault bit
      break;
    }
    Serial.print( "." );
    delay( 1000 );
  }
  Serial.printf( "pobFault = %X\n", (uint)pobFault );
  timeClient.begin();
  timeClient.setTimeOffset( timeZone * 3600 );
  timeClient.update();

  lastCmdMillis = millis() - MQTT_STATUS_INTERVAL;
  lastNTPTime = millis() - NTP_TIME_INTERVAL;

  FastLED.addLeds<WS2812B, LED_DATA_PIN, RGB>( leds, NUM_LEDS );  // GRB ordering is typical
  FastLED.setBrightness( LED_BRIGHTNESS );

  sprintf( hostname, "%s_%0X", ARDUINO_OTA_HOSTNAME, mac );
  if( digitalRead( TEST_PIN ) == 0 )
    strcat( hostname, "_test" );
  otaInit( hostname );

  // initialize the LED blinking business
  blinkPobInterval = BLINK_POB_INTERVAL_DEFAULT;  // some small value
  blinkPobLast = millis() + blinkPobInterval;
  blinkHappyInterval = BLINK_HAPPY_INTERVAL_DEFAULT;
  blinkHappyLast = millis() + blinkHappyLast;

  enableLoopWDT();

  printMenuSettings();

  rebootTimer = millis() + REBOOT_TIMER_INTERVAL;
}   // setup()

/*---------------------------------------------------------------------------*/
/* loop()
*----------------------------------------------------------------------------*/
void loop(){
  char buf[BUF_SIZE];  // buffer used to transmit and receive
  char *topic, *payload;
  const char s[2] = ",";
  int static lastPOBFault=0;

  ArduinoOTA.handle();

  rs485Do();

  if( pobFault == 0 ){
    rebootTimer = millis();
  }else{
    if( millis() - rebootTimer > REBOOT_TIMER_INTERVAL ){
      //rebootTimer = millis();
      Serial.println( "Rebooting..." );
      delay( 1000 );
      ESP.restart();
    }
  }

  // ========================================================================
  // update current output status (used by MQTT and Serial debug)
  if( lastOutputStatus != (int)OutputStatus ){
    int i;
    int out[9];
    OutputStatusStr[0] = 0;
    for( i=0; i<NUM_OUTPUTS; i++ ){
        out[i] = ((OutputStatus >> i) & 0x01) + '0';
        if( ((OutputStatus >> i) & 0x01) == 0 )
          strcat( OutputStatusStr, "off " );
        else
          strcat( OutputStatusStr, "on " );
    }
    out[i] = '\0';
    Serial.printf( "OutputStatus: %s\n", OutputStatusStr );
    if( lastOutputStatus != (int)OutputStatus ){
      // status has changed, force sending new status now
      lastMqttStatus = millis() -  mqttStatusInterval - 1; 
    }
    lastOutputStatus = (int)OutputStatus;
  }

  mqttDo();

  // ========================================================================
  // NTP stuff
  // get local time 
  if( (millis() - lastNTPTime) >= NTP_TIME_INTERVAL || pobFault != lastPOBFault ){
    lastPOBFault = pobFault;
    lastNTPTime = millis();
    Serial.printf( "%s %0X\n", VersionMsg, mac );
    timeClient.update();
    ntpTime = timeClient.getFormattedTime();
    Serial.println( ntpTime );
    char buf[40];
    sprintf( buf, "%s %0X", VersionMsg, mac );
    Serial.printf( "%s\n", buf );
    mqttClient.publish( mqtt_id_topic, (const char *)buf );
    if( pobFault > 0 )
      Serial.printf( "POBFault: %0X\n", (uint)pobFault );
  }

  // ========================================================================
  // LED and pobFault stuff
  if( pobFault != 0 ){
    blinkPobFault();
  }else{
    blinkHappyLED();
  }
} // loop()

/*----------------------------------------------------------------------------*/
/* FUNCTION: blinkHappyLED()
* blink green led to indicate all good
*-----------------------------------------------------------------------------*/
void blinkHappyLED(){
  static int color = 0;
  if( (millis() - blinkHappyLast) >= blinkHappyInterval ){
    //Serial.printf( "no pob fault %d, time to blink green LED\n", pobFault );
    blinkHappyLast = millis();
    if( color == 0 ){
      leds[0] = CRGB::Green;
      color = 1;
    }else{
      //leds[0] = CRGB::Blue;
      leds[0] = CRGB::Black;
      //FastLED.clear( true );
      color = 0;
    }
    FastLED.show();
  }
} // blinkHappyLED()

/*----------------------------------------------------------------------------*/
/* FUNCTION: blinkPobFault()
* blink red led to indicate pobFault (see POB_FAULT_DEFINITIONS in general.h)
*-----------------------------------------------------------------------------*/
void blinkPobFault( void ){
  #define START	0
  #define ONE		1
  #define TWO		2
	static int i = 0;
  static int state = START;
	static int j = 0;
	int delay = 0;
  long nowTime = millis();

	if( (nowTime - blinkPobLast) >= blinkPobInterval ){
		if( pobFault & 1<<i ){
			switch( state ){
				case START:
					j = i;
          leds[0] = CRGB::Red;
          FastLED.show();
					delay = 300;
          if( buzzerOn )
            tone( BUZZER_OUT_PIN, 1000, delay );
					state = ONE;
					break;
				case ONE:
          leds[0] = CRGB::Black;
          FastLED.show();
					delay = 300;
          if( buzzerOn )
            tone( BUZZER_OUT_PIN, 0, 0 );
					state = TWO;
					break;
				case TWO:
					j--;
					if( j < 0 ){
						// make delay medium and get ready for next bit
						delay = 550;
						i++;
						state = START;
					}else{
            leds[0] = CRGB::Red;
            FastLED.show();
						delay = 300;
            if( buzzerOn )
              tone( BUZZER_OUT_PIN, 1000, delay );
						state = ONE;
					}
					break;
			}
		}else{
			// check next bit
			do{
				i++;
			}while( ! (pobFault & 1<<i) && i < sizeof( pobFault ) * 8 );
			if( i == (sizeof( pobFault ) * 8) ){
				// no more bits set, make delay very long and start over
				i = 0; 
				delay = 800;
			}else
				delay = 550;
			state = START;
      //Serial.printf( "pobFault = %X\n", (uint)pobFault );
		}
    blinkPobInterval = delay;
    blinkPobLast = nowTime;
    //Serial.printf( "state: %d, i: %d, j: %d, delay: %d\n", state, i, j, delay );
	}
} // blinkPobFault()

/*---------------------------------------------------------------------------*/
/* otaInit()
*
*----------------------------------------------------------------------------*/
void otaInit( char *hostname ){

  // OTA hostname defaults to esp3232-[MAC]
  ArduinoOTA.setHostname( hostname );

  // No OTA authentication by commenting out this line
  ArduinoOTA.setPassword( ARDUINO_OTA_PW );

  // Password can be set with its md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  // set OTA callbacks
  ArduinoOTA
    .onStart([]() {
      String type;
      if( ArduinoOTA.getCommand() == U_FLASH ){
        type = "sketch";
      }else{  // U_SPIFFS
        type = "filesystem";
      }

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println( "Start updating " + type );
    })
    .onEnd([]() {
      Serial.println( "\nEnd" );
    })
    .onProgress([]( unsigned int progress, unsigned int total ){
      Serial.printf( "Progress: %u%%\n", (progress / (total / 100)) );
    })
    .onError([]( ota_error_t error ){
      Serial.printf( "Error[%u]: ", error );
      if( error == OTA_AUTH_ERROR ){
        Serial.println( "Auth Failed" );
      }else if( error == OTA_BEGIN_ERROR ){
        Serial.println( "Begin Failed" );
      }else if( error == OTA_CONNECT_ERROR ){
        Serial.println( "Connect Failed" );
      }else if( error == OTA_RECEIVE_ERROR ){
        Serial.println( "Receive Failed" );
      }else if( error == OTA_END_ERROR ){
        Serial.println( "End Failed" );
      }
    });

  ArduinoOTA.begin();
  Serial.println( "Ready" );
  Serial.print( "IP address: " );
  Serial.println( WiFi.localIP() );
  Serial.printf( "Starting ArduinoOTA %s\n", hostname );
} // otaInit()

/*---------------------------------------------------------------------------*/
/* SettingsSave()
*----------------------------------------------------------------------------*/
void SettingsSave( void ){
  preferences.putBytes( "menu_val", (byte *)(&menu_val), sizeof( menu_val ));
  preferences.putUChar( "buzzerOn", (uchar)buzzerOn );
}   // SettingsSave()


