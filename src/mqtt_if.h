/*===========================================================================
* mqtt_if.h
*
* MQTT Application layer for the ESP family of microcontrollers
*===========================================================================*/
#ifndef _MQTT_IF_H_
#define _MQTT_IF_H_

#include "general.h"
#include <WiFi.h>
#include <MQTT.h>


enum MQTT_STATE {
  MQTT_DISCONNECTED = 0,
  MQTT_READY_TO_CONNECT,
  MQTT_CONNECTING,
  MQTT_CONNECTED
};

/* ----- MQTT broker access settings ---- */
#define MQTT_HOST       IPAddress( 192, 168, 0, 92 )
#define MQTT_PORT       1883
#define MQTT_USER       "mqtt"
#define MQTT_PASSWORD   "123mqtt456"
#define MQTT_CONNECT_TIMEOUT    2    // seconds. Bounds how long a single reconnect attempt can block the main loop (and RS485 polling with it)
#define MQTT_RECONNECT_INTERVAL 5000UL // mS between reconnect attempts while disconnected, so a down broker doesn't stall the loop every pass

/* ----- MQTT topics ----- */
#define MQTT_STATUS_INTERVAL  60000UL    // Number of milliseconds between status messages intended to keep the MQTT port open, set to 0 to disable.
// all topics will be preceeded by MTT_TEST_TOPIC if the test switch is closed (TEST_MODE)
#define MQTT_ID_TOPIC              "lightcontroller/id"
#define MQTT_CMD_TOPIC             "lightcontroller/cmd"
#define MQTT_STATUS_TOPIC          "lightcontroller/status"
#define MQTT_SETTING_TOPIC         "lightcontroller/setting"
#define MQTT_RS485_STATUS_TOPIC    "lightcontroller/rs485_status"
#define MQTT_RXDATA_TOPIC          "lightcontroller/rxdata"
#define MQTT_TEST_TOPIC            "test/"
// other MQTT stuff
#define BUF_SIZE                    255       // size of Tx/Rx Buffer. Make big enough for the longest message
#define MAX_TOPIC_LEN               127
#define MAX_PAYLOAD_LEN             127

enum DEBUG_MODES {
  DEBUG_OFF,
  DEBUG_RXDATA,
  NUM_DEBUG_MODES
};

struct MQTT_MSG_STRUCT {
    char topic[MAX_TOPIC_LEN];
    int tLen;                         // tLen is length of topic, including \0
    char payload[MAX_PAYLOAD_LEN];
    int pLen;                         // pLen is length of payload, including \0
    bool pending;
};

/* ----- shared variables ----- */
extern MQTTClient mqttClient;
extern MQTT_MSG_STRUCT mqttMsg;
extern int mqttStatus;
extern unsigned long lastMqttStatus;
extern unsigned long mqttStatusInterval;
extern char mqtt_id_topic[];
extern char mqtt_cmd_topic[];
extern char mqtt_status_topic[];
extern char mqtt_setting_topic[];
extern char mqtt_rxdata_topic[];


/* ----- prototypes ----- */
bool mqttInit( void );
int mqttDo( void );
void mqttMessageReceived( String &topic, String &payload );
bool mqttProcessCommand( char *cmd, int len );
void mqttPrintStatus( void );
bool mqttConnect( void );
void mqttSendRelayStatus( void );
void FactoryReset( void );

#endif  // _MQTT_IF_H_
