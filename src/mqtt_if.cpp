/*===========================================================================
* mqtt_if.cpp
*
* handles mqtt setup and communication
*===========================================================================*/
#include "mqtt_if.h"

MQTT_MSG_STRUCT mqttMsg;

int mqttStatus = MQTT_DISCONNECTED;
unsigned long lastMqttStatus = 0;
unsigned long lastMqttConnectAttempt = 0;
unsigned long mqttStatusInterval = MQTT_STATUS_INTERVAL;  // if set to 0, mqtt does not send status messages
char mqtt_id_topic[40]={0};
char mqtt_cmd_topic[40]={0};
char mqtt_status_topic[40]={0};
char mqtt_setting_topic[40]={0};
char mqtt_rs485_status_topic[40]={0};
char mqtt_rxdata_topic[40]={0};
int debug=0;

/*---------------------------------------------------------------------------
* mqttInit()
*
*---------------------------------------------------------------------------*/
bool mqttInit( void ){
  if( digitalRead( TEST_PIN ) == 0 ){
    strcpy( mqtt_id_topic, MQTT_TEST_TOPIC );
    strcpy( mqtt_cmd_topic, MQTT_TEST_TOPIC );
    strcpy( mqtt_status_topic, MQTT_TEST_TOPIC );
    strcpy( mqtt_setting_topic, MQTT_TEST_TOPIC );
    strcpy( mqtt_rs485_status_topic, MQTT_TEST_TOPIC );
    strcpy( mqtt_rxdata_topic, MQTT_TEST_TOPIC );
  }
  strcat( mqtt_id_topic, MQTT_ID_TOPIC );
  strcat( mqtt_cmd_topic, MQTT_CMD_TOPIC );
  strcat( mqtt_status_topic, MQTT_STATUS_TOPIC );
  strcat( mqtt_setting_topic, MQTT_SETTING_TOPIC );
  strcat( mqtt_rs485_status_topic, MQTT_RS485_STATUS_TOPIC );
  strcat( mqtt_rxdata_topic, MQTT_RXDATA_TOPIC );
  
  mqttClient.onMessage( mqttMessageReceived );
  mqttClient.setWill( mqtt_status_topic, "Disconnected" );
  if(  mqttConnect() ){
    mqttStatus = MQTT_CONNECTED;
    return( true );
  }else
    mqttStatus = MQTT_DISCONNECTED;
  return( false );
} // mqttInit()

/*---------------------------------------------------------------------------
* mqttDo()
*
*---------------------------------------------------------------------------*/
int mqttDo(){
  mqttClient.loop();
  if( mqttClient.connected() ){
    if( mqttStatus != MQTT_CONNECTED ){
      mqttStatus = MQTT_CONNECTED;
      Serial.println( "MQTT reconnected" );
    }
    if( mqttStatusInterval > 0 && millis() - lastMqttStatus >= mqttStatusInterval ){
      lastMqttStatus = millis();
      mqttSendRelayStatus();
    }
  }else{
    if( mqttStatus != MQTT_DISCONNECTED ){
      mqttStatus = MQTT_DISCONNECTED;
      Serial.println( "MQTT disconnected" );
    }
    // Only retry every MQTT_RECONNECT_INTERVAL, not every loop() pass, so a down
    // broker can't repeatedly stall RS485 polling back-to-back.
    if( millis() - lastMqttConnectAttempt >= MQTT_RECONNECT_INTERVAL ){
      lastMqttConnectAttempt = millis();
      mqttConnect();
    }
    return( POB_FAULT_NO_MQTT );
  }
  // have we received an mqtt message?
  if( mqttMsg.pending ){
    //Serial.printf( "doMqtt(): mqttMsg.pending: %s,%s\n", mqttMsg.topic, mqttMsg.payload );
    if( strncmp( mqttMsg.topic, mqtt_cmd_topic, strlen( mqtt_cmd_topic )) == 0 ){
      if( !mqttProcessCommand( mqttMsg.payload, mqttMsg.pLen ))
        Serial.printf( "cmd \"%s\" not found or failed.\n", mqttMsg.payload );
    }
    mqttMsg.pending = false;
  }
  return( 0 );
} // mqttDo()

/*---------------------------------------------------------------------------
* FUNCTION: mqttSendStatus()
* works for up to 8 channels
*---------------------------------------------------------------------------*/
void mqttSendRelayStatus(){
    int i;
    OutputStatusStr[0] = 0;
    for( i=0; i<NUM_OUTPUTS; i++ ){
        if( ((OutputStatus >> i) & 0x01) == 0 )
          sprintf( (char *)OutputStatusStr + strlen( (const char *)OutputStatusStr ), "%d=off ", i+1 );
        else
          sprintf( (char *)OutputStatusStr + strlen( (const char *)OutputStatusStr ), "%d=on ", i+1 );
    }
    if( buzzerOn )
      strcat( (char *)OutputStatusStr, " buzzer=on " );
    else
      strcat( (char *)OutputStatusStr, " buzzer=off " );
    if( pobFault > 0 )
      sprintf( (char *)OutputStatusStr + strlen( (const char *)OutputStatusStr ), " pob=%d", pobFault );
    else
      strcat( (char *)OutputStatusStr, "pob=ok" );
    mqttClient.publish( mqtt_status_topic, (const char *)OutputStatusStr );
    Serial.printf( "Publishing %s, %s\n", mqtt_status_topic, (const char *)OutputStatusStr );
} // mqttSendRelayStatus()

/*---------------------------------------------------------------------------*/
/* mqttMessageReceived()
*
* copy the received message into the mqttMsg structure and notify the 
* main loop we've got something.
*----------------------------------------------------------------------------*/
void mqttMessageReceived( String &topic, String &payload ){
  Serial.println( "mqttMsgReceived: " + topic + ", " + payload );
  strncpy( mqttMsg.topic, topic.c_str(), MAX_TOPIC_LEN-2 );
  mqttMsg.topic[MAX_TOPIC_LEN-1] = 0;     // just in case
  mqttMsg.tLen = strlen( mqttMsg.topic ); // the actual length
  strncpy( mqttMsg.payload, payload.c_str(), MAX_PAYLOAD_LEN-2 );
  mqttMsg.payload[MAX_PAYLOAD_LEN-1] = 0; // just in case
  mqttMsg.pLen = strlen( mqttMsg.payload ); // the actual length
  mqttMsg.pending = true;
}   // mqttMessageReceived()

/*---------------------------------------------------------------------------*/
/* mqttProcessCommand()
* Called when receiving a message with topic MQTT_CMD_TOPIC
* Process the payload
* The payload is formatted as a command char followed by optional data
*----------------------------------------------------------------------------*/
bool mqttProcessCommand( char *cmd, int len ){
  bool ret = false;
  const char s[2] = ",";
  int i;
  char outputBuf[MAX_PAYLOAD_LEN];
  bool prevBuz = buzzerOn;
  Serial.printf( "processMqttCommand: %s\n", cmd );
  switch( cmd[0] ){
    case'0':
      // send controller ID
      char buf[40];
      sprintf( buf, "%s %0X", VersionMsg, mac );
      mqttClient.publish( mqtt_id_topic, buf );
      Serial.printf( "Publishing %s, %s %0X\n", mqtt_id_topic, VersionMsg, mac );
      ret = true;
      break;
    case '1':
      // send controller output status
      char OutputStr[35];
      for( i=0; i<8; i++ ){
        if( ((OutputStatus >> i) & 0x01) == 0 )
          strcat( OutputStr, "off " );
        else
          strcat( OutputStr, "on " );
      }
      mqttClient.publish( mqtt_status_topic, OutputStr );
      Serial.println( OutputStr );
      ret = true;
      break;
    case 'B':
      // turn buzzer on or off
      // 'B0' -> buzzer off
      // 'B1' -> buzzer on
      if( (uchar)cmd[1] == '1' ){
        buzzerOn = true;
        ret = true;
      }else if( (uchar)cmd[1] == '0' ){
        buzzerOn = false;
        ret = true;
      }
      if( prevBuz != buzzerOn )
        SettingsSave();
      break;
    case 'C':
      // control a channel on/off
      // C<ndx>,<cmd> where ndx is a lightchannel 0 -> NUM_OUTPUTS-1 and cmd is 0|1 for off|on
      // example: C0,1 turns on Relay #1
      if( (uchar)(cmd[1]-'0') < NUM_OUTPUTS && (uchar)(cmd[3]-'0') < 2 ){
        OutputCommand( cmd[1]-'0', cmd[3]-'0' );
        ret = true;  
      }
      break;
    case 'D':
      // debug mode
      if( sscanf( cmd+1, "%d", &i ) == 1 && i < NUM_DEBUG_MODES ){
        debug = i;
        ret = true;
      }
      break;
    case 'I':
      // change I60000 (and force sending new status right away)
      long l;
      if( sscanf( cmd+1, "%d", &l ) == 1 && l > 10000 ){
        mqttStatusInterval = l;
        lastMqttStatus = millis() + l;
        ret = true;
      }
      break;
    case 'R':
      // query or change switch status
      if( cmd[1] == 0 ){
        // query status
        outputBuf[0] = 0;
        for( int i=0; i<NUM_REMOTE_DEVS; i++ ){
          if( OnLine[i] > 0 ){
            sprintf( outputBuf + strlen(outputBuf), "%d:%d, ", i, OnLine[i] );
            if( strlen( outputBuf ) > MAX_PAYLOAD_LEN-7 )
              // can't put any more
              break;
          }
        }
        if( strlen( outputBuf ) == 0 )
          strcpy( outputBuf, "no switch online" );
        if( mqttClient.publish( mqtt_rs485_status_topic, outputBuf )){
          Serial.println( "Publish successful" );
          ret = true;
        }
      }else{
        // control a channel on/off
        // R<ndx>,<cmd> where ndx is a relay 1 -> NUM_OUTPUTS and cmd is 0|1 for off|on
        // example: R1,1 turns on Relay #1
        // this works like "C" command except that the channel starts at 1 instead of 0
        if( (uchar)(cmd[1]-'1') < NUM_OUTPUTS && (uchar)(cmd[3]-'0') < 2 ){
          OutputCommand( cmd[1]-'1', cmd[3]-'0' );
          ret = true;  
        }
      }
      break;
    case 'S':
      // display or change which channel is controlled by the switch <ndx>
      // 'S' -> display current channel assignments as MQTT_SETTING_TOPIC
      // "S<ndx>,<new_value>"" -> change a menu_val[] value
      // "S<ndx>,?"" returns current setting as MQTT_SETTING_TOPIC
      // ndx = (0-31), new_value = (0-8) 0 -> switch not assigned, 1-8 -> switch assigned to output 1-8
      // Examples to reset to defaults:
      // topic: lightcontroller/cmd data: S0,5  -> switch 0A controls output 5 (LIVING_ROOM)
      // topic: lightcontroller/cmd data: S2,1  -> switch 1A controls output 1 (OUTSIDE_FRONT)
      // topic: lightcontroller/cmd data: S3,3  -> switch 1B controls output 3 (FRONT_PORCH)
      // topic: lightcontroller/cmd data: S4,5  -> switch 2A controls output 5 (LIVING_ROOM)
      // topic: lightcontroller/cmd data: S5,6  -> switch 2B controls output 6 (GARAGE_DRIVEWAY)
      // topic: lightcontroller/cmd data: S6,5  -> switch 3A controls output 5 (LIVING_ROOM)
      // topic: lightcontroller/cmd data: S8,4  -> switch 4A controls output 4 (OUTSIDE_BACK)
      char *p, a;
      int ndx, val;
      outputBuf[0] = 0;
      if( cmd[1] == 0 ){
        for( int i=0; i<NUM_REMOTE_DEVS; i++ ){
          if( (i%2) == 0 )
            a = 'a';
          else
            a = 'b';
          //sprintf( outputBuf + strlen(outputBuf), "%d%c:%d, ", i/2, a, menu_val[i] );
          sprintf( outputBuf + strlen(outputBuf), "%d, ", menu_val[i] );
        }
        Serial.println( outputBuf );
        if( mqttClient.publish( mqtt_setting_topic, outputBuf ))
          Serial.println( "Publish successful" );
        return( true );
      }
      p = strtok( cmd+1, s );
      if( p != NULL ){
        if( sscanf( p, "%d", &ndx ) == 1 && ndx < (2*NUM_REMOTE_DEVS) ){
          p = strtok( NULL, s );
          if( p != NULL ){
            if( sscanf( p, "%d", &val ) == 1 && val <= NUM_OUTPUTS ){
              menu_val[ndx] = val;
              SettingsSave();
              ret = true;
            }else if( *p == '?' ){
              // return current setting now
              Serial.print( "Got setting request: " );
              sprintf( outputBuf, "S%d,%d", ndx, menu_val[ndx] );
              Serial.println( outputBuf );
              if( mqttClient.publish( mqtt_setting_topic, outputBuf ))
                Serial.println( "Publish successful" );
              ret = true;
            }else{
              Serial.println( "val not a number or > NUM_OUTPUTS" );
            }
          }else{
            Serial.println( "val not found" );
          }
        }else{
          Serial.println( "ndx not a number or >= 2*NUM_REMOTE_DEVS" );
        }
      }else{
        Serial.println( "ndx not found" );
      }
      break;
    case '*':
      // factory reset
      FactoryReset();
      Serial.println( "Factory Reset" );
      ret = true;
      break;
    default:
      Serial.println( "Unknown command" );
      break;
  }
  return( ret );
} // mqttProcessCommand()

void FactoryReset( void ){
  for( int i = 0; i < NUM_REMOTE_DEVS; i++ ){
    menu_val[i] = menu_val_default[i];
  }
}   // FactoryReset()

/*---------------------------------------------------------------------------*/
/* mqttPrintStatus()
*----------------------------------------------------------------------------*/
void mqttPrintStatus(){
  char stat[20];
  switch( mqttStatus ){
    case MQTT_DISCONNECTED:
      strcpy( stat, "Disconnected" );
      break;
    case MQTT_READY_TO_CONNECT:
      strcpy( stat, "Ready to Connect" );
      break;
    case MQTT_CONNECTING:
      strcpy( stat, "Connecting" );
      break;
    case MQTT_CONNECTED:
      strcpy( stat, "Connected" );
      break;
  }
  Serial.printf( "mqttStatus = %s\n", stat );
}   // mqttPrintStatus()

/*---------------------------------------------------------------------------*/
/* mqttConnect()
*----------------------------------------------------------------------------*/
bool mqttConnect(){
  char buf[40]={0};
  if( mqttClient.connected() ){
    mqttStatus = MQTT_CONNECTED;
    return true;
  }
  mqttStatus = MQTT_DISCONNECTED;
  Serial.println( "connecting to MQTT..." );
  if( digitalRead( TEST_PIN ) == 0 )
    strcpy( buf, "test_" );
  strcat( buf, "light_controller" );
  // Single bounded attempt (WiFiClient connect timeout + MQTT protocol timeout, see
  // net.setTimeout()/mqttClient.setTimeout()). No internal retry loop: mqttDo() paces
  // retries via MQTT_RECONNECT_INTERVAL so RS485 polling never gets stalled for long.
  bool ret = mqttClient.connect( buf, MQTT_USER, MQTT_PASSWORD );
  if( ret ){
    mqttStatus = MQTT_CONNECTED;
    mqttPrintStatus();
    int qos = 0;
    mqttClient.subscribe( mqtt_cmd_topic, qos );
    Serial.printf( "Subscribing to %s\n", mqtt_cmd_topic );
    char idBuf[40];
    sprintf( idBuf, "%s %0X", VersionMsg, mac );
    mqttClient.publish( mqtt_id_topic, idBuf );
    Serial.printf( "Publishing %s, %s\n", mqtt_id_topic, (const char *)idBuf );
    lastMqttStatus = millis() - mqttStatusInterval;
  }else{
    Serial.println( "Failed to connect to mqtt broker." );
  }
  return ret;
}   // mqttConnect()
