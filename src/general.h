
#ifndef _GENERAL_
#define _GENERAL_

#include <Arduino.h>
#include <HardwareSerial.h> //HardwareSerial Sabus( SABUS_UART_NUM );

/* ----- Definitions ----- */
#define uchar           unsigned char
//#define uint            unsigned int
#define FALSE           false
#define TRUE            true

/* ----- pin definitions ----- */
// waveshare board https://www.waveshare.com/wiki/ESP32-S3-Relay-6CH
#define RX_PIN	        18
#define TX_PIN		    17
#define BUZZER_OUT_PIN  21
#define LED_DATA_PIN    38 // the WS2812B-0807 RGB LED
#define TEST_PIN		8	// when grounded, mqtt topics are preceeded with /test/<mac>

//#define TEST_MODE           true    // when true, topics start with "test/"

#define NUM_OUTPUTS			6		// output channels (not more than 8, or need to change type for OutputStatus)
#define NUM_REMOTE_DEVS     16  // remote switche modules (that means up to 32 individual switches are supported)
#define ID_STRING_LEN       17  // maximum length of the remote switches firmware version string
#define NUM_STATUS_BYTES    4   // number of bytes in remote switch status
#define TXRX_BUF_LEN        12
#define MAX_TMO             3   // number of successive timeouts before we declare a remote switch disconnected

/* ----- POB Faults ----- */
#define POB_FAULT_DEFINITIONS				-1	// just a placeholder
#define POB_FAULT_NO_WIFI           1   // blink once
#define POB_FAULT_NO_MQTT           2   // blink twice
#define POB_FAULT_NO_RS485          4   // blink three times


/* ----- EEPROM ----- */
#define EEPROM_INIT_VAL     123 // used to determine if preferences have been initialized

/* ----- for SABus ----- */
#define NULL_CHAR     0

enum {
	WAIT_FOR_STX,
	WAIT_FOR_ADDRESS,
	WAIT_FOR_CMD_ETX,
	WAIT_FOR_SUM,
	WAIT_FOR_FLUSH_ETX,
	WAIT_FOR_FLUSH_SUM
};

/* -----SABus message formatting characters ----- */
#define STX				    2
#define ETX				    3
#define ACK				    6	  // 0x 0000 0110
#define NAK				    21	// 0x 0001 0101
#define EOT				    4	  // 0x 0000 0100 for USM

#define EVEN_PARITY		0x00
#define PARITY_BIT		0x80

#define PARITY_MASK		0x7F

#define PROTOCOL_BIT  0x40

/* declarations for special locations in the TxRxBuf */
enum {
  IO_BUF_STX_INDEX,
  IO_BUF_ADDR_INDEX,
  IO_BUF_CMD_INDEX,
  IO_BUF_DATA_INDEX
};

#define CMD_TIMEOUT             20    // how many mS we wait before declaring a timeout
#define CMD_INTERVAL            30    // how many mS we wait between QueryDev, default 30

/*enum {
	COMM_TMO_TIMER, 			    // 0
	NOWIFI_TIMER,				      // 1
	SAVE_TIMER,					      // 2
	NUM_SLOW_TIMERS				    // 3
};*/

/* ----- remote commands ----- */
#define ID_QUERY_CHAR				'0'
#define STATUS_RQ					  '1'
#define LIGHT_OFF						'2'
#define LIGHT_ON						'3'
#define RESET						    '9'

/* ----- prototypes ----- */
void blinkPobFault( void );
bool DoChannelChangeRequest( uchar addr, uchar stat );
void DoSwitchRequest( uchar addr, uchar stat );
void lightCntl_Init( void );
void OutputCommand( uchar channel, bool cmd );
void printMenuSettings( void );
void ProcessSabusMsg( char *rxBuf );
void ProcessSabusRxChar( int tchar );
void QueryDev( int addr );
void RxIDQuery( uchar addr );
void RxModeCmd( uchar addr );
void RxStatusQuery( uchar addr );
void rs485Do( void );
void rs485Init( void );
char sendCmd( char *pTxMsg, char addr );
void SettingsSave( void );

/* ----- shared variables ----- */
extern bool sabusMsgPending;
extern char VersionMsg[];
extern char rxBuf[];
extern uchar IDString[NUM_REMOTE_DEVS][ID_STRING_LEN];
extern uchar RemoteStat[NUM_REMOTE_DEVS][NUM_STATUS_BYTES];
extern uchar OnLine[NUM_REMOTE_DEVS];
extern uchar menu_val[2*NUM_REMOTE_DEVS];
extern uchar menu_val_default[];
extern uchar OutputStatus;
extern uchar OnLine[];
extern long saveTime;
extern bool changeRequestInProcess[NUM_REMOTE_DEVS];
extern int pobFault;
extern long blinkPobLast;
extern int blinkInterval;
extern char OutputStatusStr[35];
extern const uchar light_pins[];
extern long lastCmdMillis;
extern int lastOffLineScanned;
extern int offLineCounter;
extern int lastOutputStatus;
extern bool buzzerOn;
extern uint mac;
extern long rebootTimer;
extern int debug;

#endif    // _GENERAL_
