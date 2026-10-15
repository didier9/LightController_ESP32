/*===========================================================================
* rs485_if.cpp 
*===========================================================================*/

#include "general.h"
#include "mqtt_if.h"
extern struct EE    ee;

/* ----- SABus stuff ----- */
#define SABUS_UART_NUM          2
HardwareSerial Sabus( SABUS_UART_NUM );
#define MAX_WAIT_MS             10    // timeout in mS for transmitting data
#define SABUS_BAUDRATE	        9600
//#define TXRX_BUF_LEN	          128   // more than big enough

// output assignments
/*---------------------------------------------------------------------------
* ESP32-S3-Relay-6CH
*
GPIO  Function	Description
GP0	  BOOT Key	BOOT Key Control Pin
GP21	BUZZER	  Buzzer Control Pin
GP38	RGB	      RGB LED Control Pin
GP1	  CH1	Relay No.1 Control Pin   OUTSIDE FRONT
GP2	  CH2	Relay No.2 Control Pin   SPARE
GP41	CH3	Relay No.3 Control Pin   FRONT PORCH
GP42	CH4	Relay No.4 Control Pin   OUTSIDE BACK
GP45	CH5	Relay No.5 Control Pin   LIVING ROOM
GP46	CH6	Relay No.6 Control Pin   GARAGE DRIVEWAY
GP17	TXD	UART  TX pin, converted to RS485
GP18	RXD	UART  RX pin, converted to RS485
*---------------------------------------------------------------------------*/
#define LIGHT_1_PIN		      1     // OUTSIDE FRONT
#define LIGHT_2_PIN		      2     // SPARE
#define LIGHT_3_PIN		      41    // FRONT PORCH
#define LIGHT_4_PIN		      42    // OUTSIDE BACK
#define LIGHT_5_PIN		      45    // LIVING ROOM
#define LIGHT_6_PIN		      46    // GARAGE DRIVEWAY

// default switch assignment: which relay drives which light
#define OUTSIDE_FRONT       1
#define SPARE_OUT           2
#define FRONT_PORCH         3
#define OUTSIDE_BACK        4
#define LIVING_ROOM         5
#define GARAGE_DRIVEWAY     6

const uchar light_pins[] = { LIGHT_1_PIN, LIGHT_2_PIN, LIGHT_3_PIN, LIGHT_4_PIN, LIGHT_5_PIN, LIGHT_6_PIN };

// Default assignments for 47 Magnolia
// These can be changed at run-time via mqtt messages or directly using the switches themselves (with switch firmware v006 or later)
// To restore default settings through MQTTExplorer, send these commands:
// topic: lightcontroller/cmd data: S0,5  -> switch 0A controls output 5 (LIVING_ROOM)
// topic: lightcontroller/cmd data: S2,1  -> switch 1A controls output 1 (OUTSIDE_FRONT)
// topic: lightcontroller/cmd data: S3,3  -> switch 1B controls output 3 (FRONT_PORCH)
// topic: lightcontroller/cmd data: S4,5  -> switch 2A controls output 5 (LIVING_ROOM)
// topic: lightcontroller/cmd data: S5,6  -> switch 2B controls output 6 (GARAGE_DRIVEWAY)
// topic: lightcontroller/cmd data: S6,5  -> switch 3A controls output 5 (LIVING_ROOM)
// topic: lightcontroller/cmd data: S8,4  -> switch 4A controls output 4 (OUTSIDE_BACK)
uchar menu_val_default[] = { LIVING_ROOM,0,               // 0A, 0B
                            OUTSIDE_FRONT,FRONT_PORCH,    // 1A, 1B
                            LIVING_ROOM,GARAGE_DRIVEWAY,  // 2A, 2B
                            LIVING_ROOM,0,                // 3A, 3B
                            OUTSIDE_BACK,0,               // 4A, 4B
                            0,0,                          // 5A, 5B
                            0,0,                          // 6A, 6B
                            0,0,                          // 7A, 7B
                            0,0,                          // 8A, 8B
                            0,0,                          // 9A, 9B
                            0,0,                          // 10A, 10B
                            0,0,                          // 11A, 11B
                            0,0,                          // 12A, 12B
                            0,0,                          // 13A, 13B
                            0,0,                          // 14A, 14B
                            0,0,                          // 15A, 15B
                            0,0,                          // 16A, 16B
                           };

#define OUTPUT_OFF          0
#define OUTPUT_ON           1

uchar OutputStatus;	// system supports up to 8 outputs

#define ALL_OUTPUTS_ON    (2^NUM_OUTPUTS-1)

uchar RemoteStat[NUM_REMOTE_DEVS][NUM_STATUS_BYTES];
uchar OnLine[NUM_REMOTE_DEVS];
uchar Output[NUM_REMOTE_DEVS];

uchar IDString[NUM_REMOTE_DEVS][ID_STRING_LEN];	// 16 characters + \0. Used to store remote switch firmware version

// The menu_val[] array is used to define which light each switch controls. It is normally stored in NV memory and can be changed by the user.
// There are 8 LIGHT outputs.
// There are up to 16 remote switch modules, each remote switch module supports two channels (push-buttons), channel A and channel B
// A channel can control a single output but multiple channels can control the same output
// So menu_val has 32 elements, one for each channel. and each menu_val[] element is a bit map of the light being controlled.
// If a channel/button is not assigned to any light output, the corresponding menu_val[] is 0
// If a button is assigned to output x (0-7), menu_val[] = 1<<x
uchar menu_val[2*NUM_REMOTE_DEVS];  // each menu_val[] element has 8 bits, 
                                    // each bit correspond to an output to be controlled by the remote switch corresponding to that menu_val[]

uchar Device;		// Device is the address of the last device that got a user command
                // it is used to display the state of the last commanded device.

uchar CurDev;	// CurDev is the address of the device currently being polled by the background routine
int cmdTimeOut = CMD_TIMEOUT; // set to CMD_TIMEOUT when sending a command

// SABus response type
enum {
  R_NAK,
  R_ACK_NO_DATA,
  R_ACK
};

/* ----- SABus related variables declarations ----- */
char *pRxBuf = &rxBuf[0];
char msgState=0;
char msgSum=0;
char rxBuf[TXRX_BUF_LEN];
bool sabusMsgPending = false;

long lastCmdMillis;           // millis() when we sent the last command. That command was sent to curDev
int lastOffLineScanned=0;     // used to periodically check switch modules that could have gone offline
int offLineCounter = 0;       // used to periodically check switch modules that could have gone offline
char NAKMsg[] = "NAK";
char OutputStatusStr[35];
int lastOutputStatus = -1; // to force an update at power up
bool changeRequestInProcess[NUM_REMOTE_DEVS];

void debugPrint( char *p, int len );

/*---------------------------------------------------------------------------
* printMenuSettings()
*--------------------------------------------------------------------------*/
void printMenuSettings(){
  for( int i = 0; i < NUM_REMOTE_DEVS; i++ ){
    Serial.printf( "switch %dA: %d\n", i, menu_val[2*i] );
    Serial.printf( "switch %dB: %d\n", i, menu_val[2*i+1] );
  }
}   // printMenuSettings()

/*----------------------------------------------------------------------------*/
/* FUNCTION: lightCntl_Init()
*-----------------------------------------------------------------------------*/
void lightCntl_Init( void ){
  uchar i, j;
	for( i=0; i<NUM_REMOTE_DEVS; i++ ){
		RemoteStat[i][0] = 0;
		OnLine[i] = MAX_TMO;
    changeRequestInProcess[i] = true;
    for( int j=0; j<ID_STRING_LEN-1; j++ )
			IDString[i][j] = ' ';
		IDString[i][j] = '\0';
	}
	CurDev = NUM_REMOTE_DEVS-2;
	Device = 0;
  for( i = 0; i < NUM_OUTPUTS; i++ ){
    pinMode( light_pins[i], OUTPUT );
    digitalWrite( light_pins[i], 0 );
  }
  OutputStatus = 0;  // a bit set to 1 in output status means that output is on
} // lightCntl_Init()

/*----------------------------------------------------------------------------*/
/* FUNCTION: rs485Init()
*-----------------------------------------------------------------------------*/
void rs485Init( void ){
    // setup Sabus
  Sabus.begin( SABUS_BAUDRATE, SERIAL_8N1, RX_PIN, TX_PIN );
  Sabus.println( "Testing..." );
}   // rs485Init()

/*---------------------------------------------------------------------------*/
/* FUNCTION: QueryDev()
*
* Send a status query command '1' followed by the current output status for 
* the two lights operated by that switch module, to the module's address.
*
* We need to find which outputs that remote switch module controls and move the
* OutputStatus for those outputs into b0 and b1
*
* Note: the remote switch expects reverse polarity status bits (0 -> on, 1 -> off)
* because the original relay board was inverting (0 -> on). 
*----------------------------------------------------------------------------*/
void QueryDev( int addr ){
  char k;
  char buf[6];
  buf[0] = '1';
  // send to the remote switch module the status of the outputs it controls (for local display of the on/off status on the switch itself)
  if( menu_val[2*addr] > NUM_OUTPUTS ){
    // master switch
    if( OutputStatus == ALL_OUTPUTS_ON )
      k = 0;
    else
       k = 0x01;
  }else if( menu_val[2*addr] > 0 ){
    // switch A is assigned, copy the output status into bit0
    k = (~OutputStatus >> (menu_val[2*addr]-1)) & 0x01;
  }

  if( menu_val[1+2*addr] > 8 ){
    // master switch
    if( OutputStatus != ALL_OUTPUTS_ON )
      k |= 0x02;
  }else if( menu_val[1+2*addr] > 1 )
    // switch B is assigned to an output other than LIGHT_1, shift OutputStatus right to bit1 (or don't shift if menu_val[1+2*addr] is 2)
    k += (~OutputStatus >> (menu_val[1+2*addr]-2)) & 0x02;
  else if( menu_val[1+2*addr] == 1 )
    // switch B is assigned to the LIGHT_1 output, shift OutputStatus left to bit1
    k += (~OutputStatus << 1) & 0x02;

  buf[1] = (k & 0x3F) | PROTOCOL_BIT;	// only 6 data bits at most

  if( changeRequestInProcess[addr] ) // acknowledge one time
    buf[2] = '1';
  else
    buf[2] = '0';
  changeRequestInProcess[addr] = false;

  buf[3] = '\0';
  sendCmd( buf, addr );
}   // QueryDev()

/*---------------------------------------------------------------------------
* FUNCTION: rs485Do()
*
*---------------------------------------------------------------------------*/
void rs485Do(){
  static int currentAddr = -1, currentRetry = -1;
  static uchar lastMsg, lastAddr;
  static bool time_displayed = false;
  static int offLineCounter = 0;

    // query the next remote switch status
  if( (millis() - lastCmdMillis) >= CMD_INTERVAL ){
    // it is time to send a status query
    lastCmdMillis = millis();
    int i=0;  // make sure we don't get stuck in the do{} loop if there are no remote devices talking
    do{
      currentAddr++;
      if( currentAddr >= NUM_REMOTE_DEVS ){
        currentAddr = 0;
      }
    }while( OnLine[currentAddr] == 0 && i++ < NUM_REMOTE_DEVS );
    if( i >= NUM_REMOTE_DEVS )
      pobFault = POB_FAULT_NO_RS485;
    QueryDev( currentAddr );
    sabusMsgPending = false;
    // every so often recheck offline devices
    if( offLineCounter-- <= 0 ){
      offLineCounter = 20;
      int i = NUM_REMOTE_DEVS;
      do{ 
        lastOffLineScanned++;
        if( lastOffLineScanned >= NUM_REMOTE_DEVS )
          lastOffLineScanned = 0;
        if( OnLine[lastOffLineScanned] == 0 ){
          OnLine[lastOffLineScanned] = 1;
          break;
        }
      }while( i-- > 0 );
    }
  }
  // do we have a time out on a status query?
  if( cmdTimeOut > 0 && (millis() - lastCmdMillis) >= cmdTimeOut ){
    if( OnLine[currentAddr] > 0 ){
      OnLine[currentAddr]--;
      //Serial.printf( "OnLine[%d]=%d\n", currentAddr, OnLine[currentAddr] );
    }
  }
  // are we getting stuff from the serial port?
	while( Sabus.available() ){
		int inByte = Sabus.read();
		inByte = 0x7F & inByte; // strip parity
		ProcessSabusRxChar( inByte );
    pobFault &= ~POB_FAULT_NO_RS485;  // if we are getting traffic, there is some activity on the bus
  }
  // did we get a complete SABus message?
	if( sabusMsgPending == true ){
    	sabusMsgPending = false;
      OnLine[currentAddr] = MAX_TMO;
      cmdTimeOut = 0;
      ProcessSabusMsg( rxBuf );
	}
}   // rs485Do()

/*----------------------------------------------------------------------------*/
/* FUNCTION: OutputCommand()
*
* Control the relay outputs
*	channel = 0 through NUM_OUTPUTS-1
*	cmd = 1 (On) or 0 (Off)
* if( channel >= NUM_OUTPUTS )
*   if all lights are ON, turn all OFF.
*   else turn them all ON
*-----------------------------------------------------------------------------*/
void OutputCommand( uchar channel, bool cmd ){
	uchar oldOutput = OutputStatus;
  //Serial.printf( "OutputCommand: channel: %d cmd = %d\n", channel, cmd );
  if( channel > (NUM_OUTPUTS-1) ){
    // cmd does not mean anything when channel is >= NUM_OUTPUTS, it's just a toggle
    if( OutputStatus == ALL_OUTPUTS_ON ){
      // all lights are ON, turn all off
      for( int i=0; i < NUM_OUTPUTS; i++ )
        digitalWrite( light_pins[i], 0 );
      OutputStatus = 0;
      Serial.println( "All lights OFF" );
    }else{
      // not all are ON, turn all ON
      for( int i=0; i < NUM_OUTPUTS; i++ )
        digitalWrite( light_pins[i], 1 );
      OutputStatus = ALL_OUTPUTS_ON;
      Serial.println( "All lights ON" );
    }
    return;
  }else{
    digitalWrite( light_pins[channel], cmd );
	}

	if( cmd )
		OutputStatus |= 1<<channel;
	else
		OutputStatus &= ~(1<<channel);

  if( oldOutput != OutputStatus ){
    if( cmd == 0 )
      Serial.printf( "Output %d is OFF\n", channel+1 );
    else
      Serial.printf( "Output %d is ON\n", channel+1 );
  }else
    Serial.println( "Output not changed" );
} //  OutputCommand()

/*----------------------------------------------------------------------------*/
/* FUNCTION: RxIDQuery()
*
* not used
*-----------------------------------------------------------------------------*/
void RxIDQuery( uchar addr ){
	uchar i;
	char *p_buf;
	p_buf = &rxBuf[0] + IO_BUF_DATA_INDEX;
	for( i=0; i<ID_STRING_LEN; i++ ){
		IDString[addr][i] = rxBuf[IO_BUF_DATA_INDEX+i]; //*(p_buf++) & 0x3F;
		if( IDString[addr][i] == '\0' )
			break;
	}
	IDString[addr][ID_STRING_LEN-1] = '\0';	// to make sure
}  // RxIDQuery()

/*---------------------------------------------------------------------------*/
/* FUNCTION: RxStatusQuery()
*
* Process the remote switch status message.
* Here is the data in the remote data packet:
*	TxD[0] = DesiredSwitchStatus where bit 0 and/or 1 are set to 1 
*	to change the state of the corresponding switch.
*	TxD[1] is set to the desired channel for IN_1
*	TxD[2] is set to the desired channel for IN_2
*----------------------------------------------------------------------------*/
void RxStatusQuery( uchar addr ){
	uchar i, len;
	char *p_buf;
  if( addr > NUM_REMOTE_DEVS-1 ){
    Serial.printf( "Invalid address %d in RxStatusQuery\n", addr );
    return;
  }
	p_buf = &rxBuf[0] + IO_BUF_DATA_INDEX;
  len = strlen( (char *)p_buf );
	// copy status string of remote device into buffer (data only)
	for( i=0; i<NUM_STATUS_BYTES; i++ ){
		RemoteStat[addr][i] = *(p_buf++) & 0x3F;  // strip protocol and parity bits
	}
  if( debug & DEBUG_RXDATA ){
    Serial.printf( "switch %d, len %d ", addr, len );
    debugPrint( (char *)RemoteStat[addr], len );
  }
	if( RemoteStat[addr][0] != 0 ){
    // if RemoteStat[addr][0] != 0, then it contains a request from the remote switch to change the state of one of the outputs,
		DoSwitchRequest( addr, RemoteStat[addr][0] );
  }
  if( len > 2 ){
    // new remote switch software has more data bytes
    if( RemoteStat[addr][1] != 0 ){
      // if RemoteStat[addr][1] != 0, then it contains the new channel assignment for switch A, 
      DoChannelChangeRequest( 2*addr, RemoteStat[addr][1] );
    }else if( RemoteStat[addr][2] != 0 ){
      // if RemoteStat[addr][2] != 0, then it contains the new channel assignment for switch B, 
      DoChannelChangeRequest( 2*addr+1, RemoteStat[addr][2] );
    }
  }
} // RxStatusQuery()

/*--------------------------------------------------------------------------*/
/* FUNCTION: DoSwitchRequest()
*
* process a request from the remote switch to change the state of an output
*	addr is the sabus address of the Switch module number, 0 thru 15
* The function looks through the menu_val[] table to see which output this switch 
* module controls, and if the output is ON, it will be turned OFF and conversely.
*
*	stat is the first data byte, has bit 0 or bit 1 set
*	stat = E 0x45 -> bit 0 set, change state of switch pointed to by menu_val[addr]
*	stat = F 0x46 -> bit 1 set, change state of switch pointed to by menu_val[1+addr]
*	stat = D 0x44 -> do nothing
*--------------------------------------------------------------------------*/
void DoSwitchRequest( uchar addr, uchar stat ){
  //Serial.println( "Switch Request" );
	static uchar i;
	static int j, k;
	uchar sr_ndx;

	j = menu_val[2*addr];
	k = menu_val[1+2*addr];
	i = addr;
	if( stat & 0x02 ){
		sr_ndx = 2*addr+1;
		if( menu_val[1+2*addr] > 0 )
			i = menu_val[1+2*addr]-1;
	}else if( stat & 0x01 ){
		sr_ndx = 2*addr;
		if( menu_val[2*addr] > 0 )
			i = menu_val[2*addr]-1;
	}else
		return;

  if( i == 9 ){
    OutputCommand( 9, 0 );
  }else{
    if( (OutputStatus & (1<<i)) != 0 )
      // switch is currently on, turn it off
      OutputCommand( i, 0 );
    else
      OutputCommand( i, 1 );
	}
} // DoSwitchRequest()

/*---------------------------------------------------------------------------
* FUNCTION: DoChannelChangeRequest()
*
* Process a request from the switch to change its channel assignment
*---------------------------------------------------------------------------*/
bool DoChannelChangeRequest( uchar ndx, uchar stat ){
  uchar i;
  if( stat > NUM_OUTPUTS ){
    Serial.printf( "Invalid channel assignment %d for switch %d\n", stat, ndx );
    return false;
  }
  Serial.printf( "Received new channel %d for switch %d\n", stat, ndx );
  menu_val[ndx] = stat;
  SettingsSave();
  //Serial.printf( "Wrote %d to menu_val[%d]\n", stat, ndx );
  changeRequestInProcess[ndx/2] = true;
  return true;
} // DoChannelChangeRequest()

/*--------------------------------------------------------------------------*/
/* FUNCTION: ProcessSabusMsg()
*---------------------------------------------------------------------------*/
void ProcessSabusMsg( char *rxBuf ){
	uchar i = rxBuf[IO_BUF_ADDR_INDEX] - '0';
	//OnLine[i] = MAX_TMO; // + TotOffLine;
	switch( rxBuf[IO_BUF_CMD_INDEX] ){
		case '0':	// ID Query
			RxIDQuery( i );
			break;
		case '1':	// Status Query/Switch Request
			RxStatusQuery( i );
			break;
		default:	// other commands
			//RxMiscCmd( i );
			break;
	}
} // ProcessSabusMsg()

/*---------------------------------------------------------------------*/
/*  FUNCTION: ProcessSabusRxChar()
*----------------------------------------------------------------------*/
void ProcessSabusRxChar( int tchar ){
	// Be Deaf if already processing a command
	if( sabusMsgPending ){
		return;
	}
	tchar &= PARITY_MASK;
	// Phase Receiver with ACK
	if( (tchar == ACK) && (msgState != WAIT_FOR_SUM) ){
		msgState = WAIT_FOR_ADDRESS;
		rxBuf[0] = tchar;
		pRxBuf = &rxBuf[0] + 1;
		msgSum = tchar;
		return;
	}
	switch( msgState ){
		case WAIT_FOR_ADDRESS:
			*pRxBuf++ = tchar;
			msgSum ^= tchar;
			msgState = WAIT_FOR_CMD_ETX;
			break;
		case WAIT_FOR_CMD_ETX:
			msgSum ^= tchar;
			if( tchar == ETX )
				msgState = WAIT_FOR_SUM;
			else
				*pRxBuf++ = tchar;
			break;
		case WAIT_FOR_SUM:
			msgSum ^= tchar;
			if( msgSum == 0 ){ // good checksum
				sabusMsgPending = TRUE;
      }
			msgState = WAIT_FOR_STX;
			*pRxBuf = NULL_CHAR;
			pRxBuf = &rxBuf[0];
			break;
		case WAIT_FOR_FLUSH_ETX:
			if( tchar == ETX )
				msgState = WAIT_FOR_FLUSH_SUM;
			break;
		case WAIT_FOR_FLUSH_SUM:
			msgState = WAIT_FOR_STX;
			break;
		default:
			break;
	}
} // ProcessSabusRxChar()

/*-------------------------------------------------------------------------------*/
/* FUNCTION: sendCmd()
*--------------------------------------------------------------------------------*/
char sendCmd( char *pTxMsg, char addr ){
	uchar i, j, parity, cs=0, tchar, len;
  char txBuf[TXRX_BUF_LEN];
	// put message together in SABus format
	// make sure data is 7 bits, add SABus header, footer, protocol bit and calculate checksum
  len = strlen( pTxMsg );
  if( len == 0 )
    return( 0 );
 	txBuf[0] = STX;
  cs = txBuf[0];
  txBuf[1] = addr + '0';
  cs ^= txBuf[1];
  for( i=0; i<len; i++ ){
    txBuf[i+2] = *(pTxMsg++);
    txBuf[i+2] &= 0x7F; // blank parity bit
    if( txBuf[i+2] < 0x20 )
      txBuf[i+2] |= PROTOCOL_BIT; // add protocol bit 
    cs ^= txBuf[i+2];   // compute checksum
  }
  len += 2;
  txBuf[len++] = ETX; // add end of text
  cs ^= ETX;
  txBuf[len++] = cs;  // add checksum
  for( i=0; i<len; i++ )
    Sabus.write( txBuf[i] );
  // get ready to receive
  pRxBuf = rxBuf;
  msgState = 0;
  msgSum = 0;
 	return( len );
}   // sendCmd()

void debugPrint( char *p, int len ){
  while( len-- > 0 ){
    Serial.printf( "%02x ", *p++ );
  }
  Serial.println("");
}
