#include <errlog.h>
#include <epicsMutex.h>
#include <asynPortDriver.h>
#include <asynOctetSyncIO.h>

#include <stdint.h>
#include <string.h>
#include <math.h>    /* pow */

#include <ipmiMsg.h>
#include <ipmiDef.h>

#undef DEBUG

float REPLY_TIMEOUT = 0.25;

/* Convert 2-element array (which stores LS byte first) to integer */
uint16_t 
arrayToUint16(uint8_t *data )
{
unsigned i;
uint16_t value = 0;

	for ( i = 0 ; i < 2 ; i++)		
			value |= *data++ << i*8;

	return value;
}

/* Increment value of 2-element uint8_t array (which stores LS byte first)
 * Roll over to 0 when max possible value is reached.
 */
void 
incr2Uint8Array(uint8_t *data, int incr) 
{    
unsigned i, nBytes = 2;
uint16_t value = 0;

	value = arrayToUint16( data );

	if ( value >= (pow(2,8*nBytes) - incr) ) 
		value = 0;
	else
		value += incr; 

	/* Copy incremented value to array */
	for ( i = 0 ; i < nBytes ; i++) 
		*data++ = (value >> i*8) & 0xFF;	
}

/* Convert 4-element array (which stores LS byte first) to integer */
uint32_t 
arrayToUint32(uint8_t *data)
{
unsigned i;
uint32_t value = 0;

	for ( i = 0 ; i < 4 ; i++)			
			value |= *data++ << i*8;

	return value;
}

/* Increment value of 4-element uint8_t array (which stores LS byte first)
 * Roll over to 0 when max possible value is reached.
 */
void 
incr4Uint8Array(uint8_t *data, int incr) 
{    
unsigned i, nBytes = 4;
uint32_t    value  = 0;

	value = arrayToUint32( data );

/* test rollover */
	if ( value >= (pow(2,8*nBytes) - incr) ) 
		value = 0;
	else
		value += incr; 

	/* Copy incremented value to array */
	for ( i = 0 ; i < nBytes ; i++)
		*data++ = (value >> i*8) & 0xFF;	
}

uint8_t
calcTwosComplementChecksum(uint8_t *data, unsigned n)
{
unsigned i;
uint32_t cs = 0;   
 
	for ( i = 0; i < n ; i++) {
		cs += *data++;
	}

	return ( 0x100 - (cs & 0xFF) );	
}

/* 
 * Set session sequence number and session ID for IPMI message,
 * depending on message type. Copy to header.
 */
void
ipmiMsgSetSeqId(MchData mchData, uint8_t *message, uint8_t cmd)
{
int i;
	/* Session sequence number is not incremented (or used) for messages outside of a session */
	if ( cmd != IPMI_MSG_CMD_GET_CHAN_AUTH && cmd != IPMI_MSG_CMD_GET_SESSION_CHALLENGE && cmd != IPMI_MSG_CMD_ACTIVATE_SESSION && cmd != IPMI_MSG_CMD_SET_PRIV_LEVEL && cmd )
		incr4Uint8Array( mchData->seqSend , 1 );

	/* Close Session command contains the session ID */
	if ( cmd == IPMI_MSG_CMD_CLOSE_SESSION ) {
		for ( i = 0; i < IPMI_MSG2_ID_LENGTH ; i++)
			message[IPMI_MSG2_ID_OFFSET + i] = mchData->id[i];
	}

	/* Copy data to IPMI header */
	for ( i = 0; i < IPMI_MSG_HDR_SEQ_LENGTH ; i++)
		message[IPMI_MSG_HDR_SEQ_OFFSET + i] = mchData->seqSend[i];

	for ( i = 0; i < IPMI_MSG_HDR_ID_LENGTH ; i++)
		message[IPMI_MSG_HDR_ID_OFFSET + i]  = mchData->id[i];
}

/*
 *
 * Build IPMI outgoing message. Message structure (see ipmiMsg.h for more details):
 *
 * {RMCP header}{IPMI header}{IPMI msg 1(checksum)}{IPMI msg 2 [Bridged msg 1(checksum)][Bridged msg 2(checksum)](checksum)}
 *                                                              |_____________________________________________|      |
 *                                                                                     |                             |
 *                                                                          Bridged message is optional           Checksum for
 *                                                                                                                IPMI msg 2
 *                                                                                                               (does not include
 *                                                                                                                bridged msg)
 *   RETURNS:
 *            number of bytes in message
 */
int			                    
ipmiMsgBuild(MchData mchData, uint8_t *message, uint8_t cmd, uint8_t *imsg2, size_t imsg2Size, uint8_t *bmsg1, size_t bmsg1Size, uint8_t *bmsg2, size_t bmsg2Size)
{
size_t   iheaderSize = sizeof(IPMI_HEADER); 
size_t   imsg1Size   = sizeof(IPMI_MSG1);
uint8_t  iheader[iheaderSize];
uint8_t  imsg1[imsg1Size];
int      n, i, offset = 0;
uint8_t  cs2;

	memcpy( iheader, IPMI_HEADER, iheaderSize );
	memcpy( imsg1,   IPMI_MSG1  , imsg1Size   );

	ipmiMsgSetSeqId( mchData, iheader, cmd );

	/* Activate Session command echoes the challenge string */
	if ( cmd == IPMI_MSG_CMD_ACTIVATE_SESSION ) {
		for ( i = 0; i < IPMI_MSG2_STR_LENGTH ; i++)
			imsg2[IPMI_MSG2_STR_OFFSET + i] = mchData->str[i];
	}

	/* Number of bytes in message */
	iheader[IPMI_MSG_HDR_NBYTES_OFFSET] = bmsg1Size ? 2*imsg1Size + imsg2Size + bmsg2Size : imsg1Size + imsg2Size;

	n = sizeof(RMCP_HEADER) + sizeof(IPMI_HEADER) + iheader[IPMI_MSG_HDR_NBYTES_OFFSET];

	/* Build message */
	memcpy( message, RMCP_HEADER, sizeof(RMCP_HEADER) );

	offset += sizeof(RMCP_HEADER);

	memcpy( message + offset, iheader, iheaderSize );

	/* Calculate checksums */
	imsg1[imsg1Size - 1] = calcTwosComplementChecksum( (uint8_t *)imsg1, imsg1Size - 1 );
	cs2 = imsg2[imsg2Size - 1] = calcTwosComplementChecksum( (uint8_t *)imsg2, imsg2Size - 1 );

	offset += iheaderSize;
	memcpy( message + offset, imsg1, imsg1Size );

	offset += imsg1Size;

	if ( bmsg1Size ) {
		/* Calculate checksums */
		bmsg1[imsg1Size - 1] = calcTwosComplementChecksum( (uint8_t *)bmsg1, imsg1Size - 1 );
		bmsg2[bmsg2Size - 1] = calcTwosComplementChecksum( (uint8_t *)bmsg2, bmsg2Size - 1 );

		memcpy( message + offset, imsg2, imsg2Size -1 );

		offset += imsg2Size - 1;
		memcpy( message + offset, bmsg1, imsg1Size );

		offset += imsg1Size;
		memcpy( message + offset, bmsg2, bmsg2Size );

		offset += bmsg2Size;
		memcpy( message + offset, &cs2, 1 );
	}
	else
		memcpy( message + offset, imsg2, imsg2Size );

	return n;
}

/*
 * Send message and read response
 * 
 * The response message length varies, so we specify a longer 
 * response length than we may need. Thus, asyn always returns
 * status 'timeout', which we ignore.
 */
void
ipmiMsgWriteRead(const char *name, uint8_t *message, size_t messageSize, uint8_t *response)
{
size_t     numSent;
size_t     responseLen;
int        eomReason;
asynStatus status;
asynUser  *pasynUser;

	pasynUser = pasynManager->createAsynUser(0, 0);
	status    = pasynOctetSyncIO->connect(name, 0, &pasynUser, NULL);

	status    =  pasynOctetSyncIO->writeRead( /*mchData->*/pasynUser, (const char *)message, messageSize, (char *)response, MSG_MAX_LENGTH, REPLY_TIMEOUT, &numSent, &responseLen, &eomReason );
}


/* Call ipmiMsgWriteRead. 
 * If there is no response but MCH is alive, start a new session and then optionally
 * re-send original message.
 *
 *   RETURNS: 0 if non-zero response
 *           -1 if no response or failed to start new session
 */
int
ipmiMsgWriteReadHelper(MchData mchData, uint8_t *message, size_t messageSize, uint8_t *response, int retry, uint8_t cmd)
{

       	if ( mchIsAlive[mchData->instance] ) {

		memset( response, 0, MSG_MAX_LENGTH ); /* Initialize response to 0s in order to detect an actual read timeout */

		ipmiMsgWriteRead( mchData->name, message, messageSize, response );

		if ( 0 == *response ) {

			if ( retry != MSG_NO_RETRY ) {

				/* Start new session and optionally re-send message */
				if ( mchNewSession( mchData ) )
					return -1;
				else {

					if ( retry == MSG_RESEND ) {
						ipmiMsgSetSeqId( mchData, message + IPMI_MSG_HEADER_OFFSET, cmd );
						ipmiMsgWriteRead( mchData->name, message, messageSize, response );

						if ( 0 == *response ) /* If still no response, return error */
							return -1;
					}
				}
			}
		}
      	}
       	else
	       	return -1;

	return 0;
}

/*
 * To start a session, perform the following sequence of messages:
 * 
 *    Get Channel Authentication Capabilities
 *    Get Session Challenge
 *    Activate Session
 *    Set Session Privilege
 *
 * Messages sent outside of a session or to close a session 
 * use ipmiMsgWriteRead directly, which does not try to re-send 
 * or start a new session.
 */

/* Get Channel Authentication Capabilities */
void
ipmiMsgGetChanAuth(MchData mchData, uint8_t *data)
{
uint8_t  message[MSG_MAX_LENGTH];
uint8_t *imsg2       = GET_AUTH_MSG;
size_t   imsg2Size   = sizeof( GET_AUTH_MSG );
size_t   messageSize = ipmiMsgBuild( mchData, message, IPMI_MSG_CMD_GET_CHAN_AUTH, imsg2, imsg2Size, 0, 0, 0, 0 );

	ipmiMsgWriteRead( mchData->name, message, messageSize, data );
}

/* Get Session Challenge */
void
ipmiMsgGetSess(MchData mchData, uint8_t *data)
{
uint8_t  message[MSG_MAX_LENGTH];
uint8_t *imsg2       = GET_SESS_MSG;
size_t   imsg2Size   = sizeof( GET_SESS_MSG );
size_t   messageSize = ipmiMsgBuild( mchData, message, IPMI_MSG_CMD_GET_SESSION_CHALLENGE, imsg2, imsg2Size, 0, 0, 0, 0 );

	ipmiMsgWriteRead( mchData->name, message, messageSize, data );
}

/* Activate Session */
void
ipmiMsgActSess(MchData mchData, uint8_t *data)
{
uint8_t  message[MSG_MAX_LENGTH];
uint8_t *imsg2     = ACT_SESS_MSG;
size_t   imsg2Size = sizeof( ACT_SESS_MSG );
size_t   messageSize;
int      i;

	/* Copy challenge string */
	for ( i = 0; i < IPMI_MSG2_STR_LENGTH ; i++)
		imsg2[IPMI_MSG2_STR_OFFSET + i] = mchData->str[i];

	messageSize = ipmiMsgBuild( mchData, message, IPMI_MSG_CMD_ACTIVATE_SESSION, imsg2, imsg2Size, 0, 0, 0, 0 );

	ipmiMsgWriteRead( mchData->name, message, messageSize, data );
}

/* Set Privilege Level */
void
ipmiMsgSetPriv(MchData mchData, uint8_t *data, uint8_t level)
{
uint8_t  message[MSG_MAX_LENGTH];
uint8_t *imsg2     = SET_PRIV_MSG;
size_t   imsg2Size = sizeof( SET_PRIV_MSG );
size_t   messageSize;

	imsg2[IPMI_MSG2_PRIV_LEVEL_OFFSET] = level;

  	messageSize = ipmiMsgBuild( mchData, message, IPMI_MSG_CMD_SET_PRIV_LEVEL, imsg2, imsg2Size, 0, 0, 0, 0 );

	ipmiMsgWriteRead( mchData->name, message, messageSize, data );
}

/* Close Session */
void
ipmiMsgCloseSess(MchData mchData, uint8_t *data)
{
uint8_t  message[MSG_MAX_LENGTH];
uint8_t *imsg2     = CLOSE_SESS_MSG;
size_t   imsg2Size = sizeof( CLOSE_SESS_MSG );
size_t   messageSize;

	messageSize = ipmiMsgBuild( mchData, message, IPMI_MSG_CMD_CLOSE_SESSION, imsg2, imsg2Size, 0, 0, 0, 0 );

	ipmiMsgWriteRead( mchData->name, message, messageSize, data );
}

/* Cold Reset. Do not reconnect; next request message will handle reconnection */
void
ipmiMsgColdReset(MchData mchData, uint8_t *data)
{
uint8_t  message[MSG_MAX_LENGTH];
uint8_t *imsg2     = BASIC_MSG;
size_t   imsg2Size = sizeof( BASIC_MSG );
size_t   messageSize;

	imsg2[IPMI_MSG2_CMD_OFFSET] = IPMI_MSG_CMD_COLD_RESET;

	messageSize = ipmiMsgBuild( mchData, message, IPMI_MSG_CMD_COLD_RESET, imsg2, imsg2Size, 0, 0, 0, 0 );

	ipmiMsgWriteReadHelper( mchData, message, messageSize, data , MSG_RESEND, IPMI_MSG_CMD_COLD_RESET );
}

/* Chassis Control */
void
ipmiMsgChassisControl(MchData mchData, uint8_t *data, uint8_t parm)
{
uint8_t  message[MSG_MAX_LENGTH];
size_t   imsg2Size   = sizeof( SEND_MSG_MSG );
size_t   bmsg1Size   = sizeof( IPMI_MSG1 );
size_t   bmsg2Size   = sizeof( CHAS_CTRL_MSG );
uint8_t  imsg2[imsg2Size]; 
uint8_t  bmsg1[bmsg1Size];
uint8_t  bmsg2[bmsg2Size];
size_t   messageSize;

	memcpy( imsg2, SEND_MSG_MSG , imsg2Size );
	memcpy( bmsg1, IPMI_MSG1    , bmsg1Size );
	memcpy( bmsg2, CHAS_CTRL_MSG, bmsg2Size );

	imsg2[IPMI_MSG2_CHAN_OFFSET] = 0x40/*channel*/;

	bmsg1[IPMI_MSG1_RSADDR_OFFSET] = IPMI_MSG_ADDR_CM;
	bmsg1[IPMI_MSG1_NETFN_OFFSET]  = IPMI_MSG_NETFN_CHASSIS;

	bmsg2[IPMI_MSG2_CMD_OFFSET]    = IPMI_MSG_CMD_CHAS_CTRL;
	bmsg2[IPMI_MSG2_SENSOR_OFFSET] = parm;

	messageSize = ipmiMsgBuild( mchData, message, IPMI_MSG_CMD_CHAS_CTRL, imsg2, imsg2Size, bmsg1, bmsg1Size, bmsg2, bmsg2Size );

	ipmiMsgWriteReadHelper( mchData, message, messageSize, data , MSG_RESEND, IPMI_MSG_CMD_CHAS_CTRL );
}

/* Get FRU Inventory Info */
void
ipmiMsgGetFruInfo(MchData mchData, uint8_t *data, uint8_t id)
{
uint8_t  message[MSG_MAX_LENGTH];
size_t   imsg2Size   = sizeof( SEND_MSG_MSG );
size_t   bmsg1Size   = sizeof( IPMI_MSG1 );
size_t   bmsg2Size   = sizeof( SENS_READ_MSG );
uint8_t  imsg2[imsg2Size];
uint8_t  bmsg1[bmsg1Size];
uint8_t  bmsg2[bmsg2Size];
size_t   messageSize;

	memcpy( imsg2, SEND_MSG_MSG , imsg2Size );
	memcpy( bmsg1, IPMI_MSG1    , bmsg1Size );
	memcpy( bmsg2, SENS_READ_MSG, bmsg2Size );

	imsg2[IPMI_MSG2_CHAN_OFFSET] = 0x40/*channel*/;

	bmsg1[IPMI_MSG1_RSADDR_OFFSET] = IPMI_MSG_ADDR_CM;
	bmsg1[IPMI_MSG1_NETFN_OFFSET]  = IPMI_MSG_NETFN_STORAGE;

	bmsg2[IPMI_MSG2_CMD_OFFSET]    = IPMI_MSG_CMD_GET_FRU_INFO;
	bmsg2[IPMI_MSG2_SENSOR_OFFSET] = id;

	messageSize = ipmiMsgBuild( mchData, message, IPMI_MSG_CMD_GET_FRU_INFO, imsg2, imsg2Size, bmsg1, bmsg1Size, bmsg2, bmsg2Size );

	ipmiMsgWriteReadHelper( mchData, message, messageSize, data , MSG_RESEND, IPMI_MSG_CMD_GET_FRU_INFO );
}

/* Read FRU data */
void
ipmiMsgReadFru(MchData mchData, uint8_t *data, uint8_t id, uint8_t *readOffset, uint8_t readSize)
{
uint8_t  message[MSG_MAX_LENGTH];
size_t   imsg2Size   = sizeof( SEND_MSG_MSG );
size_t   bmsg1Size   = sizeof( IPMI_MSG1 );
size_t   bmsg2Size   = sizeof( FRU_READ_MSG );
uint8_t  imsg2[imsg2Size];
uint8_t  bmsg1[bmsg1Size];
uint8_t  bmsg2[bmsg2Size];
size_t   messageSize;

	memcpy( imsg2, SEND_MSG_MSG , imsg2Size );
	memcpy( bmsg1, IPMI_MSG1    , bmsg1Size );
	memcpy( bmsg2, FRU_READ_MSG, bmsg2Size );

	imsg2[IPMI_MSG2_CHAN_OFFSET] = 0x40/*channel*/;

	bmsg1[IPMI_MSG1_RSADDR_OFFSET] = IPMI_MSG_ADDR_CM;
	bmsg1[IPMI_MSG1_NETFN_OFFSET]  = IPMI_MSG_NETFN_STORAGE;

	bmsg2[IPMI_MSG2_CMD_OFFSET]          = IPMI_MSG_CMD_READ_FRU_DATA;
	bmsg2[IPMI_MSG2_READ_FRU_ID_OFFSET]  = id;
	bmsg2[IPMI_MSG2_READ_FRU_LSB_OFFSET] = readOffset[0];
	bmsg2[IPMI_MSG2_READ_FRU_MSB_OFFSET] = readOffset[1];
	bmsg2[IPMI_MSG2_READ_FRU_CNT_OFFSET] = readSize;

	messageSize = ipmiMsgBuild( mchData, message, IPMI_MSG_CMD_READ_FRU_DATA, imsg2, imsg2Size, bmsg1, bmsg1Size, bmsg2, bmsg2Size );

	ipmiMsgWriteReadHelper( mchData, message, messageSize, data , MSG_RESEND, IPMI_MSG_CMD_READ_FRU_DATA );
}

/* Get SDR (Sensor Data Record) Repository Info */
void
ipmiMsgGetSdrRepInfo(MchData mchData, uint8_t *data)
{
uint8_t  message[MSG_MAX_LENGTH];
size_t   imsg2Size   = sizeof( SEND_MSG_MSG );
size_t   bmsg1Size   = sizeof( IPMI_MSG1 );
size_t   bmsg2Size   = sizeof( BASIC_MSG );
uint8_t  imsg2[imsg2Size];
uint8_t  bmsg1[bmsg1Size];
uint8_t  bmsg2[bmsg2Size];
size_t   messageSize;

	memcpy( imsg2, SEND_MSG_MSG , imsg2Size );
	memcpy( bmsg1, IPMI_MSG1    , bmsg1Size );
	memcpy( bmsg2, BASIC_MSG, bmsg2Size );

	imsg2[IPMI_MSG2_CHAN_OFFSET] = 0x40/*channel*/;

	bmsg1[IPMI_MSG1_RSADDR_OFFSET] = IPMI_MSG_ADDR_CM;
	bmsg1[IPMI_MSG1_NETFN_OFFSET]  = IPMI_MSG_NETFN_STORAGE;

	bmsg2[IPMI_MSG2_RSADDR_OFFSET] = IPMI_MSG_ADDR_BMC;
	bmsg2[IPMI_MSG2_CMD_OFFSET]    = IPMI_MSG_CMD_GET_SDRREP_INFO;

	messageSize = ipmiMsgBuild( mchData, message, IPMI_MSG_CMD_GET_SDRREP_INFO, imsg2, imsg2Size, bmsg1, bmsg1Size, bmsg2, bmsg2Size );

	ipmiMsgWriteReadHelper( mchData, message, messageSize, data , MSG_RESEND, IPMI_MSG_CMD_GET_SDRREP_INFO );
}

/* 
 * Get SDR (Sensor Data Record)
 * 
 * Offset and reservation ID are 0 unless doing partial read that begins at an 
 * offset into the SDR other than 0.
 *
 * Used for both Get SDR (parm = 0) and Get Device SDR (parm = 1). 
 * Only differences in the message are the network function and command code.
 */

void
ipmiMsgGetSdr(MchData mchData, uint8_t *data, uint8_t *id, uint8_t *res, uint8_t offset, uint8_t readSize, uint8_t parm)
{
uint8_t  message[MSG_MAX_LENGTH];
size_t   imsg2Size   = sizeof( SEND_MSG_MSG );
size_t   bmsg1Size   = sizeof( IPMI_MSG1 );
size_t   bmsg2Size   = sizeof( GET_SDR_MSG );
uint8_t  imsg2[imsg2Size];
uint8_t  bmsg1[bmsg1Size];
uint8_t  bmsg2[bmsg2Size];
size_t   messageSize;
uint8_t  cmd = parm ? IPMI_MSG_CMD_GET_DEV_SDR : IPMI_MSG_CMD_GET_SDR;

	memcpy( imsg2, SEND_MSG_MSG , imsg2Size );
	memcpy( bmsg1, IPMI_MSG1    , bmsg1Size );
	memcpy( bmsg2, GET_SDR_MSG  , bmsg2Size );

	imsg2[IPMI_MSG2_CHAN_OFFSET] = 0x40/*channel*/;

	bmsg1[IPMI_MSG1_RSADDR_OFFSET] = IPMI_MSG_ADDR_CM;
	bmsg1[IPMI_MSG1_NETFN_OFFSET]  = parm? IPMI_MSG_NETFN_SENSOR_EVENT : IPMI_MSG_NETFN_STORAGE;

	bmsg2[IPMI_MSG2_CMD_OFFSET] = cmd;
	bmsg2[IPMI_MSG2_GET_SDR_RES_LSB_OFFSET] = res[0];
	bmsg2[IPMI_MSG2_GET_SDR_RES_MSB_OFFSET] = res[1];
	bmsg2[IPMI_MSG2_GET_SDR_ID_LSB_OFFSET]  = id[0];
	bmsg2[IPMI_MSG2_GET_SDR_ID_MSB_OFFSET]  = id[1];
	bmsg2[IPMI_MSG2_GET_SDR_OFFSET_OFFSET]  = offset;
	bmsg2[IPMI_MSG2_GET_SDR_CNT_OFFSET]     = readSize;

	messageSize = ipmiMsgBuild( mchData, message, cmd, imsg2, imsg2Size, bmsg1, bmsg1Size, bmsg2, bmsg2Size );

	ipmiMsgWriteReadHelper( mchData, message, messageSize, data , MSG_RESEND, cmd );
}

/* Get Device Sensor Data Record (SDR) Info */
void
ipmiMsgGetDevSdrInfo(MchData mchData, uint8_t *data, uint8_t parm)
{
uint8_t  message[MSG_MAX_LENGTH];
size_t   imsg2Size   = sizeof( SEND_MSG_MSG );
size_t   bmsg1Size   = sizeof( IPMI_MSG1 );
size_t   bmsg2Size   = sizeof( GET_DEV_SDR_INFO_MSG );
uint8_t  imsg2[imsg2Size];
uint8_t  bmsg1[bmsg1Size];
uint8_t  bmsg2[bmsg2Size];
size_t   messageSize;

	memcpy( imsg2, SEND_MSG_MSG        , imsg2Size );
	memcpy( bmsg1, IPMI_MSG1           , bmsg1Size );
	memcpy( bmsg2, GET_DEV_SDR_INFO_MSG, bmsg2Size );

	imsg2[IPMI_MSG2_CHAN_OFFSET] = 0x40/*channel*/;

	bmsg1[IPMI_MSG1_RSADDR_OFFSET] = IPMI_MSG_ADDR_CM;
	bmsg1[IPMI_MSG1_NETFN_OFFSET]  = IPMI_MSG_NETFN_SENSOR_EVENT;

	bmsg2[IPMI_MSG2_RSADDR_OFFSET] = IPMI_MSG_ADDR_BMC;
	bmsg2[IPMI_MSG2_CMD_OFFSET]    = IPMI_MSG_CMD_GET_DEV_SDR_INFO;
        bmsg2[IPMI_MSG2_GET_DEV_SDR_INFO_OP_OFFSET] = parm;

	messageSize = ipmiMsgBuild( mchData, message, IPMI_MSG_CMD_GET_DEV_SDR_INFO, imsg2, imsg2Size, bmsg1, bmsg1Size, bmsg2, bmsg2Size );

	ipmiMsgWriteReadHelper( mchData, message, messageSize, data , MSG_RESEND, IPMI_MSG_CMD_GET_DEV_SDR_INFO );
}
	
/* Get Sensor Reading */
void
ipmiMsgReadSensor(MchData mchData, uint8_t *data, uint8_t sens, uint16_t addr)
{
uint8_t  message[MSG_MAX_LENGTH];
size_t   imsg2Size   = sizeof( SEND_MSG_MSG );
size_t   bmsg1Size   = sizeof( IPMI_MSG1 );
size_t   bmsg2Size   = sizeof( SENS_READ_MSG );
uint8_t  imsg2[imsg2Size];
uint8_t  bmsg1[bmsg1Size];
uint8_t  bmsg2[bmsg2Size];
size_t   messageSize;

	memcpy( imsg2, SEND_MSG_MSG , imsg2Size );
	memcpy( bmsg1, IPMI_MSG1    , bmsg1Size );
	memcpy( bmsg2, SENS_READ_MSG, bmsg2Size );

	imsg2[IPMI_MSG2_CHAN_OFFSET] = 0x40/*channel*/;

	bmsg1[IPMI_MSG1_RSADDR_OFFSET] = IPMI_MSG_ADDR_CM;
	bmsg1[IPMI_MSG1_NETFN_OFFSET]  = IPMI_MSG_NETFN_SENSOR_EVENT;

	bmsg2[IPMI_MSG2_SENSOR_OFFSET] = sens;

	messageSize = ipmiMsgBuild( mchData, message, IPMI_MSG_CMD_SENSOR_READ, imsg2, imsg2Size, bmsg1, bmsg1Size, bmsg2, bmsg2Size );

	ipmiMsgWriteReadHelper( mchData, message, messageSize, data, MSG_RESEND, IPMI_MSG_CMD_SENSOR_READ );
}

/* Set FRU Activation */
void
ipmiMsgSetFruActivation(MchData mchData, uint8_t *data, uint8_t fru, uint8_t parm)
{
uint8_t  message[MSG_MAX_LENGTH];
size_t   imsg2Size   = sizeof( SEND_MSG_MSG );
size_t   bmsg1Size   = sizeof( IPMI_MSG1 );
size_t   bmsg2Size   = sizeof( SET_FRU_ACT_MSG );
uint8_t  imsg2[imsg2Size];
uint8_t  bmsg1[bmsg1Size];
uint8_t  bmsg2[bmsg2Size];
size_t   messageSize;

	memcpy( imsg2, SEND_MSG_MSG   , imsg2Size );
	memcpy( bmsg1, IPMI_MSG1      , bmsg1Size );
	memcpy( bmsg2, SET_FRU_ACT_MSG, bmsg2Size );

	imsg2[IPMI_MSG2_CHAN_OFFSET] = 0x40/*channel*/;

	bmsg1[IPMI_MSG1_RSADDR_OFFSET] = IPMI_MSG_ADDR_CM;
	bmsg1[IPMI_MSG1_NETFN_OFFSET]  = IPMI_MSG_NETFN_PICMG;

	bmsg2[IPMI_MSG2_SET_FRU_ACT_FRU_OFFSET] = fru;
	bmsg2[IPMI_MSG2_SET_FRU_ACT_CMD_OFFSET] = parm;

	messageSize = ipmiMsgBuild( mchData, message, IPMI_MSG_CMD_SET_FRU_ACT, imsg2, imsg2Size, bmsg1, bmsg1Size, bmsg2, bmsg2Size );

	ipmiMsgWriteReadHelper( mchData, message, messageSize, data, MSG_RESEND, IPMI_MSG_CMD_SET_FRU_ACT );
}

/* Set FRU Activation Policy
 *
 * Used to activate/deactivate FRU. See ATCA spec 3.2.4.2.2
 *
 * Activate version causes FRU to transition from "Inactive" to "Activation Request"
 * Deactivate version causes FRU to transition from "Active" to "Deactivation Request"
 */
void
ipmiMsgSetFruActPolicy(MchData mchData, uint8_t *data, uint8_t fru, uint8_t mask, uint8_t bits)
{
uint8_t  message[MSG_MAX_LENGTH];
size_t   imsg2Size   = sizeof( SEND_MSG_MSG );
size_t   bmsg1Size   = sizeof( IPMI_MSG1 );
size_t   bmsg2Size   = sizeof( SET_FRU_POLICY_MSG );
uint8_t  imsg2[imsg2Size];
uint8_t  bmsg1[bmsg1Size];
uint8_t  bmsg2[bmsg2Size];
size_t   messageSize;

	memcpy( imsg2, SEND_MSG_MSG   , imsg2Size );
	memcpy( bmsg1, IPMI_MSG1      , bmsg1Size );
	memcpy( bmsg2, SET_FRU_POLICY_MSG, bmsg2Size );

	imsg2[IPMI_MSG2_CHAN_OFFSET] = 0x40/*channel*/;

	bmsg1[IPMI_MSG1_RSADDR_OFFSET] = IPMI_MSG_ADDR_CM;
	bmsg1[IPMI_MSG1_NETFN_OFFSET]  = IPMI_MSG_NETFN_PICMG;

	bmsg2[IPMI_MSG2_SET_FRU_ACT_FRU_OFFSET] = fru;
	bmsg2[IPMI_MSG2_SET_FRU_POLICY_MASK_OFFSET] = mask;
	bmsg2[IPMI_MSG2_SET_FRU_POLICY_BITS_OFFSET] = bits;

	messageSize = ipmiMsgBuild( mchData, message, IPMI_MSG_CMD_SET_FRU_POLICY, imsg2, imsg2Size, bmsg1, bmsg1Size, bmsg2, bmsg2Size );

	ipmiMsgWriteReadHelper( mchData, message, messageSize, data, MSG_RESEND, IPMI_MSG_CMD_SET_FRU_POLICY );
}

/* 
 * Based on parm (0 for Deactivate, 1 for Activate),
 * set appropriate mask and bits and call Set FRU Activation Policy
 *
 * RETURN:
 *         -1 if illegal parm value
 *          0 otherwise
 */
int
ipmiMsgSetFruActPolicyHelper(MchData mchData, uint8_t *data, uint8_t fru, int parm) 
{
uint8_t mask;
uint8_t bits;

	if ( parm == 0 ) {
		mask = 1 << 1; /* make this #define */
		bits = 0;
	}
	else if ( parm == 1 ){
		mask = 1;
		bits = 0;
	} 
	else
		return -1;

       	ipmiMsgSetFruActPolicy( mchData, data, fru, mask, bits );

	return 0;

}
/* Get Device ID */
void
ipmiMsgGetDeviceId(MchData mchData, uint8_t *data, uint8_t rsAddr)
{
uint8_t  message[MSG_MAX_LENGTH];
size_t   imsg2Size   = sizeof( SEND_MSG_MSG );
size_t   bmsg1Size   = sizeof( IPMI_MSG1 );
size_t   bmsg2Size   = sizeof( BASIC_MSG );
uint8_t  imsg2[imsg2Size];
uint8_t  bmsg1[bmsg1Size];
uint8_t  bmsg2[bmsg2Size];
size_t   messageSize;

	memcpy( imsg2, SEND_MSG_MSG, imsg2Size );
	memcpy( bmsg1, IPMI_MSG1,    bmsg1Size );
	memcpy( bmsg2, BASIC_MSG,    bmsg2Size );

	imsg2[IPMI_MSG2_CHAN_OFFSET] = 0x40/*channel*/;

	bmsg1[IPMI_MSG1_RSADDR_OFFSET] = rsAddr ? rsAddr : IPMI_MSG_ADDR_CM;
	bmsg1[IPMI_MSG1_NETFN_OFFSET]  = IPMI_MSG_NETFN_APP_REQUEST;

	bmsg2[IPMI_MSG2_CMD_OFFSET] = IPMI_MSG_CMD_GET_DEVICE_ID;
                                                                          
	messageSize = ipmiMsgBuild( mchData, message, IPMI_MSG_CMD_GET_DEVICE_ID, imsg2, imsg2Size, bmsg1, bmsg1Size, bmsg2, bmsg2Size );

	ipmiMsgWriteReadHelper( mchData, message, messageSize, data, MSG_RESEND, IPMI_MSG_CMD_GET_DEVICE_ID );
}

/* Get Fan Speed Properties */
void
ipmiMsgGetFanProp(MchData mchData, uint8_t *data, uint8_t fru)
{
uint8_t  message[MSG_MAX_LENGTH];
size_t   imsg2Size   = sizeof( SEND_MSG_MSG );
size_t   bmsg1Size   = sizeof( IPMI_MSG1 );
size_t   bmsg2Size   = sizeof( GET_FAN_PROP_MSG );
uint8_t  imsg2[imsg2Size];
uint8_t  bmsg1[bmsg1Size];
uint8_t  bmsg2[bmsg2Size];
size_t   messageSize;

	memcpy( imsg2, SEND_MSG_MSG    , imsg2Size );
	memcpy( bmsg1, IPMI_MSG1       , bmsg1Size );
	memcpy( bmsg2, GET_FAN_PROP_MSG, bmsg2Size );

	imsg2[IPMI_MSG2_CHAN_OFFSET] = 0x40/*channel*/;

	bmsg1[IPMI_MSG1_RSADDR_OFFSET] = IPMI_MSG_ADDR_CM;
	bmsg1[IPMI_MSG1_NETFN_OFFSET]  = IPMI_MSG_NETFN_PICMG;

	bmsg2[IPMI_MSG2_SET_FRU_ACT_FRU_OFFSET] = fru;

	messageSize = ipmiMsgBuild( mchData, message, IPMI_MSG_CMD_GET_FAN_PROP, imsg2, imsg2Size, bmsg1, bmsg1Size, bmsg2, bmsg2Size );

	ipmiMsgWriteReadHelper( mchData, message, messageSize, data, MSG_RESEND, IPMI_MSG_CMD_GET_FAN_PROP );
}

/* Get Fan Level */
void
ipmiMsgGetFanLevel(MchData mchData, uint8_t *data, uint8_t fru)
{
uint8_t  message[MSG_MAX_LENGTH];
size_t   imsg2Size   = sizeof( SEND_MSG_MSG );
size_t   bmsg1Size   = sizeof( IPMI_MSG1 );
size_t   bmsg2Size   = sizeof( GET_FAN_LEVEL_MSG );
uint8_t  imsg2[imsg2Size];
uint8_t  bmsg1[bmsg1Size];
uint8_t  bmsg2[bmsg2Size];
size_t   messageSize;

	memcpy( imsg2, SEND_MSG_MSG     , imsg2Size );
	memcpy( bmsg1, IPMI_MSG1        , bmsg1Size );
	memcpy( bmsg2, GET_FAN_LEVEL_MSG, bmsg2Size );

	imsg2[IPMI_MSG2_CHAN_OFFSET] = 0x40/*channel*/;

	bmsg1[IPMI_MSG1_RSADDR_OFFSET] = IPMI_MSG_ADDR_CM;
	bmsg1[IPMI_MSG1_NETFN_OFFSET]  = IPMI_MSG_NETFN_PICMG;

	bmsg2[IPMI_MSG2_SET_FRU_ACT_FRU_OFFSET] = fru;

	messageSize = ipmiMsgBuild( mchData, message, IPMI_MSG_CMD_GET_FAN_LEVEL, imsg2, imsg2Size, bmsg1, bmsg1Size, bmsg2, bmsg2Size );

	ipmiMsgWriteReadHelper( mchData, message, messageSize, data, MSG_RESEND, IPMI_MSG_CMD_GET_FAN_LEVEL );
}

/* Set Fan Level */
void
ipmiMsgSetFanLevel(MchData mchData, uint8_t *data, uint8_t fru, uint8_t level)
{
uint8_t  message[MSG_MAX_LENGTH];
size_t   imsg2Size   = sizeof( SEND_MSG_MSG );
size_t   bmsg1Size   = sizeof( IPMI_MSG1 );
size_t   bmsg2Size   = sizeof( SET_FAN_LEVEL_MSG );
uint8_t  imsg2[imsg2Size];
uint8_t  bmsg1[bmsg1Size];
uint8_t  bmsg2[bmsg2Size];
size_t   messageSize;

	memcpy( imsg2, SEND_MSG_MSG     , imsg2Size );
	memcpy( bmsg1, IPMI_MSG1        , bmsg1Size );
	memcpy( bmsg2, SET_FAN_LEVEL_MSG, bmsg2Size );

	imsg2[IPMI_MSG2_CHAN_OFFSET] = 0x40/*channel*/;

	bmsg1[IPMI_MSG1_RSADDR_OFFSET] = IPMI_MSG_ADDR_CM;
	bmsg1[IPMI_MSG1_NETFN_OFFSET]  = IPMI_MSG_NETFN_PICMG;

	bmsg2[IPMI_MSG2_SET_FRU_ACT_FRU_OFFSET] = fru;
	bmsg2[IPMI_MSG2_SET_FAN_LEVEL_LEVEL_OFFSET] = level;

	messageSize = ipmiMsgBuild( mchData, message, IPMI_MSG_CMD_SET_FAN_LEVEL, imsg2, imsg2Size, bmsg1, bmsg1Size, bmsg2, bmsg2Size );

	ipmiMsgWriteReadHelper( mchData, message, messageSize, data, MSG_RESEND, IPMI_MSG_CMD_SET_FAN_LEVEL );
}
