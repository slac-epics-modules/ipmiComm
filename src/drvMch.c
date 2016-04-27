#include <stdint.h>
#include <stdio.h>   /* fopen, etc. */
#include <string.h>
#include <math.h>    /* floor, pow */
#include <ctype.h>   /* toupper */

#include <arpa/inet.h>
#include <netinet/in.h>

#include <errlog.h>
#include <epicsExport.h>
#include <drvSup.h>
#include <iocsh.h>
#include <epicsMutex.h>
#include <cantProceed.h>
#include <epicsThread.h>
#include <dbScan.h>

#include <drvMch.h>
#include <drvMchMsg.h>
#include <ipmiMsg.h>
#include <picmgDef.h>

#undef DEBUG 

#define PING_PERIOD  5

char mchDescString[MCH_TYPE_MAX][MCH_DESC_MAX_LENGTH] =
                {   "Unknown device\0", 
                    "MicroTCA Crate, VT MCH\0",
                    "MicroTCA Crate, NAT MCH\0",
                    "Supermicro Server\0",
                    "",
                    "",
                    "",
                    "",
                    "",
                    ""
                };


int mchCounter = 0;

epicsMutexId mchStatMtx[MAX_MCH];
uint32_t     mchStat[MAX_MCH] = { 0 };

static int mchSdrGetDataAll(MchData mchData);
static int mchFruGetDataAll(MchData mchData);
int  mchCnfg(MchData mchData);

static void
mchSeqInit(IpmiSess ipmiSess)
{
int i;
	/* Initialize IPMI sequence */
	ipmiSess->seq = 0;

        /* Initialize our stored sequence number for messages from MCH */
	ipmiSess->seqRply[0] = IPMI_WRAPPER_SEQ_INITIAL;
        for ( i = 1; i < IPMI_RPLY_SEQ_LENGTH - 1 ; i++)
                ipmiSess->seqRply[i] = 0;	
}

static void
mchSetAuth(MchSess mchSess, IpmiSess ipmiSess, uint8_t authByte)
{
	ipmiSess->authReq = 0xFF; /* Default to illegal value */
	ipmiSess->authSup = IPMI_AUTH_TYPE_SUPPORT( authByte );

	if ( ipmiSess->authSup & (1<<IPMI_MSG_AUTH_TYPE_NONE) ) {
		ipmiSess->authReq = IPMI_MSG_AUTH_TYPE_NONE;
	}
	else if (  ipmiSess->authSup & (1<<IPMI_MSG_AUTH_TYPE_PWD_KEY) ) {
		ipmiSess->authReq = IPMI_MSG_AUTH_TYPE_PWD_KEY;
	}
	else
		printf("mchSetAuth: No supported auth type for %s, supported mask is 0x%02x\n", 
		    mchSess->name, ipmiSess->authSup);
}

/* Start communication session with MCH
 * Multi-step handshaking sequence
 *
 * Caller must perform locking.
 *
 *   RETURNS:
 *         0 on success
 *         non-zero on failure
 */		
static int 
mchCommStart(MchSess mchSess, IpmiSess ipmiSess)
{	
uint8_t response[MSG_MAX_LENGTH] = { 0 };
int     i;

	if ( MCH_DBG( mchStat[mchSess->instance] ) )
		printf("%s Connecting...\n", mchSess->name);

	mchSeqInit( ipmiSess );

	if ( mchMsgGetChanAuth( mchSess, ipmiSess, response ) )
		return -1;	

	mchSetAuth( mchSess, ipmiSess, response[IPMI_RPLY_IMSG2_AUTH_CAP_AUTH_OFFSET] );

	if ( mchMsgGetSess( mchSess, ipmiSess, response ) )
		return -1;

        /* Extract temporary session ID */
        for ( i = 0; i < IPMI_RPLY_IMSG2_SESSION_ID_LENGTH ; i++)
                ipmiSess->id[i] = response[IPMI_RPLY_IMSG2_GET_SESS_TEMP_ID_OFFSET + i];

        /* Extract challenge string */
        for ( i = 0; i < IPMI_RPLY_CHALLENGE_STR_LENGTH ; i++)
                ipmiSess->str[i] = response[IPMI_RPLY_IMSG2_GET_SESS_CHALLENGE_STR_OFFSET + i];

	if ( mchMsgActSess( mchSess, ipmiSess, response ) )
		goto bail;;

        /* Extract session ID */
        for ( i = 0; i < IPMI_RPLY_IMSG2_SESSION_ID_LENGTH ; i++)
                ipmiSess->id[i] = response[IPMI_RPLY_IMSG2_ACT_SESS_ID_OFFSET + i];

        /* Extract initial sequence number for messages to MCH */
        for ( i = 0; i < IPMI_RPLY_INIT_SEND_SEQ_LENGTH ; i++)
                ipmiSess->seqSend[i] = response[IPMI_RPLY_IMSG2_ACT_SESS_INIT_SEND_SEQ_OFFSET + i];

	/* Need a non-hard-coded way to determine privelige level */
	if ( mchMsgSetPriv( mchSess, ipmiSess, response, IPMI_MSG_PRIV_LEVEL_OPER ) )
		goto bail;

	return 0;

bail:
	mchMsgCloseSess( mchSess, ipmiSess, response );
	return -1;

}

/* Start new session with MCH
 * Reset session sequence number and session ID to 0,
 * then call mchCommStart to initiate new session
 *
 * Caller must perform locking.
 *
 *   RETURNS: (return val from mchCommStart)
 *           0 on success
 *           non-zero on failure          
 */
int
mchNewSession(MchSess mchSess, IpmiSess ipmiSess)
{
int i, rval = -1;

	for ( i = 0; i < IPMI_WRAPPER_SEQ_LENGTH ; i++)
	        ipmiSess->seqSend[i] = 0;

	for ( i = 0; i < IPMI_WRAPPER_ID_LENGTH ; i++)
		ipmiSess->id[i] = 0;

	if ( MCH_ONLN( mchStat[mchSess->instance] ) )
		rval = mchCommStart( mchSess, ipmiSess );

	return rval;
}

static uint8_t
bcdPlusConvert(uint8_t raw)
{

	switch( raw ) {

		default:
			printf("Illegal or reserved BCD PLUS byte 0x%02x\n", raw);
		case 0:
			return '0';
		case 1:
			return '1';
			break;
		case 2:
			return '2';
			break;
		case 3:
			return '3';
			break;
		case 4:
			return '4';
			break;
		case 5:
			return '5';
			break;
		case 6:
			return '6';
			break;
		case 7:
			return '7';
			break;
		case 8:
			return '8';
			break;
		case 9:
			return '9';
			break;
		case 0xA:
			return ' ';
			break;
		case 0xB:
			return '-';
			break;
		case 0xC:
			return '.';
			break;
	}
}

static void
sixBitAsciiConvert(uint8_t *input, uint8_t *output, uint8_t n_input, uint8_t n_output)
{
int i = 0, j = 0;

	while( 1 ) {

			output[i++] = (input[j*3 + 0] & 0x3F) + 0x20;
			if ( i >= n_output)
				break;

			output[i++] = (((input[j*3 + 0] & 0xC0) >> 6) | ((input[j*3 + 1] & 0xF) << 2)) + 0x20;
			if ( i >= n_output)
				break;

			output[i++] = (((input[j*3 + 1] & 0xF0) >> 4) | ((input[j*3 + 2] & 0x3) << 4)) + 0x20;
			if ( i >= n_output)
				break;

			output[i++] = ((input[j*3 + 2] & 0xFC) >> 2) + 0x20;
			if ( i >= n_output)
				break;

			j++;
	}
}

/* If error, set field->length to zero to indicate no valid data */
static int
mchFruFieldConvertData(FruField field, uint8_t lang)
{
int i;

	switch( field->type ) {

	default:
			printf("FRU field data type %i is not supported\n", field->type);
			return -1;

		case FRU_DATA_TYPE_BINARY:
			printf("FRU field data type binary or unspecified. Add support!\n");

		case FRU_DATA_TYPE_BCDPLUS:

			field->length = field->rlength;
			if ( !( field->data = ipmiReallocZeros( field->data, field->length ) ) ) {
				printf("No memory for FRU field data\n");
				field->length = 0;
				return -1;
			}
			else {
				for ( i = 0; i < field->length; i++ )
					field->data[i] = bcdPlusConvert( field->rdata[i] );
				return 0;
			}

		case FRU_DATA_TYPE_6BITASCII:

			field->length = ceil( 8*((float)field->rlength/6) );
			if ( field->rlength % 3 )
				printf("6-bit ASCII data length not integer multiple of 3, something may be wrong.\n");

			if ( !( field->data = ipmiReallocZeros( field->data, field->length ) ) ) {
				printf("No memory for FRU field data\n");
				field->length = 0;
				return -1;
			}
			else {
				sixBitAsciiConvert( field->rdata, field->data, field->rlength, field->length);
				return 0;
			}

		case FRU_DATA_TYPE_LANG:

			if ( IPMI_DATA_LANG_ENGLISH( lang ) ) {

				field->length = field->rlength;
				if ( !( field->data = ipmiReallocZeros( field->data, field->length ) ) ) {
					printf("No memory for FRU field data\n");
					field->length = 0;
					return -1;
				}

				memcpy( field->data, field->rdata, field->length );
				return 0;
			}
			else
				printf("Warning FRU data language %i is not english; need to add 2-byte unicode support\n", lang);		   
	}
	return 0;
}

/* Copy FRU area field to data structure
 *
 * Caller must perform locking.
 *
 *   RETURNS:
 *           0 on success
 *          -1 if no data to store
 */
static int
mchFruFieldGet(FruField field, uint8_t *raw, unsigned *offset, uint8_t lang)
{
int i;
        if ( ( field->rlength = IPMI_DATA_LENGTH( raw[*offset] ) ) ) {

                field->type = IPMI_DATA_TYPE( raw[*offset] );

                (*offset)++;

                if ( !( field->rdata = ipmiReallocZeros( field->rdata, field->rlength ) ) )
                        printf("No memory for FRU field data\n");
                else {
                        for ( i = 0; i < field->rlength; i++ )
                                field->rdata[i] = raw[*offset + i];

                        *offset += field->rlength;

                        return mchFruFieldConvertData( field, lang );
                }
        }
        return -1;
}

/* 
 * Copy FRU Chassis area data to chassis structure
 *
 * Caller must perform locking.
 */
static void
mchFruChassisDataGet(FruChassis chas, uint8_t *raw, unsigned *offset)
{
	if ( 0 != (*offset = 8*raw[FRU_DATA_COMMON_HEADER_OFFSET + FRU_DATA_COMMON_HEADER_CHASSIS_AREA_OFFSET]) ) {

		chas->type = raw[*offset + FRU_DATA_CHASSIS_TYPE_OFFSET];

		*offset += FRU_DATA_CHASSIS_AREA_PART_LENGTH_OFFSET;

		if ( mchFruFieldGet( &(chas->part),   raw, offset, IPMI_DATA_LANG_CODE_ENGLISH1 ) )
			(*offset)++;
		if ( mchFruFieldGet( &(chas->sn),     raw, offset, IPMI_DATA_LANG_CODE_ENGLISH1 ) )
			(*offset)++;
	}
}

/* 
 * Copy FRU Product area data to product structure
 *
 * Caller must perform locking.
 */
static void
mchFruProdDataGet(FruProd prod, uint8_t *raw, unsigned *offset)
{
	if ( 0 != (*offset = 8*raw[FRU_DATA_COMMON_HEADER_OFFSET + FRU_DATA_COMMON_HEADER_PROD_AREA_OFFSET]) ) {

		prod->lang = raw[*offset + FRU_DATA_BOARD_AREA_LANG_OFFSET];

		*offset += FRU_DATA_PROD_AREA_MANUF_LENGTH_OFFSET;

		if ( mchFruFieldGet( &(prod->manuf),   raw, offset, prod->lang ) )
			(*offset)++;
		if ( mchFruFieldGet( &(prod->prod),    raw, offset, prod->lang) )
			(*offset)++;
		if ( mchFruFieldGet( &(prod->part),    raw, offset, prod->lang ) )
			(*offset)++;
		if ( mchFruFieldGet( &(prod->version), raw, offset, prod->lang ) )
			(*offset)++;
		if ( mchFruFieldGet( &(prod->sn),      raw, offset, prod->lang ) )
			(*offset)++;
	}
}

/* 
 * Copy FRU Board area data to board structure
 *
 * Caller must perform locking.
 */
static void
mchFruBoardDataGet(FruBoard board, uint8_t *raw, unsigned *offset)
{
	if ( 0 != (*offset = 8*raw[FRU_DATA_COMMON_HEADER_OFFSET + FRU_DATA_COMMON_HEADER_BOARD_AREA_OFFSET] ) ) {

		board->lang = raw[*offset + FRU_DATA_BOARD_AREA_LANG_OFFSET];

		*offset += FRU_DATA_BOARD_AREA_MANUF_LENGTH_OFFSET;

		if ( mchFruFieldGet( &(board->manuf), raw, offset, board->lang ) )
			(*offset)++;
		if ( mchFruFieldGet( &(board->prod),  raw, offset, board->lang ) )
			(*offset)++;
		if ( mchFruFieldGet( &(board->sn),    raw, offset, board->lang ) )
			(*offset)++;
		if ( mchFruFieldGet( &(board->part),  raw, offset, board->lang ) )
			(*offset)++;	       
	}
}

/* 
 * Get data for one FRU
 *
 * Read FRU inventory info. If error in response
 * or FRU data area size is zero, return. Else read FRU data
 * and call mchFru*DataGet to store in FRU structure.
 *
 * Caller must perform locking.
 */
static int
mchFruDataGet(MchData mchData, Fru fru, uint8_t id) 
{
MchSess    mchSess = mchData->mchSess;
int        inst    = mchSess->instance;
uint8_t    response[MSG_MAX_LENGTH] = { 0 };
uint8_t   *raw = 0; 
int        i;
uint16_t   sizeInt;  /* Size of FRU data area in bytes */
unsigned   nread;    /* Number of FRU data reads */
unsigned   offset;   /* Offset into FRU data */
int        rval;

	/* Get FRU Inventory Info */
	if ( mchMsgGetFruInvInfo( mchData, response, id ) )
		return -1;

	fru->size[0] = response[IPMI_RPLY_IMSG2_FRU_AREA_SIZE_LSB_OFFSET];
	fru->size[1] = response[IPMI_RPLY_IMSG2_FRU_AREA_SIZE_MSB_OFFSET];
	fru->access  = response[IPMI_RPLY_IMSG2_FRU_AREA_ACCESS_OFFSET];  

	if ( 0 == (sizeInt = arrayToUint16( fru->size )) )
		return 0;

	if ( MCH_DBG( mchStat[inst] ) )
		printf("%s mchFruDataGet: FRU %i inventory info size %i\n", mchSess->name, id, sizeInt);

	if ( !(raw = ipmiReallocZeros( raw, sizeInt ) ) ) {
		printf("mchFruDataGet: No memory for FRU %i data\n",id);
		return -1;
	}

	/* Too many reads! But no way to determine the end of the data until we read it, so for now... */
	if ( ( nread = floor( sizeInt/MSG_FRU_DATA_READ_SIZE ) ) > 50 )
		nread = 50;

	/* Initialize read offset to 0 */
	for ( i = 0; i < sizeof( fru->readOffset ); i ++ )
		fru->readOffset[i] = 0;

	/* Read FRU data, store in raw, increment our read offset for next read. If error returned for requested FRU ID, abort */
	for ( i = 0; i < nread; i++ ) {
		fru->read = i;

		rval = mchMsgReadFru( mchData, response, id, fru->readOffset, MSG_FRU_DATA_READ_SIZE );
		if ( rval ) {
			if ( IPMI_COMP_CODE_REQUESTED_DATA == rval )
				break;
		}
		else
			memcpy( raw + i*MSG_FRU_DATA_READ_SIZE, response + IPMI_RPLY_IMSG2_FRU_DATA_READ_OFFSET, MSG_FRU_DATA_READ_SIZE );
		incr2Uint8Array( fru->readOffset, MSG_FRU_DATA_READ_SIZE );
	}

	if ( MCH_DBG( mchStat[inst] ) >= MCH_DBG_HIGH ) {
		printf("%s FRU %i raw data, size %i: \n", mchSess->name, id, sizeInt);
		for ( i = 0; i < sizeInt; i++)
			printf("0x%02x ",raw[i]);
		printf("\n");

		printf("%s FRU %i raw data, size %i: \n", mchSess->name, id, sizeInt);
		for ( i = 0; i < sizeInt; i++)
			printf("%c ",raw[i]);
		printf("\n");
	}

	/* Add chassis data get */
	mchFruBoardDataGet( &(fru->board), raw, &offset );
	mchFruProdDataGet(  &(fru->prod) , raw, &offset );
	mchFruChassisDataGet( &(fru->chassis), raw, &offset );
	free( raw );

	return 0;
}

/* 
 * Get data for all FRUs. Must be done after SDR data has been stored in FRU struct.
 * If FRU is cooling unit, get fan properties.
 *
 * FRUs are indexed by FRU number
 * Later, change this to also discover FRUs that do not have SDRs (like our SLAC board)?
 *
 * Caller must perform locking.
 */
static int
mchFruGetDataAll(MchData mchData)
{
MchSess mchSess = mchData->mchSess;
MchSys  mchSys  = mchData->mchSys;
uint8_t response[MSG_MAX_LENGTH] = { 0 };
uint8_t i;
Fru fru;
int rval = 0;

	for ( i = 0; i < MAX_FRU ; i++ ) {

	       	fru = &mchSys->fru[i];
	
		if ( fru->sdr.recType ) {

			if ( mchFruDataGet( mchData, fru , i ) )
				rval = -1;

			/* If not MCH, do not do MicroTCA-specific tasks; later change this to callbacks */
			if ( !MCH_IS_MICROTCA( mchSess->type ) ) {
			}
			else if ( (i >= UTCA_FRU_TYPE_CU_MIN) && (i <= UTCA_FRU_TYPE_CU_MAX) ) {

				if ( mchMsgGetFanPropHelper( mchData, response, fru->sdr.fruId ) )
					rval = -1;
				fru->fanMin  = response[IPMI_RPLY_IMSG2_GET_FAN_PROP_MIN_OFFSET];
				fru->fanMax  = response[IPMI_RPLY_IMSG2_GET_FAN_PROP_MAX_OFFSET];
				fru->fanNom  = response[IPMI_RPLY_IMSG2_GET_FAN_PROP_NOM_OFFSET];
				fru->fanProp = response[IPMI_RPLY_IMSG2_GET_FAN_PROP_PROP_OFFSET];

				if ( MCH_DBG( mchStat[mchSess->instance] ) >= MCH_DBG_MED )
					printf("FRU %i fan properties min %i max %i nom %i prop 0x%02x\n", i, fru->fanMin, fru->fanMax, fru->fanNom, fru->fanProp);
			}
		}
	}
	if ( MCH_DBG( mchStat[mchSess->instance] ) >= MCH_DBG_MED ) {
		printf("%s mchFruGetDataAll: FRU Summary:\n", mchSess->name);
		for ( i = 0; i < MAX_FRU  ; i++) {
			fru = &mchSys->fru[i];
			if ( fru->sdr.fruId || (arrayToUint16( fru->size ) > 0) )
				printf("SDR FRU ID %i %s was found, id %02x instance %02x, addr %02x dev %02x lun %02x\n", 
				fru->sdr.fruId, fru->board.prod.data, fru->sdr.entityId, fru->sdr.entityInst, fru->sdr.addr, fru->sdr.fruId, fru->sdr.lun);
		}
	}

	return rval;
}

/* 
 * Get sensor/FRU association
 *
 * -Given a sensor index, find the FRU ID (which equals FRU index) and store 
 *  it in the sensor structure. For AMC and RTM non-FRU entities, identify
 *  the associated FRU and do the same. For NAT hot-swap sensors, parse 
 *  description string to find associated FRU ID.
 *
 * Note: these indices refer to our arrays of Sensors and FRUs--
 * they do not refer to the IPMI sensor number or IPMI SDR Record ID
 *
 *
 * Caller must perform locking.
 */
static void
mchSensorGetFru(int type, MchSys mchSys, uint8_t index)
{
int  i, natid, id;
char buff[9];
Sensor sens = &mchSys->sens[index];
SdrFull sdr = &sens->sdr;

	sens->fruId = sens->mgmtIndex = -1; /* Set default indices to indicate no corresponding fru/mgmt */

	/* Supermicro does not use FRU device locator records, so no means to read a FRU ID
	 * Associate all Supermicro sensors with single FRU, arbitrarily choose...0?
	 */
	if ( MCH_IS_SUPERMICRO( type ) ) {
		sens->fruId = 0;
		return;
	}

	/* NAT associates hotswap aka M-state sensors with carrier manager but
         * Vadatech associates them with the actual FRU described by the sensor.
	 * For NAT, parse sensor description to get associated FRU number
         */
	if ( MCH_IS_NAT( type ) && (sdr->sensType == SENSOR_TYPE_HOTSWAP) ) {
		if ( 2 != sscanf( (const char *)(sdr->str), "HS %d %s", &natid, buff ) )
			return;
		sens->fruId = natid;
		return;
	}

	/* Loop through FRUs and Management Controllers
	 * Start at 1 for MicroTCA because FRU 0 is reserved logical 
	 * entity with same entityId and entityInst as MCH 1 
	 */
	for ( i = 0; i < MAX_FRU_MGMT; i++ ) {

		if ( (i == 0) && MCH_IS_MICROTCA( type ) ) {
		}
		else {
			if ( i < MAX_FRU ) {

				if ( (sens->sdr.entityId == mchSys->fru[i].sdr.entityId) && (sens->sdr.entityInst == mchSys->fru[i].sdr.entityInst ) ) {
					sens->fruId = mchSys->fru[i].sdr.fruId; /* FRU ID for associated FRU */
					return;
				}
			}
			else {
				id = i - MAX_FRU;
				if ( (sens->sdr.entityId == mchSys->mgmt[id].sdr.entityId) && (sens->sdr.entityInst == mchSys->mgmt[id].sdr.entityInst) ) {
					sens->mgmtIndex = id;
					return;
				}
			}
		}
	}

	/* If not MCH, do not do MicroTCA-specific tasks; later change this to callbacks */
	if ( !MCH_IS_MICROTCA( type ) )
		return;

	/* Find entities that do not have SDRs */
	if ( sens->sdr.entityId == VT_ENTITY_ID_AMC ) {
		sens->fruId = UTCA_FRU_TYPE_AMC_MIN + sens->sdr.entityInst - 0x60 - 1;
		return;
	}
	else if ( sens->sdr.entityId == VT_ENTITY_ID_RTM ) {
	       	sens->fruId = UTCA_FRU_TYPE_RTM_MIN + sens->sdr.entityInst - 0x60 - 1;
		return;
	}
}

/* Caller must perform locking */
static int
mchSdrRepGetInfoMsg(MchData mchData, uint8_t *response) {

	return mchMsgGetSdrRepInfo( mchData, response );

}

/* Caller must perform locking */
static void
mchSdrRepGetTs(uint8_t *response, uint32_t *addTs, uint32_t *delTs) {

       	*addTs = ntohl( *(uint32_t *)(response + IPMI_RPLY_IMSG2_SDRREP_ADD_TS_OFFSET) );
       	*delTs = ntohl( *(uint32_t *)(response + IPMI_RPLY_IMSG2_SDRREP_DEL_TS_OFFSET) );
}

/* Compare SDR repository timestamps to detect changes
 *
 * Caller must perform locking
 */
static int
mchSdrRepTsDiff(MchData mchData) {
MchSess   mchSess = mchData->mchSess;
MchSys    mchSys  = mchData->mchSys;
uint32_t  add, del;
uint32_t *addTs = &mchSys->sdrRep.addTs, *delTs = &mchSys->sdrRep.delTs;
uint8_t   buff[MSG_MAX_LENGTH] = { 0 };

	if ( mchSdrRepGetInfoMsg( mchData, buff ) )
		return 0;

	mchSdrRepGetTs( buff, &add, &del );

	if ( (add != *addTs) || (del != *delTs) ) {

		if ( MCH_DBG( mchStat[mchSess->instance] ) )
			printf("%s SDR rep TS before: 0x%08x 0x%08x, after: 0x%08x 0x%08x\n", mchSess->name, *addTs, *delTs, add, del);

		*addTs = add;
		*delTs = del;

		return -1;
	}

	return 0;
}

/* 
 * Store SDR Repository info 
 *
 * Caller must perform locking.
 */
static int
mchSdrRepGetInfo(MchData mchData)
{
MchSys  mchSys  = mchData->mchSys;
uint8_t response[MSG_MAX_LENGTH] = { 0 };
uint8_t flags;

	if ( mchSdrRepGetInfoMsg( mchData, response ) )
		return -1;

	mchSys->sdrRep.ver     = response[IPMI_RPLY_IMSG2_SDRREP_VER_OFFSET];
	mchSys->sdrRep.size[0] = response[IPMI_RPLY_IMSG2_SDRREP_CNT_LSB_OFFSET];
	mchSys->sdrRep.size[1] = response[IPMI_RPLY_IMSG2_SDRREP_CNT_MSB_OFFSET];

	mchSdrRepGetTs( response, &mchSys->sdrRep.addTs, &mchSys->sdrRep.delTs );

	mchMsgGetDevSdrInfo( mchData, response, 1 );

	if ( response[0] ) // completion code
		return 0; /* We don't currently support dev sdr, so don't return error */

       	mchSys->sdrRep.devSdrSize = response[IPMI_RPLY_IMSG2_DEV_SDR_INFO_CNT_OFFSET];
	flags = response[IPMI_RPLY_IMSG2_DEV_SDR_INFO_FLAGS_OFFSET];
       	mchSys->sdrRep.devSdrDyn  = DEV_SENSOR_DYNAMIC(flags);
       	mchSys->sdrRep.lun0       = DEV_SENSOR_LUN0(flags);
       	mchSys->sdrRep.lun1       = DEV_SENSOR_LUN1(flags);
       	mchSys->sdrRep.lun2       = DEV_SENSOR_LUN2(flags);
       	mchSys->sdrRep.lun3       = DEV_SENSOR_LUN3(flags);

	return 0;
}

/* 
 * Store SDR for one FRU into FRU data structure
 *
 * Caller must perform locking.
 */
static void
mchSdrFruDev(SdrFru sdr, uint8_t *raw)
{
int n, l, i;
	n = SDR_HEADER_LENGTH + raw[SDR_LENGTH_OFFSET];

	sdr->id[0]      = raw[SDR_ID_LSB_OFFSET];
	sdr->id[1]      = raw[SDR_ID_MSB_OFFSET];
	sdr->ver        = raw[SDR_VER_OFFSET];
	sdr->recType    = raw[SDR_REC_TYPE_OFFSET];
	sdr->length     = raw[SDR_LENGTH_OFFSET];
	sdr->addr       = raw[SDR_FRU_ADDR_OFFSET];
	sdr->fruId      = raw[SDR_FRU_ID_OFFSET];
	sdr->lun        = raw[SDR_FRU_LUN_OFFSET];
	sdr->chan       = raw[SDR_FRU_CHAN_OFFSET];
	sdr->devType    = raw[SDR_FRU_TYPE_OFFSET];
	sdr->devMod     = raw[SDR_FRU_TYPE_MOD_OFFSET];
	sdr->entityId   = raw[SDR_FRU_ENTITY_ID_OFFSET];
	sdr->entityInst = raw[SDR_FRU_ENTITY_INST_OFFSET];
	sdr->strLength  = raw[SDR_FRU_STR_LENGTH_OFFSET];

	l = IPMI_DATA_LENGTH( sdr->strLength );
	for ( i = 0; i < l; i++ )
	       	sdr->str[i] = raw[SDR_FRU_STR_OFFSET + i];
       	sdr->str[i+1] = '\0';
}

/* 
 * Store SDR for one management controller device into data structure
 *
 * Caller must perform locking.
 */
static void
mchSdrMgmtCtrlDev(SdrMgmt sdr, uint8_t *raw)
{
int n, l, i;
	n = SDR_HEADER_LENGTH + raw[SDR_LENGTH_OFFSET];

	sdr->id[0]      = raw[SDR_ID_LSB_OFFSET];
	sdr->id[1]      = raw[SDR_ID_MSB_OFFSET];
	sdr->ver        = raw[SDR_VER_OFFSET];
	sdr->recType    = raw[SDR_REC_TYPE_OFFSET];
	sdr->length     = raw[SDR_LENGTH_OFFSET];
	sdr->addr       = raw[SDR_MGMT_ADDR_OFFSET];
	sdr->chan       = raw[SDR_MGMT_CHAN_OFFSET];
	sdr->pwr        = raw[SDR_MGMT_PWR_OFFSET];
	sdr->cap        = raw[SDR_MGMT_CAP_OFFSET];
	sdr->entityId   = raw[SDR_MGMT_ENTITY_ID_OFFSET];
	sdr->entityInst = raw[SDR_MGMT_ENTITY_INST_OFFSET];
	sdr->strLength  = raw[SDR_MGMT_STR_LENGTH_OFFSET];
			
	l = IPMI_DATA_LENGTH( sdr->strLength );
	for ( i = 0; i < l; i++ )
	       	sdr->str[i] = raw[SDR_MGMT_STR_OFFSET + i];
       	sdr->str[i+1] = '\0';
}

/* 
 * Store SDR for one sensor into sensor data structure
 *
 * Caller must perform locking.
 */
static void
mchSdrFullSens(SdrFull sdr, uint8_t *raw, int type)
{
int n, i, l = 0;
int m = 0, b = 0;

	n = SDR_HEADER_LENGTH + raw[SDR_LENGTH_OFFSET];

	sdr->id[0]      = raw[SDR_ID_LSB_OFFSET];
	sdr->id[1]      = raw[SDR_ID_MSB_OFFSET];
	sdr->ver        = raw[SDR_VER_OFFSET];
	sdr->recType    = raw[SDR_REC_TYPE_OFFSET];
	sdr->length     = raw[SDR_LENGTH_OFFSET];
	sdr->owner      = raw[SDR_OWNER_OFFSET];
	sdr->lun        = raw[SDR_LUN_OFFSET];
	sdr->number     = raw[SDR_NUMBER_OFFSET];
	sdr->entityId   = raw[SDR_ENTITY_ID_OFFSET];
	sdr->entityInst = raw[SDR_ENTITY_INST_OFFSET];
	sdr->init       = raw[SDR_INIT_OFFSET];
	sdr->cap        = raw[SDR_CAP_OFFSET];
	sdr->sensType   = raw[SDR_SENS_TYPE_OFFSET];
	sdr->readType   = raw[SDR_READ_TYPE_OFFSET];

	if ( type == SDR_TYPE_COMPACT_SENSOR ) {
		sdr->strLength  = raw[SDR_COMPACT_STR_LENGTH_OFFSET];    

		l = IPMI_DATA_LENGTH( sdr->strLength );
		for ( i = 0; i < l; i++ )
			sdr->str[i] = raw[SDR_COMPACT_STR_OFFSET + i];
		sdr->str[i+1] = '\0';
	}

	/* Full Sensor fields */
	if ( type == SDR_TYPE_FULL_SENSOR ) {

		sdr->units1     = raw[SDR_UNITS1_OFFSET];      
		sdr->units2     = raw[SDR_UNITS2_OFFSET];    
		sdr->units3     = raw[SDR_UNITS3_OFFSET];    
		sdr->linear     = raw[SDR_LINEAR_OFFSET];   
		sdr->M          = raw[SDR_M_OFFSET];   
		sdr->MTol       = raw[SDR_M_TOL_OFFSET];   
		sdr->B          = raw[SDR_B_OFFSET];   
		sdr->BAcc       = raw[SDR_B_ACC_OFFSET];
		sdr->acc        = raw[SDR_ACC_OFFSET];   
		sdr->RexpBexp   = raw[SDR_EXP_OFFSET];   
		sdr->anlgChar   = raw[SDR_ANLG_CHAR_OFFSET];   
		sdr->nominal    = raw[SDR_NOMINAL_OFFSET];   
		sdr->normMax    = raw[SDR_NORM_MAX_OFFSET];   
		sdr->normMin    = raw[SDR_NORM_MIN_OFFSET];   
		sdr->strLength  = raw[SDR_STR_LENGTH_OFFSET];    

		m = SENSOR_CONV_M_B( sdr->M, sdr->MTol );
		b = SENSOR_CONV_M_B( sdr->B, sdr->BAcc );

		sdr->m    = TWOS_COMP_SIGNED_NBIT( m, 10 );
		sdr->b    = TWOS_COMP_SIGNED_NBIT( b, 10 );
		sdr->rexp = TWOS_COMP_SIGNED_NBIT(SENSOR_CONV_REXP( sdr->RexpBexp ), 4 );
		sdr->bexp = TWOS_COMP_SIGNED_NBIT(SENSOR_CONV_BEXP( sdr->RexpBexp ), 4 );


		l = IPMI_DATA_LENGTH( sdr->strLength );
		for ( i = 0; i < l; i++ )
			sdr->str[i] = raw[SDR_STR_OFFSET + i];
		sdr->str[i+1] = '\0';
       	}     

#ifdef DEBUG
printf("mchSdrFullSens: owner 0x%02x lun %i sdr number %i type 0x%02x, m %i, b %i, rexp %i bexp %i\n", 
	sdr->owner, sdr->lun, sdr->number, sdr->sensType, sdr->m, sdr->b, sdr->rexp, sdr->bexp);
if ( l > 0 )
printf("string %s\n", sdr->str);
#endif
}

/* If sensor ..., return error and indicate 'unavailable' in sensor data structure */
/* This is stricter than it used to be!! Now any non-zero comp code leads to discarding sensor; need to make sure this is okay */
int
mchGetSensorReadingStat(MchData mchData, Sensor sens, uint8_t *response, uint8_t number, uint8_t lun, size_t *sensReadMsgLength)
{
uint8_t bits;
uint8_t rval;

	rval = mchMsgReadSensor( mchData, response, number, lun, sensReadMsgLength );

	/* If error code ... */
	if ( rval ) {
		if ( rval == IPMI_COMP_CODE_REQUESTED_DATA )	
			sens->unavail = 1;
		return -1;
	}

	bits = response[IPMI_RPLY_IMSG2_SENSOR_ENABLE_BITS_OFFSET];
	if ( IPMI_SENSOR_READING_DISABLED(bits) || IPMI_SENSOR_SCANNING_DISABLED(bits) ) {
		if ( MCH_DBG( mchStat[mchData->mchSess->instance] ) )
			printf("%s mchGetSensorReadingStat: sensor %i reading/state unavailable or scanning disabled. Bits: %02x\n", mchData->mchSess->name, number, bits);
		return -1;
	}

	return 0;
}

/*
 * Read sensor to save sensor reading response length; it varies
 * by sensor and this prevents read timeouts later.
 * Get sensor thresholds and store them for use by device support.
 * (Assuming thresholds do not change)
 *
 * Caller must perform locking.
 */
static void
mchGetSensorInfo(MchData mchData, Sensor sens)
{
uint8_t  response[MSG_MAX_LENGTH] = { 0 };
size_t   sensReadMsgLength;

	sensReadMsgLength = sens->readMsgLength = IPMI_RPLY_IMSG2_SENSOR_READ_MAX_LENGTH;

	if( !(MCH_ONLN( mchStat[mchData->mchSess->instance] )) ) {
		if ( MCH_DBG( mchStat[mchData->mchSess->instance] ) )
			printf("%s mchGetSensorInfo: MCH offline; aborting\n", mchData->mchSess->name);
		return;
	}

	/* If sensor does not exist, do not read thresholds, etc. */
	mchGetSensorReadingStat( mchData, sens, response, sens->sdr.number, (sens->sdr.lun & 0x3) , &sensReadMsgLength );/*) {*/	

	if ( sens->unavail )
		return;

	sens->readMsgLength = sensReadMsgLength;

	sens->tmask = 0; /* Set default to no readable thresholds */

	if ( IPMI_SENSOR_THRESH_IS_READABLE( IPMI_SDR_SENSOR_THRESH_ACCESS( sens->sdr.cap ) ) ) {

		if (  mchMsgGetSensorThresholds( mchData, response, sens->sdr.number, (sens->sdr.lun & 0x3) ) ) {
			if ( MCH_DBG( mchStat[mchData->mchSess->instance] ) )
				printf("%s mchGetSensorInfo: mchMsgGetSensorThresholds error, assume no thresholds are readable\n", mchData->mchSess->name);
			return;
		}

		sens->tmask = response[IPMI_RPLY_IMSG2_SENSOR_THRESH_MASK_OFFSET];
		sens->tlnc  = response[IPMI_RPLY_IMSG2_SENSOR_THRESH_LNC_OFFSET];
		sens->tlc   = response[IPMI_RPLY_IMSG2_SENSOR_THRESH_LC_OFFSET];
		sens->tlnr  = response[IPMI_RPLY_IMSG2_SENSOR_THRESH_LNR_OFFSET];
		sens->tunc  = response[IPMI_RPLY_IMSG2_SENSOR_THRESH_UNC_OFFSET];
		sens->tuc   = response[IPMI_RPLY_IMSG2_SENSOR_THRESH_UC_OFFSET];
		sens->tunr  = response[IPMI_RPLY_IMSG2_SENSOR_THRESH_UNR_OFFSET];

		if ( MCH_DBG( mchStat[mchData->mchSess->instance] ) >= MCH_DBG_HIGH )
			printf("sensor %s thresholds tmask 0x%02x, tlnc %i tlc %i tlnr %i tunc %i tuc %i tunr %i\n", 
				sens->sdr.str, sens->tmask, sens->tlnc, sens->tlc, sens->tlnr, sens->tunc, sens->tuc, sens->tunr);
	}
}

/*
 * Read sensor data records. Call ipmiMsgGetSdr twice per record;
 * once to get record length, then to read record. This prevents timeouts,
 * saving much delay. 
 *
 * Caller must perform locking.
 */				  
static int				  
mchSdrGetDataAll(MchData mchData)
{
MchSess  mchSess = mchData->mchSess;
MchSys   mchSys  = mchData->mchSys;
uint8_t  response[MSG_MAX_LENGTH] = { 0 };
uint16_t sdrCount;
uint8_t  id[2]  = { 0 };
uint8_t  res[2] = { 0 };
Sensor   sens   = 0;
uint8_t  offset = 0;
uint8_t  type   = 0;
uint8_t *raw    = 0;
int      err    = 0;
int      size; /* SDR record read size (after header) */
int      i, iFull = 0, iFru = 0, iMgmt = 0, fruId;
int      rval = -1;
int      inst = mchSess->instance;
Fru      fru;
Mgmt     mgmt;
int      remainder = 0;

	if ( mchSdrRepGetInfo( mchData ) )
		return rval;

	sdrCount = arrayToUint16( mchSys->sdrRep.size );

	if ( !(raw = ipmiReallocZeros( raw, sdrCount*SDR_MAX_LENGTH ) ) ) {
		printf("mchSdrGetDataAll: No memory for raw SDR data for %s\n", mchSess->name);
		goto bail;
	}

	if ( mchMsgReserveSdrRep( mchData, response ) ) {
		printf("mchSdrGetDataAll: Error reserving SDR repository %s\n", mchSess->name);
		goto bail;
	}

	res[0] = response[IPMI_RPLY_IMSG2_GET_SDR_RES_LSB_OFFSET];
	res[1] = response[IPMI_RPLY_IMSG2_GET_SDR_RES_MSB_OFFSET];

       	for ( i = 0; i < sdrCount; i++) {
		offset = 0;

		/* readSize = 5 because 5th byte is remaining record length; 0xFF reads entire record */
       		if ( (mchMsgGetSdr( mchData, response, id, res, offset, 5, 0, 0 )) ) {
			i--;
			if ( err++ > 5 ) {
			        printf("mchSdrGetDataAll: too many errors reading SDR for %s\n", mchSess->name);
				goto bail;
			}
		}
		else {

			size = response[IPMI_RPLY_IMSG2_GET_SDR_DATA_OFFSET + SDR_LENGTH_OFFSET] + SDR_HEADER_LENGTH;
			if ( size > SDR_MAX_LENGTH )
				size = SDR_MAX_LENGTH;
			type = response[IPMI_RPLY_IMSG2_GET_SDR_DATA_OFFSET + SDR_REC_TYPE_OFFSET];

			if ( size > SDR_MAX_READ_SIZE ) {
				remainder = size - SDR_MAX_READ_SIZE;
				size = SDR_MAX_READ_SIZE;
			}

			while ( size > 0 ) {
				if ( mchMsgGetSdr( mchData, response, id, res, offset, size, 0, 0 /* recordsize, can be removed */ ) ) {
					i--;
					if ( err++ > 5 ) {
						printf("mchSdrGetDataAll: too many errors reading SDR for %s", mchSess->name);
						goto bail;
					}
					break;
				}

				else {
					memcpy( raw + (i*SDR_MAX_LENGTH) + offset, response + IPMI_RPLY_IMSG2_GET_SDR_DATA_OFFSET, size );
					offset += size;
				}

				if ( remainder > SDR_MAX_READ_SIZE ) {
					remainder = remainder - SDR_MAX_READ_SIZE;
					size = SDR_MAX_READ_SIZE;
				}
				else {
					size = remainder;
					remainder = 0;
				}
			}

			switch ( type ) {

				default:
					break;

				case SDR_TYPE_FULL_SENSOR:
					mchSys->sensCount++;
					break;

				case SDR_TYPE_COMPACT_SENSOR:
					mchSys->sensCount++;
					break;

				case SDR_TYPE_FRU_DEV:
					break;

				case SDR_TYPE_MGMT_CTRL_DEV:
					break;
			}
			id[0]  = response[IPMI_RPLY_IMSG2_GET_SDR_NEXT_ID_LSB_OFFSET];
			id[1]  = response[IPMI_RPLY_IMSG2_GET_SDR_NEXT_ID_MSB_OFFSET];

			if ( arrayToUint16( id ) == SDR_ID_LAST_SENSOR ) // last record in SDR
				break;
		}
       	}

	sdrCount = i;

// no need for both sdrCount and sens; change to only use one or at least stop using sdrCount once sens is assigned

	if ( !(mchSys->sens = ipmiReallocZeros( mchSys->sens, mchSys->sensCount*sizeof(*sens) ) ) ) {
		printf("mchSdrGetDataAll: No memory for sensor data for %s\n", mchSess->name);
		goto bail;
	}

	/* Store SDR data; for now we only support Compact/Full Sensor Records and FRU Device Locator Records */
	for ( i = 0; i < sdrCount; i++) {
		type = raw[i*SDR_MAX_LENGTH + SDR_REC_TYPE_OFFSET];

		if ( (type == SDR_TYPE_FULL_SENSOR) || (type == SDR_TYPE_COMPACT_SENSOR) ) {
			mchSdrFullSens( &(mchSys->sens[iFull].sdr) , raw + i*SDR_MAX_LENGTH, type );
			mchSys->sens[iFull].instance = 0; /* Initialize instance to 0 */

			mchGetSensorInfo( mchData, &mchSys->sens[iFull] );
			mchSys->sens[iFull].cnfg = 0;

			iFull++;
		}
	        else if ( type == SDR_TYPE_FRU_DEV ) {
			fruId = raw[SDR_FRU_ID_OFFSET + i*SDR_MAX_LENGTH];
			mchSdrFruDev( &(mchSys->fru[fruId].sdr), raw + i*SDR_MAX_LENGTH );
			mchSys->fru[fruId].instance = 0;     /* Initialize instance to 0 */
			iFru++;
		}
	        else if ( type == SDR_TYPE_MGMT_CTRL_DEV ) {
			mchSdrMgmtCtrlDev( &(mchSys->mgmt[iMgmt].sdr), raw + i*SDR_MAX_LENGTH );
			iMgmt++;
		}
	}

	if ( MCH_DBG( mchStat[inst] ) >= MCH_DBG_MED ) {
		printf("%s mchSdrGetDataAll Sumary:\n", mchSess->name);
		for ( i = 0; i < iFull; i++ ) {
			sens = &mchSys->sens[i];
			printf("SDR %i, %s entity ID 0x%02x, entity inst 0x%02x, sensor number %i, sens type 0x%02x, "
				"owner 0x%02x, LUN %i, RexpBexp %i, M %i, MTol %i, B %i, BAcc %i\n", 
				sens->sdr.number, sens->sdr.str, sens->sdr.entityId, sens->sdr.entityInst, 
				sens->sdr.number, sens->sdr.sensType, sens->sdr.owner, sens->sdr.lun, 
				sens->sdr.RexpBexp, sens->sdr.M, sens->sdr.MTol, sens->sdr.B, sens->sdr.BAcc);
		}
		for ( i = 0; i < MAX_FRU; i++ ) {
			fru = &mchSys->fru[i];
			if ( fru->sdr.recType )
				printf("FRU %i, entity ID 0x%02x, entity inst 0x%02x, FRU id %i, %s\n", 
				i, fru->sdr.entityId, fru->sdr.entityInst, fru->sdr.fruId, fru->sdr.str);
		}
		for ( i = 0; i < iMgmt; i++ ) {
			mgmt = &mchSys->mgmt[i];
       			printf("Mgmt Ctrl %i, entity ID 0x%02x, entity inst 0x%02x, %s, cap 0x%02x\n", 
				i, mgmt->sdr.entityId, mgmt->sdr.entityInst, mgmt->sdr.str, mgmt->sdr.cap);
		}

	}

	rval = 0;

bail:

	if ( raw )
		free( raw );
	return rval;
}

/* 
 *  Periodically ping MCH. This runs in its own thread.
 *  If MCH online/offline status changes, update global
 *  variable and process status record. Less frequently set
 *  flag to check that MCH data structs match real hardware.
 *
 *  These messages are outside of a session and we don't
 *  modify our shared structure, so there is no need to lock
 *  during the message write. We call ipmiMsgWriteRead directly, 
 *  instead of using the helper routine which tries to recover a 
 *  disconnected session.
 */

void
mchPing(void *arg)
{
MchDev  mch     = arg;
MchData mchData = mch->udata;
MchSess mchSess = mchData->mchSess;
uint8_t message[MSG_MAX_LENGTH]  = { 0 };
uint8_t response[MSG_MAX_LENGTH] = { 0 };
size_t  responseSize, responseLen; /* expected, actual */
int     cos; /* change of state */
int     inst = mchSess->instance, i = 0;

	memcpy( message, RMCP_HEADER, sizeof( RMCP_HEADER ) );

	message[RMCP_MSG_CLASS_OFFSET] = RMCP_MSG_CLASS_ASF;

	memcpy( message + ASF_MSG_OFFSET, ASF_MSG, sizeof( ASF_MSG ) );

	while (1) {

		cos = 0;
		responseSize = RMCP_MSG_HEADER_LENGTH + ASF_MSG_HEADER_LENGTH + ASF_RPLY_PONG_PAYLOAD_LENGTH;

		ipmiMsgWriteRead( mchSess->name, message, sizeof( RMCP_HEADER ) + sizeof( ASF_MSG ), 
			response, &responseSize, RPLY_TIMEOUT_DEFAULT, &responseLen );

		if ( responseLen == 0 ) {

			if ( MCH_ONLN( mchStat[inst] ) ) {
				if ( MCH_DBG( mchStat[inst] ) )
					printf("%s mchPing now offline\n", mchSess->name);
				mchStatSet( inst, MCH_MASK_ONLN, 0 );
				cos = 1;
			}
		}
		else {
			if ( !MCH_ONLN( mchStat[inst] ) ) {
				if ( MCH_DBG( mchStat[inst] ) )
					printf("%s mchPing now online\n", mchSess->name);
				mchStatSet( inst, MCH_MASK_ONLN, MCH_MASK_ONLN );
				cos = 1;
			}
			if ( i > 60/PING_PERIOD ) {
				mchStatSet( inst, MCH_MASK_CNFG_CHK, MCH_MASK_CNFG_CHK );
				i = 0;
			}
		}

		if ( cos ) {
       			if ( drvMchStatScan )
       				scanIoRequest( drvMchStatScan );
		}

		epicsThreadSleep( PING_PERIOD );
		i++;
	}
}

/* Check if MCH configuration has changed or if
 * MCH is online and we have not read its configuration.
 * If either, get MCH data and store in our data structs
 */
int
mchCnfgChk(MchData mchData) {
int    inst   = mchData->mchSess->instance;

	mchStatSet( inst, MCH_MASK_CNFG_CHK, 0 );

	if ( MCH_INIT_NOT_DONE( mchStat[inst] ) )
		return mchCnfg( mchData );

	else if ( MCH_INIT_DONE( mchStat[inst] ) ) {
		if ( mchSdrRepTsDiff( mchData ) )
			return mchCnfg( mchData );
	}

	return 0;
}

/* Set all elements of 3D array to same integer value */
void
set3DArrayVals(int a, int b, int c, int arr[a][b][c], int val) {
int i, j, k;

	for ( i = 0; i < a; i++ ) {
		for ( j = 0; j < b; j++ ) {
			for ( k = 0; k < c; k++ ) {
				arr[i][j][k] = val;
			}
		}
	}
}
/* 
 * Modify MCH status mask. If record-related changes
 * were made, scan associated EPICS records
 */
void
mchStatSet(int inst, uint32_t clear, uint32_t set) {

	epicsMutexLock(   mchStatMtx[inst] );
	mchStat[inst] &= ~clear;
	mchStat[inst] |=  set;
	epicsMutexUnlock( mchStatMtx[inst] );

	if ( clear == MCH_MASK_INIT) {
		if ( drvMchInitScan )
			scanIoRequest( drvMchInitScan );
	}

	if ( clear == MCH_MASK_ONLN) {
		if ( drvMchStatScan )
			scanIoRequest( drvMchStatScan );
	}
}

static void
mchSetFeatures(MchData mchData)
{
MchSess  mchSess  = mchData->mchSess;
IpmiSess ipmiSess = mchData->ipmiSess;

	if ( mchSess->type == MCH_TYPE_VT )
		mchSess->timeout = ipmiSess->timeout = RPLY_TIMEOUT_SENDMSG_RPLY;
	else
		mchSess->timeout = ipmiSess->timeout = RPLY_TIMEOUT_DEFAULT;

	if ( mchSess->type == MCH_TYPE_VT )
		ipmiSess->features |= MCH_FEAT_SENDMSG_RPLY;		
}	

/* 
 * Use Manufacturer ID (from Get Device ID command)
 * to determine MCH type
 */
static int
mchIdentify(MchData mchData)
{
MchSess  mchSess = mchData->mchSess;
int      i;
uint8_t  response[MSG_MAX_LENGTH] = { 0 };
uint8_t  tmp[4] = { 0 }, vers;
uint32_t mf;

	if ( mchMsgGetDeviceId( mchData, response, 0, 0 ) ) {
		printf("mchIdentify: Error from Get Device ID command\n");
mchMsgCloseSess( mchSess, mchData->ipmiSess, response );
		return -1;
	}

	/* Extract Manufacturer ID */
	for ( i = 0; i < IPMI_RPLY_MANUF_ID_LENGTH ; i++)
		tmp[i] = response[IPMI_RPLY_IMSG2_GET_DEVICE_ID_MANUF_ID_OFFSET + i];

       	vers = response[IPMI_RPLY_IMSG2_GET_DEVICE_ID_IPMI_VERS_OFFSET];

	mf  = arrayToUint32( tmp  );
	mf  = IPMI_MANUF_ID( mf  );

	switch ( mf ) {

		default: 
			printf("mchIdentify: Unknown type of MCH, failed to match manuf ID 0x%08x\n", mf);
			mchSess->type = MCH_TYPE_UNKNOWN;
			return -1;

		case MCH_MANUF_ID_NAT:
			mchSess->type = MCH_TYPE_NAT;
			printf("Identified %s to be N.A.T. IPMI version %i.%i\n", mchSess->name, IPMI_VER_MSD( vers ), IPMI_VER_LSD( vers ));
			break;

		case MCH_MANUF_ID_VT:
			mchSess->type = MCH_TYPE_VT;
			printf("Identified %s to be Vadatech. IPMI version %i.%i\n", mchSess->name, IPMI_VER_MSD( vers ), IPMI_VER_LSD( vers ));
			break;
/*  Not yet supported     
		case MCH_MANUF_ID_DELL:
			mchSess->type = MCH_TYPE_DELL;
			printf("Identified %s to be Dell. IPMI version %i.%i\n", mchSess->name, IPMI_VER_MSD( vers ), IPMI_VER_LSD( vers ));
			break;
*/
		case MCH_MANUF_ID_SUPERMICRO:
			mchSess->type = MCH_TYPE_SUPERMICRO; // Was previously DELL; need to figure this out
			printf("Identified %s to be Supermicro. IPMI version %i.%i\n", mchSess->name, IPMI_VER_MSD( vers ), IPMI_VER_LSD( vers ));
			break;

	}

	mchSetFeatures( mchData );
        return 0;
}

static void
mchSensorFruGetInstance(int type, MchSys mchSys)
{
int     s = mchSys->sensCount;
uint8_t sensTypeInst[MAX_FRU_MGMT][MAX_SENSOR_TYPE] = { { 0 } };    /* Counts of sensor type per ID instance */
int     j, id, mgmtId;
Fru     fru  = 0;
Mgmt    mgmt = 0;
Sensor  sens;
uint8_t mgmtEntId, mgmtEntInst;

	/* Find FRU or management controller associated with this sensor, assign sensor an instance based on type */
	for ( j = 0; j < s; j++ ) {

		sens = &mchSys->sens[j];

		if ( -1 == (id = sens->fruId) )
			continue;

		/* Skip FRU 0 for MicroTCA because it is reserved logical entity with same entityId and entityInst as MCH 1 */
		if ( (id == 0) && MCH_IS_MICROTCA( type ) )
			continue;

		if ( id < MAX_FRU ) {

			fru = &mchSys->fru[id];

			/* If a real entity */
			if ( fru->sdr.recType ) {
				sens->instance = ++sensTypeInst[id][sens->sdr.sensType];
				if ( sens->instance > MAX_SENS_INST ) {
					printf("WARNING: FRU %i sensor type 0x%02x instance %i exceeds allowed instance %i\n", id, sens->sdr.sensType, sens->instance, MAX_SENS_INST);
					continue;
				}
				if ( !sens->unavail )
					mchSys->sensLkup[id][sens->sdr.sensType][sens->instance] = j;
			}
		}
		else {
			id = sens->fruId;
			mgmtId = id - MAX_FRU;
			mgmt        = &mchSys->mgmt[mgmtId];
			mgmtEntId   = mgmt->sdr.entityId;
			mgmtEntInst = mgmt->sdr.entityInst;

			if ( mchSys->sens->mgmtIndex == mgmtId ) {

				/* If a real entity */
				if ( mgmt->sdr.recType ) {
					sens->instance = ++sensTypeInst[id][sens->sdr.sensType];
					if ( sens->instance > MAX_SENS_INST ) {
						printf("WARNING: FRU %i sensor type 0x%02x instance %i exceeds allowed instance %i\n", id, sens->sdr.sensType, sens->instance, MAX_SENS_INST);
						continue;
					}
					if ( !sens->unavail )
						mchSys->sensLkup[id][sens->sdr.sensType][sens->instance] = j;
				}
			}
		}
	}

#ifdef DEBUG
int i;
printf("mchSensorFruGetInstance:\n");
for ( i = 0; i < s; i++ ) {
	sens = &mchSys->sens[i];
	if ( sens->fruId != -1 ) {
		fru  = &mchSys->fru[sens->fruId];
		printf("FRU sensor %s %i, type 0x%02x, inst %i, FRU entId 0x%02x, entInst 0x%02x, sens entId 0x%02x  entInst 0x%02x recType %i fruId %i fruId %i unavail %i\n",
			sens->sdr.str, sens->sdr.number, sens->sdr.sensType, sens->instance, fru->sdr.entityId, fru->sdr.entityInst, 
			sens->sdr.entityId, sens->sdr.entityInst, sens->sdr.recType, sens->fruId, sens->fruId, sens->unavail);
	}
	else if ( sens->mgmtIndex != -1 ) {
		mgmt  = &mchSys->mgmt[sens->mgmtIndex];
		printf("Mgmt sensor %s %i, type 0x%02x, inst %i, MGMT entId 0x%02x, entInst 0x%02x, sens entId 0x%02x  entInst 0x%02x recType %i unavail %i\n",
			sens->sdr.str, sens->sdr.number, sens->sdr.sensType, sens->instance, mgmt->sdr.entityId, mgmt->sdr.entityInst, 
			sens->sdr.entityId, sens->sdr.entityInst, sens->sdr.recType, sens->unavail);
	}
	else
		printf("Sensor %s %i, type 0x%02x, inst %i, entId 0x%02x  entInst 0x%02x recType %i, no corresponding FRU or MGTM unavail %i\n",
			sens->sdr.str, sens->sdr.number, sens->sdr.sensType, sens->instance, sens->sdr.entityId, sens->sdr.entityInst, sens->sdr.recType, sens->unavail);
}
#endif
}

/* Get info about MCH, shelf, FRUs, sensors
 * Caller must perform locking
 */
int
mchCnfg(MchData mchData) {
MchSess mchSess = mchData->mchSess;
MchSys  mchSys  = mchData->mchSys;
int inst = mchSess->instance;
int i;

	mchStatSet( inst, MCH_MASK_INIT, MCH_MASK_INIT_IN_PROGRESS );

	set3DArrayVals( MAX_FRU_MGMT, MAX_SENSOR_TYPE, MAX_SENS_INST, mchSys->sensLkup, -1 );

	mchSeqInit( mchData->ipmiSess );

	/* Turn on debug messages during initial messages with device */
	mchStatSet( inst, MCH_MASK_DBG, MCH_DBG_SET(MCH_DBG_LOW) );

	/* Initiate communication session with MCH */
	if ( mchCommStart( mchSess, mchData->ipmiSess ) ) {
		printf("Error initiating session with %s; cannot complete initialization\n",mchSess->name);
		goto bail;
	}

        /* Determine MCH type */
        if ( mchIdentify( mchData ) ) {
	       	printf("Failed to identify %s MCH type; cannot complete initialization\n",mchSess->name);	
		goto bail;
	}

	/* Turn off debug messages after initial messages with device */
	mchStatSet( inst, MCH_MASK_DBG, MCH_DBG_SET(MCH_DBG_OFF) );

       	/* Get SDR data */
       	if ( mchSdrGetDataAll( mchData ) ) {
       		printf("Failed to read %s SDR; cannot complete initialization\n",mchSess->name);
		goto bail;
       	}

	/* Get FRU data; errors are not fatal, but could cause missing FRU data */
	if ( mchFruGetDataAll( mchData ) )
       		printf("Warning: errors getting %s FRU data; some data may be missing\n",mchSess->name);

	/* Get Sensor/FRU association */
	for ( i = 0; i < mchSys->sensCount; i++ )
		mchSensorGetFru( mchSess->type, mchSys, i );

	mchSensorFruGetInstance( mchSess->type, mchSys );

	mchStatSet( inst, MCH_MASK_INIT, MCH_MASK_INIT_DONE );

	if ( drvMchFruScan )
		scanIoRequest( drvMchFruScan );

	return 0;

bail:

	mchStatSet( inst, MCH_MASK_INIT, MCH_MASK_INIT_NOT_DONE );
	return -1;
}


// !! need to make sure initial values do not lead to records sending messages i.e. assume not initialized if cannot establish session

/* 
 * MCH initialization. Called before iocInit.
 *
 * Initial release requires MCH/shelf to be on-line
 * and populated and creates new epics records script.  
 * Later release should derive info from previously
 * created script.
 */
void
mchInit(const char *name)
{
MchDev   mch     = 0; /* Device support data structure */
MchData  mchData = 0; /* MCH-specific info */
MchSess  mchSess = 0;
IpmiSess ipmiSess = 0;
MchSys   mchSys  = 0;
char     taskName[MAX_NAME_LENGTH+10];
int      inst;

	/* Allocate memory for MCH data structures */
	if ( ! (mchData = calloc( 1, sizeof( *mchData ))) )
		cantProceed("FATAL ERROR: No memory for MchData structure for %s\n", name);

	if ( ! (mchSess = calloc( 1, sizeof( *mchSess ))) )
		cantProceed("FATAL ERROR: No memory for MchSess structure for %s\n", name);

	if ( ! (ipmiSess = calloc( 1, sizeof( *ipmiSess ))) )
		cantProceed("FATAL ERROR: No memory for IpmiSess structure for %s\n", name);

	if ( ! (mchSys = calloc( 1, sizeof( *mchSys ))) )
		cantProceed("FATAL ERROR: No memory for MchSys structure for %s\n", name);

	if ( !(mchSys->fru = calloc( MAX_FRU , sizeof(FruRec) ) ) )
		cantProceed("FATAL ERROR: No memory for FRU data for %s for %s\n", name);

	if ( !(mchSys->mgmt = calloc( MAX_MGMT , sizeof(MgmtRec) ) ) )
		cantProceed("FATAL ERROR: No memory for Management Controller data for %s for %s\n", name);

	ipmiSess->wrf = (IpmiWriteReadHelper)mchMsgWriteReadHelper;

	inst = mchSess->instance = mchCounter++;

	mchStatMtx[inst] = epicsMutexMustCreate(); /* Used for global mchStat mask */

       	/* Allocate memory for MCH device support structure */
       	if ( ! (mch = calloc( 1, sizeof( *mch ) )) )
       		cantProceed("FATAL ERROR: No memory for MchDev structure\n");

       	if ( ! (mch = devMchRegister( name )) )
       		printf("FATAL ERROR: Unable to register MCH %s with device support\n", name);

	mchData->ipmiSess = ipmiSess;
	mchData->mchSess = mchSess;
	mchData->mchSys  = mchSys;

       	strncpy( mchSess->name, mch->name, MAX_NAME_LENGTH );
       	strncpy( mchSys->name,  mch->name, MAX_NAME_LENGTH ); // okay to remove this and from drvMch.h?
       	mch->udata = mchData;

       	mchSess->timeout = ipmiSess->timeout = RPLY_TIMEOUT_SENDMSG_RPLY; /* Default, until determine type */
	mchSess->session = 1;   /* Default: enable session with MCH */

	/* Start task to periodically ping MCH */
	sprintf( taskName, "%s-PING", mch->name ); 
	mchSess->pingThreadId = epicsThreadMustCreate( taskName, epicsThreadPriorityMedium, epicsThreadGetStackSize(epicsThreadStackMedium), mchPing, mch );

	/* Wait for updated status from ping thread */ 
	epicsThreadSleep( 2 ); 

	if (  MCH_ONLN( mchStat[inst] ) ) {
		epicsMutexLock( mch->mutex );
		mchCnfg( mchData );
		epicsMutexUnlock( mch->mutex );
	}
	else
		printf("No response from %s; cannot complete initialization\n",mch->name);
}

static long
drvMchReport(int level)
{
	printf("MicroTCA MCH communication driver support\n");
	return 0;
}

static long
drvMchInit(void)
{
	return 0;
}

static struct {
	long number;
	DRVSUPFUN report;
	DRVSUPFUN init;
} drvMch={
	2,
	drvMchReport,
	drvMchInit
};

epicsExportAddress(drvet,drvMch);


/* 
 * IOC shell command registration
 */
static const iocshArg mchInitArg0        = { "port name",iocshArgString};
static const iocshArg *mchInitArgs[1]    = { &mchInitArg0 };
static const iocshFuncDef mchInitFuncDef = { "mchInit", 1, mchInitArgs };

static void mchInitCallFunc(const iocshArgBuf *args)
{
	mchInit(args[0].sval);
}

static void
drvMchRegisterCommands(void)
{
	static int firstTime = 1;
	if ( firstTime ) {
		iocshRegister(&mchInitFuncDef, mchInitCallFunc);
		firstTime = 0;
	}
}

epicsExportRegistrar(drvMchRegisterCommands);
