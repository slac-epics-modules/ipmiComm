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

#undef DEBUG 

#define PING_PERIOD  5

int mchCounter = 0;

epicsMutexId mchStatMtx[MAX_MCH];
uint32_t     mchStat[MAX_MCH] = { 0 };

int  mchSdrGetDataAll(MchData mchData);
int  mchFruGetDataAll(MchData mchData);
int  mchCnfg(MchData mchData);

void
mchSeqInit(IpmiSess ipmiSess)
{
int i;
	/* Initialize IPMI sequence */
	ipmiSess->seq = 0;

        /* Initialize our stored sequence number for messages from MCH */
	ipmiSess->seqRply[0] = IPMI_MSG_HEADER_SEQ_INITIAL;
        for ( i = 1; i < IPMI_RPLY_SEQ_LENGTH - 1 ; i++)
                ipmiSess->seqRply[i] = 0;	
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
int 
mchCommStart(MchSess mchSess, IpmiSess ipmiSess)
{	
uint8_t response[MSG_MAX_LENGTH] = { 0 };
int     i;

	if ( MCH_DBG( mchStat[mchSess->instance] ) )
		printf("%s Connecting...\n", mchSess->name);

	mchSeqInit( ipmiSess );

	if ( ipmiMsgGetChanAuth( mchSess, ipmiSess, response ) )
		return -1;	

	if ( ipmiMsgGetSess( mchSess, ipmiSess, response ) )
		return -1;

        /* Extract temporary session ID */
        for ( i = 0; i < IPMI_RPLY_TEMP_ID_LENGTH ; i++)
                ipmiSess->id[i] = response[IPMI_RPLY_TEMP_ID_OFFSET + i];

        /* Extract challenge string */
        for ( i = 0; i < IPMI_RPLY_STR_LENGTH ; i++)
                ipmiSess->str[i] = response[IPMI_RPLY_STR_OFFSET + i];

	if ( ipmiMsgActSess( mchSess, ipmiSess, response ) )
		return -1;

        /* Extract session ID */
        for ( i = 0; i < IPMI_RPLY_ID_LENGTH ; i++)
                ipmiSess->id[i] = response[IPMI_RPLY_ID_OFFSET + i];

        /* Extract initial sequence number for messages to MCH */
        for ( i = 0; i < IPMI_RPLY_INIT_SEND_SEQ_LENGTH ; i++)
                ipmiSess->seqSend[i] = response[IPMI_RPLY_INIT_SEND_SEQ_OFFSET + i];

	if ( ipmiMsgSetPriv( mchSess, ipmiSess, response, IPMI_MSG_PRIV_LEVEL_ADMIN ) )
		return -1;

	return 0;
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

	for ( i = 0; i < IPMI_MSG_HDR_SEQ_LENGTH ; i++)
	        ipmiSess->seqSend[i] = 0;

	for ( i = 0; i < IPMI_MSG_HDR_ID_LENGTH ; i++)
		ipmiSess->id[i] = 0;

	if ( MCH_ONLN( mchStat[mchSess->instance] ) )
		rval = mchCommStart( mchSess, ipmiSess );

	return rval;
}

/* Copy FRU area field to data structure
 *
 * Caller must perform locking.
 *
 *   RETURNS:
 *           0 on success
 *          -1 if no data to store
 */
int
mchFruFieldGet(FruField field, uint8_t *raw, unsigned *offset)
{
int i;
        if ( ( field->length = IPMI_DATA_LENGTH( raw[*offset] ) ) ) {

                field->type = IPMI_DATA_TYPE( raw[*offset] );
                (*offset)++;

                if ( !( field->data = calloc( field->length, 1 ) ) )
                        printf("No memory for FRU field data\n");
                else {
                        for ( i = 0; i < field->length; i++ )
                                field->data[i] = raw[*offset + i];

                        *offset += field->length;

                        return 0;
                }
        }
        return -1;
}

/* 
 * Copy FRU product area data to product structure
 *
 * Caller must perform locking.
 */
void
mchFruProdDataGet(FruProd prod, uint8_t *raw, unsigned *offset)
{
	if ( 0 != (*offset = prod->offset = 8*raw[FRU_DATA_COMMON_HEADER_OFFSET + FRU_DATA_COMMON_HEADER_PROD_AREA_OFFSET] ) ) {

		prod->lang = raw[*offset + FRU_DATA_BOARD_AREA_LANG_OFFSET];

		*offset += FRU_DATA_PROD_AREA_MANUF_LENGTH_OFFSET;

		if ( mchFruFieldGet( &(prod->manuf),   raw, offset ) )
			(*offset)++;
		if ( mchFruFieldGet( &(prod->prod),    raw, offset ) )
			(*offset)++;
		if ( mchFruFieldGet( &(prod->part),    raw, offset ) )
			(*offset)++;
		if ( mchFruFieldGet( &(prod->version), raw, offset ) )
			(*offset)++;
		if ( mchFruFieldGet( &(prod->sn),      raw, offset ) )
			(*offset)++;
	}
}

/* 
 * Copy FRU board area data to board structure
 *
 * Caller must perform locking.
 */
void
mchFruBoardDataGet(FruBoard board, uint8_t *raw, unsigned *offset)
{
	if ( 0 != (*offset = board->offset = 8*raw[FRU_DATA_COMMON_HEADER_OFFSET + FRU_DATA_COMMON_HEADER_BOARD_AREA_OFFSET] ) ) {

		board->lang = raw[*offset + FRU_DATA_BOARD_AREA_LANG_OFFSET];

		*offset += FRU_DATA_BOARD_AREA_MANUF_LENGTH_OFFSET;

		if ( mchFruFieldGet( &(board->manuf), raw, offset ) )
			(*offset)++;
		if ( mchFruFieldGet( &(board->prod),  raw, offset ) )
			(*offset)++;
		if ( mchFruFieldGet( &(board->sn),    raw, offset ) )
			(*offset)++;
		if ( mchFruFieldGet( &(board->part),  raw, offset ) )
			(*offset)++;	       
	}
}

/* 
 * Get data for one FRU
 * Read FRU inventory info. If error in response
 * or FRU data area size is zero, return. Else read FRU data
 * and call mchFru*DataGet to store in FRU structure.
 *
 * Caller must perform locking.
 */
int
mchFruDataGet(MchData mchData, Fru fru, uint8_t id) 
{
MchSess    mchSess = mchData->mchSess;
int        inst    = mchSess->instance;
uint8_t    response[MSG_MAX_LENGTH] = { 0 };
uint8_t   *raw; 
int        i;
uint16_t   sizeInt;  /* Size of FRU data area in bytes */
unsigned   nread;    /* Number of FRU data reads */
unsigned   offset;   /* Offset into FRU data */
int        offs = ( MCH_IS_NAT( mchSess->type ) ) ? IPMI_RPLY_OFFSET_NAT : 0; /* Offset into reply message */

	/* Get FRU Inventory Info */
	if ( mchMsgGetFruInfo( mchData, response, id ) )
		return -1;

	fru->size[0] = response[IPMI_RPLY_FRU_AREA_SIZE_LSB_OFFSET + offs];
	fru->size[1] = response[IPMI_RPLY_FRU_AREA_SIZE_MSB_OFFSET + offs];
	fru->access  = response[IPMI_RPLY_FRU_AREA_ACCESS_OFFSET   + offs];  

	if ( 0 == (sizeInt = arrayToUint16( fru->size )) )
		return 0;


	if ( MCH_DBG( mchStat[inst] ) )
		printf("%s mchFruDataGet: FRU %i inventory info size %i\n", mchSess->name, id, sizeInt);

	if ( !(raw = calloc( sizeInt ,1 ) ) ) {
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
		if ( (IPMI_COMP_CODE_REQUESTED_DATA == mchMsgReadFru( mchData, response, id, fru->readOffset, MSG_FRU_DATA_READ_SIZE )) )
			break;
		memcpy( raw + i*MSG_FRU_DATA_READ_SIZE, response + IPMI_RPLY_FRU_DATA_READ_OFFSET + offs, MSG_FRU_DATA_READ_SIZE );
		incr2Uint8Array( fru->readOffset, MSG_FRU_DATA_READ_SIZE );
	}

	if ( MCH_DBG( mchStat[inst] ) > 2 ) {
		printf("%s FRU %i raw data, size %i: \n", mchSess->name, id, sizeInt);
		for ( i = 0; i < sizeInt; i++)
			printf("%u ",raw[i]);
		printf("\n");
	}
	/* Add chassis data get */
	mchFruBoardDataGet( &(fru->board), raw, &offset );
	mchFruProdDataGet(  &(fru->prod) , raw, &offset );

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
int
mchFruGetDataAll(MchData mchData)
{
MchSess mchSess = mchData->mchSess;
MchSys  mchSys  = mchData->mchSys;
uint8_t response[MSG_MAX_LENGTH] = { 0 };
uint8_t i;
Fru fru;
int offs = ( MCH_IS_NAT( mchSess->type ) ) ? IPMI_RPLY_2ND_BRIDGED_OFFSET_NAT : 0;;
int rval = 0;

	for ( i = 0; i < MAX_FRU ; i++ ) {

	       	fru = &mchSys->fru[i];
	
		if ( fru->sdr.entityInst ) {

			if ( mchFruDataGet( mchData, fru , i ) )
				rval = -1;

			if ( (i >= UTCA_FRU_TYPE_CU_MIN) && (i <= UTCA_FRU_TYPE_CU_MAX) ) {

				if ( mchMsgGetFanPropHelper( mchData, response, fru->sdr.fruId ) )
					rval = -1;
				fru->fanMin  = response[IPMI_RPLY_GET_FAN_PROP_MIN_OFFSET  + offs];
				fru->fanMax  = response[IPMI_RPLY_GET_FAN_PROP_MAX_OFFSET  + offs];
				fru->fanNom  = response[IPMI_RPLY_GET_FAN_PROP_NOM_OFFSET  + offs];
				fru->fanProp = response[IPMI_RPLY_GET_FAN_PROP_PROP_OFFSET + offs];
			}
		}			
	}
	if ( MCH_DBG( mchStat[mchSess->instance] ) > 1 ) {
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
 *  the associated FRU and do the same.
 *
 * -For all hot-swap sensors, store sensor index in FRU structure
 *
 * Note: these indices refer to our arrays of Sensors and FRUs--
 * they do not refer to the IPMI sensor number or IPMI SDR Record ID
 *
 *
 * Caller must perform locking.
 */
void
mchSensorGetFru(int type, MchSys mchSys, uint8_t index)
{
int  i, natid;
char buff[9];
int  id     = -1;
Sensor sens = &mchSys->sens[index];
SdrFull sdr = &sens->sdr;
char test[20];

	/* NAT associates hotswap aka M-state sensors with carrier manager but
         * Vadatech associates them with the actual FRU described by the sensor.
	 * For NAT, parse sensor description to get associated FRU number
         */
	if ( (type == MCH_TYPE_NAT) && (sdr->sensType == SENSOR_TYPE_HOTSWAP) ) {
		if ( 2 != sscanf( (const char *)(sdr->str), "HS %d %s", &natid, buff ) ) {
			printf("sscanf parse error %s\n", sdr->str);
			return;
		}
		sens->fruIndex = natid;
		return;
	}
		
	/* First loop through FRUs */
	for ( i = 0; i < MAX_FRU; i++ ) {
		if ( (sens->sdr.entityId == mchSys->fru[i].sdr.entityId) && (sens->sdr.entityInst == mchSys->fru[i].sdr.entityInst ) ) {
			id = i;
			break;
		}
	}

	/* Next find non-FRU entities */
	if ( id == -1 ) {
		if ( sens->sdr.entityId == VT_ENTITY_ID_AMC )
			id = UTCA_FRU_TYPE_AMC_MIN + sens->sdr.entityInst - 0x60 - 1;
		else if ( sens->sdr.entityId == VT_ENTITY_ID_RTM )
		       	id = UTCA_FRU_TYPE_RTM_MIN + sens->sdr.entityInst - 0x60 - 1;
	}

	sens->fruIndex = id;
}

/* Caller must perform locking */
int
mchSdrRepGetInfoMsg(MchData mchData, uint8_t *response) {

	return mchMsgGetSdrRepInfo( mchData, response );

}

/* Caller must perform locking */

void
mchSdrRepGetTs(uint8_t *response, uint32_t *addTs, uint32_t *delTs, int offs) {

       	*addTs = ntohl( *(uint32_t *)(response + IPMI_RPLY_SDRREP_ADD_TS_OFFSET + offs) );
       	*delTs = ntohl( *(uint32_t *)(response + IPMI_RPLY_SDRREP_DEL_TS_OFFSET + offs) );
}

/* Compare SDR repository timestamps to detect changes
 *
 * Caller must perform locking
 */
int
mchSdrRepTsDiff(MchData mchData) {
MchSess   mchSess = mchData->mchSess;
MchSys    mchSys  = mchData->mchSys;
uint32_t  add, del;
uint32_t *addTs = &mchSys->sdrRep.addTs, *delTs = &mchSys->sdrRep.delTs;
uint8_t   buff[MSG_MAX_LENGTH] = { 0 };
int offs = ( MCH_IS_NAT( mchSess->type ) ) ? IPMI_RPLY_OFFSET_NAT : 0;

	if ( mchSdrRepGetInfoMsg( mchData, buff ) )
		return 0;

	mchSdrRepGetTs( buff, &add, &del, offs);


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
int
mchSdrRepGetInfo(MchData mchData)
{
MchSess   mchSess = mchData->mchSess;
MchSys    mchSys  = mchData->mchSys;
uint8_t response[MSG_MAX_LENGTH] = { 0 };
uint8_t flags;
int     offs = ( MCH_IS_NAT( mchSess->type ) ) ? IPMI_RPLY_OFFSET_NAT : 0;

	if ( mchSdrRepGetInfoMsg( mchData, response ) )
		return -1;

	mchSys->sdrRep.ver     = response[IPMI_RPLY_SDRREP_VER_OFFSET     + offs];
	mchSys->sdrRep.size[0] = response[IPMI_RPLY_SDRREP_CNT_LSB_OFFSET + offs];
	mchSys->sdrRep.size[1] = response[IPMI_RPLY_SDRREP_CNT_MSB_OFFSET + offs];

	mchSdrRepGetTs( response, &mchSys->sdrRep.addTs, &mchSys->sdrRep.delTs, offs );

	mchMsgGetDevSdrInfo( mchData, response, 1 );

	if ( response[IPMI_RPLY_COMPLETION_CODE_OFFSET + offs] )
		return 0; /* We don't currently support dev sdr, so don't return error */

       	mchSys->sdrRep.devSdrSize = response[IPMI_RPLY_DEV_SDR_CNT_OFFSET + offs];
	flags = response[IPMI_RPLY_DEV_SDR_FLAGS_OFFSET + offs];
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
void
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
void
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
void
mchSdrFullSens(SdrFull sdr, uint8_t *raw, int type)
{
int n, i, l = 0;

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

		l = IPMI_DATA_LENGTH( sdr->strLength );
		for ( i = 0; i < l; i++ )
			sdr->str[i] = raw[SDR_STR_OFFSET + i];
		sdr->str[i+1] = '\0';
       	}     

#ifdef DEBUG
printf("mchSdrFullSens: owner 0x%02x lun %i sdr number %i type 0x%02x, str l %i\n", sdr->owner, sdr->lun, sdr->number, sdr->sensType, l);
if ( l > 0 )
printf("string %s\n", sdr->str);
#endif

}

/*
 * Read sensor to save sensor reading response length; it varies
 * by sensor and this prevents read timeouts later.
 * Get sensor thresholds and store them for use by device support.
 * (Assuming thresholds do not change)
 *
 * Caller must perform locking.
 */
void
mchGetSensorInfo(MchData mchData, Sensor sens)
{
uint8_t  response[MSG_MAX_LENGTH] = { 0 };
size_t   responseSize;
int      offs = ( MCH_IS_NAT( mchData->mchSess->type ) ) ? IPMI_RPLY_OFFSET_NAT : 0; /* Offset into reply message */


	responseSize = sens->readMsgLength = ( MCH_IS_NAT( mchData->mchSess->type ) ) ? IPMI_RPLY_SENSOR_READ_MAX_LENGTH_NAT : IPMI_RPLY_SENSOR_READ_MAX_LENGTH_VT;

	if( !(MCH_ONLN( mchStat[mchData->mchSess->instance] )) ) {
		if ( MCH_DBG( mchStat[mchData->mchSess->instance] ) )
			printf("%s mchGetSensorInfo: MCH offline; aborting\n", mchData->mchSess->name);
		return;
	}

	/* Do not check status because timeout is allowed here */
	mchMsgReadSensor( mchData, response, sens->sdr.number, (sens->sdr.lun & 0x3) , &responseSize );	
	sens->readMsgLength = responseSize;

	responseSize = ( MCH_IS_NAT( mchData->mchSess->type ) ) ? IPMI_RPLY_GET_SENSOR_THRESH_LENGTH_NAT : IPMI_RPLY_GET_SENSOR_THRESH_LENGTH_VT;

/* not ready for prime time; seems MCH does not respond if discrete sensor
	if (  mchMsgGetSensorThresholds( mchData, response, sens->sdr.number, (sens->sdr.lun & 0x3), &responseSize ) ) {
		if ( MCH_DBG( mchStat[mchData->mchSess->instance] ) )
			printf("%s mchGetSensorInfo: mchMsgGetSensorThresholds error, assume no thresholds are readable\n", mchData->mchSess->name);
		sens->tmask = 0;
		return;
	}
*/

	sens->tmask = response[IPMI_RPLY_SENSOR_THRESH_MASK_OFFSET + offs];
	sens->tlnc  = response[IPMI_RPLY_SENSOR_THRESH_LNC_OFFSET  + offs];
	sens->tlc   = response[IPMI_RPLY_SENSOR_THRESH_LC_OFFSET   + offs];
	sens->tlnr  = response[IPMI_RPLY_SENSOR_THRESH_LNR_OFFSET  + offs];
	sens->tunc  = response[IPMI_RPLY_SENSOR_THRESH_UNC_OFFSET  + offs];
	sens->tuc   = response[IPMI_RPLY_SENSOR_THRESH_UC_OFFSET   + offs];
	sens->tunr  = response[IPMI_RPLY_SENSOR_THRESH_UNR_OFFSET  + offs];
}

/*
 * Read sensor data records. Call ipmiMsgGetSdr twice per record;
 * once to get record length, then to read record. This prevents timeouts,
 * saving much delay. 
 *
 * Caller must perform locking.
 */				  
int				  
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
uint8_t  type   = 0, addr = 0;
uint8_t *raw    = 0;
int      err;
int      size; /* SDR record size (after header) */
int      i, iFull = 0, iFru = 0, iMgmt = 0, fruId;
size_t   responseSize;
int      rval = -1;
int      offs = ( MCH_IS_NAT( mchSess->type ) ) ? IPMI_RPLY_OFFSET_NAT : 0;
int      base = ( MCH_IS_NAT( mchSess->type ) ) ? IPMI_RPLY_GET_SDR_BASE_LENGTH_NAT : IPMI_RPLY_GET_SDR_BASE_LENGTH_VT;
int      inst = mchSess->instance;
Fru      fru;
SdrMgmt  mgmt;

	if ( mchSdrRepGetInfo( mchData ) )
		return rval;

	sdrCount = arrayToUint16( mchSys->sdrRep.size );

	if ( !(raw = calloc( sdrCount, SDR_MAX_LENGTH ) ) ) {
		printf("mchSdrGetDataAll: No memory for raw SDR data for %s\n", mchSess->name);
		goto bail;
	}
       	for ( i = 0; i < sdrCount; i++) {
		       
		err = 0;
		/* readSize = 5 because 5th byte is remaining record length; 0xFF reads entire record */
       		if ( mchMsgGetSdr( mchData, response, id, res, offset, 5, 0, 0 ) ) {

			i--;
			if ( err++ > 5 ) {
			        printf("mchSdrGetDataAll: too many errors reading SDR for %s", mchSess->name);
				goto bail;
			}
		}
		else {
			size = response[base - 1];
			if ( mchMsgGetSdr( mchData, response, id, res, offset, 0xFF, 0, size ) ) {
				i--;
				if ( err++ > 5 ) {
					printf("mchSdrGetDataAll: too many errors reading SDR for %s", mchSess->name);
					goto bail;
				}
			}

			else {
				memcpy( raw + (i*SDR_MAX_LENGTH), response + IPMI_RPLY_GET_SDR_DATA_OFFSET + offs, SDR_MAX_LENGTH );

				switch ( response[IPMI_RPLY_GET_SDR_DATA_OFFSET + offs + SDR_REC_TYPE_OFFSET] ) {

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
						mchSys->mgmtCount++;
						break;
				}
				id[0]  = response[IPMI_RPLY_GET_SDR_NEXT_ID_LSB_OFFSET + offs];
				id[1]  = response[IPMI_RPLY_GET_SDR_NEXT_ID_MSB_OFFSET + offs];

				if ( arrayToUint16( id ) == 0xFFFF ) // last record in SDR
					break;
			}
		}
       	}

	sdrCount = i;

	if ( !(mchSys->sens = calloc( mchSys->sensCount, sizeof(*sens) ) ) ) {
		printf("mchSdrGetDataAll: No memory for sensor data for %s\n", mchSess->name);
		goto bail;
	}

	if ( !(mchSys->mgmt = calloc( mchSys->mgmtCount, sizeof(*mgmt) ) ) ) {
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
			mchSys->fru[iFru].instance = 0;     /* Initialize instance to 0 */
			iFru++;
		}
	        else if ( type == SDR_TYPE_MGMT_CTRL_DEV ) {
			mchSdrMgmtCtrlDev( &(mchSys->mgmt[iMgmt]), raw + i*SDR_MAX_LENGTH );
			mchSys->mgmt[iMgmt].instance = 0;     /* Initialize instance to 0 */
			iMgmt++;
		}
	}

	if ( MCH_DBG( mchStat[inst] ) > 1 ) {
		printf("%s mchSdrGetDataAll Sumary:\n", mchSess->name);
		for ( i = 0; i < iFull; i++ ) {
			sens = &mchSys->sens[i];
			printf("SDR %i, %s entity ID 0x%02x, entity inst 0x%02x, sensor number %i, sens type 0x%02x, owner 0x%02x, LUN %i, RexpBexp %i, M %i, MTol %i, B %i, BAcc %i\n", i, sens->sdr.str, sens->sdr.entityId, sens->sdr.entityInst, sens->sdr.number, sens->sdr.sensType, sens->sdr.owner, sens->sdr.lun, sens->sdr.RexpBexp, sens->sdr.M, sens->sdr.MTol, sens->sdr.B, sens->sdr.BAcc);
		}
		for ( i = 0; i < MAX_FRU; i++ ) {
			fru = &mchSys->fru[i];
			if ( fru->sdr.entityInst )
				printf("FRU %i, entity ID 0x%02x, entity inst 0x%02x, FRU id %i, %s\n", 
				i, fru->sdr.entityId, fru->sdr.entityInst, fru->sdr.fruId, fru->sdr.str);
		}
		for ( i = 0; i < iMgmt; i++ ) {
			mgmt = &mchSys->mgmt[i];
       			printf("Mgmt Ctrl %i, entity ID 0x%02x, entity inst 0x%02x, %s\n", 
				i, mgmt->entityId, mgmt->entityInst, mgmt->str);
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
 *  during the message write We call ipmiMsgWriteRead directly, 
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
		responseSize = IPMI_RPLY_PONG_LENGTH;

		ipmiMsgWriteRead( mchSess->name, message, sizeof( RMCP_HEADER ) + sizeof( ASF_MSG ), response, &responseSize, RPLY_TIMEOUT_NAT, &responseLen );

		if ( responseSize == 0 ) {

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
	
/* 
 * Use Manufacturer ID (from Get Device ID command)
 * to determine MCH type
 */
int
mchIdentify(MchData mchData)
{
MchSess  mchSess = mchData->mchSess;
int      i;
uint8_t  response[MSG_MAX_LENGTH] = { 0 };
uint8_t  tmpvt[4] = { 0 }, tmpnat[4] = { 0 }, vers;
uint32_t mfvt, mfnat;

	if ( mchMsgGetDeviceId( mchData, response, IPMI_MSG_ADDR_CM ) ) {
                printf("mchIdentify: Error from Get Device ID command\n");
                return -1;
	}

        /* Extract Manufacturer ID */
        for ( i = 0; i < IPMI_RPLY_MANUF_ID_LENGTH ; i++)
		tmpvt[i] = response[IPMI_RPLY_MANUF_ID_OFFSET + i];
        for ( i = 0; i < IPMI_RPLY_MANUF_ID_LENGTH ; i++)
		tmpnat[i] = response[IPMI_RPLY_MANUF_ID_OFFSET + i + IPMI_RPLY_OFFSET_NAT];

	mfvt  = arrayToUint32( tmpvt  );
	mfnat = arrayToUint32( tmpnat );

	mfvt  = IPMI_MANUF_ID( mfvt  );
	mfnat = IPMI_MANUF_ID( mfnat );

        if ( mfvt == MCH_MANUF_ID_VT ) {
                mchSess->type = MCH_TYPE_VT;
		mchSess->timeout = RPLY_TIMEOUT_VT;
		vers = response[IPMI_RPLY_IPMI_VERS_OFFSET];
		printf("Identified %s to be Vadatech. IPMI version %i.%i\n", mchSess->name, IPMI_VER_MSD( vers ), IPMI_VER_LSD( vers ));
        }
        else if ( mfnat == MCH_MANUF_ID_NAT ) {
                mchSess->type = MCH_TYPE_NAT;
		vers = response[IPMI_RPLY_IPMI_VERS_OFFSET + IPMI_RPLY_OFFSET_NAT];
		printf("Identified %s to be N.A.T. IPMI version %i.%i\n", mchSess->name, IPMI_VER_MSD( vers ), IPMI_VER_LSD( vers ));
        }
        else {
                printf("mchIdentify: Unknown type of MCH, Manufacturer ID 0x%08x or 0x%08x?\n", mfvt, mfnat);
                mchSess->type = MCH_TYPE_UNKNOWN;
                return -1;
        }

        return 0;
}

void
mchSensorFruGetInstance(MchSys mchSys)
{
int     f = MAX_FRU;
int     s = mchSys->sensCount;
uint8_t fruEntId[f];                         /* Entity IDs */
uint8_t fruEntInst[f];                       /* Counts of each ID */
uint8_t sensTypeInst[f][MAX_SENSOR_TYPE];    /* Counts of sensor type per ID instance */
int i, j, found, id;
Fru     fru;
Sensor  sens;

	memset( fruEntId,     0, f        );
	memset( fruEntInst,   0, f        );
	memset( sensTypeInst, 0, f*MAX_SENSOR_TYPE );

	for ( i = 0; i < f; i++ ) {

		fru   = &mchSys->fru[i];
                id    = fru->sdr.fruId;
		found = 0;

		/* Find sensors associated with this FRU, assign each an instance based on sensor type */
		for ( j = 0; j < s; j++ ) {

			sens = &mchSys->sens[j];

			if ( mchSys->fru[sens->fruIndex].sdr.fruId == id ) {

				/* If a real entity */
				if ( fru->sdr.entityInst ) {
					sens->instance = ++sensTypeInst[i][sens->sdr.sensType]; 
					mchSys->sensLkup[id][sens->sdr.sensType][sens->instance] = j;
				}
			}
		}
	}

#ifdef DEBUG
printf("mchSensorFruGetInstance:\n");
for ( i = 0; i < mchSys->sensCount; i++ ) {
	sens = &mchSys->sens[i];
	int fruIndex = sens->fruIndex;
	fru  = &mchSys->fru[fruIndex];
	printf("Sensor %i, type %02x, inst %i, FRU entId %02x, entInst %02x, sens entId %02x  entInst %02x recType %i\n",sens->sdr.number, sens->sdr.sensType, sens->instance, fru->sdr.entityId, fru->sdr.entityInst, sens->sdr.entityId, sens->sdr.entityInst, sens->sdr.recType);
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

	set3DArrayVals( MAX_FRU, MAX_SENSOR_TYPE, MAX_SENS, mchSys->sensLkup, -1 );

	mchSeqInit( mchData->ipmiSess );

        /* Determine MCH type */
        if ( mchIdentify( mchData ) ) {
	       	printf("Failed to identify %s MCH type; cannot complete initialization\n",mchSess->name);	
		mchStatSet( inst, MCH_MASK_INIT, MCH_MASK_INIT_NOT_DONE );
		return -1;
	}

	/* Initiate communication session with MCH */
	if ( mchCommStart( mchSess, mchData->ipmiSess ) ) {
		printf("Error initiating session with %s; cannot complete initialization\n",mchSess->name);
		mchStatSet( inst, MCH_MASK_INIT, MCH_MASK_INIT_NOT_DONE );
		return -1;
	}

       	/* Get SDR data */
       	if ( mchSdrGetDataAll( mchData ) ) {
       		printf("Failed to read %s SDR; cannot complete initialization\n",mchSess->name);
       		mchStatSet( inst, MCH_MASK_INIT, MCH_MASK_INIT_NOT_DONE );
       		return -1;
       	}

	/* Get FRU data; errors are not fatal, but could cause missing FRU data */
	if ( mchFruGetDataAll( mchData ) )
       		printf("Warning: errors getting %s FRU data; some data may be missing\n",mchSess->name);

	/* Get Sensor/FRU association */
	for ( i = 0; i < mchSys->sensCount; i++ )
		mchSensorGetFru( mchSess->type, mchSys, i );

	mchSensorFruGetInstance( mchSys );

	mchStatSet( inst, MCH_MASK_INIT, MCH_MASK_INIT_DONE );

	if ( drvMchFruScan )
		scanIoRequest( drvMchFruScan );

	return 0;
}

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
MchDev  mch     = 0; /* Device support data structure */
MchData mchData = 0; /* MCH-specific info */
MchSess mchSess = 0;
IpmiSess ipmiSess = 0;
MchSys  mchSys  = 0;
char    taskName[50];
int     inst;

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

       	mchSess->name = mch->name;
       	mchSys->name  = mch->name;
       	mch->udata = mchData;

       	mchSess->timeout = RPLY_TIMEOUT_NAT; /* Default */
	mchSess->session = 1;   /* Default: enable session with MCH */

	/* Start task to periodically ping MCH */
	sprintf( taskName, "%s-PING", mch->name ); 
	mchSess->pingThreadId = epicsThreadMustCreate( taskName, epicsThreadPriorityMedium, epicsThreadGetStackSize(epicsThreadStackMedium), mchPing, mch );

	/* Wait for updated status from ping thread */ 
	epicsThreadSleep( PING_PERIOD ); 

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
