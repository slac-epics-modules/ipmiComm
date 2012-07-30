#include <epicsExport.h>
#include <epicsInterrupt.h>
#include <drvSup.h>
#include <iocsh.h>
#include <epicsExport.h>
#include <errlog.h>
#include <epicsMutex.h>
#include <cantProceed.h>
#include <asynPortDriver.h>
#include <asynOctetSyncIO.h>
#include <epicsThread.h>
#include <dbScan.h>

#include <stdint.h>
#include <stdio.h>   /* fopen, etc. */
#include <string.h>
#include <math.h>    /* floor, pow */
#include <ctype.h>   /* toupper */

#include <drvMch.h>
#include <drvMchUtil.h>
#include <ipmiDef.h>
#include <ipmiMsg.h>

#define MCH_STAT_NORESPONSE 0
#define MCH_STAT_OK         1

#define MAX_MCH      255

#define PING_PERIOD  5

#undef DEBUG

int mchCounter = 0;
int mchInitDone[MAX_MCH] = { 0 };
int mchIsAlive[MAX_MCH]  = { 0 };

/* Start communication session with MCH
 * Multi-step handshaking sequence
 *
 * RETURN:
 *         0 on success
 *         non-zero on failure
 */		
int 
mchCommStart(MchData mchData)
{	
uint8_t response[MSG_MAX_LENGTH];
int     i;

/*	mchData->pasynUser = pasynUser;*/
	
	ipmiMsgGetChanAuth( mchData, response );

	ipmiMsgGetSess( mchData, response );

        /* Extract temporary session ID */
        for ( i = 0; i < IPMI_RPLY_TEMP_ID_LENGTH ; i++)
                mchData->id[i] = response[IPMI_RPLY_TEMP_ID_OFFSET + i];

        /* Extract challenge string */
        for ( i = 0; i < IPMI_RPLY_STR_LENGTH ; i++)
                mchData->str[i] = response[IPMI_RPLY_STR_OFFSET + i];

	ipmiMsgActSess( mchData, response );

        /* Extract session ID */
        for ( i = 0; i < IPMI_RPLY_ID_LENGTH ; i++)
                mchData->id[i] = response[IPMI_RPLY_ID_OFFSET + i];

        /* Extract initial sequence number for messages from MCH */
        for ( i = 0; i < IPMI_RPLY_INIT_SEND_SEQ_LENGTH ; i++)
                mchData->seqSend[i] = response[IPMI_RPLY_INIT_SEND_SEQ_OFFSET + i];

	ipmiMsgSetPriv( mchData, response, IPMI_MSG_PRIV_LEVEL_ADMIN );

	return response[IPMI_RPLY_COMPLETION_CODE_OFFSET];
}

/* Start new session with MCH
 * Reset session sequence number and session ID to 0,
 * then call mchCommStart to initiate new session
 *
 * RETURN: (return val from mchCommStart)
 *         0 on success
 *         non-zero on failure          
 */
int
mchNewSession(MchData mchData)
{
int i, rval = -1;

	for ( i = 0; i < IPMI_MSG_HDR_SEQ_LENGTH ; i++)
	        mchData->seqSend[i] = 0;

	for ( i = 0; i < IPMI_MSG_HDR_ID_LENGTH ; i++)
		mchData->id[i] = 0;

	if ( mchIsAlive[mchData->instance] )	
		rval = mchCommStart( mchData );

	return rval;
}

/* Copy FRU area field to data structure
 *
 * RETURNS:
 *          0 on success
 *         -1 if no data to store
 */
int
mchFruFieldGet(FruField field, uint8_t *raw, unsigned *offset)
{
int i;
        if ( ( field->length = IPMI_DATA_LENGTH( raw[*offset] ) ) ) {

                field->type = IPMI_DATA_TYPE( raw[*offset] );
                (*offset)++;

                if ( !( field->data = calloc( field->length, 1 ) ) )
                        errlogPrintf("No memory for FRU field data\n");
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
 * Copy FRU product area data to prod structure
 */
void
mchFruProdDataGet(FruProd prod, uint8_t *raw, unsigned *offset)
{
	if ( 0 != (*offset = prod->offset = 8*raw[FRU_DATA_COMMON_HEADER_OFFSET + FRU_DATA_COMMON_HEADER_PROD_AREA_OFFSET] ) ) {

		prod->lang = raw[*offset + FRU_DATA_BOARD_AREA_LANG_OFFSET];

		*offset += FRU_DATA_PROD_AREA_MANUF_LENGTH_OFFSET;

		if ( mchFruFieldGet( &(prod->manuf), raw, offset ) )
			(*offset)++;
		if ( mchFruFieldGet( &(prod->prod),  raw, offset ) )
			(*offset)++;
		if ( mchFruFieldGet( &(prod->part),  raw, offset ) )
			(*offset)++;
		if ( mchFruFieldGet( &(prod->ver),   raw, offset ) )
			(*offset)++;
		if ( mchFruFieldGet( &(prod->sn),    raw, offset ) )
			(*offset)++;
	}
}

/* 
 * Copy FRU board area data to board structure
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
 */
void
mchFruDataGet(MchData mchData, Fru fru, uint8_t id) 
{
uint8_t    response[MSG_MAX_LENGTH];
uint8_t   *raw; 
int        i;
uint16_t   sizeInt;  /* Size of FRU data area in bytes */
unsigned   nread;    /* Number of FRU data reads */
unsigned   offset;
uint8_t   *readOffset = fru->readOffset;

	/* Get FRU Inventory Info */
	ipmiMsgGetFruInfo( mchData, response, id );

	if ( response[IPMI_RPLY_COMPLETION_CODE_OFFSET] )
		return;

	fru->size[0] = response[IPMI_RPLY_FRU_AREA_SIZE_LSB_OFFSET];
	fru->size[1] = response[IPMI_RPLY_FRU_AREA_SIZE_MSB_OFFSET];
	fru->access  = response[IPMI_RPLY_FRU_AREA_ACCESS_OFFSET];  

	if ( 0 == (sizeInt = arrayToUint16( fru->size )) )
		return;

	if ( !(raw = calloc( sizeInt ,1 ) ) ) {
		errlogPrintf("mchFruDataGet: No memory for FRU %i data\n",id);
		return;
	}

	/* Too many reads! Later, detect no more data */
	if ( (nread = floor( sizeInt/MSG_FRU_DATA_READ_SIZE )) > 50 )
		nread = 30;

	/* Read FRU data, store in raw, increment our read offset for next read */
	for ( i = 0; i < nread; i++ ) {
		fru->read = i;
		ipmiMsgReadFru( mchData, response, id, readOffset, MSG_FRU_DATA_READ_SIZE );
		memcpy( raw + i*MSG_FRU_DATA_READ_SIZE, response + IPMI_RPLY_FRU_DATA_READ_OFFSET, MSG_FRU_DATA_READ_SIZE );
		incr2Uint8Array( fru->readOffset, MSG_FRU_DATA_READ_SIZE );
	}

#ifdef DEBUG
printf("FRU %i raw data: \n",id);
for ( i = 0; i < sizeInt; i++)
	printf("%u ",raw[i]);
printf("\n");
#endif

/* filter out FRUs with no data so that we don't load records for them */

	/* Add chassis data get */
	mchFruBoardDataGet( &(fru->board), raw, &offset );
	mchFruProdDataGet(  &(fru->prod) , raw, &offset );

	free( raw );
}

/* 
 * Get data for all FRUs. Must be done after SDR data stored in fru struct.
 * FRUs are indexed by FRU number
 * Later, change this to also discover FRUs that do not have SDRs (like our SLAC board)
 */
void
mchFruGetDataAll(MchData mchData)
{
uint8_t *response  = malloc(MSG_MAX_LENGTH);
uint8_t  i;
Fru fru;
	for ( i = 0; i < MAX_FRU ; i++) {

		fru = &mchData->fru[i];

		if ( fru->sdr.entityInst ) {

			mchFruDataGet( mchData, fru , i);

			if ( (i >= UTCA_FRU_TYPE_CU_MIN) && (i <= UTCA_FRU_TYPE_CU_MAX) ) {
				ipmiMsgGetFanProp( mchData, response, fru->sdr.fruId );
				fru->fanMin  = response[IPMI_RPLY_GET_FAN_PROP_MIN_OFFSET];
				fru->fanMax  = response[IPMI_RPLY_GET_FAN_PROP_MAX_OFFSET];
				fru->fanNom  = response[IPMI_RPLY_GET_FAN_PROP_NOM_OFFSET];
				fru->fanProp = response[IPMI_RPLY_GET_FAN_PROP_PROP_OFFSET];
			}
		}			
	}

	free( response );

#ifdef DEBUG
printf("mchFruGetDataAll: FRU Summary:\n");
for ( i = 0; i < MAX_FRU  ; i++) {
	fru = &mchData->fru[i];
	if ( fru->sdr.fruId )
		printf("FRU %i %s was found, id %02x instance %02x\n", fru->sdr.fruId, fru->board.prod.data, fru->sdr.entityId, fru->sdr.entityInst);
}
#endif

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
 */
void
mchSensorGetFru(MchData mchData, uint8_t index)
{
int i;
int id      = -1;
Sensor sens = &mchData->sens[index];

	/* First loop through FRUs */
	for ( i = 0; i < MAX_FRU; i++ ) {
		if ( (sens->sdr.entityId == mchData->fru[i].sdr.entityId) && (sens->sdr.entityInst == mchData->fru[i].sdr.entityInst ) ) {
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
	
	/* Save hotswap sensor index to FRU structure (used by devSup) */
	if ( (id != 0 ) && (sens->sdr.sensType == SENSOR_TYPE_HOT_SWAP) )
       		mchData->fru[id].hotswap = index;
}


/* Store SDR Repository info */
void
mchSdrRepGetInfo(MchData mchData)
{
uint8_t *response  = malloc(MSG_MAX_LENGTH);
uint8_t  flags;

	ipmiMsgGetSdrRepInfo( mchData, response );

	if ( response[IPMI_RPLY_COMPLETION_CODE_OFFSET] ) {
		errlogPrintf("mchSdrRepGetInfo: Error reading SDR Repository info for %s\n", mchData->name);
		return;
	}

	mchData->sdrRep.ver     = response[IPMI_RPLY_SDRREP_VER_OFFSET];
	mchData->sdrRep.size[0] = response[IPMI_RPLY_SDRREP_CNT_LSB_OFFSET];
	mchData->sdrRep.size[1] = response[IPMI_RPLY_SDRREP_CNT_MSB_OFFSET];

	ipmiMsgGetDevSdrInfo( mchData, response, 1 );

	if ( response[IPMI_RPLY_COMPLETION_CODE_OFFSET] )
		return;

       	mchData->sdrRep.devSdrSize = response[IPMI_RPLY_DEV_SDR_CNT_OFFSET];
	flags = response[IPMI_RPLY_DEV_SDR_FLAGS_OFFSET];
       	mchData->sdrRep.devSdrDyn  = DEV_SENSOR_DYNAMIC(flags);
       	mchData->sdrRep.lun0       = DEV_SENSOR_LUN0(flags);
       	mchData->sdrRep.lun1       = DEV_SENSOR_LUN1(flags);
       	mchData->sdrRep.lun2       = DEV_SENSOR_LUN2(flags);
       	mchData->sdrRep.lun3       = DEV_SENSOR_LUN3(flags);

	free( response );
}

/* 
 * Store SDR for one FRU
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
 * Store SDR for one sensor or FRU
 */
void
mchSdrFullSens(SdrFull sdr, uint8_t *raw, int type)
{
int n, l, i;
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
}
				  
void				  
mchSdrGetDataAll(MchData mchData)
{
uint8_t  response[MSG_MAX_LENGTH] = { 0 };
uint16_t sdrCount;
uint16_t lastId = 0;
uint8_t  id[2]  = { 0 };
uint8_t  res[2] = { 0 };
Sensor   sens   = 0;
Fru      fru    = 0;
uint8_t  offset = 0;
uint8_t  type   = 0;
uint8_t *raw    = 0;
int      i, iFull = 0, iFru = 0, fruId;

	mchSdrRepGetInfo( mchData );

	sdrCount = arrayToUint16( mchData->sdrRep.size );

	if ( !(raw = calloc( sdrCount, SDR_MAX_LENGTH ) ) ) {
		errlogPrintf("mchSdrGetDataAll: No memory for raw SDR data for %s\n", mchData->name);
		goto bail;
	}
    
       	for ( i = 0; i < sdrCount; i++) {
		       
       		ipmiMsgGetSdr( mchData, response, id, res, offset, 0xFF, 0 );

       		memcpy( raw + (i*SDR_MAX_LENGTH), response + IPMI_RPLY_GET_SDR_DATA_OFFSET, SDR_MAX_LENGTH );

       		switch ( response[IPMI_RPLY_GET_SDR_DATA_OFFSET + SDR_REC_TYPE_OFFSET] ) {

       			default:
       				break;

       			case SDR_TYPE_FULL_SENSOR:
       				mchData->sensCount++;
       				break;

       			case SDR_TYPE_COMPACT_SENSOR:
       				mchData->sensCount++;
       				break;

       			case SDR_TYPE_FRU_DEV:
       				mchData->fruCount++;
       				break;
       		}
       		lastId = arrayToUint16( id );
       		id[0]  = response[IPMI_RPLY_GET_SDR_NEXT_ID_LSB_OFFSET];
       		id[1]  = response[IPMI_RPLY_GET_SDR_NEXT_ID_MSB_OFFSET];
       		if ( lastId == 0xFFFF ) {
       			break;
       		}
       	}

	sdrCount = i;

	if ( !(mchData->sens = calloc( mchData->sensCount, sizeof(*sens) ) ) ) {
		errlogPrintf("mchSdrGetDataAll: No memory for sensor data for %s\n", mchData->name);
		goto bail;
	}

	if ( !(mchData->fru = calloc( MAX_FRU , sizeof(*fru) ) ) ) {
		errlogPrintf("mchSdrGetDataAll: No memory for FRU data for %s\n", mchData->name);
		goto bail;
	}
	/* Store SDR data; for now we only support Compact/Full Sensor Records and FRU Device Locator Records */
	for ( i = 0; i < sdrCount; i++) {
		type = raw[i*SDR_MAX_LENGTH + SDR_REC_TYPE_OFFSET];

		if ( (type == SDR_TYPE_FULL_SENSOR) || (type == SDR_TYPE_COMPACT_SENSOR) ) {
			mchSdrFullSens( &(mchData->sens[iFull].sdr) , raw + i*SDR_MAX_LENGTH, type );
			mchData->sens[iFull].instance = 0; /* Initialize instance to 0 */
			iFull++;
		}
	        else if ( type == SDR_TYPE_FRU_DEV ) {
			fruId = raw[SDR_FRU_ID_OFFSET + i*SDR_MAX_LENGTH];
			mchSdrFruDev( &(mchData->fru[fruId].sdr), raw + i*SDR_MAX_LENGTH );
			mchData->fru[iFru].instance = 0;     /* Initialize instance to 0 */
			iFru++;
		}
	}

#ifdef DEBUG
printf("mchSdrGetDataAll Sumary:\n");
for ( i = 0; i < iFull; i++ ) {
	sens = &mchData->sens[i];
	printf("SDR %i, %s entity ID %02x, entity inst %02x, sensor number %02x, sens type %02x, owner %02x, LUN %02x, RexpBexp %i, M %i, MTol %i, B %i, BAcc %i\n", i, sens->sdr.str, sens->sdr.entityId, sens->sdr.entityInst, sens->sdr.number, sens->sdr.sensType, sens->sdr.owner, sens->sdr.lun, sens->sdr.RexpBexp, sens->sdr.M, sens->sdr.MTol, sens->sdr.B, sens->sdr.BAcc);
}
for ( i = 0; i < MAX_FRU; i++ ) {
	fru = &mchData->fru[i];
	if ( fru->sdr.entityInst )
		printf("SDR %i, entity ID %02x, entity inst %02x, FRU id %02x, %s\n", i, fru->sdr.entityId, fru->sdr.entityInst, fru->sdr.fruId, fru->sdr.str);
}
#endif

bail:
	free( raw );
}

/* 
 *  Periodically ping MCH. This runs in its own thread.
 *  If MCH online/offline status changes, update global
 *  variable and process status record.
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
uint8_t message[MSG_MAX_LENGTH];
uint8_t response[MSG_MAX_LENGTH];
int    *alive = &(mchIsAlive[mchData->instance]);

	memcpy( message, RMCP_HEADER, sizeof( RMCP_HEADER ) );

	message[RMCP_MSG_CLASS_OFFSET] = RMCP_MSG_CLASS_ASF;

	memcpy( message + ASF_MSG_OFFSET, ASF_MSG, sizeof( ASF_MSG ) );

	while (1) {

		memset( response, 0, MSG_MAX_LENGTH ); /* Initialize response to 0 to detect no response */

		ipmiMsgWriteRead( mchData->name, message, sizeof ( RMCP_HEADER ) + sizeof( ASF_MSG ), response );

		if ( 0 == *response ) {
			if ( *alive == MCH_STAT_OK ) { 
				epicsMutexLock( mch->mutex );
				*alive = MCH_STAT_NORESPONSE;
				epicsMutexUnlock( mch->mutex );
				if ( drvMchStatScan )
					scanIoRequest( drvMchStatScan );
			}
		}
		else {
			if ( *alive == MCH_STAT_NORESPONSE ) {
				epicsMutexLock( mch->mutex );
				*alive = MCH_STAT_OK;
				epicsMutexUnlock( mch->mutex );
				if ( drvMchStatScan )
					scanIoRequest( drvMchStatScan );
			}
		}

		epicsThreadSleep( PING_PERIOD );
	}
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
uint8_t i;
char    taskName[50];

	/* Allocate memory for MCH data structure */
	if ( ! (mchData = calloc( 1, sizeof( *mchData ))) )
		cantProceed("FATAL ERROR: No memory for MchData structure\n");

       	/* Allocate memory for MCH device support structure */
       	if ( ! (mch = calloc( 1, sizeof( *mch ) )) )
       		cantProceed("FATAL ERROR: No memory for MchDev structure\n");

       	if ( ! (mch = devMchRegister( name )) )
       		errlogPrintf("FATAL ERROR: Unable to register MCH %s with device support\n", name);

       	mchData->name = mch->name;
       	mch->udata = mchData;

	/* Start task to periodically ping MCH */
	mchData->instance = mchCounter++;
	sprintf( taskName, "%s-PING", mch->name ); 
	mchData->pingThreadId = epicsThreadMustCreate( taskName, epicsThreadPriorityMedium, epicsThreadGetStackSize(epicsThreadStackMedium), mchPing, mch );

	/* Wait for one ping cycle */ 
	epicsThreadSleep( PING_PERIOD ); 

	if ( mchIsAlive[mchData->instance] ) {

		epicsMutexLock( mch->mutex );

		/* Initiate communication session with MCH */
		mchCommStart( mchData );

		/* Get SDR data */
		mchSdrGetDataAll( mchData );

		/* Get FRU data */
		mchFruGetDataAll( mchData );

		/* Get Sensor/FRU association */
		for ( i = 0; i < mchData->sensCount; i++ )
			mchSensorGetFru( mchData, i );

		mchSensorFruGetInstance( mchData );

		/* Create script to load records */
		sensorFruRecordScript( mchData );

		mchInitDone[mchData->instance] = 1;

		epicsMutexUnlock( mch->mutex );

		REPLY_TIMEOUT = 0.1; /* After getting all configuration data, reduce timeout */
	}
	else
		errlogPrintf("No response from %s; cannot complete initialization\n",mch->name);
}

static long
drvMchReport(int level)
{
	errlogPrintf("MicroTCA MCH communication driver support\n");
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
static const iocshArg mchInitArg0       = { "port name",iocshArgString};
static const iocshArg *mchInitArgs[1]   = { &mchInitArg0 };
static const iocshFuncDef mchInitFuncDef = 
	{ "mchInit", 1, mchInitArgs };
static void mchInitCallFunc(const iocshArgBuf *args)
{
	mchInit(args[0].sval);
}

static const iocshArg mchCreateFileArg0        = { "file name",iocshArgString};
static const iocshArg *mchCreateFileArgs[1]    = { &mchCreateFileArg0 };
static const iocshFuncDef mchCreateFileFuncDef = 
	{ "mchCreateFile", 1, mchCreateFileArgs };
static void mchCreateFileCallFunc(const iocshArgBuf *args)
{
	mchCreateFile(args[0].sval);
}

static void
drvMchRegisterCommands(void)
{
	static int firstTime = 1;
	if ( firstTime ) {
		iocshRegister(&mchInitFuncDef, mchInitCallFunc);
		iocshRegister(&mchCreateFileFuncDef, mchCreateFileCallFunc);
		firstTime = 0;
	}
}

epicsExportRegistrar(drvMchRegisterCommands);
