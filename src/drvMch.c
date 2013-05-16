#include <epicsExport.h>
#include <epicsInterrupt.h>
#include <drvSup.h>
#include <iocsh.h>
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
int mchInitDone[MAX_MCH]   = { 0 };
int mchIsAlive[MAX_MCH]    = { 0 };
int mchWasOffline[MAX_MCH] = { 0 };

/* MCH system info, frus, sensors, etc. */
MchSys mchSysData[MAX_MCH] = { 0 };

int  mchSdrGetDataAll(MchSess mchSess, MchSys mchSys);
void mchFruGetDataAll(MchSess mchSess, MchSys mchSys);

/* Compare installed FRUs with those stored in mchData structure.
 * Will be used to detect changes in installed FRUs.
 * (Not in use yet)
 *
 * Caller must perform locking.
 *
 *   RETURNS:
 *            0 if no changes
 *           -1 if changes
 */
int
mchFruDiff(MchSess mchSess, MchSys mchSys)
{
MchSys mchSysTmp = 0;
Fru fru, fruTmp;
int i, j;
int rval = 0;
uint8_t *part = 0, *partTmp = 0, *sn = 0, *snTmp = 0;
uint8_t inst, instTmp, lpart, lpartTmp, lsn, lsnTmp;

	/* Allocate memory for temporary MCH data structure */
	if ( ! (mchSysTmp = calloc( 1, sizeof( *mchSysTmp ))) )
		cantProceed("FATAL ERROR: No memory for temporary MchData structure\n");

       	/* Get SDR data */
       	mchSdrGetDataAll( mchSess, mchSysTmp );

       	/* Get FRU data */
       	mchFruGetDataAll( mchSess, mchSysTmp );

	for ( i = 0; i < MAX_FRU; i++ ) {

		fru     = &mchSys->fru[i];
		fruTmp  = &mchSysTmp->fru[i];
		inst    = (fru->sdr.entityInst)    ? 1 : 0;
		instTmp = (fruTmp->sdr.entityInst) ? 1 : 0;

		if ( inst != instTmp ) {

			if ( IPMICOMM_DEBUG )
				printf("FRU %i presence; before: %i, now %i\n", i, inst, instTmp);
       			rval = -1;
       			break;
       		}

		/* If this FRU exists */
		if ( inst ) {

			lpart    = fru->board.part.length;
			lpartTmp = fruTmp->board.part.length;
			lsn      = fru->board.sn.length;
			lsnTmp   = fruTmp->board.sn.length;

			if ( (lpart != lpartTmp) || (lsn != lsnTmp) ) {
				if ( IPMICOMM_DEBUG )
					printf("FRU %i part or sn data lengths differ; part before %i now %i, sn before %i now %i\n", i, lpart, lpartTmp, lsn, lsnTmp);
       				rval = -1;
       				break;
			}

			for ( j = 0; j < lpart; j++ ) {

				if ( part[j] != partTmp[j] ) {
					if ( IPMICOMM_DEBUG )
						printf("FRU %i part byte % different; before %i now %i\n", i, j, part[j], partTmp[j]);
					rval = -1;
					break;
				}	
			}		

			if ( rval )
				break;

			for ( j = 0; j < lsn; j++ ) {

				if ( sn[j] != snTmp[j] ) {
					if ( IPMICOMM_DEBUG )
						printf("FRU %i sn byte % different; before %i now %i\n", i, j, sn[j], snTmp[j]);
					rval = -1;
					break;
				}	
			}		

			if ( rval )
				break;
		}
	}
		
       	free( mchSysTmp );
	return rval;
}



/* Start communication session with MCH
 * Multi-step handshaking sequence
 *
 * If we have just reconnected after the
 * MCH was offline, determine if the 
 * installed FRUs have changed. If they have,
 * we can not assume our sensor addresses
 * are correct. Set "is initialized"
 * flag to "no" and scan corresponding EPICS
 * record.
 *
 * Caller must perform locking.
 *
 *   RETURNS:
 *         0 on success
 *         non-zero on failure
 */		
int 
mchCommStart(MchSess mchSess)
{	
uint8_t response[MSG_MAX_LENGTH] = { 0 };
int     i;
int    *init = &mchInitDone[mchSess->instance];
int     inst = mchSess->instance;

	if ( IPMICOMM_DEBUG )
		errlogPrintf("Connecting to %s\n", mchSess->name);

	/* Initialize IPMI sequence */
	mchSess->seq = 0;

        /* Initialize our stored sequence number for messages from MCH */
	mchSess->seqRply[0] = IPMI_MSG_HEADER_SEQ_INITIAL;
        for ( i = 1; i < IPMI_RPLY_SEQ_LENGTH - 1 ; i++)
                mchSess->seqRply[i] = 0;
	
	if ( ipmiMsgGetChanAuth( mchSess, response ) )
		return -1;	

	if ( ipmiMsgGetSess( mchSess, response ) )
		return -1;

        /* Extract temporary session ID */
        for ( i = 0; i < IPMI_RPLY_TEMP_ID_LENGTH ; i++)
                mchSess->id[i] = response[IPMI_RPLY_TEMP_ID_OFFSET + i];

        /* Extract challenge string */
        for ( i = 0; i < IPMI_RPLY_STR_LENGTH ; i++)
                mchSess->str[i] = response[IPMI_RPLY_STR_OFFSET + i];

	if ( ipmiMsgActSess( mchSess, response ) )
		return -1;

        /* Extract session ID */
        for ( i = 0; i < IPMI_RPLY_ID_LENGTH ; i++)
                mchSess->id[i] = response[IPMI_RPLY_ID_OFFSET + i];

        /* Extract initial sequence number for messages to MCH */
        for ( i = 0; i < IPMI_RPLY_INIT_SEND_SEQ_LENGTH ; i++)
                mchSess->seqSend[i] = response[IPMI_RPLY_INIT_SEND_SEQ_OFFSET + i];

	if ( ipmiMsgSetPriv( mchSess, response, IPMI_MSG_PRIV_LEVEL_ADMIN ) )
		return -1;

	if ( mchWasOffline[inst] ) {

			*init = MCH_INIT_NOT_DONE;
			if ( drvMchInitScan )
				scanIoRequest( drvMchInitScan );
			errlogPrintf("%s MCH back online, but FRUs and sensor addresses may have changed\n", mchSess->name);
	}

	/* Leave commented for now until ready to handle dynamic sensor addresses and shelf contents
	if ( mchWasOffline[inst] ) {

       		mchWasOffline[inst] = 0;

       		*init = MCH_INIT_IN_PROGRESS;
       		if ( drvMchInitScan )
       			scanIoRequest( drvMchInitScan );

		if ( mchFruDiff( mchSess, mchSysData[inst] ) ) {

			*init = MCH_INIT_NOT_DONE;
			if ( drvMchInitScan )
				scanIoRequest( drvMchInitScan );
			errlogPrintf("%s MCH back online with different FRU configuration\n", mchSess->name);
		}
		else {
			*init = MCH_INIT_DONE;
			if ( drvMchInitScan )
				scanIoRequest( drvMchInitScan );
			errlogPrintf("%s MCH back online with same FRU configuration\n", mchSess->name);
		}
	}
	*/

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
mchNewSession(MchSess mchSess)
{
int i, rval = -1;

	for ( i = 0; i < IPMI_MSG_HDR_SEQ_LENGTH ; i++)
	        mchSess->seqSend[i] = 0;

	for ( i = 0; i < IPMI_MSG_HDR_ID_LENGTH ; i++)
		mchSess->id[i] = 0;

	if ( mchIsAlive[mchSess->instance] )	
		rval = mchCommStart( mchSess );

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
void
mchFruDataGet(MchSess mchSess, MchSys mchSys, Fru fru, uint8_t id) 
{
uint8_t    response[MSG_MAX_LENGTH] = { 0 };
uint8_t   *raw; 
int        i;
uint16_t   sizeInt;  /* Size of FRU data area in bytes */
unsigned   nread;    /* Number of FRU data reads */
unsigned   offset;   /* Offset into FRU data */
int        offs = ( mchSess->type == MCH_TYPE_NAT ) ? IPMI_RPLY_OFFSET_NAT : 0; /* Offset into reply message */

	/* Get FRU Inventory Info */
	if ( ipmiMsgGetFruInfo( mchSess, response, id ) > 0 )
		return;

	fru->size[0] = response[IPMI_RPLY_FRU_AREA_SIZE_LSB_OFFSET + offs];
	fru->size[1] = response[IPMI_RPLY_FRU_AREA_SIZE_MSB_OFFSET + offs];
	fru->access  = response[IPMI_RPLY_FRU_AREA_ACCESS_OFFSET   + offs];  

	if ( 0 == (sizeInt = arrayToUint16( fru->size )) )
		return;

	if ( IPMICOMM_DEBUG )
		printf("mchFruDataGet: FRU %i inventory info size %i\n", id, sizeInt);

	if ( !(raw = calloc( sizeInt ,1 ) ) ) {
		errlogPrintf("mchFruDataGet: No memory for FRU %i data\n",id);
		return;
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
		if ( (IPMI_COMP_CODE_REQUESTED_DATA == ipmiMsgReadFru( mchSess, response, id, fru->readOffset, MSG_FRU_DATA_READ_SIZE )) )
			break;
		memcpy( raw + i*MSG_FRU_DATA_READ_SIZE, response + IPMI_RPLY_FRU_DATA_READ_OFFSET + offs, MSG_FRU_DATA_READ_SIZE );
		incr2Uint8Array( fru->readOffset, MSG_FRU_DATA_READ_SIZE );
	}

#ifdef DEBUG
printf("FRU %i raw data, size %i: \n",id, sizeInt);
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
 * Get data for all FRUs. Must be done after SDR data has been stored in FRU struct.
 * If FRU is cooling unit, get fan properties.
 *
 * FRUs are indexed by FRU number
 * Later, change this to also discover FRUs that do not have SDRs (like our SLAC board)?
 *
 * Caller must perform locking.
 */
void
mchFruGetDataAll(MchSess mchSess, MchSys mchSys)
{
uint8_t response[MSG_MAX_LENGTH] = { 0 };
uint8_t i;
Fru fru;
int offs = ( mchSess->type == MCH_TYPE_NAT ) ? IPMI_RPLY_2ND_BRIDGED_OFFSET_NAT : 0;;

	for ( i = 0; i < MAX_FRU ; i++ ) {

	       	fru = &mchSys->fru[i];
	
		if ( fru->sdr.entityInst ) {

			mchFruDataGet( mchSess, mchSys, fru , i );

			if ( (i >= UTCA_FRU_TYPE_CU_MIN) && (i <= UTCA_FRU_TYPE_CU_MAX) ) {

				ipmiMsgGetFanPropHelper( mchSess, response, fru->sdr.fruId );
				fru->fanMin  = response[IPMI_RPLY_GET_FAN_PROP_MIN_OFFSET  + offs];
				fru->fanMax  = response[IPMI_RPLY_GET_FAN_PROP_MAX_OFFSET  + offs];
				fru->fanNom  = response[IPMI_RPLY_GET_FAN_PROP_NOM_OFFSET  + offs];
				fru->fanProp = response[IPMI_RPLY_GET_FAN_PROP_PROP_OFFSET + offs];
			}
		}			
	}

#ifdef DEBUG
printf("mchFruGetDataAll: FRU Summary:\n");
for ( i = 0; i < MAX_FRU  ; i++) {
	fru = &mchSys->fru[i];
	if ( fru->sdr.fruId )
		printf("FRU %i %s was found, id %02x instance %02x, addr %02x dev %02x lun %02x\n", fru->sdr.fruId, fru->board.prod.data, fru->sdr.entityId, fru->sdr.entityInst, fru->sdr.addr, fru->sdr.fruId, fru->sdr.lun);
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
 *
 * Caller must perform locking.
 */
void
mchSensorGetFru(MchSys mchSys, uint8_t index)
{
int i;
int id      = -1;
Sensor sens = &mchSys->sens[index];

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
	
	/* Save hotswap sensor index to FRU structure (used by devSup) */
	if ( (id != 0 ) && ((sens->sdr.sensType == SENSOR_TYPE_HOTSWAP_VT) || (sens->sdr.sensType == SENSOR_TYPE_HOTSWAP_NAT) ) )
       		mchSys->fru[id].hotswap = index;
}


/* 
 * Store SDR Repository info 
 *
 * Caller must perform locking.
 */
int
mchSdrRepGetInfo(MchSess mchSess, MchSys mchSys)
{
uint8_t response[MSG_MAX_LENGTH] = { 0 };
uint8_t flags;
int     offs = ( mchSess->type == MCH_TYPE_NAT ) ? IPMI_RPLY_OFFSET_NAT : 0;

	ipmiMsgGetSdrRepInfo( mchSess, response );

	if ( response[IPMI_RPLY_COMPLETION_CODE_OFFSET + offs] ) {
		errlogPrintf("mchSdrRepGetInfo: Error reading SDR Repository info for %s\n", mchSess->name);
		return -1;
	}

	mchSys->sdrRep.ver     = response[IPMI_RPLY_SDRREP_VER_OFFSET     + offs];
	mchSys->sdrRep.size[0] = response[IPMI_RPLY_SDRREP_CNT_LSB_OFFSET + offs];
	mchSys->sdrRep.size[1] = response[IPMI_RPLY_SDRREP_CNT_MSB_OFFSET + offs];

	ipmiMsgGetDevSdrInfo( mchSess, response, 1 );

	if ( response[IPMI_RPLY_COMPLETION_CODE_OFFSET + offs] )
		return 0; /* We don't currently use dev sdr, so don't return error */

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
 * Store SDR for one sensor into sensor data structure
 *
 * Caller must perform locking.
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

/*
 *
 * Caller must perform locking.
 */				  
int				  
mchSdrGetDataAll(MchSess mchSess, MchSys mchSys)
{
uint8_t  response[MSG_MAX_LENGTH] = { 0 };
uint16_t sdrCount;
uint8_t  id[2]  = { 0 };
uint8_t  res[2] = { 0 };
Sensor   sens   = 0;
Fru      fru    = 0;
uint8_t  offset = 0;
uint8_t  type   = 0, addr = 0;
uint8_t *raw    = 0;
int      i, iFull = 0, iFru = 0, fruId;
size_t   responseSize;
int      rval = -1;
int      offs = ( mchSess->type == MCH_TYPE_NAT ) ? IPMI_RPLY_OFFSET_NAT : 0;

	if ( mchSdrRepGetInfo( mchSess, mchSys ) )
		return -1;

	sdrCount = arrayToUint16( mchSys->sdrRep.size );

	if ( !(raw = calloc( sdrCount, SDR_MAX_LENGTH ) ) ) {
		errlogPrintf("mchSdrGetDataAll: No memory for raw SDR data for %s\n", mchSess->name);
		goto bail;
	}
    
       	for ( i = 0; i < sdrCount; i++) {
		       
       		if ( ipmiMsgGetSdr( mchSess, response, id, res, offset, 0xFF, 0 ) )
			i--;

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
					mchSys->fruCount++;
					break;
			}
			id[0]  = response[IPMI_RPLY_GET_SDR_NEXT_ID_LSB_OFFSET + offs];
			id[1]  = response[IPMI_RPLY_GET_SDR_NEXT_ID_MSB_OFFSET + offs];

			if ( arrayToUint16( id ) == 0xFFFF )
				break;
		}

       	}

	sdrCount = i;

	if ( !(mchSys->sens = calloc( mchSys->sensCount, sizeof(*sens) ) ) ) {
		errlogPrintf("mchSdrGetDataAll: No memory for sensor data for %s\n", mchSess->name);
		goto bail;
	}

	if ( !(mchSys->fru = calloc( MAX_FRU , sizeof(*fru) ) ) ) {
		errlogPrintf("mchSdrGetDataAll: No memory for FRU data for %s\n", mchSess->name);
		goto bail;
	}

	/* Store SDR data; for now we only support Compact/Full Sensor Records and FRU Device Locator Records */
	for ( i = 0; i < sdrCount; i++) {
		type = raw[i*SDR_MAX_LENGTH + SDR_REC_TYPE_OFFSET];

		if ( (type == SDR_TYPE_FULL_SENSOR) || (type == SDR_TYPE_COMPACT_SENSOR) ) {
			mchSdrFullSens( &(mchSys->sens[iFull].sdr) , raw + i*SDR_MAX_LENGTH, type );
			mchSys->sens[iFull].instance = 0; /* Initialize instance to 0 */

			/* Save sensor reading response length for future reads (response length varies per sensor) */
			responseSize = mchSys->sens[iFull].readMsgLength = 0;
			ipmiMsgReadSensor( mchSess, response, mchSys->sens[iFull].sdr.number, addr, &responseSize );	

			mchSys->sens[iFull].readMsgLength = responseSize;

			iFull++;
		}
	        else if ( type == SDR_TYPE_FRU_DEV ) {
			fruId = raw[SDR_FRU_ID_OFFSET + i*SDR_MAX_LENGTH];
			mchSdrFruDev( &(mchSys->fru[fruId].sdr), raw + i*SDR_MAX_LENGTH );
			mchSys->fru[iFru].instance = 0;     /* Initialize instance to 0 */
			iFru++;
		}
	}

#ifdef DEBUG
printf("mchSdrGetDataAll Sumary:\n");
for ( i = 0; i < iFull; i++ ) {
	sens = &mchSys->sens[i];
	printf("SDR %i, %s entity ID %02x, entity inst %02x, sensor number %02x, sens type %02x, owner %02x, LUN %02x, RexpBexp %i, M %i, MTol %i, B %i, BAcc %i\n", i, sens->sdr.str, sens->sdr.entityId, sens->sdr.entityInst, sens->sdr.number, sens->sdr.sensType, sens->sdr.owner, sens->sdr.lun, sens->sdr.RexpBexp, sens->sdr.M, sens->sdr.MTol, sens->sdr.B, sens->sdr.BAcc);
}
for ( i = 0; i < MAX_FRU; i++ ) {
	fru = &mchSys->fru[i];
	if ( fru->sdr.entityInst )
		printf("SDR %i, entity ID %02x, entity inst %02x, FRU id %02x, %s\n", i, fru->sdr.entityId, fru->sdr.entityInst, fru->sdr.fruId, fru->sdr.str);
}
#endif

	rval = 0;

bail:
	free( raw );
	return rval;
}

/* 
 *  Periodically ping MCH. This runs in its own thread.
 *  If MCH online/offline status changes, update global
 *  variable and process status record.
  *
  *  These messages are outside of a session and we don't
  *  modify our shared structure, so there is no need to lock
  *  during the message write (just while we modify the global
  *  variable mchIsAlive.) We call ipmiMsgWriteRead directly, 
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
size_t  responseSize;
int    *alive   = &(mchIsAlive[mchSess->instance]);
int    *offline = &(mchWasOffline[mchSess->instance]);
int     cos;

	memcpy( message, RMCP_HEADER, sizeof( RMCP_HEADER ) );

	message[RMCP_MSG_CLASS_OFFSET] = RMCP_MSG_CLASS_ASF;

	memcpy( message + ASF_MSG_OFFSET, ASF_MSG, sizeof( ASF_MSG ) );

	while (1) {

		cos = 0;
		responseSize = IPMI_RPLY_PONG_LENGTH;

		ipmiMsgWriteRead( mchSess->name, message, sizeof( RMCP_HEADER ) + sizeof( ASF_MSG ), response, &responseSize, RPLY_TIMEOUT );

       		epicsMutexLock( mch->mutex );

		if ( responseSize == 0 ) {
			if ( *alive == MCH_STAT_OK ) { 
				*alive = MCH_STAT_NORESPONSE;
				*offline = 1;
				cos = 1;
			}
		}
		else {
			if ( *alive == MCH_STAT_NORESPONSE ) {
				*alive = MCH_STAT_OK;
				cos = 1;
			}
		}

       	       	epicsMutexUnlock( mch->mutex );

		if ( cos ) {
       			if ( drvMchStatScan )
       				scanIoRequest( drvMchStatScan );
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
MchSess mchSess = 0;
MchSys  mchSys  = 0;
uint8_t i;
char    taskName[50];

	/* Allocate memory for MCH data structures */
	if ( ! (mchData = calloc( 1, sizeof( *mchData ))) )
		cantProceed("FATAL ERROR: No memory for MchData structure\n");

	if ( ! (mchSess = calloc( 1, sizeof( *mchSess ))) )
		cantProceed("FATAL ERROR: No memory for MchSess structure\n");

	if ( ! (mchSys = calloc( 1, sizeof( *mchSys ))) )
		cantProceed("FATAL ERROR: No memory for MchSys structure\n");

	mchSess->instance = mchCounter++;
	mchSysData[mchSess->instance] = mchSys;    

       	/* Allocate memory for MCH device support structure */
       	if ( ! (mch = calloc( 1, sizeof( *mch ) )) )
       		cantProceed("FATAL ERROR: No memory for MchDev structure\n");

       	if ( ! (mch = devMchRegister( name )) )
       		errlogPrintf("FATAL ERROR: Unable to register MCH %s with device support\n", name);

	mchData->mchSess = mchSess;

       	mchSess->name = mch->name;
       	mchSys->name  = mch->name;
       	mch->udata = mchData;

       	mchSess->timeout = RPLY_TIMEOUT;
	mchSess->session = 1;   /* Default: enable session with MCH */

	/* Start task to periodically ping MCH */
	sprintf( taskName, "%s-PING", mch->name ); 
	mchSess->pingThreadId = epicsThreadMustCreate( taskName, epicsThreadPriorityMedium, epicsThreadGetStackSize(epicsThreadStackMedium), mchPing, mch );

	/* Wait for updated status from ping thread */ 
	epicsThreadSleep( PING_PERIOD ); 

	if ( mchIsAlive[mchSess->instance] ) {

		epicsMutexLock( mch->mutex );

                /* Determine MCH type */
                mchIdentify( mchSess );

		/* Initiate communication session with MCH */
		if ( !mchCommStart( mchSess ) ) {

			/* Get SDR data */
			if ( !mchSdrGetDataAll( mchSess, mchSys ) ) {

				/* Get FRU data */
				mchFruGetDataAll( mchSess, mchSys );

				/* Get Sensor/FRU association */
				for ( i = 0; i < mchSys->sensCount; i++ )
					mchSensorGetFru( mchSys, i );

				mchSensorFruGetInstance( mchSys );

				mchInitDone[mchSess->instance] = MCH_INIT_DONE;
			}
			else {
				errlogPrintf("Failed to read %s SDR; cannot complete initialization\n",mch->name);
			}

		}
		else {
			errlogPrintf("Error initiating session with %s; cannot complete initialization\n",mch->name);
		}


		epicsMutexUnlock( mch->mutex );
	}
	else {
		errlogPrintf("No response from %s; cannot complete initialization\n",mch->name);
	}

       	/* Create script to load records; done at init with no other threads modifying mchData, so need to lock */
       	sensorFruRecordScript( mchSys, mchInitDone[mchSess->instance] );

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
static const iocshArg mchInitArg0        = { "port name",iocshArgString};
static const iocshArg *mchInitArgs[1]    = { &mchInitArg0 };
static const iocshFuncDef mchInitFuncDef = { "mchInit", 1, mchInitArgs };

static void mchInitCallFunc(const iocshArgBuf *args)
{
	mchInit(args[0].sval);
}

static const iocshArg mchCreateFileArg0        = { "file name",iocshArgString};
static const iocshArg *mchCreateFileArgs[1]    = { &mchCreateFileArg0 };
static const iocshFuncDef mchCreateFileFuncDef = { "mchCreateFile", 1, mchCreateFileArgs };

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
