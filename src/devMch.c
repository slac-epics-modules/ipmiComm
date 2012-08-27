/*
=============================================================

  Abs:  EPICS Device Support for MicroTCA MCH/shelf 

  Name: devMch.c

         Utilities:
         -------------------------------------------------------
	 *   devMchRegister           - Register device
	 *   devMchFind               - Find registered device
         *   sensorConversion         - Convert sensor raw reading, according to sensor SDR info

         Analog Input Device Support:
         -------------------------------------------------------

	 devAiMch
         --------
         *   init_ai_record           - Record initialization
         *   read_ai                  - Read analog input

	   Supported operations:

             * "TEMP": read temperature sensor
             * "FAN" : read fan speed sensor
             * "V"   : read voltage sensor
             * "I"   : read current sensor
	   
	 devAiFru
         --------
         *   init_fru_ai_record           - Record initialization
         *   read_fru_ai                  - Read analog input

	   Supported operations:

             * "FAN" : read fan level
             * "BSN" : FRU board serial number
             * "PSN" : FRU product serial number	   

         Binary Output Device Support:
         -------------------------------------------------------

	 devBoMch
         --------

         *   init_bo_record           - Record initialization
         *   write_bo                 - Write binary output

	   Supported operations:

             * "RESET": perform cold reset (reboots MCH)
             * "SESS":  close/open session with MCH
             * "INIT":  override 'MCH communication initialized' bit

         Binary Input Device Support:
         -------------------------------------------------------

	 devBiMch
         ----------

	 *   init_bi                   - bi initialization     
         *   bi_ioint_info             - Add to i/o scan list   
         *   init_bi_record            - Record initialization
         *   read_bi                   - Read binary input

	   Supported operations:

             * "STAT": read crate online/offline status

         Multibit-Binary Input Device Support:
         -------------------------------------------------------

	 devMbbiMch
         ----------

	 *   init_mbbi                 - mbbi initialization     
         *   mbbi_ioint_info           - Add to i/o scan list   
         *   init_mbbi_record          - Record initialization
         *   read_mbbi                 - Read multi-bit binary input

	   Supported operations:

             * "INIT": communication with crate initialized
             * "FAN":  read fan auto-adjustment
             * "HS":   read FRU hot-swap sensor 


         Multibit-Binary Output Device Support:
         -------------------------------------------------------

	 devMbboMch
         ----------

         *   init_mbbo_record          - Record initialization
         *   write_mbbo                - Write multi-bit binary output

	   Supported operations:

             * "CHAS": chassis control on/off/reset
             * "FRU":  FRU on/off

         String Input Device Support:
         -------------------------------------------------------

	 devStringinFru
         --------------

         *   init_fru_stringin_record     - Record initialization
         *   read_fru_stringin            - Read string input

	   Supported operations:

             * "BMF":  FRU board manufacturer
             * "BP":   FRU board product name
             * "BPN":  FRU board part number
             * "PMF":  FRU product manufacturer
             * "PP":   FRU product product name
             * "PPN":  FRU product part number

         Long Integer Output Device Support:
         -------------------------------------------------------

	 devLongoutFru
         -------------

         *   init_fru_longout_record      - Record initialization
         *   write_longout                - Write long integer output

	   Supported operations:

             * "FAN":  fan level control

=============================================================
*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>

#include <math.h> /* can be removed with ipmiSensorConversion */

#include <registry.h>
#include <alarm.h>
#include <dbAccess.h>
#include <errlog.h>
#include <alarm.h>
#include <dbDefs.h>
#include <dbAccess.h>
#include <recGbl.h>
#include <recSup.h>
#include <devSup.h>
#include <aiRecord.h>
#include <biRecord.h>
#include <longoutRecord.h>
#include <stringinRecord.h>
#include <mbboRecord.h>
#include <mbbiRecord.h>
#include <boRecord.h>
#include <epicsExport.h>
#include <asynDriver.h>
#include <dbScan.h>

#include <ipmiDef.h>
#include <ipmiMsg.h>
#include <drvMch.h>
#include <devMch.h>

#undef DEBUG

extern volatile int IPMICOMM_DEBUG;

#define MAX_STRING_LENGTH 39

int initTrap = 0;

/* Device support prototypes */
static long init_ai_record(struct aiRecord *pai);
static long read_ai(struct aiRecord *pai);

static long init_bo_record(struct  boRecord *pbo);
static long write_bo(struct boRecord *pbo);

static long init_bi(struct biRecord *pbi);
static long init_bi_record(struct biRecord *pbi);
static long read_bi(struct biRecord *pbi);
static long bi_ioint_info(int cmd, struct biRecord *pbi, IOSCANPVT *iopvt);

static long init_mbbi(struct biRecord *pmbbi);
static long init_mbbi_record(struct mbbiRecord *pmbbi);
static long read_mbbi(struct mbbiRecord *pmbbi);
static long mbbi_ioint_info(int cmd, struct mbbiRecord *mbpbi, IOSCANPVT *iopvt);

static long init_mbbo_record(struct mbboRecord *pmbbo);
static long write_mbbo(struct mbboRecord *pmbbo);

static long init_fru_ai_record(struct aiRecord *pai);
static long read_fru_ai(struct aiRecord *pai);
/*static long ai_fru_ioint_info(int cmd, struct aiRecord *pai, IOSCANPVT *iopvt);*/

static long init_fru_longout_record(struct longoutRecord *plongout);
static long write_fru_longout(struct longoutRecord *plongout);

static long init_fru_stringin_record(struct stringinRecord *pstringin);
static long read_fru_stringin(struct stringinRecord *pstringin);
/*static long stringin_fru_ioint_info(int cmd, struct stringinRecord *pstringin, IOSCANPVT *iopvt);*/

/* global struct for devSup */
typedef struct {
        long            number;
        DEVSUPFUN       report;
        DEVSUPFUN       init;
        DEVSUPFUN       init_record;
        DEVSUPFUN       get_ioint_info;
        DEVSUPFUN       read_write;
        DEVSUPFUN       special_linconv;
} MCH_DEV_SUP_SET;

/* Add reporting */
MCH_DEV_SUP_SET devAiMch         = {6, NULL, NULL, init_ai_record,             NULL, read_ai,           NULL};
MCH_DEV_SUP_SET devBoMch         = {6, NULL, NULL, init_bo_record,             NULL, write_bo,          NULL};
MCH_DEV_SUP_SET devMbboMch       = {6, NULL, NULL, init_mbbo_record,           NULL, write_mbbo,        NULL};
MCH_DEV_SUP_SET devBiMch         = {6, NULL, init_bi, init_bi_record,     bi_ioint_info,   read_bi,     NULL};
MCH_DEV_SUP_SET devMbbiMch       = {6, NULL, init_mbbi, init_mbbi_record, mbbi_ioint_info, read_mbbi,   NULL};
MCH_DEV_SUP_SET devAiFru         = {6, NULL, NULL, init_fru_ai_record,         NULL, read_fru_ai,       NULL};
MCH_DEV_SUP_SET devLongoutFru    = {6, NULL, NULL, init_fru_longout_record,    NULL, write_fru_longout, NULL};
MCH_DEV_SUP_SET devStringinFru   = {6, NULL, NULL, init_fru_stringin_record,   NULL, read_fru_stringin, NULL};

epicsExportAddress(dset, devAiMch);
epicsExportAddress(dset, devBoMch);
epicsExportAddress(dset, devBiMch);
epicsExportAddress(dset, devMbbiMch);
epicsExportAddress(dset, devMbboMch);
epicsExportAddress(dset, devAiFru);
epicsExportAddress(dset, devLongoutFru);
epicsExportAddress(dset, devStringinFru);


/*--- Register functions stolen from devBusMapped ---*/

/* just any unique address */
static void     *registryId = (void*)&registryId;
static void     *ioRegistryId = (void*)&ioRegistryId;
static void     *ioscanRegistryId = (void*)&ioscanRegistryId;

/* Register a device's base address and return a pointer to a
 * freshly allocated 'MchDev' struct or NULL on failure.
 */
MchDev
devMchRegister(const char *name)
{
MchDev rval = 0, d;

        if ( (d = malloc(sizeof(*rval) + strlen(name))) ) {
                /* pre-load the allocated structure -  'registryAdd()'
                 * is atomical...
                 */
                /*d->baseAddr = baseAddress;*/
                strcpy((char*)d->name, name);
                if ( (d->mutex = epicsMutexCreate()) ) {
                        /* NOTE: the registry keeps a pointer to the name and
                         *       does not copy the string, therefore we keep one.
                         *       (_must_ pass d->name, not 'name'!!)
                         */
                        if ( registryAdd( registryId, d->name, d ) ) {
                                rval = d; d = 0;
                        }
                }
        }

        if (d) {
                if (d->mutex)
                        epicsMutexDestroy(d->mutex);
                free(d);
        }
        return rval;
}

/* Find the 'MchDev' of a registered device by name */
MchDev
devMchFind(const char *name)
{
#ifdef DEBUG
printf("devMchFind: name is %s\n",name);
#endif
	return (MchDev)registryFind(registryId, name);
}


/*--- end stolen ---*/

float 
sensorConversion(SdrFull sdr, uint8_t raw)
{
int l, units, m, b, rexp, bexp;
float value;

	l = SENSOR_LINEAR( sdr->linear );

	m     = TWOS_COMP_SIGNED_NBIT(SENSOR_CONV_M_B( sdr->M, sdr->MTol ), 10 );
	b     = TWOS_COMP_SIGNED_NBIT(SENSOR_CONV_M_B( sdr->B, sdr->BAcc ), 10 );
	rexp  = TWOS_COMP_SIGNED_NBIT(SENSOR_CONV_REXP( sdr->RexpBexp ), 4 );
	bexp  = TWOS_COMP_SIGNED_NBIT(SENSOR_CONV_BEXP( sdr->RexpBexp ), 4 );
	units = sdr->units2;    

       	value = ((m*raw) + (b*pow(10,bexp)))*pow(10,rexp);

	if ( l == SENSOR_CONV_LINEAR )
		value = value;
	if ( l == SENSOR_CONV_LN )
		value = log( value );
	if ( l == SENSOR_CONV_LOG10 )
		value = log10( value );
	if ( l == SENSOR_CONV_LOG2 )
		value = log( value )/log( 2 );
	if ( l == SENSOR_CONV_E )
		value = exp( value );
	if ( l == SENSOR_CONV_EXP10 )
		value = pow( value, 10 );
	if ( l == SENSOR_CONV_EXP2 )
		value = pow( value, 2 );
	if ( l == SENSOR_CONV_1_X )
		value = 1/value;
	if ( l == SENSOR_CONV_SQR )
		value = pow( value, 2);
	if ( l == SENSOR_CONV_CUBE )
		value = pow( value, 1/3 );
	if ( l == SENSOR_CONV_SQRT )
		value = sqrt( value );
	if ( l == SENSOR_CONV_CUBE_NEG1 )
		value = pow( value, -1/3 );

	return value;
}

static long 
init_ai_record(struct aiRecord *pai)
{
MchRec   recPvt  = 0; /* Info stored with record */
MchDev   mch     = 0; /* MCH device data structures */
char    *node;        /* Network node name, stored in parm */
char    *task    = 0; /* Optional additional parameter appended to parm */
char    *p;
short    c, s;
long     status  = 0;
char     str[40];

        if ( VME_IO != pai->inp.type ) {
		status = S_dev_badBus;
		goto bail;
	}

        c = pai->inp.value.vmeio.card;
        s = pai->inp.value.vmeio.signal;

        if ( c < 0 ) {
		status = S_dev_badCard;
		goto bail;
	}

        if ( s < 0 ) {
		status = S_dev_badSignal;
		goto bail;
	}

	if ( ! (recPvt = calloc( 1, sizeof( *recPvt ) )) ) {
		sprintf( str, "No memory for recPvt structure");
		status = S_rec_outMem;
		goto bail;
	}

	/* Break parm into node name and optional parameter */
	node = strtok( pai->inp.value.vmeio.parm, "+" );
	if ( (p = strtok( NULL, "+" )) ) {
		task = p;
		if ( strcmp( task, "TEMP" ) && strcmp( task, "FAN" ) && strcmp( task, "V") && strcmp( task, "I") ) { 
			sprintf( str, "Unknown task parameter %s", task);
			status = S_dev_badSignal;
		}
	}

	/* Find data structure in registry by MCH name */
	if ( (mch = devMchFind( node )) ) {
		recPvt->mch  = mch;
                recPvt->task = task;
		pai->dpvt    = recPvt;
	}
	else {
		sprintf( str, "Failed to locate data structure" );
		status = S_dev_noDeviceFound;
		goto bail;
	}

	/* To do: implement alarms 
	mchData = mch->udata;
	sdr     = &mchSys->sens[s].sdr;

	if ( sdr->recType == SDR_TYPE_FULL_SENSOR ) {

		if ( SENSOR_NOMINAL_GIVEN( sdr->anlgChar ) || (sdr->nominal) ) {

			switch ( SENSOR_NOMINAL_FORMAT( sdr->units1 ) ) {

				default:
					goto bail;

				case SENSOR_NOMINAL_UNSIGNED:

					raw = sdr->nominal;
					break;

				case SENSOR_NOMINAL_ONES_COMP:

					raw = ONES_COMP_SIGNED_NBIT( sdr->nominal, 8 );
					break;

				case SENSOR_NOMINAL_TWOS_COMP:

					raw = TWOS_COMP_SIGNED_NBIT( sdr->nominal, 8 );
					break;

				case SENSOR_NOMINAL_NONNUMERIC:
					goto bail;

			}

			nominal = sensorConversion( sdr, raw );
			if ( sdr->units2 == SENSOR_UNITS_DEGC)
				nominal = nominal*9/5 + 32;
		}

	} */

bail:
	if ( status ) {
	       recGblRecordError( status, (void *)pai , (const char *)str );
	       pai->pact=TRUE;
	}

        return status;
}

static long 
read_ai(struct aiRecord *pai)
{
MchRec   recPvt  = pai->dpvt;
MchDev   mch;
MchData  mchData;
MchSess  mchSess;
MchSys   mchSys;
char    *task;
uint8_t  data[MSG_MAX_LENGTH] = { 0 };
uint8_t  raw, sensor;
short    index   = pai->inp.value.vmeio.signal; /* Sensor index */
long     status  = NO_CONVERT;
SdrFull  sdr;
int      units, s = 0;
float    value = 0;
size_t   responseSize;
uint8_t  lun;

	if ( !recPvt )
		return status;

	mch     = recPvt->mch;
	mchData = mch->udata;
	mchSess = mchData->mchSess;
	mchSys  = mchSysData[mchSess->instance];
	task    = recPvt->task;

	if ( mchIsAlive[mchSess->instance] && mchSess->session && mchInitDone[mchSess->instance] ) {

		responseSize = mchSys->sens[index].readMsgLength;
		sensor = mchSys->sens[index].sdr.number;
		lun    = mchSys->sens[index].sdr.lun & 0x3;

		epicsMutexLock( mch->mutex );

		if ( !(s = ipmiMsgReadSensor( mchSess, data, sensor, lun, &responseSize )) ) {
			raw = data[IPMI_RPLY_SENSOR_READING_OFFSET];
			mchSys->sens[index].val = raw;
		}
	
		epicsMutexUnlock( mch->mutex );

		if ( s ) {
			if ( IPMICOMM_DEBUG )
				printf("%s writeread error sensor %02x index %i\n",pai->name, sensor, index);
			recGblSetSevr( pai, READ_ALARM, INVALID_ALARM );
			return ERROR;
		}

		sdr = &mchSys->sens[index].sdr;

		/* All of our conversions are for Full Sensor SDRs */
		if ( sdr->recType != SDR_TYPE_FULL_SENSOR )
			return status;

		value = sensorConversion( sdr, raw );

		units = sdr->units2;
		pai->rval = value;

		/* Perform appropriate conversion - check units */
		if ( !(strcmp( task, "TEMP")) ) {
			if ( units == SENSOR_UNITS_DEGC)
				pai->val = value*9/5 + 32;
			else if ( units == SENSOR_UNITS_DEGF)
				pai->val = value;
		}
		else if ( !(strcmp( task, "FAN")) ) {
			if ( units == SENSOR_UNITS_RPM )
				pai->val  = value;
		}
		else if ( !(strcmp( task, "V")) ) {
			if ( units == SENSOR_UNITS_VOLTS )
				pai->val  = value;
		}
		else if ( !(strcmp( task, "I")) ) {
			if ( units == SENSOR_UNITS_AMPS )
				pai->val  = value;
		}

       		if ( IPMICOMM_DEBUG > 2 )
			printf("%s: raw %02x, sensor %02x, owner %i, lun %i, index %i, value %.1f\n",pai->name, raw, sensor, mchSys->sens[index].sdr.owner, mchSys->sens[index].sdr.lun, index, pai->val);

#ifdef DEBUG
printf("read_ai: %s sensor index is %i, task is %s, sensor number is %i, value is %.0f, rval is %i\n",
    pai->name, index, task, mchSys->sens[index].sdr.number, pai->val, pai->rval);
#endif

		pai->udf = FALSE;
		return status;
	}
	else {
		recGblSetSevr( pai, READ_ALARM, INVALID_ALARM );
		return ERROR;
	}


}

static long 
init_bo_record(struct boRecord *pbo)
{
MchRec  recPvt  = 0; /* Info stored with record */
MchDev  mch     = 0; /* MCH device data structures */
char    *node;       /* Network node name, stored in parm */
char    *task   = 0; /* Optional additional parameter appended to parm */
char    *p;
short    c, s;
long    status = 0;
char    str[40];

        if ( VME_IO != pbo->out.type ) {
		status = S_dev_badBus;
		goto bail;
	}

        c = pbo->out.value.vmeio.card;
        s = pbo->out.value.vmeio.signal;

        if ( c < 0 )
		status = S_dev_badCard;

        if ( s < 0 )
		status = S_dev_badSignal;

        if ( ! (recPvt = calloc( 1, sizeof( *recPvt ) )) ) {
		sprintf( str, "No memory for recPvt structure");
		status = S_rec_outMem;
		goto bail;
        }

        /* Break parm into node name and optional parameter */
        node = strtok( pbo->out.value.vmeio.parm, "+" );
        if ( (p = strtok( NULL, "+")) ) {
                task = p;
                if ( strcmp( task, "RESET") && strcmp( task, "SESS") && strcmp( task, "INIT") ) {
			sprintf( str, "Unknown task parameter %s", task);
			status = S_dev_badSignal;
		}
        }

        /* Find data structure in registry by MCH name */
        if ( (mch = devMchFind(node)) ) {
                recPvt->mch  = mch;
                recPvt->task = task;
                pbo->dpvt  = recPvt;                
        }
        else {
		sprintf( str, "Failed to locate data structure" );
		status = S_dev_noDeviceFound;
        }

bail:
	if ( status ) {
	       recGblRecordError( status, (void *)pbo , (const char *)str );
	       pbo->pact=TRUE;
	}

        return status;
}

static long 
write_bo(struct boRecord *pbo)
{
uint8_t  data[MSG_MAX_LENGTH] = { 0 };
MchRec   recPvt = pbo->dpvt;
MchDev   mch;
MchData  mchData;
MchSess  mchSess;
MchSys   mchSys;
char    *task;
long     status = NO_CONVERT;
int      s = 0;

	if ( !recPvt )
		return status;

	mch     = recPvt->mch;
	mchData = mch->udata;
	mchSess = mchData->mchSess;
	mchSys  = mchSysData[mchSess->instance];

	task    = recPvt->task;

	if ( mchIsAlive[mchSess->instance] ) {

		if ( !(strcmp( task, "SESS" )) ) {

			epicsMutexLock( mch->mutex );

			if ( pbo->val )
				mchSess->session = 1; /* Re-enable session */
			else {
				mchSess->session = 0;
				ipmiMsgCloseSess( mchSess, data );
			}
			
			epicsMutexUnlock( mch->mutex );
		}

		else if ( !(strcmp( task, "RESET" )) && mchSess->session ) {

			epicsMutexLock( mch->mutex );
		       	s = ipmiMsgColdReset( mchSess, data );
			epicsMutexUnlock( mch->mutex );

			if ( s ) {
				recGblSetSevr( pbo, WRITE_ALARM, INVALID_ALARM );
				return ERROR;
			}
		}

		else if ( !(strcmp( task, "INIT" )) && mchSess->session ) {

			epicsMutexLock( mch->mutex );
		       	mchInitDone[mchSess->instance] = (pbo->val) ? MCH_INIT_DONE : MCH_INIT_NOT_DONE;
			epicsMutexUnlock( mch->mutex );

			if ( drvMchInitScan )
				scanIoRequest( drvMchInitScan );
		}


		pbo->udf = FALSE;
		return status;
	}
	else {
		recGblSetSevr( pbo, WRITE_ALARM, INVALID_ALARM );
		return ERROR;
	}
}

static long
init_bi(struct biRecord *pbi)
{
	scanIoInit( &drvMchStatScan );
	return 0;
}

/*
** Add this record to our IOSCANPVT list.
*/
static long 
bi_ioint_info(int cmd, struct biRecord *pbi, IOSCANPVT *iopvt)
{
       *iopvt = drvMchStatScan;
       return 0;
} 

static long 
init_bi_record(struct biRecord *pbi)
{
MchRec  recPvt  = 0; /* Info stored with record */
MchDev  mch     = 0; /* MCH device data structures */
char    *node;       /* Network node name, stored in parm */
char    *task   = 0; /* Optional additional parameter appended to parm */
char    *p;
short    c, s;
long     status = 0;
char     str[40];

        if ( VME_IO != pbi->inp.type ) {
		status = S_dev_badBus;
		goto bail;
	}

        c = pbi->inp.value.vmeio.card;
        s = pbi->inp.value.vmeio.signal;

        if ( c < 0 )
		status = S_dev_badCard;

        if ( s < 0 )
		status = S_dev_badSignal;

        if ( ! (recPvt = calloc( 1, sizeof( *recPvt ) )) ) {
		sprintf( str, "No memory for recPvt structure");
		status = S_rec_outMem;
		goto bail;
        }

        /* Break parm into node name and optional parameter */
        node = strtok( pbi->inp.value.vmeio.parm, "+" );
        if ( (p = strtok( NULL, "+")) ) {
                task = p;
                if ( strcmp( task, "STAT" ) ) {
			sprintf( str, "Unknown task parameter %s", task);
			status = S_dev_badSignal;
		}
        }

        /* Find data structure in registry by MCH name */
        if ( (mch = devMchFind(node)) ) {
                recPvt->mch  = mch;
                recPvt->task = task;
                pbi->dpvt  = recPvt;
                
        }
        else {
		sprintf( str, "Failed to locate data structure" );
		status = S_dev_noDeviceFound;
        }

bail:
	if ( status ) {
	       recGblRecordError( status, (void *)pbi , (const char *)str );
	       pbi->pact=TRUE;
	}

	return status;
}

static long 
read_bi(struct biRecord *pbi)
{
MchRec   recPvt = pbi->dpvt;
MchDev   mch;
MchData  mchData;
MchSess  mchSess;
MchSys   mchSys;
char    *task;
long     status = 0;
int      s = 0;

	if ( !recPvt )
		return status;

	mch     = recPvt->mch;
	mchData = mch->udata;
	mchSess = mchData->mchSess;
	mchSys  = mchSysData[mchSess->instance];

	task    = recPvt->task;

	if ( task ) {

		if ( !(strcmp( task, "STAT" )) )

			pbi->rval = mchIsAlive[mchSess->instance];
	}

	pbi->udf = FALSE;
	return status;
}

static long
init_mbbi(struct biRecord *pmbbi)
{
	scanIoInit( &drvMchInitScan );
	return 0;
}

/*
** Add this record to our IOSCANPVT list.
*/
static long 
mbbi_ioint_info(int cmd, struct mbbiRecord *pmbbi, IOSCANPVT *iopvt)
{
       *iopvt = drvMchInitScan;
       return 0;
} 

static long 
init_mbbi_record(struct mbbiRecord *pmbbi)
{
MchRec  recPvt  = 0; /* Info stored with record */
MchDev  mch     = 0; /* MCH device data structures */
char    *node;       /* Network node name, stored in parm */
char    *task   = 0; /* Optional additional parameter appended to parm */
char    *p;
short    c, s;
long     status = 0;
char     str[40];

        if ( VME_IO != pmbbi->inp.type ) {
		status = S_dev_badBus;
		goto bail;
	}

        c = pmbbi->inp.value.vmeio.card;
        s = pmbbi->inp.value.vmeio.signal;

        if ( c < 0 )
		status = S_dev_badCard;

        if ( s < 0 )
		status = S_dev_badSignal;

        if ( ! (recPvt = calloc( 1, sizeof( *recPvt ) )) ) {
		sprintf( str, "No memory for recPvt structure");
		status = S_rec_outMem;
		goto bail;
        }

        /* Break parm into node name and optional parameter */
        node = strtok( pmbbi->inp.value.vmeio.parm, "+" );
        if ( (p = strtok( NULL, "+")) ) {
                task = p;
                if ( strcmp( task, "HS") && strcmp( task, "FAN") && strcmp( task, "INIT" ) ) {
			sprintf( str, "Unknown task parameter %s", task);
			status = S_dev_badSignal;
		}
        }

        /* Find data structure in registry by MCH name */
        if ( (mch = devMchFind(node)) ) {
                recPvt->mch  = mch;
                recPvt->task = task;
                pmbbi->dpvt  = recPvt;
                
        }
        else {
		sprintf( str, "Failed to locate data structure" );
		status = S_dev_noDeviceFound;
        }

bail:
	if ( status ) {
	       recGblRecordError( status, (void *)pmbbi , (const char *)str );
	       pmbbi->pact=TRUE;
	}

	return status;
}

static long 
read_mbbi(struct mbbiRecord *pmbbi)
{
uint8_t  data[MSG_MAX_LENGTH] = { 0 };
MchRec   recPvt = pmbbi->dpvt;
MchDev   mch;
MchData  mchData;
MchSess  mchSess;
MchSys   mchSys;
char    *task;
uint8_t  value = 0, sensor;
uint16_t addr = 0; /* get addr */
short    index  = pmbbi->inp.value.vmeio.signal; /* Sensor index or FRU id */
long     status = 0;
int      s = 0;
size_t   responseSize;

	if ( !recPvt )
		return status;

	mch     = recPvt->mch;
	mchData = mch->udata;
	mchSess = mchData->mchSess;
	mchSys  = mchSysData[mchSess->instance];

	task    = recPvt->task;

	if ( task ) {

		if ( !(strcmp( task, "FAN" )) ) {

			value = ( mchSys->fru[index].fanProp & (1<<7) ) ? 1 : 0;
			pmbbi->rval = value;
		}

		else if ( !(strcmp( task, "INIT" )) )

			pmbbi->rval = mchInitDone[mchSess->instance];

		else if ( !(strcmp( task, "HS")) && mchSess->session && mchInitDone[mchSess->instance] ) {

			if ( mchIsAlive[mchSess->instance] ) {

				responseSize = mchSys->sens[index].readMsgLength;
				sensor = mchSys->sens[index].sdr.number;

				epicsMutexLock( mch->mutex );

				if ( !(s = ipmiMsgReadSensor( mchSess, data, sensor, addr, &responseSize )) ) {				
					/* Store raw sensor reading */
					value = data[IPMI_RPLY_HS_SENSOR_READING_OFFSET];
					mchSys->sens[index].val = value;
				}

				epicsMutexUnlock( mch->mutex );

				if ( s ) {
					recGblSetSevr( pmbbi, READ_ALARM, INVALID_ALARM );
					return ERROR;
				}
				pmbbi->rval = value;
			}
		}	
	}

	pmbbi->udf = FALSE;
	return status;
}

/* Create the dset for mbbo support */

static long 
init_mbbo_record(struct mbboRecord *pmbbo)
{
MchRec  recPvt  = 0; /* Info stored with record */
MchDev  mch     = 0; /* MCH device data structures */
char    *node;       /* Network node name, stored in parm */
char    *task   = 0; /* Optional additional parameter appended to parm */
char    *p;
short    c, s;
long     status = 0;
char     str[40];

        if ( VME_IO != pmbbo->out.type ) {
		status = S_dev_badBus;
		goto bail;
	}

        c = pmbbo->out.value.vmeio.card;
        s = pmbbo->out.value.vmeio.signal;

        if ( c < 0 )
		status = S_dev_badCard;

        if ( s < 0 )
		status = S_dev_badSignal;

        if ( ! (recPvt = calloc( 1, sizeof( *recPvt ) )) ) {
		sprintf( str, "No memory for recPvt structure");
		status = S_rec_outMem;
		goto bail;
        }

        /* Break parm into node name and optional parameter */
        node = strtok( pmbbo->out.value.vmeio.parm, "+" );
        if ( (p = strtok( NULL, "+")) ) {
                task = p;
                if ( strcmp( task, "CHAS" ) && strcmp( task, "FRU") ) { 
			sprintf( str, "Unknown task parameter %s", task);
			status = S_dev_badSignal;
		}
        }

        /* Find data structure in registry by MCH name */
        if ( (mch = devMchFind(node)) ) {
                recPvt->mch  = mch;
                recPvt->task = task;
                pmbbo->dpvt  = recPvt;
        }
        else {
		sprintf( str, "Failed to locate data structure" );
		status = S_dev_noDeviceFound;
        }

bail:
	if ( status ) {
	       recGblRecordError( status, (void *)pmbbo , (const char *)str );
	       pmbbo->pact=TRUE;
	}

	return status;
}

static long 
write_mbbo(struct mbboRecord *pmbbo)
{
uint8_t  data[MSG_MAX_LENGTH] = { 0 };
MchRec   recPvt = pmbbo->dpvt;
MchDev   mch;
MchData  mchData;
MchSess  mchSess;
MchSys   mchSys;
char    *task;
long     status = NO_CONVERT;
int      cmd;
int      id     = pmbbo->out.value.vmeio.signal;
int      index, i = 0, s = 0;
volatile uint8_t val;

	if ( !recPvt )
		return status;

	mch     = recPvt->mch;
	mchData = mch->udata;
	mchSess = mchData->mchSess;
	mchSys  = mchSysData[mchSess->instance];

	task    = recPvt->task;

	if ( mchIsAlive[mchSess->instance] && mchSess->session ) {

		if ( !(strcmp( task, "CHAS" )) && mchInitDone[mchSess->instance] ) {
			epicsMutexLock( mch->mutex );
			ipmiMsgChassisControl( mchSess, data, pmbbo->val );
			epicsMutexUnlock( mch->mutex );
		}
		else if ( !(strcmp( task, "FRU" )) ) {

			if ( pmbbo->val > 1 ) { /* reset not supported yet */
				recGblSetSevr( pmbbo, STATE_ALARM, MAJOR_ALARM );
				return ERROR;
			}

			cmd = ( pmbbo->val == 2 ) ? 0 : pmbbo->val; 

			epicsMutexLock( mch->mutex );

			ipmiMsgSetFruActPolicyHelper( mchSess, data, id, cmd );

			epicsMutexUnlock( mch->mutex );
	      
			if ( pmbbo->val == 2 ) {

				index = mchSys->fru[id].hotswap;
				val   = (volatile uint8_t)mchSys->sens[index].val;

				while( val != 2 ) {
					epicsThreadSleep(1);
					i++;
					if (i > RESET_TIMEOUT ) {
						recGblSetSevr( pmbbo, TIMEOUT_ALARM, MAJOR_ALARM );
						return ERROR; /* also set FRU status */
					}
				}

				epicsMutexLock( mch->mutex );

				s = ipmiMsgSetFruActPolicyHelper( mchSess, data, id, 1 );

				epicsMutexUnlock( mch->mutex );
			 }	    
		}


		if ( s ) {
			recGblSetSevr( pmbbo, WRITE_ALARM, INVALID_ALARM );
			return ERROR;
		}

		pmbbo->udf = FALSE;
		return status;
	}
	else {
		recGblSetSevr( pmbbo, WRITE_ALARM, INVALID_ALARM );
		return ERROR;
	}
}

/* 
 * Device support to read FRU data
 */

static long 
init_fru_ai_record(struct aiRecord *pai)
{
MchRec  recPvt  = 0; /* Info stored with record */
MchDev  mch     = 0; /* MCH device data structures */
char   *node;        /* Network node name, stored in parm */
char   *task    = 0; /* Optional additional parameter appended to parm */
char   *p;
short   c, s;
long    status = 0;
char    str[40];

        if ( VME_IO != pai->inp.type ) {
		status = S_dev_badBus;
		goto bail;
	}

        c = pai->inp.value.vmeio.card;
        s = pai->inp.value.vmeio.signal;

        if ( c < 0 )
		status = S_dev_badCard;

        if ( s < 0 )
		status = S_dev_badSignal;

	if ( ! (recPvt = calloc( 1, sizeof( *recPvt ) )) ) {
		sprintf( str, "No memory for recPvt structure");
		status = S_rec_outMem;
		goto bail;
	}

	/* Break parm into node name and optional parameter */
	node = strtok( pai->inp.value.vmeio.parm, "+" );
	if ( (p = strtok( NULL, "+" )) ) {
		task = p;
		if ( strcmp( task, "TYPE" ) && strcmp( task, "BSN" ) && strcmp( task, "PSN" ) && strcmp( task, "FAN")  ) {
			sprintf( str, "Unknown task parameter %s", task);
			status = S_dev_badSignal;
		}
	}

	/* Find data structure in registry by MCH name */
	if ( (mch = devMchFind(node)) ) {
		recPvt->mch  = mch;
                recPvt->task = task;
		pai->dpvt    = recPvt;                
	}
	else {
		sprintf( str, "Failed to locate data structure" );
		status = S_dev_noDeviceFound;
		goto bail;
        }

bail:
	if ( status ) {
	       recGblRecordError( status, (void *)pai , (const char *)str );
	       pai->pact=TRUE;
	}

	return status;
}

static long 
read_fru_ai(struct aiRecord *pai)
{
uint8_t  data[MSG_MAX_LENGTH] = { 0 };
MchRec   recPvt  = pai->dpvt;
MchDev   mch;
MchData  mchData;
MchSess  mchSess;
MchSys   mchSys;
char    *task;
int      id   = pai->inp.value.vmeio.signal;
long     status = NO_CONVERT;
Fru      fru;
int      s = 0;

	if ( !recPvt )
		return status;

	mch     = recPvt->mch;
	mchData = mch->udata;
	mchSess = mchData->mchSess;
	mchSys  = mchSysData[mchSess->instance];

	task    = recPvt->task;
	fru     = &mchSys->fru[id];

	if ( mchInitDone[mchSess->instance] ) {

		if ( !(strcmp( task, "BSN" )) && fru->board.sn.data )
       			pai->val = (epicsFloat64)(*fru->board.sn.data);

		else if ( !(strcmp( task, "PSN" )) && fru->prod.sn.data )
       			pai->val = (epicsFloat64)(*fru->prod.sn.data);

		else if ( !(strcmp( task, "FAN")) ) {

			if ( mchIsAlive[mchSess->instance] && mchSess->session ) {

				epicsMutexLock( mch->mutex );

				if ( !(s = ipmiMsgGetFanLevel( mchSess, data, id )) ) 
					pai->rval = data[IPMI_RPLY_GET_FAN_LEVEL_OFFSET];

				epicsMutexUnlock( mch->mutex );

				if ( s ) {
					recGblSetSevr( pai, READ_ALARM, INVALID_ALARM );
					return ERROR;
				}

				pai->val  = pai->rval;
			}

			pai->udf = FALSE;
		}

#ifdef DEBUG
printf("read_fru_ai: %s FRU id is %i, value is %.0f\n",pai->name, id, pai->val);
#endif
		pai->udf = FALSE;
		return status;
	}
	else {
		recGblSetSevr( pai, READ_ALARM, INVALID_ALARM );
		return ERROR;
	}
}

/*
** Add this record to our IOSCANPVT list.

static long 
ai_fru_ioint_info(int cmd, struct aiRecord *pai, IOSCANPVT *iopvt)
{
       *iopvt = drvMchFruScan;
       return 0;
} 
*/

/* Create the dset for stringin support */
static long 
init_fru_stringin_record(struct stringinRecord *pstringin)
{
MchRec  recPvt  = 0; /* Info stored with record */
MchDev  mch     = 0; /* MCH device data structures */
char    *node;       /* Network node name, stored in parm */
char    *task   = 0; /* Optional additional parameter appended to parm */
char    *p;
short    c, s; 
long    status  = 0;
char    str[40];

        if ( VME_IO != pstringin->inp.type ) {
		status = S_dev_badBus;
		goto bail;
        }

        c = pstringin->inp.value.vmeio.card;
        s = pstringin->inp.value.vmeio.signal;

        if ( c < 0 )
		status = S_dev_badCard;

        if ( s < 0 )
		status = S_dev_badSignal;

	if ( ! (recPvt = calloc( 1, sizeof( *recPvt ) )) ) {
		sprintf( str, "No memory for recPvt structure");
		status = S_rec_outMem;
		goto bail;
	}

	/* Break parm into node name and optional parameter */
	node = strtok( pstringin->inp.value.vmeio.parm, "+" );
	if ( (p = strtok( NULL, "+" )) ) {
		task = p;
		if ( strcmp( task, "BMF" ) && strcmp( task, "BP" ) && strcmp( task, "PMF" ) && strcmp( task, "PP") && strcmp( task, "BPN" ) && strcmp( task, "PPN")) {
			sprintf( str, "Unknown task parameter %s", task);
			status = S_dev_badSignal;
		}
	}

	/* Find data structure in registry by MCH name */
	if ( (mch = devMchFind(node)) ) {
		recPvt->mch  = mch;
                recPvt->task = task;
		pstringin->dpvt    = recPvt;
                
	}
	else {
		sprintf( str, "Failed to locate data structure" );
		status = S_dev_noDeviceFound;
        }

bail:

	if ( status ) {
	       recGblRecordError( status, (void *)pstringin , (const char *)str );
	       pstringin->pact=TRUE;
	}

	return status;
}

static long 
read_fru_stringin(struct stringinRecord *pstringin)
{
MchRec   recPvt  = pstringin->dpvt;
MchDev   mch;
MchData  mchData;
MchSess  mchSess;
MchSys   mchSys;
char    *task;
short    id = pstringin->inp.value.vmeio.signal;   /* FRU ID */
int      i;
long     status = NO_CONVERT;
Fru      fru;
uint8_t  l = 0, *d = 0; /* FRU data length and raw */

	if ( !recPvt )
		return status;

	mch     = recPvt->mch;
	mchData = mch->udata;
	mchSess = mchData->mchSess;
	mchSys  = mchSysData[mchSess->instance];

	task    = recPvt->task;
	fru     = &mchSys->fru[id];

	if ( mchInitDone[mchSess->instance] ) {

		if ( !(strcmp( task, "BMF" )) ) {
			d = fru->board.manuf.data;
			l = fru->board.manuf.length;
		}
		else if ( !(strcmp( task, "BP" )) ) {
			d = fru->board.prod.data;
			l = fru->board.prod.length;
		}
		else if ( !(strcmp( task, "PMF" )) ) {
			d = fru->board.manuf.data;
			l = fru->prod.manuf.length;
		}
		else if ( !(strcmp( task, "PP" )) ) {
			d = fru->prod.prod.data;
			l = fru->prod.prod.length;
		}
		else if ( !(strcmp( task, "BPN" )) ) {
			d = fru->board.part.data;
			l = fru->board.part.length;
		}
		else if ( !(strcmp( task, "PPN" )) ) {
			d = fru->prod.part.data;
			l = fru->prod.part.length;
		}

		if ( d ) {

			l = ( l <= MAX_STRING_LENGTH ) ? l : MAX_STRING_LENGTH;
			for ( i = 0; i < l; i++ )
				pstringin->val[i] = d[i];
		}

#ifdef DEBUG
errlogPrintf("read_fru_stringin: %s, task is %s, card is %i, id into FRU array is %i\n", pstringin->name, task, pstringin->inp.value.vmeio.card, pstringin->inp.value.vmeio.signal);
#endif
		pstringin->udf = FALSE;
		return status;
	}
	else {
		recGblSetSevr( pstringin, READ_ALARM, INVALID_ALARM );
		return ERROR;
	}
}

/*
** Add this record to our IOSCANPVT list.

static long 
stringin_fru_ioint_info(int cmd, struct stringinRecord *pstringin, IOSCANPVT *iopvt)
{
       *iopvt = drvMchFruScan;
       return 0;
} 
*/

static long
init_fru_longout_record(struct longoutRecord *plongout)
{
MchRec  recPvt  = 0; /* Info stored with record */
MchDev  mch     = 0; /* MCH device data structures */
MchData mchData = 0;
MchSess mchSess = 0;
MchSys  mchSys  = 0;
char   *node;        /* Network node name, stored in parm */
char   *task    = 0; /* Optional additional parameter appended to parm */
char   *p;
short   c, s;
long    status = 0;
char    str[40];

        if ( VME_IO != plongout->out.type ) {
		status = S_dev_badBus;
		goto bail;
	}

        c = plongout->out.value.vmeio.card;
        s = plongout->out.value.vmeio.signal; /* FRU ID */

        if ( c < 0 )
		status = S_dev_badCard;

        if ( s < 0 )
		status = S_dev_badSignal;

	if ( ! (recPvt = calloc( 1, sizeof( *recPvt ) )) ) {
		sprintf( str, "No memory for recPvt structure");
		status = S_rec_outMem;
		goto bail;
	}

	/* Break parm into node name and optional parameter */
	node = strtok( plongout->out.value.vmeio.parm, "+" );
	if ( (p = strtok( NULL, "+" )) ) {
		task = p;
		if ( strcmp( task, "FAN" ) ) {
			sprintf( str, "Unknown task parameter %s", task);
			status = S_dev_badSignal;
		}
	}

	/* Find data structure in registry by MCH name */
	if ( (mch = devMchFind(node)) ) {
		recPvt->mch  = mch;
                recPvt->task = task;
		plongout->dpvt    = recPvt;                
	}
	else {
		sprintf( str, "Failed to locate data structure" );
		status = S_dev_noDeviceFound;
		goto bail;
        }

	if ( !(strcmp( task, "FAN" )) ) {
		mchData = mch->udata;
		mchSess = mchData->mchSess;
		mchSys  = mchSysData[mchSess->instance];
       		plongout->drvl = plongout->lopr = mchSys->fru[s].fanMin;
	       	plongout->drvh = plongout->hopr = mchSys->fru[s].fanMax;
	}

bail:
	if ( status ) {
	       recGblRecordError( status, (void *)plongout , (const char *)str );
	       plongout->pact=TRUE;
	}

	return status;
}

static long 
write_fru_longout(struct longoutRecord *plongout)
{
MchRec   recPvt  = plongout->dpvt;
MchDev   mch;
MchData  mchData;
MchSess  mchSess;
MchSys   mchSys;
char    *task;
int      id      = plongout->out.value.vmeio.signal; /* FRU ID */
long     status  = NO_CONVERT;
Fru      fru;
uint8_t  data[MSG_MAX_LENGTH] = { 0 };
int      s = 0;

	if ( !recPvt )
		return status;

	mch     = recPvt->mch;
	mchData = mch->udata;
	mchSess = mchData->mchSess;
	mchSys  = mchSysData[mchSess->instance];

	task    = recPvt->task;
	fru     = &mchSys->fru[id];

#ifdef DEBUG
printf("write_fru_longout: %s FRU id is %i, value is %.0f\n",plongout->name, id, plongout->val);
#endif

	if ( mchIsAlive[mchSess->instance] && mchSess->session && mchInitDone[mchSess->instance] ) {
		if ( !(strcmp( task, "FAN" )) ) {

			epicsMutexLock( mch->mutex );

			s = ipmiMsgSetFanLevel( mchSess, data, id, plongout->val );

			epicsMutexUnlock( mch->mutex );

			if ( s ) {
				recGblSetSevr( plongout, WRITE_ALARM, INVALID_ALARM );
				return ERROR;
			}
		}

		plongout->udf = FALSE;
		return status;
	}
	else {
       	       	recGblSetSevr( plongout, WRITE_ALARM, INVALID_ALARM );
		return ERROR;		
	}
}

/*
** Add this record to our IOSCANPVT list.

static long 
longout_fru_ioint_info(int cmd, struct longoutRecord *plongout, IOSCANPVT *iopvt)
{
       *iopvt = drvMchFruScan;
       return 0;
} 
*/
