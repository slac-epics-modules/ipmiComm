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

             * "sens": read sensor
	   
	 devAiFru
         --------
         *   init_fru_ai                  - Record initialization
         *   fru_ai_ioint_info            - Record initialization
         *   init_fru_ai_record           - Record initialization
         *   read_fru_ai                  - Read analog input

	   Supported operations:

             * "fan"     : Get fan level
             * "pwr"     : Get FRU power levels (steady state and early, 
                                                 desired and actual)

         Binary Output Device Support:
         -------------------------------------------------------

	 devBoMch
         --------

         *   init_bo_record           - Record initialization
         *   write_bo                 - Write binary output

	   Supported operations:

             * "reset": perform IPMI cold reset
             * "sess":  close/open session with MCH
             * "init":  override 'MCH communication initialized' bit

         Binary Input Device Support:
         -------------------------------------------------------

	 devBiMch
         ----------

	 *   init_bi                   - bi initialization     
         *   bi_ioint_info             - Add to i/o scan list   
         *   init_bi_record            - Record initialization
         *   read_bi                   - Read binary input

	   Supported operations:

             * "stat": read crate online/offline status
             * "spres": sensor presence
             * "fpres": FRU presence

         Multibit-Binary Input Device Support:
         -------------------------------------------------------

	 devMbbiMch
         ----------

	 *   init_mbbi                 - mbbi initialization     
         *   mbbi_ioint_info           - Add to i/o scan list   
         *   init_mbbi_record          - Record initialization
         *   read_mbbi                 - Read multi-bit binary input

	   Supported operations:

             * "init": communication with crate initialized
             * "fan":  read fan auto-adjustment
             * "hs":   read FRU hot-swap sensor 
	     * "mch":  MCH vendor


         Multibit-Binary Output Device Support:
         -------------------------------------------------------

	 devMbboMch
         ----------

         *   init_mbbo_record          - Record initialization
         *   write_mbbo                - Write multi-bit binary output

	   Supported operations:

             * "chas": chassis control on/off/reset
             * "fru":  FRU on/off
             * "dbg":   control debug messages for one MCH

         String Input Device Support:
         -------------------------------------------------------

	 devStringinFru
         --------------

         *   init_fru_stringin            - stringin initialization
         *   fru_stringin_ioint_info      - Add to i/o scan list
         *   init_fru_stringin_record     - Record initialization
         *   read_fru_stringin            - Read string input

	   Supported operations:

             * "bmf":  FRU board manufacturer
             * "bp":   FRU board product name
             * "bpn":  FRU board part number
             * "pmf":  FRU product manufacturer
             * "pp":   FRU product product name
             * "ppn":  FRU product part number

         Long Integer Output Device Support:
         -------------------------------------------------------

	 devLongoutFru
         -------------

         *   init_fru_longout_record      - Record initialization
         *   write_longout                - Write long integer output

	   Supported operations:

             * "fan":  fan level control

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
#include <link.h>

#include <ipmiDef.h>
#include <ipmiMsg.h>
#include <drvMch.h>
#include <drvMchMsg.h>
#include <devMch.h>

extern uint32_t mchStat[MAX_MCH];

#define MAX_STRING_LENGTH 39
#define MAX_EGU_LENGTH 16

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

static long init_fru_ai(struct aiRecord *pai);
static long init_fru_ai_record(struct aiRecord *pai);
static long read_fru_ai(struct aiRecord *pai);
static long ai_fru_ioint_info(int cmd, struct aiRecord *pai, IOSCANPVT *iopvt);

static long init_fru_longout_record(struct longoutRecord *plongout);
static long write_fru_longout(struct longoutRecord *plongout);

static long init_fru_stringin(struct stringinRecord *pstringin);
static long init_fru_stringin_record(struct stringinRecord *pstringin);
static long read_fru_stringin(struct stringinRecord *pstringin);
static long stringin_fru_ioint_info(int cmd, struct stringinRecord *pstringin, IOSCANPVT *iopvt);

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
MCH_DEV_SUP_SET devAiMch         = {6, NULL, NULL,              init_ai_record,           NULL,                    read_ai,           NULL};
MCH_DEV_SUP_SET devBoMch         = {6, NULL, NULL,              init_bo_record,           NULL,                    write_bo,          NULL};
MCH_DEV_SUP_SET devMbboMch       = {6, NULL, NULL,              init_mbbo_record,         NULL,                    write_mbbo,        NULL};
MCH_DEV_SUP_SET devBiMch         = {6, NULL, init_bi,           init_bi_record,           bi_ioint_info,           read_bi,           NULL};
MCH_DEV_SUP_SET devMbbiMch       = {6, NULL, init_mbbi,         init_mbbi_record,         mbbi_ioint_info,         read_mbbi,         NULL};
MCH_DEV_SUP_SET devAiFru         = {6, NULL, init_fru_ai,       init_fru_ai_record,       ai_fru_ioint_info,       read_fru_ai,       NULL};
MCH_DEV_SUP_SET devLongoutFru    = {6, NULL, NULL,              init_fru_longout_record,  NULL,                    write_fru_longout, NULL};
MCH_DEV_SUP_SET devStringinFru   = {6, NULL, init_fru_stringin, init_fru_stringin_record, stringin_fru_ioint_info, read_fru_stringin, NULL};

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
	return (MchDev)registryFind(registryId, name);
}


/*--- end stolen ---*/

static float 
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
	else if ( l == SENSOR_CONV_LN )
		value = log( value );
	else if ( l == SENSOR_CONV_LOG10 )
		value = log10( value );
	else if ( l == SENSOR_CONV_LOG2 )
		value = log( value )/log( 2 );
	else if ( l == SENSOR_CONV_E )
		value = exp( value );
	else if ( l == SENSOR_CONV_EXP10 )
		value = pow( value, 10 );
	else if ( l == SENSOR_CONV_EXP2 )
		value = pow( value, 2 );
	else if ( l == SENSOR_CONV_1_X )
		value = 1/value;
	else if ( l == SENSOR_CONV_SQR )
		value = pow( value, 2);
	else if ( l == SENSOR_CONV_CUBE )
		value = pow( value, 1/3 );
	else if ( l == SENSOR_CONV_SQRT )
		value = sqrt( value );
	else if ( l == SENSOR_CONV_CUBE_NEG1 )
		value = pow( value, -1/3 );
	else
		printf("unknown sensor conversion algorithm\n");

	return value;
}

static short
sensLkup(MchSys mchSys, struct camacio link) {

	return mchSys->sensLkup[link.b][link.c][link.n];
}

static void
sensEgu(char *egu, unsigned units) {

	switch ( units ) {

		default:
			strcpy( egu, "" );

		case SENSOR_UNITS_UNSPEC:
			strcpy( egu, "Unspecified");
			break;

		case SENSOR_UNITS_DEGC:   
			strcpy( egu, "DegC");
			break;

		case SENSOR_UNITS_DEGF:   
			strcpy( egu, "DegF");
			break;

		case SENSOR_UNITS_DEGK:   
			strcpy( egu, "DegK");
			break;

		case SENSOR_UNITS_VOLTS:  
			strcpy( egu, "V");
			break;

		case SENSOR_UNITS_AMPS:   
			strcpy( egu, "A");
			break;

		case SENSOR_UNITS_WATTS:  
			strcpy( egu, "W");
			break;

		case SENSOR_UNITS_JOULES: 
			strcpy( egu, "J");
			break;

		case SENSOR_UNITS_COULOMBS:
			strcpy( egu, "C");
			break;

		case SENSOR_UNITS_RPM:
			strcpy( egu, "RPM");
			break;
	}

}		

/* Perform some common dev sup checks */
static MchRec
init_record_chk(DBLINK *plink, long *status, char *str) {
short  f, t, s;
MchRec rval = 0;

        if ( CAMAC_IO != plink->type ) {
		*status = S_dev_badBus;
		return rval;
	}

        f = plink->value.camacio.b;
        t = plink->value.camacio.c;
        s = plink->value.camacio.n;

        if ( f < 0 ) {
	       	sprintf( str, "FRU id (Branch) is %i; must >= 0", f);
		*status = S_dev_badSignal;
		return rval;
	}

        if ( t < 0 ) {
	       	sprintf( str, "Sensor type (Crate) is %i; must >= 0", t);
		*status = S_dev_badSignal;
	        return rval;
	}

        if ( s < 0 ) {
	       	sprintf( str, "Sensor instance (Station) is %i; must >= 0", s);
		*status = S_dev_badSignal;
		return rval;
	}

	if ( ! (rval = calloc( 1, sizeof( *rval ) )) ) {
		sprintf( str, "No memory for recPvt structure");
		*status = S_rec_outMem;
		return rval;
	}

	return rval;
}

/* Find data structure in registry by MCH name */
static long
init_record_find(MchDev mch, MchRec recPvt, char *node, char *task, long *status, char *str) {

	if ( (mch = devMchFind( node )) ) {
		recPvt->mch  = mch;
                recPvt->task = task;
		return 0;
	}
	else {
		sprintf( str, "Failed to locate data structure" );
		*status = S_dev_noDeviceFound;
		return -1;
	}
}

static long 
init_ai_record(struct aiRecord *pai)
{
MchRec   recPvt  = 0; /* Info stored with record */
MchDev   mch     = 0; /* MCH device data structures */
char    *node    = 0; /* Network node name, stored in parm */
char    *task    = 0; /* Optional additional parameter appended to parm */
char    *p;
long     status  = 0;
char     str[40];
DBLINK  *plink   = &pai->inp;

	if ( ! ( recPvt = init_record_chk( plink, &status, str )) )
		goto bail;

	/* Break parm into node name and optional parameter */
	node = strtok( pai->inp.value.camacio.parm, "+" );
	if ( (p = strtok( NULL, "+" )) ) {
		task = p;
		if ( strcmp( task, "sens" ) ) { 
			sprintf( str, "Unknown task parameter %s", task);
			status = S_dev_badSignal;
		}
	}

	if ( init_record_find( mch, recPvt, node, task, &status, str ) )
		goto bail;
	else
		pai->dpvt = recPvt;

	/* To do: implement alarms; use upper, lower, critical, non-critical etc.
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

/* Take advantage of regular read_ai calls
 * to periodically check that our MCH data matches
 * the live configuration. Thus read_ai does not check
 * that MCH_INIT_DONE is true, but other read routines do
 */
static long 
read_ai(struct aiRecord *pai)
{
MchRec   recPvt  = pai->dpvt;
MchDev   mch;
MchData  mchData;
MchSess  mchSess;
MchSys   mchSys;
Sensor   sens;
SdrFull  sdr;
char     egu[16];
uint8_t  data[MSG_MAX_LENGTH] = { 0 };
uint8_t  raw = 0, sensor, bits;
short    index; /* Sensor index */
long     status = NO_CONVERT;
int      s = 0, inst;
size_t   responseSize;
uint8_t  lun;
int      offs; 

	if ( !recPvt )
		return status;

	mch     = recPvt->mch;
	mchData = mch->udata;
	mchSess = mchData->mchSess;
	mchSys  = mchData->mchSys;
	inst    = mchSess->instance;

	if ( MCH_ONLN( mchStat[inst] ) && mchSess->session ) {    

		epicsMutexLock( mch->mutex );

		/* If flag is set, check MCH configuration */
		if ( MCH_CNFG_CHK( mchStat[inst] ) ) {
			if ( mchCnfgChk( mchData ) ) {
				epicsMutexUnlock( mch->mutex );
				return ERROR;
			}
		}

		if ( -1 == (index = sensLkup( mchSys, pai->inp.value.camacio )) ) {
       			epicsMutexUnlock( mch->mutex );
			return ERROR;
		}

		sens = &mchSys->sens[index];
		offs = ( mchSess->type == MCH_TYPE_NAT ) ? IPMI_RPLY_OFFSET_NAT : 0;

		responseSize = sens->readMsgLength;
		sensor       = sens->sdr.number;
		lun          = sens->sdr.lun & 0x3;

		if ( !(s = mchMsgReadSensor( mchData, data, sensor, lun, &responseSize )) ) {
			bits = data[IPMI_RPLY_SENSOR_ENABLE_BITS_OFFSET + offs];

			if ( IPMI_SENSOR_READING_DISABLED(bits) || IPMI_SENSOR_SCANNING_DISABLED(bits) ) {
				if ( MCH_DBG( mchStat[inst] ) )
					printf("%s sensor reading/state unavailable or scanning disabled. Bits: %02x\n", pai->name, bits);
				s = ERROR;
			}
			else
				sens->val = raw = data[IPMI_RPLY_SENSOR_READING_OFFSET + offs];
		}
	
		epicsMutexUnlock( mch->mutex );

		sdr = &sens->sdr;

		if ( !mchSys->sens[index].cnfg ) {
			sensEgu( egu, sdr->units2 );
			strcpy( pai->egu,  egu );
			if ( sdr->str )
				strcpy( pai->desc, sdr->str );
			sens->cnfg = 1;
		}

		if ( s ) {

			if ( MCH_DBG( mchStat[inst] ) )
				printf("%s writeread error sensor %02x index %i\n", pai->name, sensor, index);
			recGblSetSevr( pai, READ_ALARM, INVALID_ALARM );
			return ERROR;
		}

		/* All of our conversions are for Full Sensor SDRs */
		if ( sdr->recType != SDR_TYPE_FULL_SENSOR )
			pai->val = raw;
		else
			pai->val = (float)sensorConversion( sdr, raw );

		if ( MCH_DBG( mchStat[inst] ) > 1 )
			printf("%s read_ai: sensor index is %i, sensor number is %i, value is %.0f, rval is %i\n",
			pai->name, index, sdr->number, pai->val, pai->rval);

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
long    status  = 0;
char    str[40];
DBLINK  *plink  = &pbo->out;

	if ( ! ( recPvt = init_record_chk( plink, &status, str )) )
		goto bail;

        /* Break parm into node name and optional parameter */
        node = strtok( pbo->out.value.camacio.parm, "+" );
        if ( (p = strtok( NULL, "+")) ) {
                task = p;
                if ( strcmp( task, "reset") && strcmp( task, "sess") && strcmp( task, "init") ) {
			sprintf( str, "Unknown task parameter %s", task);
			status = S_dev_badSignal;
		}
        }

	if ( init_record_find( mch, recPvt, node, task, &status, str ) )
		goto bail;
	else
		pbo->dpvt = recPvt;

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
long     status = 0;
int      inst;

	if ( !recPvt )
		return status;

	mch     = recPvt->mch;
	mchData = mch->udata;
	mchSess = mchData->mchSess;
	mchSys  = mchData->mchSys;
	inst    = mchSess->instance;

	task    = recPvt->task;

	if (  MCH_ONLN( mchStat[inst] ) ) {

		if ( !(strcmp( task, "sess" )) ) {

			epicsMutexLock( mch->mutex );

			if ( pbo->val ) /* could change this to be purely soft; session will time out */
				mchSess->session = 1; /* Re-enable session */
			else {
				mchSess->session = 0;
				mchMsgCloseSess( mchSess, mchData->ipmiSess, data );
			}
			
			epicsMutexUnlock( mch->mutex );
		}

		else if ( !(strcmp( task, "reset" )) && mchSess->session ) {

			epicsMutexLock( mch->mutex );
		       	ipmiMsgColdReset( mchSess, mchData->ipmiSess, data );
			epicsMutexUnlock( mch->mutex );
		}

		else if ( !(strcmp( task, "init" )) && mchSess->session )
			mchStatSet( inst, MCH_MASK_INIT, (pbo->val) ? MCH_MASK_INIT_DONE : MCH_MASK_INIT_NOT_DONE );

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
long     status = 0;
char     str[40];
DBLINK  *plink  = &pbi->inp;

	if ( ! ( recPvt = init_record_chk( plink, &status, str )) )
		goto bail;

        /* Break parm into node name and optional parameter */
        node = strtok( pbi->inp.value.camacio.parm, "+" );
        if ( (p = strtok( NULL, "+")) ) {
                task = p;
                if ( strcmp( task, "stat" ) && strcmp( task, "spres") && strcmp( task, "fpres") ) {
			sprintf( str, "Unknown task parameter %s", task);
			status = S_dev_badSignal;
		}
        }

	if ( init_record_find( mch, recPvt, node, task, &status, str ) )
		goto bail;
	else
		pbi->dpvt = recPvt;

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
Fru      fru;
char    *task;
long     status = 0;
int      inst;
short    id, index;

	if ( !recPvt )
		return status;

	mch     = recPvt->mch;
	mchData = mch->udata;
	mchSess = mchData->mchSess;
	mchSys  = mchData->mchSys;
	inst    = mchSess->instance;

	task    = recPvt->task;

       	if ( !(strcmp( task, "stat" )) ) 
       		pbi->rval = MCH_ONLN( mchStat[inst] );

	else if ( MCH_INIT_DONE( mchStat[inst] ) ) {

		if ( !(strcmp( task, "spres")) ) {

			index     = sensLkup( mchSys, pbi->inp.value.camacio );
			pbi->rval = ( -1 == index ) ? 0 : 1;
		}

		else if ( !(strcmp( task, "fpres")) ) {
			id  = pbi->inp.value.camacio.b;   /* FRU id */
			fru = &mchSys->fru[id];

			pbi->rval = ( fru->sdr.entityInst ) ? 1 : 0;
		}
	}
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
long     status = 0;
char     str[40];
DBLINK  *plink  = &pmbbi->inp;

	if ( ! ( recPvt = init_record_chk( plink, &status, str )) )
		goto bail;

        /* Break parm into node name and optional parameter */
        node = strtok( pmbbi->inp.value.camacio.parm, "+" );
        if ( (p = strtok( NULL, "+")) ) {
                task = p;
                if ( strcmp( task, "hs") && strcmp( task, "fan") && strcmp( task, "init" ) && strcmp( task, "pwr") && strcmp( task, "mch" ) ) {
			sprintf( str, "Unknown task parameter %s", task);
			status = S_dev_badSignal;
		}
        }

	if ( init_record_find( mch, recPvt, node, task, &status, str ) )
		goto bail;
	else
		pmbbi->dpvt = recPvt;
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
Fru      fru;
uint8_t  value  = 0, sensor, bits;
uint16_t addr   = 0; /* get addr */
short    id     = pmbbi->inp.value.camacio.b;   /* FRU id */
short    index; /* Sensor index */
long     status = 0;
int      s = 0, inst, offs;
size_t   responseSize;

	if ( !recPvt )
		return status;

	mch     = recPvt->mch;
	mchData = mch->udata;
	mchSess = mchData->mchSess;
	mchSys  = mchData->mchSys;
	inst    = mchSess->instance;

	task    = recPvt->task;
	fru     = &mchSys->fru[id];

	offs = ( mchSess->type == MCH_TYPE_NAT ) ? IPMI_RPLY_OFFSET_NAT : 0;

       	if ( !(strcmp( task, "init" )) )
       		pmbbi->rval = mchStat[inst] & MCH_MASK_INIT;

	else if ( task && MCH_INIT_DONE( mchStat[inst] ) ) {

		if ( !(strcmp( task, "fan" )) )

			pmbbi->rval = ( fru->fanProp & (1<<7) ) ? 1 : 0;

		else if ( !(strcmp( task, "pwr" )) && fru->sdr.entityInst )

			pmbbi->rval = fru->pwrDyn;

		else if ( !(strcmp( task, "mch" )) )

			pmbbi->rval = mchSess->type;

		else if ( !(strcmp( task, "hs")) && mchSess->session ) {

			if ( MCH_ONLN( mchStat[inst] ) ) {

				if ( -1 == (index = sensLkup( mchSys, pmbbi->inp.value.camacio )) )
					return ERROR;

				responseSize = mchSys->sens[index].readMsgLength;
				sensor = mchSys->sens[index].sdr.number;

				epicsMutexLock( mch->mutex );

				if ( !(s = mchMsgReadSensor( mchData, data, sensor, addr, &responseSize )) ) {	
					bits = data[IPMI_RPLY_SENSOR_ENABLE_BITS_OFFSET + offs];
					if ( IPMI_SENSOR_READING_DISABLED(bits)  || IPMI_SENSOR_SCANNING_DISABLED(bits) ) {
						if ( MCH_DBG( mchStat[inst] ) )
						    printf("%s sensor reading/state unavailable or scanning disabled. Bits: %02x\n", pmbbi->name, bits);
						s = ERROR;
					}
					else {
						/* Store raw sensor reading */
						value = data[IPMI_RPLY_HS_SENSOR_READING_OFFSET + offs];
						mchSys->sens[index].val = value;

						if ( MCH_DBG( mchStat[inst] ) > 1 )
							printf("%s read_mbbi: value %02x, sensor %02x, owner %i, lun %i, index %i, value %i\n",pmbbi->name, value, sensor, mchSys->sens[index].sdr.owner, mchSys->sens[index].sdr.lun, index, value);
					}
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
long     status = 0;
char     str[40];
DBLINK  *plink  = &pmbbo->out;

	if ( ! ( recPvt = init_record_chk( plink, &status, str )) )
		goto bail;

        /* Break parm into node name and optional parameter */
        node = strtok( pmbbo->out.value.camacio.parm, "+" );
        if ( (p = strtok( NULL, "+")) ) {
                task = p;
                if ( strcmp( task, "chas" ) && strcmp( task, "fru") && strcmp( task, "dbg") ) { 
			sprintf( str, "Unknown task parameter %s", task);
			status = S_dev_badSignal;
		}
        }

	if ( init_record_find( mch, recPvt, node, task, &status, str ) )
		goto bail;
	else
		pmbbo->dpvt = recPvt;
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
Fru      fru;
long     status = 0;
int      cmd;
int      id     = pmbbo->out.value.camacio.b;
int      inst;

	if ( !recPvt )
		return status;

	mch     = recPvt->mch;
	mchData = mch->udata;
	mchSess = mchData->mchSess;
	mchSys  = mchData->mchSys;
	inst    = mchSess->instance;

	task    = recPvt->task;
	fru     = &mchSys->fru[id];

	if ( !(strcmp( task, "dbg" )) ) {

       		mchStatSet( inst, MCH_MASK_DBG, (pmbbo->val << 4) );
       		printf("%s Setting debug message verbosity to %i\n", mchSess->name, pmbbo->val);

		pmbbo->udf = FALSE;
		return status;
	}
	else if (  MCH_ONLN(mchStat[inst] ) && mchSess->session ) {

		if ( !(strcmp( task, "chas" )) ) {
			epicsMutexLock( mch->mutex );
			status = mchMsgChassisControl( mchData, data, pmbbo->val );
			epicsMutexUnlock( mch->mutex );
		}
		else if ( !(strcmp( task, "fru" )) && fru->sdr.entityInst ) {

			if ( pmbbo->val > 1 ) { /* reset not supported yet */
				recGblSetSevr( pmbbo, STATE_ALARM, MAJOR_ALARM );
				return ERROR;
			}

			cmd = ( pmbbo->val == 2 ) ? 0 : pmbbo->val; 

			epicsMutexLock( mch->mutex );
			status = mchMsgSetFruActHelper( mchData, data, id, cmd );
			epicsMutexUnlock( mch->mutex );
		}


		if ( status ) {
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
init_fru_ai(struct aiRecord *pai)
{
	scanIoInit( &drvMchFruScan );
	return 0;
}

/*
** Add this record to our IOSCANPVT list
 */
static long 
ai_fru_ioint_info(int cmd, struct aiRecord *pai, IOSCANPVT *iopvt)
{
       *iopvt = drvMchFruScan;
       return 0;
} 

static long 
init_fru_ai_record(struct aiRecord *pai)
{
MchRec  recPvt  = 0; /* Info stored with record */
MchDev  mch     = 0; /* MCH device data structures */
char   *node;        /* Network node name, stored in parm */
char   *task    = 0; /* Optional additional parameter appended to parm */
char   *p;
long    status  = 0;
char    str[40];
DBLINK  *plink  = &pai->inp;

	if ( ! ( recPvt = init_record_chk( plink, &status, str )) )
		goto bail;

	/* Break parm into node name and optional parameter */
	node = strtok( pai->inp.value.camacio.parm, "+" );
	if ( (p = strtok( NULL, "+" )) ) {
		task = p;
		if ( strcmp( task, "type" ) && strcmp( task, "fan") && strcmp( task, "pwr")  ) {
			sprintf( str, "Unknown task parameter %s", task);
			status = S_dev_badSignal;
		}
	}

	if ( init_record_find( mch, recPvt, node, task, &status, str ) )
		goto bail;
	else
		pai->dpvt = recPvt;

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
int      id     = pai->inp.value.camacio.b;
int      parm   = pai->inp.value.camacio.c;
long     status = NO_CONVERT;
Fru      fru;
int      s = 0, inst, offs;
uint8_t  prop, level, draw, mult;

	if ( !recPvt )
		return status;

	mch     = recPvt->mch;
	mchData = mch->udata;
	mchSess = mchData->mchSess;
	mchSys  = mchData->mchSys;
	inst    = mchSess->instance;

	task    = recPvt->task;
	fru     = &mchSys->fru[id];

	offs = ( mchSess->type == MCH_TYPE_NAT ) ? IPMI_RPLY_2ND_BRIDGED_OFFSET_NAT : 0;

	if ( MCH_INIT_DONE( mchStat[inst] ) && MCH_ONLN( mchStat[inst] ) && mchSess->session && fru->sdr.entityInst ) {

		if ( !(strcmp( task, "fan")) ) {

				epicsMutexLock( mch->mutex );

				if ( !(s = mchMsgGetFanLevelHelper( mchData, data, id )) ) 
					pai->rval = data[IPMI_RPLY_GET_FAN_LEVEL_OFFSET + offs];

				epicsMutexUnlock( mch->mutex );
		}

		/* Check for systems and FRU IDs that support this query */
		else if ( !(strcmp( task, "pwr")) && MCH_IS_VT( mchSess->type ) && FRU_PWR_MSG_CMPTBL( id ) ) {

				epicsMutexLock( mch->mutex );

				if ( parm < 4 ) {

					if ( !(s = mchMsgGetPowerLevelVt( mchData, data, id, parm )) ) {

							prop  = data[IPMI_RPLY_GET_POWER_LEVEL_PROP_OFFSET + offs];

							if ( (level = FRU_PWR_LEVEL( prop )) ) {
								draw  = data[IPMI_RPLY_GET_POWER_LEVEL_DRAW_OFFSET + (level - 1)];
								mult  = data[IPMI_RPLY_GET_POWER_LEVEL_MULT_OFFSET];
								pai->rval = draw * mult * 0.1; /* Convert from 0.1 Watts to Watts */
							}

							/* If these don't change, consider i/o scanning these after initialization */
							switch ( parm ) {

								default: 
									break;

								case FRU_PWR_STEADY_STATE:
									fru->pwrDyn = FRU_PWR_DYNAMIC( prop ) ? 1 : 0;
									break;

								case FRU_PWR_EARLY:
									fru->pwrDly = data[IPMI_RPLY_GET_POWER_LEVEL_DELAY_OFFSET];
									break;
							}
				       	}
				}

				else if ( parm == 4 )
						pai->rval = fru->pwrDly * 0.1; /* Convert from 0.1 seconds to seconds */


				epicsMutexUnlock( mch->mutex );
			}

		       	if ( s ) {
		       		recGblSetSevr( pai, READ_ALARM, INVALID_ALARM );
		       		return ERROR;
		       	}

		       	pai->val  = pai->rval;

		if ( MCH_DBG( mchStat[inst] ) > 1 )
			printf("read_fru_ai: %s FRU id is %i, value is %.0f\n", pai->name, id, pai->val);

		pai->udf = FALSE;

		return status;
	}
	else {
		recGblSetSevr( pai, READ_ALARM, INVALID_ALARM );
		return ERROR;
	}
}

static long
init_fru_stringin(struct stringinRecord *pstringin)
{
	scanIoInit( &drvMchFruScan );
	return 0;
}

/*
** Add this record to our IOSCANPVT list
 */

static long 
stringin_fru_ioint_info(int cmd, struct stringinRecord *pstringin, IOSCANPVT *iopvt)
{
       *iopvt = drvMchFruScan;
       return 0;
} 

/* Create the dset for stringin support */
static long 
init_fru_stringin_record(struct stringinRecord *pstringin)
{
MchRec  recPvt  = 0; /* Info stored with record */
MchDev  mch     = 0; /* MCH device data structures */
char    *node;       /* Network node name, stored in parm */
char    *task   = 0; /* Optional additional parameter appended to parm */
char    *p;
long    status  = 0;
char    str[40];
DBLINK  *plink  = &pstringin->inp;

	if ( ! ( recPvt = init_record_chk( plink, &status, str )) )
		goto bail;

	/* Break parm into node name and optional parameter */
	node = strtok( pstringin->inp.value.camacio.parm, "+" );
	if ( (p = strtok( NULL, "+" )) ) {
		task = p;
		if ( strcmp( task, "bmf" ) && strcmp( task, "bp" ) && strcmp( task, "pmf" ) && strcmp( task, "pp") && strcmp( task, "bpn" ) && strcmp( task, "ppn") && strcmp( task, "bsn" ) && strcmp( task, "psn" )) {
			sprintf( str, "Unknown task parameter %s", task);
			status = S_dev_badSignal;
		}
	}

	if ( init_record_find( mch, recPvt, node, task, &status, str ) )
		goto bail;
	else
		pstringin->dpvt = recPvt;

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
short    id = pstringin->inp.value.camacio.b;     /* FRU ID */
int      i, inst;
long     status = NO_CONVERT;
Fru      fru;
uint8_t  l = 0, *d = 0; /* FRU data length and raw */

	if ( !recPvt )
		return status;

	mch     = recPvt->mch;
	mchData = mch->udata;
	mchSess = mchData->mchSess;
	mchSys  = mchData->mchSys;
	inst    = mchSess->instance;

	task    = recPvt->task;
	fru     = &mchSys->fru[id];

	if ( MCH_INIT_DONE( mchStat[inst] ) && fru->sdr.entityInst ) {

		if ( !(strcmp( task, "bmf" )) ) {
			d = fru->board.manuf.data;
			l = fru->board.manuf.length;
		}
		else if ( !(strcmp( task, "bp" )) ) {
			d = fru->board.prod.data;
			l = fru->board.prod.length;
		}
		else if ( !(strcmp( task, "pmf" )) ) {
			d = fru->prod.manuf.data;
			l = fru->prod.manuf.length;
		}
		else if ( !(strcmp( task, "pp" )) ) {
			d = fru->prod.prod.data;
			l = fru->prod.prod.length;
		}
		else if ( !(strcmp( task, "bpn" )) ) {
			d = fru->board.part.data;
			l = fru->board.part.length;
		}
		else if ( !(strcmp( task, "ppn" )) ) {
			d = fru->prod.part.data;
			l = fru->prod.part.length;
		}
		else if ( !(strcmp( task, "bsn" )) ) {
			d = fru->board.sn.data;
			l = fru->board.sn.length;
		} 

		else if ( !(strcmp( task, "psn" )) ) {
			d = fru->prod.sn.data;
			l = fru->prod.sn.length;
		}

		if ( d ) {

			l = ( l <= MAX_STRING_LENGTH ) ? l : MAX_STRING_LENGTH;
			for ( i = 0; i < l; i++ )
					pstringin->val[i] = d[i];
		}

		if ( MCH_DBG( mchStat[inst] ) > 1 )
			printf("%s read_fru_stringin: task is %s, FRU is %i\n", pstringin->name, task, pstringin->inp.value.camacio.b);
		pstringin->udf = FALSE;
		return status;
	}
	else {
		recGblSetSevr( pstringin, READ_ALARM, INVALID_ALARM );
		return ERROR;
	}
}

static long
init_fru_longout_record(struct longoutRecord *plongout)
{
MchRec  recPvt  = 0; /* Info stored with record */
MchDev  mch     = 0; /* MCH device data structures */
char   *node;        /* Network node name, stored in parm */
char   *task    = 0; /* Optional additional parameter appended to parm */
char   *p;
long    status = 0;
char    str[40];
DBLINK  *plink  = &plongout->out;

	if ( ! ( recPvt = init_record_chk( plink, &status, str )) )
		goto bail;

	/* Break parm into node name and optional parameter */
	node = strtok( plongout->out.value.camacio.parm, "+" );
	if ( (p = strtok( NULL, "+" )) ) {
		task = p;
		if ( strcmp( task, "fan" ) ) {
			sprintf( str, "Unknown task parameter %s", task);
			status = S_dev_badSignal;
		}
	}

	if ( init_record_find( mch, recPvt, node, task, &status, str ) )
		goto bail;
	else
		plongout->dpvt = recPvt;

bail:
	if ( status ) {
	       recGblRecordError( status, (void *)plongout , (const char *)str );
	       plongout->pact=TRUE;
	}

	return status;
}

/* scan i/o int? */
static long 
write_fru_longout(struct longoutRecord *plongout)
{
MchRec   recPvt  = plongout->dpvt;
MchDev   mch;
MchData  mchData;
MchSess  mchSess;
MchSys   mchSys;
char    *task;
int      id      = plongout->out.value.camacio.b; /* FRU ID */
long     status  = 0;
Fru      fru;
uint8_t  data[MSG_MAX_LENGTH] = { 0 };
int      s = 0, inst;

	if ( !recPvt )
		return status;

	mch     = recPvt->mch;
	mchData = mch->udata;
	mchSess = mchData->mchSess;
	mchSys  = mchData->mchSys;
	inst    = mchSess->instance;

	task    = recPvt->task;
	fru     = &mchSys->fru[id];

	if ( MCH_DBG( mchStat[inst] ) > 1 )
		printf("%s write_fru_longout: FRU id is %i, value is %.0f\n",plongout->name, id, (double)plongout->val);

	if ( MCH_ONLN( mchStat[inst] ) && mchSess->session && MCH_INIT_DONE( mchStat[inst] ) && fru->sdr.entityInst ) {
		if ( !(strcmp( task, "fan" )) ) {

       			/* For now test init done above and then always set these; change this to after detect config change ? */
       			plongout->drvl = plongout->lopr = mchSys->fru[id].fanMin;
       			plongout->drvh = plongout->hopr = mchSys->fru[id].fanMax;

			epicsMutexLock( mch->mutex );

			s = mchMsgSetFanLevelHelper( mchData, data, id, plongout->val );

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
