//////////////////////////////////////////////////////////////////////////////
// This file is part of 'ipmiComm'.
// It is subject to the license terms in the LICENSE.txt file found in the
// top-level directory of this distribution and at:
//    https://confluence.slac.stanford.edu/display/ppareg/LICENSE.html.
// No part of 'ipmiComm', including this file,
// may be copied, modified, propagated, or distributed except according to
// the terms contained in the LICENSE.txt file.
//////////////////////////////////////////////////////////////////////////////
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
         *   ai_ioint_info            - Add to i/o scan list
         *
       Supported operations:

             * "sens": read sensor

     devAiFru
         --------
         *   init_fru_ai                  - Record initialization
         *   fru_ai_ioint_info            - Add to i/o scan list
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
             * "dbg":  control debug messages for one MCH
             * "scan": sensor scan period control, one per system

         Long Integer Input Device Support:
         -------------------------------------------------------

     devLonginMch
         -------------

         *   init_longin_record       - Record initialization
         *   write_longin             - Read long integer input

       Supported operations:

         * "chas": read chassis status

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

#include <epicsTypes.h>
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
#include <boRecord.h>
#include <longinRecord.h>
#include <longoutRecord.h>
#include <mbbiRecord.h>
#include <mbboRecord.h>
#include <stringinRecord.h>
#include <epicsExport.h>
#include <asynDriver.h>
#include <dbScan.h>
#include <link.h>

#include <ipmiDef.h>
#include <picmgDef.h>
#include <ipmiMsg.h>
#include <drvMch.h>
#include <drvMchMsg.h>
#include <devMch.h>

extern uint32_t mchStat[MAX_MCH];

/* Sensor scan period options (in seconds). Must match
 * SENSOR_SCAN_PERIOD record definition in system_common.db
 */
uint8_t SENSOR_SCAN_PERIODS[5] = {5, 10, 20, 30, 60};

IOSCANPVT drvMchStatScan;
IOSCANPVT drvMchInitScan;
IOSCANPVT drvMchFruScan;

#define MAX_STRING_LENGTH 39
#define MAX_EGU_LENGTH    16

/* Device support return values */
static long SUCCESS = 0;
static long NO_CONVERT =
    2; /* Used by ai, success and indicate that devsup handles the conversion (record support does not need to) */
static long ERROR = -1;

/* Device support prototypes */
static long init_ai_record(struct aiRecord* pai);
static long read_ai(struct aiRecord* pai);
static long ai_ioint_info(int cmd, struct aiRecord* pai, IOSCANPVT* iopvt);

static long init_bo_record(struct boRecord* pbo);
static long write_bo(struct boRecord* pbo);

static long init_bi(struct biRecord* pbi);
static long init_bi_record(struct biRecord* pbi);
static long read_bi(struct biRecord* pbi);
static long bi_ioint_info(int cmd, struct biRecord* pbi, IOSCANPVT* iopvt);

static long init_mbbi(struct mbbiRecord* pmbbi);
static long init_mbbi_record(struct mbbiRecord* pmbbi);
static long read_mbbi(struct mbbiRecord* pmbbi);
static long mbbi_ioint_info(int cmd, struct mbbiRecord* mbpbi, IOSCANPVT* iopvt);

static long init_mbbo_record(struct mbboRecord* pmbbo);
static long write_mbbo(struct mbboRecord* pmbbo);

static long init_longin_record(struct longinRecord* plongin);
static long read_longin(struct longinRecord* plongin);

static long init_fru_ai(struct aiRecord* pai);
static long init_fru_ai_record(struct aiRecord* pai);
static long read_fru_ai(struct aiRecord* pai);
static long ai_fru_ioint_info(int cmd, struct aiRecord* pai, IOSCANPVT* iopvt);

static long init_fru_longout_record(struct longoutRecord* plongout);
static long write_fru_longout(struct longoutRecord* plongout);

static long init_fru_stringin(struct stringinRecord* pstringin);
static long init_fru_stringin_record(struct stringinRecord* pstringin);
static long read_fru_stringin(struct stringinRecord* pstringin);
static long stringin_fru_ioint_info(int cmd, struct stringinRecord* pstringin, IOSCANPVT* iopvt);

/* global struct for devSup */
typedef struct {
    long      number;
    DEVSUPFUN report;
    DEVSUPFUN init;
    DEVSUPFUN init_record;
    DEVSUPFUN get_ioint_info;
    DEVSUPFUN read_write;
    DEVSUPFUN special_linconv;
} MCH_DEV_SUP_SET;

/* Add reporting */
MCH_DEV_SUP_SET devAiMch       = {6, NULL, NULL, init_ai_record, ai_ioint_info, read_ai, NULL};
MCH_DEV_SUP_SET devBoMch       = {6, NULL, NULL, init_bo_record, NULL, write_bo, NULL};
MCH_DEV_SUP_SET devMbboMch     = {6, NULL, NULL, init_mbbo_record, NULL, write_mbbo, NULL};
MCH_DEV_SUP_SET devBiMch       = {6, NULL, init_bi, init_bi_record, bi_ioint_info, read_bi, NULL};
MCH_DEV_SUP_SET devMbbiMch     = {6, NULL, init_mbbi, init_mbbi_record, mbbi_ioint_info, read_mbbi, NULL};
MCH_DEV_SUP_SET devLonginMch   = {6, NULL, NULL, init_longin_record, NULL, read_longin, NULL};
MCH_DEV_SUP_SET devAiFru       = {6, NULL, init_fru_ai, init_fru_ai_record, ai_fru_ioint_info, read_fru_ai, NULL};
MCH_DEV_SUP_SET devLongoutFru  = {6, NULL, NULL, init_fru_longout_record, NULL, write_fru_longout, NULL};
MCH_DEV_SUP_SET devStringinFru = {
    6, NULL, init_fru_stringin, init_fru_stringin_record, stringin_fru_ioint_info, read_fru_stringin, NULL};

epicsExportAddress(dset, devAiMch);
epicsExportAddress(dset, devBoMch);
epicsExportAddress(dset, devBiMch);
epicsExportAddress(dset, devMbbiMch);
epicsExportAddress(dset, devMbboMch);
epicsExportAddress(dset, devLonginMch);
epicsExportAddress(dset, devAiFru);
epicsExportAddress(dset, devLongoutFru);
epicsExportAddress(dset, devStringinFru);

/*--- Register functions stolen from devBusMapped ---*/

/* just any unique address */
static void* registryId       = (void*)&registryId;
static void* ioRegistryId     = (void*)&ioRegistryId;
static void* ioscanRegistryId = (void*)&ioscanRegistryId;

/* Register a device's base address and return a pointer to a
 * freshly allocated 'MchDev' struct or NULL on failure.
 */
MchDev devMchRegister(const char* name) {
    MchDev rval = 0, d;

    if ((d = malloc(sizeof(*rval) + strlen(name)))) {

        strcpy((char*)d->name, name);
        if ((d->mutex = epicsMutexCreate())) {
            /* NOTE: the registry keeps a pointer to the name and
             *       does not copy the string, therefore we keep one.
             *       (_must_ pass d->name, not 'name'!!)
             */
            if (registryAdd(registryId, d->name, d)) {
                /* success */
                rval = d;
                d    = 0;
            }
        }
    }

    if (d) {
        /* an error happened and we need to clean up */
        if (d->mutex) {
            epicsMutexDestroy(d->mutex);
        }
        free(d);
    }

    /* return address of MchDev struct for success, 0 for error */
    return rval;
}

/* Find the 'MchDev' of a registered device by name */
MchDev devMchFind(const char* name) {
    return (MchDev)registryFind(registryId, name);
}

/*--- end stolen ---*/

static epicsFloat64 sensorConversion(SdrFull sdr, uint8_t raw, char* name) {
    int          l, units, format, m, b, rexp, bexp;
    epicsFloat64 value;

    l = SENSOR_LINEAR(sdr->linear);

    m     = sdr->m;
    b     = sdr->b;
    rexp  = sdr->rexp;
    bexp  = sdr->bexp;
    units = sdr->units2;

    format = SENSOR_NUMERIC_FORMAT(sdr->units1);

    switch (format) {

        default:
            printf("sensorConversion %s: Unknown analog data format\n", name);
            return raw;

        case SENSOR_NUMERIC_FORMAT_UNSIGNED:
            value = raw;
            break;

        case SENSOR_NUMERIC_FORMAT_ONES_COMP:
            value = ONES_COMP_SIGNED_NBIT(raw, 8);
            break;

        case SENSOR_NUMERIC_FORMAT_TWOS_COMP:
            value = TWOS_COMP_SIGNED_NBIT(raw, 8);
            break;

        case SENSOR_NUMERIC_FORMAT_NONNUMERIC:
            printf("sensorConversion %s: Non-numeric data format\n", name);
            return raw;
    }

    value = ((m * value) + (b * pow(10, bexp))) * pow(10, rexp);

    if (l == SENSOR_CONV_LINEAR) {
        value = value;
    } else if (l == SENSOR_CONV_LN) {
        value = log(value);
    } else if (l == SENSOR_CONV_LOG10) {
        value = log10(value);
    } else if (l == SENSOR_CONV_LOG2) {
        value = log(value) / log(2);
    } else if (l == SENSOR_CONV_E) {
        value = exp(value);
    } else if (l == SENSOR_CONV_EXP10) {
        value = pow(value, 10);
    } else if (l == SENSOR_CONV_EXP2) {
        value = pow(value, 2);
    } else if (l == SENSOR_CONV_1_X) {
        value = 1 / value;
    } else if (l == SENSOR_CONV_SQR) {
        value = pow(value, 2);
    } else if (l == SENSOR_CONV_CUBE) {
        value = pow(value, 1 / 3);
    } else if (l == SENSOR_CONV_SQRT) {
        value = sqrt(value);
    } else if (l == SENSOR_CONV_CUBE_NEG1) {
        value = pow(value, -1 / 3);
    } else {
        printf("sensorConversion %s: unknown sensor conversion algorithm\n", name);
    }

    return value;
}

static short fruLkup(MchSys mchSys, struct camacio link) {

    return mchSys->fruLkup[link.b];
}

static short sensLkup(MchSys mchSys, struct camacio link) {

    int fruindex = fruLkup(mchSys, link);

    if (-1 == fruindex) {
        return -1;
    }

    return mchSys->sensLkup[fruindex][link.c][link.n];
}

static int checkMchOnlnSessInitDone(MchSess mchSess) {

    return (MCH_ONLN(mchStat[mchSess->instance]) && mchSess->session && MCH_INIT_DONE(mchStat[mchSess->instance]));
}

static int checkMchOnlnSess(MchSess mchSess) {

    return (MCH_ONLN(mchStat[mchSess->instance]) && mchSess->session);
}

static int checkMchInitDone(MchSess mchSess) {
    return MCH_INIT_DONE(mchStat[mchSess->instance]);
}

static int checkMchOnln(MchSess mchSess) {
    return MCH_ONLN(mchStat[mchSess->instance]);
}

static void sensEgu(char* egu, unsigned units) {

    switch (units) {

        default:
            strcpy(egu, "");

        /* If unspecified, copy empty string to .EGU */
        case SENSOR_UNITS_UNSPEC:
            strcpy(egu, "");
            break;

        case SENSOR_UNITS_DEGC:
            strcpy(egu, "DegC");
            break;

        case SENSOR_UNITS_DEGF:
            strcpy(egu, "DegF");
            break;

        case SENSOR_UNITS_DEGK:
            strcpy(egu, "DegK");
            break;

        case SENSOR_UNITS_VOLTS:
            strcpy(egu, "V");
            break;

        case SENSOR_UNITS_AMPS:
            strcpy(egu, "A");
            break;

        case SENSOR_UNITS_WATTS:
            strcpy(egu, "W");
            break;

        case SENSOR_UNITS_JOULES:
            strcpy(egu, "J");
            break;

        case SENSOR_UNITS_COULOMBS:
            strcpy(egu, "C");
            break;

        case SENSOR_UNITS_RPM:
            strcpy(egu, "RPM");
            break;
    }
}

static void sensThresh(SdrFull sdr, Sensor sens, epicsFloat64* lolo, epicsEnum16* llsv, epicsFloat64* low,
                       epicsEnum16* lsv, epicsFloat64* high, epicsEnum16* hsv, epicsFloat64* hihi, epicsEnum16* hhsv,
                       char* name) {

    if (IPMI_SENSOR_THRESH_LC_READABLE(sens->tmask)) {
        *lolo = sensorConversion(sdr, sens->tlc, name);
        *llsv = MAJOR_ALARM;
    } else {
        *lolo = 0;
        *llsv = NO_ALARM;
    }
    if (IPMI_SENSOR_THRESH_LNC_READABLE(sens->tmask)) {
        *low = sensorConversion(sdr, sens->tlnc, name);
        *lsv = MINOR_ALARM;
    } else {
        *low = 0;
        *lsv = NO_ALARM;
    }
    if (IPMI_SENSOR_THRESH_UNC_READABLE(sens->tmask)) {
        *high = sensorConversion(sdr, sens->tunc, name);
        *hsv  = MINOR_ALARM;
    } else {
        *high = 0;
        *hsv  = NO_ALARM;
    }
    if (IPMI_SENSOR_THRESH_UC_READABLE(sens->tmask)) {
        *hihi = sensorConversion(sdr, sens->tuc, name);
        *hhsv = MAJOR_ALARM;
    } else {
        *hihi = 0;
        *hhsv = NO_ALARM;
    }
}

/* Perform some common dev sup checks */
static MchRec init_record_chk(DBLINK* plink, long* status, char* str) {
    short  f, t, s;
    MchRec rval = 0;

    if (CAMAC_IO != plink->type) {
        *status = S_dev_badBus;
        return rval;
    }

    f = plink->value.camacio.b;
    t = plink->value.camacio.c;
    s = plink->value.camacio.n;

    if (f < 0) {
        sprintf(str, "FRU id (Branch) is %i; must >= 0", f);
        *status = S_dev_badSignal;
        return rval;
    }

    if (t < 0) {
        sprintf(str, "Sensor type (Crate) is %i; must >= 0", t);
        *status = S_dev_badSignal;
        return rval;
    }

    if (s < 0) {
        sprintf(str, "Sensor instance (Station) is %i; must >= 0", s);
        *status = S_dev_badSignal;
        return rval;
    }

    if (!(rval = calloc(1, sizeof(*rval)))) {
        sprintf(str, "No memory for recPvt structure");
        *status = S_rec_outMem;
        return rval;
    }

    return rval;
}

/* Find data structure in registry by MCH name */
static long init_record_find(MchDev mch, MchRec recPvt, char* node, char* task, long* status, char* str) {

    if ((mch = devMchFind(node))) {
        recPvt->mch = mch;
        strcpy(recPvt->task, task);
        return 0;
    } else {
        sprintf(str, "Failed to locate data structure");
        *status = S_dev_noDeviceFound;
        return -1;
    }
}

/*
** Add this record to our IOSCANPVT list.
*/
static long ai_ioint_info(int cmd, struct aiRecord* pai, IOSCANPVT* iopvt) {
    MchRec  recPvt = pai->dpvt; /* Info stored with record */
    MchDev  mch;
    MchData mchData;
    int     inst;
    long    status = SUCCESS;

    if (!recPvt) {
        return NO_CONVERT;
    }

    mch     = recPvt->mch;
    mchData = mch->udata;
    inst    = mchData->mchSess->instance;

    *iopvt = drvSensorScan[inst];
    return status;
}

static long init_ai_record(struct aiRecord* pai) {
    MchRec recPvt = 0; /* Info stored with record */
    MchDev mch    = 0; /* MCH device data structures */
    char*  node   = 0; /* Network node name, stored in parm */
    char*  task   = 0; /* Optional additional parameter appended to parm */
    char*  p;
    long   status = SUCCESS;
    char   str[40];

    if (!(recPvt = init_record_chk(&pai->inp, &status, str))) {
        goto bail;
    }

    /* Break parm into node name and optional parameter */
    node = strtok(pai->inp.value.camacio.parm, "+");
    if ((p = strtok(NULL, "+"))) {
        task = p;
        if (strcmp(task, "sens")) {
            sprintf(str, "Unknown task parameter %s", task);
            status = S_dev_badSignal;
        }
    }

    if (init_record_find(mch, recPvt, node, task, &status, str)) {
        goto bail;
    } else {
        pai->dpvt = recPvt;
    }

bail:
    if (status) {
        recGblRecordError(status, (void*)pai, (const char*)str);
        pai->pact = TRUE;
    }

    return status;
}

/* Take advantage of regular read_ai calls
 * to periodically check that our MCH data matches
 * the live configuration. Thus read_ai does not check
 * that MCH_INIT_DONE is true, but other read routines do
 */
static long read_ai(struct aiRecord* pai) {
    MchRec  recPvt = pai->dpvt;
    MchDev  mch;
    MchData mchData;
    MchSess mchSess;
    MchSys  mchSys;
    Sensor  sens;
    SdrFull sdr;
    char    egu[16];
    uint8_t data[MSG_MAX_LENGTH] = {0};
    uint8_t raw                  = 0;
    short   index; /* Sensor index */
    int     s = 0, inst;

    if (!recPvt) {
        return NO_CONVERT;
    }

    mch     = recPvt->mch;
    mchData = mch->udata;
    mchSess = mchData->mchSess;
    mchSys  = mchData->mchSys;
    inst    = mchSess->instance;

    if (checkMchOnlnSess(mchSess)) {

        epicsMutexLock(mch->mutex);

        /* If flag is set, check MCH configuration */
        if (MCH_CNFG_CHK(mchStat[inst])) {
            if (mchCnfgChk(mchData)) {
                epicsMutexUnlock(mch->mutex);
                return ERROR;
            }
        }

        if (MCH_INIT_NOT_DONE(mchStat[inst])) {
            epicsMutexUnlock(mch->mutex);
            return ERROR;
        }

        /* Check if sensor exists */
        if (-1 == (index = sensLkup(mchSys, pai->inp.value.camacio))) {
            epicsMutexUnlock(mch->mutex);
            pai->udf = FALSE;
            return NO_CONVERT;
        }

        sens = &mchSys->sens[index];

        if (mchGetSensorReadingStat(mchData, data, sens)) {
            s = ERROR;
        } else {
            sens->val = raw = data[IPMI_RPLY_IMSG2_SENSOR_READING_OFFSET];
        }

        epicsMutexUnlock(mch->mutex);

        sdr = &sens->sdr;

        /* Need to reconsider how to handle alarms if sensor scanning disabled */

        if (!sens->cnfg) {
            sensEgu(egu, sdr->units2);
            strcpy(pai->egu, egu);
            if (sdr->str) {
                strcpy(pai->desc, sdr->str);
            }

            if (sdr->recType == SDR_TYPE_FULL_SENSOR) {
                sensThresh(sdr, sens, &pai->lolo, &pai->llsv, &pai->low, &pai->lsv, &pai->high, &pai->hsv, &pai->hihi,
                           &pai->hhsv, pai->name);
            }
            sens->cnfg = 1;
        }

        if (s) {
            if (MCH_DBG(mchStat[inst])) {
                printf("%s writeread error sensor owner 0x%02x number %02x index %i\n", pai->name, sdr->owner,
                       sdr->number, index);
            }
            goto bail;
        }

        /* All of our conversions are for Full Sensor SDRs */
        if (sdr->recType != SDR_TYPE_FULL_SENSOR) {
            pai->val = raw;
        } else {
            pai->val = sensorConversion(sdr, raw, pai->name);
        }

        if (MCH_DBG(mchStat[inst]) >= MCH_DBG_MED) {
            printf("%s read_ai: sensor index is %i, sensor number is %i, value is %.0f, rval is %i, raw is 0x%02x\n",
                   pai->name, index, sdr->number, pai->val, pai->rval, raw);
        }

        pai->udf = FALSE;
        return NO_CONVERT;
    }
bail:
    recGblSetSevr(pai, READ_ALARM, INVALID_ALARM);
    return ERROR;
}

static long init_bo_record(struct boRecord* pbo) {
    MchRec recPvt = 0; /* Info stored with record */
    MchDev mch    = 0; /* MCH device data structures */
    char*  node;       /* Network node name, stored in parm */
    char*  task = 0;   /* Optional additional parameter appended to parm */
    char*  p;
    long   status = SUCCESS;
    char   str[40];

    if (!(recPvt = init_record_chk(&pbo->out, &status, str))) {
        goto bail;
    }

    /* Break parm into node name and optional parameter */
    node = strtok(pbo->out.value.camacio.parm, "+");
    if ((p = strtok(NULL, "+"))) {
        task = p;
        if (strcmp(task, "reset") && strcmp(task, "sess") && strcmp(task, "init")) {
            sprintf(str, "Unknown task parameter %s", task);
            status = S_dev_badSignal;
        }
    }

    if (init_record_find(mch, recPvt, node, task, &status, str)) {
        goto bail;
    } else {
        pbo->dpvt = recPvt;
    }

bail:
    if (status) {
        recGblRecordError(status, (void*)pbo, (const char*)str);
        pbo->pact = TRUE;
    }

    return status;
}

static long write_bo(struct boRecord* pbo) {
    uint8_t data[MSG_MAX_LENGTH] = {0};
    MchRec  recPvt               = pbo->dpvt;
    MchDev  mch;
    MchData mchData;
    MchSess mchSess;
    MchSys  mchSys;
    char*   task;

    if (!recPvt) {
        return SUCCESS;
    }

    mch     = recPvt->mch;
    mchData = mch->udata;
    mchSess = mchData->mchSess;
    mchSys  = mchData->mchSys;

    task = recPvt->task;

    if (checkMchOnln(mchSess)) {

        if (!(strcmp(task, "sess"))) {

            epicsMutexLock(mch->mutex);

            if (pbo->val) {           /* could change this to be purely soft; session should time out */
                mchSess->session = 1; /* Re-enable session */
            } else {
                mchSess->session = 0;
                mchMsgCloseSess(mchSess, mchData->ipmiSess, data);
            }

            epicsMutexUnlock(mch->mutex);
        }

        else if (!(strcmp(task, "reset")) && mchSess->session) {

            epicsMutexLock(mch->mutex);
            ipmiMsgColdReset(mchSess, mchData->ipmiSess, data);
            epicsMutexUnlock(mch->mutex);
        }

        else if (!(strcmp(task, "init")) && mchSess->session) {
            mchStatSet(mchSess->instance, MCH_MASK_INIT, (pbo->val) ? MCH_MASK_INIT_DONE : MCH_MASK_INIT_NOT_DONE);
        }

        pbo->udf = FALSE;
        return SUCCESS;
    }
    recGblSetSevr(pbo, WRITE_ALARM, INVALID_ALARM);
    return ERROR;
}

static long init_bi(struct biRecord* pbi) {
    scanIoInit(&drvMchStatScan);
    return 0;
}

/*
** Add this record to our IOSCANPVT list.
*/
static long bi_ioint_info(int cmd, struct biRecord* pbi, IOSCANPVT* iopvt) {
    *iopvt = drvMchStatScan;
    return 0;
}

static long init_bi_record(struct biRecord* pbi) {
    MchRec recPvt = 0; /* Info stored with record */
    MchDev mch    = 0; /* MCH device data structures */
    char*  node;       /* Network node name, stored in parm */
    char*  task = 0;   /* Optional additional parameter appended to parm */
    char*  p;
    long   status = SUCCESS;
    char   str[40];

    if (!(recPvt = init_record_chk(&pbi->inp, &status, str))) {
        goto bail;
    }

    /* Break parm into node name and optional parameter */
    node = strtok(pbi->inp.value.camacio.parm, "+");
    if ((p = strtok(NULL, "+"))) {
        task = p;
        if (strcmp(task, "stat") && strcmp(task, "spres") && strcmp(task, "fpres")) {
            sprintf(str, "Unknown task parameter %s", task);
            status = S_dev_badSignal;
        }
    }

    if (init_record_find(mch, recPvt, node, task, &status, str)) {
        goto bail;
    } else {
        pbi->dpvt = recPvt;
    }

bail:
    if (status) {
        recGblRecordError(status, (void*)pbi, (const char*)str);
        pbi->pact = TRUE;
    }

    return status;
}

static long read_bi(struct biRecord* pbi) {
    MchRec  recPvt = pbi->dpvt;
    MchDev  mch;
    MchData mchData;
    MchSess mchSess;
    MchSys  mchSys;
    char*   task;
    long    status = SUCCESS;
    short   index;

    if (!recPvt) {
        return status;
    }

    mch     = recPvt->mch;
    mchData = mch->udata;
    mchSess = mchData->mchSess;
    mchSys  = mchData->mchSys;

    task = recPvt->task;

    if (!(strcmp(task, "stat"))) {

        pbi->rval = checkMchOnln(mchSess);
    }

    else if (checkMchInitDone(mchSess)) {

        if (!(strcmp(task, "spres"))) {

            index     = sensLkup(mchSys, pbi->inp.value.camacio);
            pbi->rval = (-1 == index) ? 0 : 1;
        }

        else if (!(strcmp(task, "fpres"))) {

            index     = fruLkup(mchSys, pbi->inp.value.camacio);
            pbi->rval = (-1 == index) ? 0 : 1;
        }
    }
    return status;
}

static long init_mbbi(struct mbbiRecord* pmbbi) {
    scanIoInit(&drvMchInitScan);
    return 0;
}

/*
** Add this record to our IOSCANPVT list.
*/
static long mbbi_ioint_info(int cmd, struct mbbiRecord* pmbbi, IOSCANPVT* iopvt) {
    *iopvt = drvMchInitScan;
    return 0;
}

static long init_mbbi_record(struct mbbiRecord* pmbbi) {
    MchRec  recPvt = 0; /* Info stored with record */
    MchDev  mch    = 0; /* MCH device data structures */
    char*   node;       /* Network node name, stored in parm */
    char*   task = 0;   /* Optional additional parameter appended to parm */
    char*   p;
    long    status = SUCCESS;
    char    str[40];
    DBLINK* plink = &pmbbi->inp;

    if (!(recPvt = init_record_chk(plink, &status, str))) {
        goto bail;
    }

    /* Break parm into node name and optional parameter */
    node = strtok(pmbbi->inp.value.camacio.parm, "+");
    if ((p = strtok(NULL, "+"))) {
        task = p;
        if (strcmp(task, "hs") && strcmp(task, "fan") && strcmp(task, "init") && strcmp(task, "pwr") &&
            strcmp(task, "mch")) {
            sprintf(str, "Unknown task parameter %s", task);
            status = S_dev_badSignal;
        }
    }

    if (init_record_find(mch, recPvt, node, task, &status, str)) {
        goto bail;
    } else {
        pmbbi->dpvt = recPvt;
    }
bail:
    if (status) {
        recGblRecordError(status, (void*)pmbbi, (const char*)str);
        pmbbi->pact = TRUE;
    }

    return status;
}

static long read_mbbi(struct mbbiRecord* pmbbi) {
    uint8_t data[MSG_MAX_LENGTH] = {0};
    MchRec  recPvt               = pmbbi->dpvt;
    MchDev  mch;
    MchData mchData;
    MchSess mchSess;
    MchSys  mchSys;
    char*   task;
    Fru     fru;
    Sensor  sens;
    uint8_t value = 0;
    short   findex, sindex; /* FRU and Sensor index */
    long    status = SUCCESS;
    int     s      = 0, inst;

    if (!recPvt) {
        return status;
    }

    mch     = recPvt->mch;
    mchData = mch->udata;
    mchSess = mchData->mchSess;
    mchSys  = mchData->mchSys;
    inst    = mchSess->instance;
    task    = recPvt->task;

    /* Read initialized status */
    if (!(strcmp(task, "init"))) {
        pmbbi->rval = mchStat[inst] & MCH_MASK_INIT;
    }

    else if (task && checkMchInitDone(mchSess)) {

        if (!(strcmp(task, "fan"))) {

            if (-1 == (findex = fruLkup(mchSys, pmbbi->inp.value.camacio))) {
                return ERROR;
            }
            fru = &mchSys->fru[findex];

            pmbbi->rval = (fru->fanProp & (1 << 7)) ? 1 : 0;
        }

        else if (!(strcmp(task, "pwr"))) {

            if (-1 == (findex = fruLkup(mchSys, pmbbi->inp.value.camacio))) {
                return ERROR;
            }
            fru = &mchSys->fru[findex];

            pmbbi->rval = fru->pwrDyn;
        }

        else if (!(strcmp(task, "mch"))) {

            pmbbi->rval = mchSess->type;
        }

        else if (!(strcmp(task, "hs")) && checkMchOnlnSess(mchSess)) {

            if (-1 == (sindex = sensLkup(mchSys, pmbbi->inp.value.camacio))) {
                pmbbi->rval = 0x100; /* default state */
                /* return 0 here rather than ERROR so we can provide "Not Available" */
                return 0;
            }

            sens = &mchSys->sens[sindex];
            /* possibly add this later
            int readoffset;
                        if ( SENSOR_NUMERIC_FORMAT( mchSys->sens[sindex].sdr.units1) == SENSOR_NUMERIC_FORMAT_NONNUMERIC
            ) readoffset = IPMI_RPLY_DISCRETE_SENSOR_READING_OFFSET;
            */
            epicsMutexLock(mch->mutex);

            if (mchGetSensorReadingStat(mchData, data, sens)) {
                s = ERROR;
            } else {
                sens->val = value = data[IPMI_RPLY_IMSG2_DISCRETE_SENSOR_READING_OFFSET];

                if (MCH_DBG(mchStat[inst]) >= MCH_DBG_MED) {
                    printf("%s read_mbbi: value %02x, sensor %i, owner %i, lun %i, index %i, value %i\n", pmbbi->name,
                           value, mchSys->sens[sindex].sdr.number, mchSys->sens[sindex].sdr.owner,
                           mchSys->sens[sindex].sdr.lun, sindex, value);
                }
            }

            epicsMutexUnlock(mch->mutex);

            if (s) {
                recGblSetSevr(pmbbi, READ_ALARM, INVALID_ALARM);
                return ERROR;
            }

            pmbbi->rval = value;
        }
    }

    pmbbi->udf = FALSE;
    return status;
}

/* Create the dset for mbbo support */

static long init_mbbo_record(struct mbboRecord* pmbbo) {
    MchRec recPvt = 0; /* Info stored with record */
    MchDev mch    = 0; /* MCH device data structures */
    char*  node;       /* Network node name, stored in parm */
    char*  task = 0;   /* Optional additional parameter appended to parm */
    char*  p;
    long   status = SUCCESS;
    char   str[40];

    if (!(recPvt = init_record_chk(&pmbbo->out, &status, str))) {
        goto bail;
    }

    /* Break parm into node name and optional parameter */
    node = strtok(pmbbo->out.value.camacio.parm, "+");
    if ((p = strtok(NULL, "+"))) {
        task = p;
        if (strcmp(task, "chas") && strcmp(task, "fru") && strcmp(task, "dbg") && strcmp(task, "scan")) {
            sprintf(str, "Unknown task parameter %s", task);
            status = S_dev_badSignal;
        }
    }

    if (init_record_find(mch, recPvt, node, task, &status, str)) {
        goto bail;
    } else {
        pmbbo->dpvt = recPvt;
    }
bail:
    if (status) {
        recGblRecordError(status, (void*)pmbbo, (const char*)str);
        pmbbo->pact = TRUE;
    }

    return status;
}

static long write_mbbo(struct mbboRecord* pmbbo) {
    uint8_t data[MSG_MAX_LENGTH] = {0};
    MchRec  recPvt               = pmbbo->dpvt;
    MchDev  mch;
    MchData mchData;
    MchSess mchSess;
    MchSys  mchSys;
    char*   task;
    Fru     fru;
    long    status = SUCCESS;
    int     cmd;
    short   index;
    int     inst;

    if (!recPvt) {
        return status;
    }

    mch     = recPvt->mch;
    mchData = mch->udata;
    mchSess = mchData->mchSess;
    mchSys  = mchData->mchSys;
    inst    = mchSess->instance;

    task = recPvt->task;

    if (!(strcmp(task, "dbg"))) {
        mchStatSet(inst, MCH_MASK_DBG, MCH_DBG_SET(pmbbo->val));
        printf("%s Setting debug message verbosity to %i\n", mchSess->name, pmbbo->val);
        pmbbo->udf = FALSE;
        return status;
    } else if (!(strcmp(task, "scan"))) {
        mchSensorScanPeriod = SENSOR_SCAN_PERIODS[pmbbo->val];
        printf("%s Setting sensor scan period to %i seconds\n", mchSess->name, SENSOR_SCAN_PERIODS[pmbbo->val]);
        pmbbo->udf = FALSE;
        return status;
    } else if (checkMchOnlnSess(mchSess)) {

        if (!(strcmp(task, "chas"))) {

            if (MCH_DBG(mchStat[inst]) >= MCH_DBG_MED) {
                printf("write_mbbo: call mchMsgChassisControl with value %i\n", pmbbo->val);
            }

            epicsMutexLock(mch->mutex);
            status = mchMsgChassisControl(mchData, data, pmbbo->val);
            epicsMutexUnlock(mch->mutex);
        } else if (!(strcmp(task, "fru"))) {

            if (-1 == (index = fruLkup(mchSys, pmbbo->out.value.camacio))) {
                return ERROR;
            }

            fru = &mchSys->fru[index];

            if (pmbbo->val > 1) { /* reset not supported yet */
                recGblSetSevr(pmbbo, STATE_ALARM, MAJOR_ALARM);
                return ERROR;
            }

            cmd = (pmbbo->val == 2) ? 0 : pmbbo->val;

            epicsMutexLock(mch->mutex);
            if (mchData->mchSys->mchcb->set_fru_act) {
                status = mchData->mchSys->mchcb->set_fru_act(mchData, data, index, cmd);
            } else {
                status = -1;
            }
            epicsMutexUnlock(mch->mutex);
        }

        if (status) {
            recGblSetSevr(pmbbo, WRITE_ALARM, INVALID_ALARM);
            return ERROR;
        }

        pmbbo->udf = FALSE;

        return status;
    } else {
        recGblSetSevr(pmbbo, WRITE_ALARM, INVALID_ALARM);
        return ERROR;
    }
}

static long init_longin_record(struct longinRecord* plongin) {
    MchRec recPvt = 0; /* Info stored with record */
    MchDev mch    = 0; /* MCH device data structures */
    char*  node   = 0; /* Network node name, stored in parm */
    char*  task   = 0; /* Optional additional parameter appended to parm */
    char*  p;
    long   status = SUCCESS;
    char   str[40];

    if (!(recPvt = init_record_chk(&plongin->inp, &status, str))) {
        goto bail;
    }

    /* Break parm into node name and optional parameter */
    node = strtok(plongin->inp.value.camacio.parm, "+");
    if ((p = strtok(NULL, "+"))) {
        task = p;
        if (strcmp(task, "chas")) {
            sprintf(str, "Unknown task parameter %s", task);
            status = S_dev_badSignal;
        }
    }

    if (init_record_find(mch, recPvt, node, task, &status, str)) {
        goto bail;
    } else {
        plongin->dpvt = recPvt;
    }

bail:
    if (status) {
        recGblRecordError(status, (void*)plongin, (const char*)str);
        plongin->pact = TRUE;
    }

    return status;
}

static long read_longin(struct longinRecord* plongin) {
    MchRec  recPvt = plongin->dpvt;
    MchDev  mch;
    MchData mchData;
    MchSess mchSess;
    uint8_t data[MSG_MAX_LENGTH] = {0};
    int     inst;

    if (!recPvt) {
        return SUCCESS;
    }

    mch     = recPvt->mch;
    mchData = mch->udata;
    mchSess = mchData->mchSess;
    inst    = mchSess->instance;

    if (checkMchOnlnSessInitDone(mchSess)) {

        epicsMutexLock(mch->mutex);

        if (mchData->mchSys->mchcb->get_chassis_status) {

            if (mchData->mchSys->mchcb->get_chassis_status(mchData, data)) {
                epicsMutexUnlock(mch->mutex);
                goto bail;
            } else {
                plongin->val = IPMI_GET_CHAS_POWER_STATE(data[IPMI_RPLY_IMSG2_GET_CHAS_POWER_STATE_OFFSET]) |
                               (IPMI_GET_CHAS_LAST_EVENT(data[IPMI_RPLY_IMSG2_GET_CHAS_LAST_EVENT_OFFSET]) << 8) |
                               (IPMI_GET_CHAS_MISC_STATE(data[IPMI_RPLY_IMSG2_GET_CHAS_MISC_STATE_OFFSET]) << 16);
                /* Or 'last event' and 'misc' bits into power state word */

                if (MCH_DBG(mchStat[inst]) >= MCH_DBG_MED) {
                    printf("%s read_longin: val %i, power state %i last event %i misc state %i\n", plongin->name,
                           plongin->val, IPMI_GET_CHAS_POWER_STATE(data[IPMI_RPLY_IMSG2_GET_CHAS_POWER_STATE_OFFSET]),
                           IPMI_GET_CHAS_LAST_EVENT(data[IPMI_RPLY_IMSG2_GET_CHAS_LAST_EVENT_OFFSET]),
                           IPMI_GET_CHAS_MISC_STATE(data[IPMI_RPLY_IMSG2_GET_CHAS_MISC_STATE_OFFSET]));
                }

                epicsMutexUnlock(mch->mutex);
                plongin->udf = FALSE;
                return SUCCESS;
            }
        } else {
            goto bail;
        }
    }

bail:
    recGblSetSevr(plongin, READ_ALARM, INVALID_ALARM);
    return ERROR;
}

/*
 * Device support to read FRU data
 */
static long init_fru_ai(struct aiRecord* pai) {
    scanIoInit(&drvMchFruScan);
    return 0;
}

/*
** Add this record to our IOSCANPVT list
*/
static long ai_fru_ioint_info(int cmd, struct aiRecord* pai, IOSCANPVT* iopvt) {
    *iopvt = drvMchFruScan;
    return 0;
}

static long init_fru_ai_record(struct aiRecord* pai) {
    MchRec recPvt = 0; /* Info stored with record */
    MchDev mch    = 0; /* MCH device data structures */
    char*  node;       /* Network node name, stored in parm */
    char*  task = 0;   /* Optional additional parameter appended to parm */
    char*  p;
    long   status = SUCCESS;
    char   str[40];

    if (!(recPvt = init_record_chk(&pai->inp, &status, str))) {
        goto bail;
    }

    /* Break parm into node name and optional parameter */
    node = strtok(pai->inp.value.camacio.parm, "+");
    if ((p = strtok(NULL, "+"))) {
        task = p;
        if (strcmp(task, "type") && strcmp(task, "fan") && strcmp(task, "pwr")) {
            sprintf(str, "Unknown task parameter %s", task);
            status = S_dev_badSignal;
        }
    }

    if (init_record_find(mch, recPvt, node, task, &status, str)) {
        goto bail;
    } else {
        pai->dpvt = recPvt;
    }

bail:
    if (status) {
        recGblRecordError(status, (void*)pai, (const char*)str);
        pai->pact = TRUE;
    }

    return status;
}

static long read_fru_ai(struct aiRecord* pai) {
    uint8_t data[MSG_MAX_LENGTH] = {0};
    MchRec  recPvt               = pai->dpvt;
    MchDev  mch;
    MchData mchData;
    MchSess mchSess;
    MchSys  mchSys;
    char*   task;
    short   index;
    int     id     = pai->inp.value.camacio.b;
    int     parm   = pai->inp.value.camacio.c;
    long    status = NO_CONVERT;
    Fru     fru;
    int     s = 0, inst;
    uint8_t prop, level, draw, mult;

    if (!recPvt) {
        return status;
    }

    mch     = recPvt->mch;
    mchData = mch->udata;
    mchSess = mchData->mchSess;
    mchSys  = mchData->mchSys;

    if (-1 == (index = fruLkup(mchSys, pai->inp.value.camacio))) {
        return ERROR;
    }

    inst = mchSess->instance;
    task = recPvt->task;
    fru  = &mchSys->fru[index];

    if (checkMchOnlnSessInitDone(mchSess)) {

        if (!(strcmp(task, "fan"))) {

            epicsMutexLock(mch->mutex);

            if (mchSys->mchcb->get_fan_level) {
                if (!(s = mchSys->mchcb->get_fan_level(mchData, data, index, &level))) {
                    pai->rval = level;
                }
            } else {
                s = -1;
            }
            epicsMutexUnlock(mch->mutex);
        } else if (!(strcmp(task, "pwr"))) {

            /* Check for systems and FRU IDs that support this query;
             * kludgey implementation, needs re-work
             */
            if (!((mchSys->mchcb->get_power_level) && (FRU_PWR_MSG_CMPTBL(id)))) {
                goto bail;
            }

            epicsMutexLock(mch->mutex);

            if (parm < 4) {

                if (!(s = mchSys->mchcb->get_power_level(mchData, data, index, parm))) {

                    prop = data[PICMG_RPLY_IMSG2_GET_POWER_LEVEL_PROP_OFFSET];

                    if ((level = FRU_PWR_LEVEL(prop))) {
                        draw      = data[PICMG_RPLY_IMSG2_GET_POWER_LEVEL_DRAW_OFFSET + (level - 1)];
                        mult      = data[PICMG_RPLY_IMSG2_GET_POWER_LEVEL_MULT_OFFSET];
                        pai->rval = draw * mult * 0.1; /* Convert from 0.1 Watts to Watts */
                    }

                    /* If these don't change, consider i/o scanning these after initialization */
                    switch (parm) {

                        default:
                            break;

                        case FRU_PWR_STEADY_STATE:
                            fru->pwrDyn = FRU_PWR_DYNAMIC(prop) ? 1 : 0;
                            break;

                        case FRU_PWR_EARLY:
                            fru->pwrDly = data[PICMG_RPLY_IMSG2_GET_POWER_LEVEL_DELAY_OFFSET];
                            break;
                    }
                }
            }

            else if (parm == 4) {
                pai->rval = fru->pwrDly * 0.1; /* Convert from 0.1 seconds to seconds */
            }

            epicsMutexUnlock(mch->mutex);
        }

        if (s) {
            goto bail;
        }

        pai->val = pai->rval;

        if (MCH_DBG(mchStat[inst]) >= MCH_DBG_MED) {
            printf("read_fru_ai: %s FRU id is %i, index is %i, value is %.0f\n", pai->name, id, index, pai->val);
        }

        pai->udf = FALSE;
        return status;
    }
bail:
    recGblSetSevr(pai, READ_ALARM, INVALID_ALARM);
    return ERROR;
}

static long init_fru_stringin(struct stringinRecord* pstringin) {
    scanIoInit(&drvMchFruScan);
    return 0;
}

/*
** Add this record to our IOSCANPVT list
*/

static long stringin_fru_ioint_info(int cmd, struct stringinRecord* pstringin, IOSCANPVT* iopvt) {
    *iopvt = drvMchFruScan;
    return 0;
}

/* Create the dset for stringin support */
static long init_fru_stringin_record(struct stringinRecord* pstringin) {
    MchRec recPvt = 0; /* Info stored with record */
    MchDev mch    = 0; /* MCH device data structures */
    char*  node;       /* Network node name, stored in parm */
    char*  task = 0;   /* Optional additional parameter appended to parm */
    char*  p;
    long   status = SUCCESS;
    char   str[40];

    if (!(recPvt = init_record_chk(&pstringin->inp, &status, str))) {
        goto bail;
    }

    /* Break parm into node name and optional parameter */
    node = strtok(pstringin->inp.value.camacio.parm, "+");
    if ((p = strtok(NULL, "+"))) {
        task = p;
        if (strcmp(task, "bmf") && strcmp(task, "bp") && strcmp(task, "pmf") && strcmp(task, "pp") &&
            strcmp(task, "bpn") && strcmp(task, "ppn") && strcmp(task, "bsn") && strcmp(task, "psn")) {
            sprintf(str, "Unknown task parameter %s", task);
            status = S_dev_badSignal;
        }
    }

    if (init_record_find(mch, recPvt, node, task, &status, str)) {
        goto bail;
    } else {
        pstringin->dpvt = recPvt;
    }

bail:
    if (status) {
        recGblRecordError(status, (void*)pstringin, (const char*)str);
        pstringin->pact = TRUE;
    }

    return status;
}

/*
static void
mchConvertData(uint8_t type, uint8_t *input, uint8_t *output, size_t bytes, size_t chars)
{
}
*/

static long read_fru_stringin(struct stringinRecord* pstringin) {
    MchRec  recPvt = pstringin->dpvt;
    MchDev  mch;
    MchData mchData;
    MchSess mchSess;
    MchSys  mchSys;
    char*   task;
    short   index;
    int     id = pstringin->inp.value.camacio.b; /* FRU ID; correct type? */
    int     i, inst;
    long    status = SUCCESS; // NO_CONVERT;
    Fru     fru;
    uint8_t l = 0, *d = 0; /* FRU data length and raw */

    if (!recPvt) {
        return status;
    }

    mch     = recPvt->mch;
    mchData = mch->udata;
    mchSess = mchData->mchSess;
    mchSys  = mchData->mchSys;
    inst    = mchSess->instance;

    task = recPvt->task;

    if (-1 == (index = fruLkup(mchSys, pstringin->inp.value.camacio))) {
        goto bail;
    }

    fru = &mchSys->fru[index];

    /* index check may be sufficient; may no longer need to check recType */

    if (checkMchInitDone(mchSess)) {

        if (!(strcmp(task, "bmf"))) {
            if (fru->board.manuf.length == 0) {
                d = "N/A";
                l = 4;
            } else {
                d = fru->board.manuf.data;
                l = fru->board.manuf.length;
            }
        } else if (!(strcmp(task, "bp"))) {
            if (fru->board.prod.length == 0) {
                d = "N/A";
                l = 4;
            } else {
                d = fru->board.prod.data;
                l = fru->board.prod.length;
            }
        } else if (!(strcmp(task, "pmf"))) {
            if (fru->prod.manuf.length == 0) {
                d = "N/A";
                l = 4;
            } else {
                d = fru->prod.manuf.data;
                l = fru->prod.manuf.length;
            }
        } else if (!(strcmp(task, "pp"))) {
            if (fru->prod.prod.length == 0) {
                d = "N/A";
                l = 4;
            } else {
                d = fru->prod.prod.data;
                l = fru->prod.prod.length;
            }
        } else if (!(strcmp(task, "bpn"))) {
            if (fru->board.part.length == 0) {
                d = "N/A";
                l = 4;
            } else {
                d = fru->board.part.data;
                l = fru->board.part.length;
            }
        } else if (!(strcmp(task, "ppn"))) {
            if (fru->prod.part.length == 0) {
                d = "N/A";
                l = 4;
            } else {
                d = fru->prod.part.data;
                l = fru->prod.part.length;
            }
        } else if (!(strcmp(task, "bsn"))) {
            if (fru->board.sn.length == 0) {
                d = "N/A";
                l = 4;
            } else {
                d = fru->board.sn.data;
                l = fru->board.sn.length;
            }
        }

        else if (!(strcmp(task, "psn"))) {
            if (fru->prod.sn.length == 0) {
                d = "N/A";
                l = 4;
            } else {
                d = fru->prod.sn.data;
                l = fru->prod.sn.length;
            }
        }

        if (d) {

            l = (l <= MAX_STRING_LENGTH) ? l : MAX_STRING_LENGTH;
            for (i = 0; i < l; i++) {
                pstringin->val[i] = d[i];
            }
        }

        if (MCH_DBG(mchStat[inst]) >= MCH_DBG_MED) {
            printf("%s read_fru_stringin: task is %s, FRU id %i index %i\n", pstringin->name, task, id, index);
        }
        pstringin->udf = FALSE;
        return status;
    }

bail:
    recGblSetSevr(pstringin, READ_ALARM, INVALID_ALARM);
    return ERROR;
}

static long init_fru_longout_record(struct longoutRecord* plongout) {
    MchRec  recPvt = 0; /* Info stored with record */
    MchDev  mch    = 0; /* MCH device data structures */
    char*   node;       /* Network node name, stored in parm */
    char*   task = 0;   /* Optional additional parameter appended to parm */
    char*   p;
    long    status = 0;
    char    str[40];
    short   index;
    MchData mchData;
    MchSess mchSess;
    MchSys  mchSys;

    if (!(recPvt = init_record_chk(&plongout->out, &status, str))) {
        goto bail;
    }

    /* Break parm into node name and optional parameter */
    node = strtok(plongout->out.value.camacio.parm, "+");
    if ((p = strtok(NULL, "+"))) {
        task = p;
        if (strcmp(task, "fan")) {
            sprintf(str, "Unknown task parameter %s", task);
            status = S_dev_badSignal;
        }
    }

    if (init_record_find(mch, recPvt, node, task, &status, str)) {
        goto bail;
    } else {

        mch     = recPvt->mch;
        mchData = mch->udata;
        mchSess = mchData->mchSess;
        mchSys  = mchData->mchSys;

        if (-1 == (index = fruLkup(mchSys, plongout->out.value.camacio))) {
            goto bail;
        }

        if (checkMchInitDone(mchSess)) {

            plongout->dpvt = recPvt;

            if (0 == strcmp(task, "fan")) {
                plongout->drvl = plongout->lopr = mchSys->fru[index].fanMin;
                plongout->drvh = plongout->hopr = mchSys->fru[index].fanMax;
            }
        }
    }

bail:
    if (status) {
        recGblRecordError(status, (void*)plongout, (const char*)str);
        plongout->pact = TRUE;
    }

    return status;
}

/* scan i/o int? */
static long write_fru_longout(struct longoutRecord* plongout) {
    MchRec  recPvt = plongout->dpvt;
    MchDev  mch;
    MchData mchData;
    MchSess mchSess;
    MchSys  mchSys;
    char*   task;
    int     id = plongout->out.value.camacio.b; /* FRU ID */
    short   index;                              /* FRU index in data structure */
    long    status = 0;
    Fru     fru;
    uint8_t data[MSG_MAX_LENGTH] = {0};
    int     s                    = 0, inst;

    if (!recPvt) {
        return status;
    }

    mch     = recPvt->mch;
    mchData = mch->udata;
    mchSess = mchData->mchSess;
    mchSys  = mchData->mchSys;
    inst    = mchSess->instance;

    task = recPvt->task;

    if (-1 == (index = fruLkup(mchSys, plongout->out.value.camacio))) {
        goto bail;
    }

    fru = &mchSys->fru[index];

    if (MCH_DBG(mchStat[inst]) >= MCH_DBG_MED) {
        printf("%s write_fru_longout: FRU id is %i, index is %i, value is %.0f\n", plongout->name, id, index,
               (double)plongout->val);
    }

    if (checkMchOnlnSessInitDone(mchSess)) {
        if (!(strcmp(task, "fan"))) {

            /* Need to change this so that these limits are also set after a config update
             * instead of setting them on every write
             */
            plongout->drvl = plongout->lopr = mchSys->fru[index].fanMin;
            plongout->drvh = plongout->hopr = mchSys->fru[index].fanMax;

            epicsMutexLock(mch->mutex);

            /* Need to test this for all archs */
            if (mchData->mchSys->mchcb->set_fan_level) {
                s = mchData->mchSys->mchcb->set_fan_level(mchData, data, index, plongout->val);
            } else {
                s = -1;
            }

            epicsMutexUnlock(mch->mutex);

            if (s) {
                goto bail;
            }
        }

        plongout->udf = FALSE;
        return status;
    }
bail:
    recGblSetSevr(plongout, WRITE_ALARM, INVALID_ALARM);
    return ERROR;
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
