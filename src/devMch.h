//////////////////////////////////////////////////////////////////////////////
// This file is part of 'ipmiComm'.
// It is subject to the license terms in the LICENSE.txt file found in the
// top-level directory of this distribution and at:
//    https://confluence.slac.stanford.edu/display/ppareg/LICENSE.html.
// No part of 'ipmiComm', including this file,
// may be copied, modified, propagated, or distributed except according to
// the terms contained in the LICENSE.txt file.
//////////////////////////////////////////////////////////////////////////////
#ifndef DEV_MCH_H
#define DEV_MCH_H

#include <dbCommon.h>
#include <dbScan.h>
#include <epicsMutex.h>
#include <epicsTypes.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define RESET_TIMEOUT 180 /* seconds */

/* Some arbitrary sizes */
#define MAX_NAME_LENGTH 50
#define MAX_TASK_LENGTH 10

    extern IOSCANPVT drvMchStatScan;
    extern IOSCANPVT drvMchInitScan;
    extern IOSCANPVT drvMchFruScan;

    /* Much of this stolen from devBusMapped */

    /* Per-MCH information kept in the registry */
    typedef struct MchDevRec_ {
        epicsMutexId mutex; /* any other driver/devSup sharing resources must with devMch MUST LOCK THIS MUTEX when
                                   performing modifications or non-atomical reads. */
        void* udata;        /* for use by the driver / user */
        const char
            name[MAX_NAME_LENGTH]; /* space for the terminating NULL; the entire string is appended here, however. */
    } MchDevRec, *MchDev;

    /* Data private to IPMI MCH device support; to be stored in record's DPVT field */
    typedef struct MchRec_ {
        MchDev mch;
        char   task[MAX_TASK_LENGTH]; /* operation type */
    }* MchRec;

    MchDev devMchRegister(const char* name);

    /* Find the structure of a registered MCH by name */
    MchDev devMchFind(const char* name);

#ifdef __cplusplus
};
#endif

#endif
