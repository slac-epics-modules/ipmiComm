#ifndef DEV_MCH_H
#define DEV_MCH_H

#include <dbCommon.h>
#include <dbScan.h>
#include <epicsMutex.h>
#include <epicsTypes.h>

#define NO_CONVERT 2 /* Return value for ai record */
#define ERROR     -1

#define RESET_TIMEOUT 180 /* seconds */

#ifdef __cplusplus
extern "C" {
#endif

IOSCANPVT drvMchStatScan;

/* Much of this stolen from devBusMapped */

/* Per-MCH information kept in the registry */
typedef struct MchDevRec_ {
	epicsMutexId  mutex;     /* any other driver/devSup sharing resources must with devMch MUST LOCK THIS MUTEX when
                                    performing modifications or non-atomical reads. */
                                      
	void         *udata;	 /* for use by the driver / user */
	const char    name[1];	 /* space for the terminating NULL; the entire string is appended here, however. */
} MchDevRec, *MchDev;

/* Data private to IPMI MCH device support; to be stored in record's DPVT field */
typedef struct MchRec_ {
	MchDev      mch;
        char       *task; /* operation type */
} *MchRec;


MchDev
devMchRegister(const char *name);

/* Find the structure of a registered MCH by name */
MchDev
devMchFind(const char *name);

#ifdef __cplusplus
};
#endif

#endif
