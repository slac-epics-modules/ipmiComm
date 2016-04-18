#ifndef DRV_MCH_H
#define DRV_MCH_H

#include <epicsThread.h>
#include <devMch.h>
#include <ipmiDef.h>

#define MAX_FRU             255
#define MAX_MGMT            5    // management controller device
#define MAX_FRU_MGMT        MAX_FRU + MAX_MGMT
#define MAX_MCH             255
#define MAX_SENS_INST       32    /* Max instances of one sensor type on one FRU or Management Controller entity */

extern uint32_t mchStat[MAX_MCH];

/* Vadatech typically sends 2 replies; NAT sends 1 */
#define RPLY_TIMEOUT_VT    0.50
#define RPLY_TIMEOUT_NAT   0.25

#define RPLY_TIMEOUT_SENDMSG_RPLY    0.50
#define RPLY_TIMEOUT_DEFAULT         0.25

/* Mask of MCH status
 * INIT uses 2 lowest bits: 
 * must AND stat with MCH_MASK_INIT and
 * then test for value, for example:
 * if ( (mchStat & MCH_MASK_INIT) == MCH_MASK_INIT_DONE ) 
 * Similarly for DBG
 */
#define MCH_MASK_INIT            (0x3) 
#define MCH_MASK_INIT_NOT_DONE    0
#define MCH_MASK_INIT_IN_PROGRESS 1
#define MCH_MASK_INIT_DONE        2
#define MCH_MASK_INIT_FAILED      3
#define MCH_MASK_ONLN            (1<<2) /* 1 = MCH responds to pings */
#define MCH_MASK_CNFG_CHK        (1<<3) /* 1 = time to check config up-to-date */
#define MCH_MASK_DBG             (0x30) 
#define MCH_INIT_DONE(x)         ((x & MCH_MASK_INIT) == MCH_MASK_INIT_DONE)
#define MCH_INIT_NOT_DONE(x)     ((x & MCH_MASK_INIT) == MCH_MASK_INIT_NOT_DONE)
#define MCH_ONLN(x)              ((x & MCH_MASK_ONLN) >> 2)
#define MCH_CNFG_CHK(x)          ((x & MCH_MASK_CNFG_CHK) >> 3)
#define MCH_DBG(x)               ((x & MCH_MASK_DBG) >> 4)
#define MCH_DBG_SET(x)           (x<<4) /* Select debug bits in mask */

#define MCH_DBG_OFF   0
#define MCH_DBG_LOW   1
#define MCH_DBG_MED   2
#define MCH_DBG_HIGH  3

#ifdef __cplusplus
extern "C" {
#endif

/* Handle for each Management Controller Device */
typedef struct MgmtRec_ {
	uint8_t      instance;      /* Instance of management controller set in drvMch.c - needed? */
	SdrMgmtRec   sdr;
} MgmtRec, *Mgmt;

/* Handle for each FRU */
typedef struct FruRec_ {
	uint8_t      type;          /* FRU type */
	uint8_t      instance;      /* Instance of this FRU type, set in drvMch.c - still in use? */
	uint8_t      size[2];       /* FRU Inventory Area size, LS byte stored first */	
	uint8_t      access;        /* Access FRU data by words or bytes */
	uint8_t      readOffset[2];  
	uint16_t     read;          /* Read number */
	FruChassisRec chassis;
	FruBoardRec  board;
	FruProdRec   prod;
	SdrFruRec    sdr;
	char         parm[10];      /* Describes FRU type, used to load EPICS records */
	uint8_t      pwrDyn;        /* 1 if FRU supports dynamic reconfiguration of power, otherwise 0 */
	uint8_t      pwrDly;        /* Delay to stable power */
/* Following used only by cooling unit FRUs */
        uint8_t      fanMin;        /* Fan minimum level */          
        uint8_t      fanMax;        /* Fan maximum level */          
        uint8_t      fanNom;        /* Fan nominal level */   
        uint8_t      fanProp;       /* [7] 1 if fan try supports automatic fan speed adjustment */
} FruRec, *Fru;

/* Handle for each SDR */
typedef struct SensorRec_ {
	SdrFullRec    sdr;          /* Full Sensor SDR */
        uint8_t       val;          /* Most recent sensor reading */
	int           fruId;        /* FRU ID for associated FRU, same as index into FRU array ( -1 if no associated FRU ) */
	int           mgmtIndex;    /* Index into MGMT array for associated Management Controller ( -1 if no associated MGMT ) */
	uint8_t       instance;     /* Instance of this sensor type on this entity (usually a FRU) */
	int           unavail;      /* 1 if sensor reading returns 'Requested Sensor, data, or record not present' */
	char          parm[10];     /* Describes signal type, used by device support */
	size_t        readMsgLength;/* Get Sensor Reading message response length */
	int           cnfg;         /* 0 if needs record fields need to be updated */
	uint8_t       tmask;        /* Mask of which thresholds are readable */
	uint8_t       tlnc;         /* Threshold lower non-critical */
	uint8_t       tlc;          /* Threshold lower critical */
	uint8_t       tlnr;         /* Threshold lower non-recoverable */
	uint8_t       tunc;         /* Threshold upper non-critical */
	uint8_t       tuc;          /* Threshold upper critical */
	uint8_t       tunr;         /* Threshold upper non-recoverable */
} SensorRec, *Sensor;

/* Struct for MCH session information */
typedef struct MchSessRec_ {
	char    name[MAX_NAME_LENGTH];  /* MCH port name used by asyn */
	int           instance;      /* MCH instance number; assigned at init */
	epicsThreadId pingThreadId;  /* Thread ID for task that periodically pings MCH */
	double        timeout;       /* Asyn read timeout */
	int           session;       /* Enable session with MCH */
	int           err;           /* Count of sequential message errors */         
	int           type;          /* MCH vendor, Vadatech, NAT, etc. - need to clean this up, perhaps merge with vendor and/or add 'features' */
} MchSessRec, *MchSess;

/* Struct for MCH system information */
typedef struct MchSysRec_ {
	char    name[MAX_NAME_LENGTH]; /* MCH port name used by asyn */
	SdrRepRec     sdrRep;
	FruRec       *fru;           /* Array of FRUs (size of MAX_FRU) */
	int           sensLkup[MAX_FRU_MGMT][MAX_SENSOR_TYPE][MAX_SENS_INST]; /* Index into sens struct array, used by devsup, -1 if not used */
/*sens count used to be 8-bit !! */
	uint32_t      sensCount;     /* Sensor count, data type must be larger than MAX_FRU_MGMT*MAX_SENSOR_TYPE*MAX_SENS_INST */
	SensorRec    *sens;          /* Array of sensors (size of sensCount) */	
	uint8_t       mgmtCount;     /* Management controller device count */
	MgmtRec      *mgmt;          /* Array of management controller devices (size of mgmtCount) */	
} MchSysRec, *MchSys;

/* Struct for MCH system information */
typedef struct MchDataRec_ {
	IpmiSess   ipmiSess;       /* IPMI session information; defined in ipmiDef.h */
	MchSess    mchSess;        /* Additional MCH session info */
	MchSys     mchSys;         /* MCH system info */
} MchDataRec, *MchData;

/*extern MchSys mchSysData[MAX_MCH];*/

int  mchCnfgChk(MchData mchData);
void mchStatSet(int inst, uint32_t clear, uint32_t set);
int  mchNewSession(MchSess mchSess, IpmiSess ipmiSess);
int  mchGetSensorReadingStat(MchData mchData, Sensor sens, uint8_t *response, uint8_t number, uint8_t lun, size_t *sensReadMsgLength);

#define IPMI_RPLY_CLOSE_SESSION_LENGTH_VT        22

#define SENSOR_TYPE_HOTSWAP        0xF0  /* Contains M-state */
#define SENSOR_TYPE_HOTSWAP_NAT    0xF2

/* IPMI device types */
#define MCH_TYPE_MAX           10 /* Number possible MCH types; increase as needed */
#define MCH_TYPE_UNKNOWN        0
#define MCH_TYPE_VT             1
#define MCH_TYPE_NAT            2
#define MCH_TYPE_SUPERMICRO     3
#define MCH_IS_VT(x)         (x == MCH_TYPE_VT)
#define MCH_IS_NAT(x)        (x == MCH_TYPE_NAT)
#define MCH_IS_SUPERMICRO(x) (x == MCH_TYPE_SUPERMICRO)
#define MCH_IS_MICROTCA(x)   ((x==MCH_TYPE_NAT)||(x==MCH_TYPE_VT))

/* Dell not yet supported 
#define MCH_TYPE_DELL           
#define MCH_IS_DELL(x)       (x == MCH_TYPE_DELL)
#define MCH_MANUF_ID_DELL       0x02A2
#define MCH_PROD_ID_DELL  0x0100
*/

#define MCH_DESC_MAX_LENGTH 40
extern char mchDescString[MCH_TYPE_MAX][MCH_DESC_MAX_LENGTH]; /* Defined in drvMch.c */

/* Manufacturer ID */
#define MCH_MANUF_ID_VT         0x5D32
#define MCH_MANUF_ID_NAT        0x6C78
#define MCH_MANUF_ID_SUPERMICRO 0x2A7C

/* Vadatech */
#define VT_ENTITY_ID_MCH      0xC2 
#define VT_ENTITY_ID_AMC      0xC1
#define VT_ENTITY_ID_CU       0x1E
#define VT_ENTITY_ID_PM       0x0A
#define VT_ENTITY_ID_RTM      0xC0  /* asked Vivek to verify */


#ifdef __cplusplus
};
#endif

#endif
