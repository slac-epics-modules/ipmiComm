#ifndef DRV_MCH_H
#define DRV_MCH_H

#include <epicsThread.h>
#include <devMch.h>
#include <ipmiDef.h>

#define MAX_FRU             255
#define MAX_MCH             255
#define MAX_SENS            32    /* CHANGE THIS? */

extern uint32_t mchStat[MAX_MCH];

/* Vadatech typically sends 2 replies; NAT sends 1 */
#define RPLY_TIMEOUT_VT    0.50
#define RPLY_TIMEOUT_NAT   0.25

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

#ifdef __cplusplus
extern "C" {
#endif

// Much of this should be moved to ipmiDef.h

typedef struct FruFieldRec_ {
	uint8_t     type;         /* Type of data */
	uint8_t     length;       /* Length of data (bytes) */
	uint8_t    *data;
} FruFieldRec, *FruField;

typedef struct FruChassisRec_ {
	uint8_t      offset;       /* Offset into FRU storage */
	uint8_t      ver;          /* Format version (0x01) */
	uint8_t      length;       /* Length of FRU area */
	uint8_t      type;         /* Chassis type */
	FruFieldRec  part;         /* Part number */
	FruFieldRec  sn;           /* Serial number */
        /* optional custom fields */	
} FruChassisRec, *FruChassis;

typedef struct FruBoardRec_ {
	uint8_t      offset;       /* Offset into FRU storage */
	uint8_t      ver;          /* Format version (0x01) */
	uint8_t      length;       /* Length of FRU area */
	uint8_t      lang;         /* Language code */
	uint8_t      date;         /* Manufacturer date/time (minutes from 00:00 1/1/96) */
	FruFieldRec  manuf;        /* Manufacturer */
	FruFieldRec  prod;         /* Product name */
	FruFieldRec  sn;           /* Serial number */
	FruFieldRec  part;         /* Part number */
        /* more */	
} FruBoardRec, *FruBoard;

typedef struct FruProdRec_ {
	uint8_t      offset;       /* Offset into FRU storage */
	uint8_t      ver;          /* Format version (0x01) */
	uint8_t      length;       /* Length of FRU area */
        uint8_t      lang;         /* Language code */
	FruFieldRec  manuf;        /* Manufacturer */
	FruFieldRec  prod;         /* Product name */
	FruFieldRec  part;         /* Part number */
	FruFieldRec  version;      /* Version number */
	FruFieldRec  sn;           /* Serial number */
        /* more */	
} FruProdRec, *FruProd;
	
/* Handle for Sensor Data Record (one per sensor) IPMI Section 43.1
 * Used for both Full and Compact SDRs
 * In this implementation, we don't support all of the Full SDR fields
 */
typedef struct SdrFullRec_ {
	uint8_t      id[2];         /* Sensor ID (can change, so key bytes must also be used to identify a sensor) */
	uint8_t      ver;           /* SDR version (0x51) */
	uint8_t      recType;       /* Record type, 0x01 for Full Sensor Record */
        uint8_t      length;        /* Number of remaining record bytes */
	uint8_t      owner;         /* Sensor owner ID, [7:1] slave address or software ID; [0] 0 = IPMB, 1 = system software */
	uint8_t      lun;           /* Sensor owner LUN, [7:4] channel number; [3:2] reserved; [1:0] sensor owner LUN */
	uint8_t      number;        /* Sensor number, unique behind given address and LUN, 0xFF reserved */
	uint8_t      entityId;      /* Entity ID (physical entity that sensor is associated with) */
	uint8_t      entityInst;    /* Entity instance, [7] 0 = physical entity, 1 = logical; [6:0] instance number 0x00-0x5F system-relative, 0x60-0x7F device-relative */
        uint8_t      init;          /* Sensor initialization (see IPMI spec page 472 for details) */
        uint8_t      cap;           /* Sensor capabilities (see IPMI spec page 473 for details) */
	uint8_t      sensType;      /* Sensor type code (see IPMI spec table 42-3) */
	uint8_t      readType;      /* Event/reading type code (see IPMI spec table 42-1) */
	/* Following fields only applicable to Full SDR */
	uint8_t      units1;        /* [7:6] analog data format, [5:3] rate unit, [2:1] modifier unit, [0] percentage */
	uint8_t      units2;        /* Units type code, IPMI 43-15 */
	uint8_t      units3;        /* Units type code, 0x00 if unused */
	uint8_t      linear;        /* Linearization [7] reserved, [6:0] formula type */
	uint8_t      M;             /* M in conversion, LS 8 bits (2's complement signed 10-bit 'M' value) */
	uint8_t      MTol;          /* [7:6] MS 2 bits of M, [5:0] tolerance 6 bits unsigned (in +/- 1/2 raw counts) */
	uint8_t      B;             /* B in conversion, LS 8 bits (2's complement signed 10-bit 'B' value) */
	uint8_t      BAcc;          /* [7:6] MS 2 bits of B, [5:0] accuracy LW 6 bits (unsigned 10-bit Basic Sensor Accuracy in 1/100 % scaled up by accuracy exponent */
	uint8_t      acc;           /* [7:4] MS 4 bits of accuracy, [3:2] accuracy exponent 2 bits unsigned, [1:0] sensor direction */
	uint8_t      RexpBexp;      /* [7:4] R (result) exponent 4 bits 2's complement signed, [3:0] B exponent 4 bits, 2's complement signed */
	uint8_t      strLength;     /* Sensor ID string type/length */
	uint8_t      anlgChar;      /* Analog characteristics [7:3] reserved, [2] normal min specified, [1] normal max specified [0], nominal reading specified */
	uint8_t      nominal;       /* Nominal reading (raw units) */
	uint8_t      normMax;       /* Normal maximum (raw units) */
	uint8_t      normMin;       /* Normal minimum (raw units) */
	char         str[17];       /* Sensor ID string, 16 bytes maximum */
	/* more... */
} SdrFullRec, *SdrFull;
	
/* 
 * Handle for FRU Device Locator Record, IPMI v2.0 Table 43-7
 */
typedef struct SdrFruRec_ {
	uint8_t      id[2];         /* Sensor ID (can change, so key bytes must also be used to identify a sensor) */
	uint8_t      ver;           /* SDR version (0x51) */
	uint8_t      recType;       /* Record type, 0x01 for Full Sensor Record */
        uint8_t      length;        /* Number of remaining record bytes */
	uint8_t      addr;          /* [7:1] slave address of controller, all 0 if dev on IPMB, [0] reserved */
	uint8_t      fruId;         /* FRU Device ID/Slave address. For logical FRU device, [7:0] FRU dev ID, for non-intelligent, [7:1] slave address, [0] reserved */
	uint8_t      lun;           /* [7] logical/physical FRU device, [6:5] reserved, [4:3] LUN for FRU commands, [2:0] private bus ID */
	uint8_t      chan;          /* [7:4] Channel number to access device, 0 if on IPMB (MS bit in next byte?); [3:0] reserved */
	uint8_t      devType;       /* Device type code Table 43-12 */
	uint8_t      devMod;        /* Device type modifier Table 43-12 */
	uint8_t      entityId;      /* Entity ID (physical entity that sensor is associated with) */
	uint8_t      entityInst;    /* Entity instance, [7] 0 = physical entity, 1 = logical; [6:0] instance number 0x00-0x5F system-relative, 0x60-0x7F device-relative */
	uint8_t      strLength;     /* Device ID string type/length Section 43.15*/
	char         str[17];       /* Device ID string, 16 bytes maximum */
} SdrFruRec, *SdrFru;
	
/* 
 * Handle for Management Controller Device Locator Record, IPMI v2.0 Table 43-8
 */
typedef struct SdrMgmtRec_ {
	uint8_t      id[2];         /* Record ID */
	uint8_t      ver;           /* SDR version */
	uint8_t      recType;       /* Record type, 0x12 for Management Controller Locator */
        uint8_t      length;        /* Number of remaining record bytes */
	uint8_t      addr;          /* [7:1] I2C slave address of device on channel, [0] reserved */
	uint8_t      chan;          /* [7:4] reserved; [3:0] Channel number for the channel that the management controller is on, use 0 for primary BMC */
	uint8_t      pwr;           /* Power state notification and Global initialization, see IPMI spec for meaning of each bit */
	uint8_t      cap;           /* Device capabilities, see IPMI spec for meaning of each bit */
	/* next 3 bytes reserved */
	uint8_t      entityId;      /* Entity ID for the FRU associated with this device, 0 if not specified */
	uint8_t      entityInst;    /* Entity instance */
	uint8_t      strLength;     /* Device ID string type/length Section 43.15*/
	char         str[17];       /* Device ID string, 16 bytes maximum */
} SdrMgmtRec, *SdrMgmt;
	

/* Handle for Sensor Data Record Repository and Dynamic Sensor Device */
typedef struct SdrRepRec_ {
	uint8_t      ver;           /* SDR Version (0x51) */
        uint8_t      size[2];       /* Number of sensor data records; LS byte stored first */
	uint8_t      free[2];       /* Free space in bytes; LS byte stored first */
	uint32_t     addTs;         /* Timestamp of most recent addition - remove: ; LS byte stored first */
        uint32_t     delTs;         /* Timestamp of most recent erase - remove: ; LS byte stored first */
        uint8_t      config[1];     /* 8 bits to describe Respository configuration, mostly supported commands */
	uint8_t      resId;         /* Reservation ID assigned when reserving SDR */
/* remaining members used by dynamic sensor device only */
        uint8_t      devSdrSize;    /* Number of sensor data records in chassis dynamic Device SDR */
        uint8_t      devSdrDyn;     /* Sensor population dynamic, 1 is dynamic, 0 is static */
        uint8_t      lun0;          /* Sensors in LUN 0, 1 yes, 0 no */
        uint8_t      lun1;          /* Sensors in LUN 1, 1 yes, 0 no */
        uint8_t      lun2;          /* Sensors in LUN 2, 1 yes, 0 no */
        uint8_t      lun3;          /* Sensors in LUN 3, 1 yes, 0 no */
} SdrRepRec, *SdrRep;

/* Handle for each FRU */
typedef struct FruRec_ {
	uint8_t      type;          /* FRU type */
	uint8_t      instance;      /* Instance of this FRU type, set in drvMch.c */
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
	int           fruIndex;     /* Index into FRU array (same as FRU ID) for associated FRU ( -1 if no associated FRU ) */
	uint8_t       instance;     /* Instance of this sensor type on this entity (usually a FRU) */
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
	const char   *name;          /* MCH port name used by asyn */
	int           instance;      /* MCH instance number; assigned at init */
	epicsThreadId pingThreadId;  /* Thread ID for task that periodically pings MCH */
	double        timeout;       /* Asyn read timeout */
	int           session;       /* Enable session with MCH */
	int           err;           /* Count of sequential message errors */         
	int           type;          /* MCH vendor, Vadatech, NAT, etc. */
} MchSessRec, *MchSess;

/* Struct for MCH system information */
typedef struct MchSysRec_ {
	const char   *name;          /* MCH port name used by asyn */
	SdrRepRec     sdrRep;
	FruRec       *fru;           /* Array of FRUs (size of MAX_FRU) */
	int           sensLkup[MAX_FRU][MAX_SENSOR_TYPE][MAX_SENS]; /* Index into sens struct array, used by devsup, -1 if not used */
	uint8_t       sensCount;     /* Sensor count */
	SensorRec    *sens;          /* Array of sensors (size of sensCount) */	
	uint8_t       mgmtCount;     /* Management controller device count */
	SdrMgmtRec   *mgmt;          /* Array of management controller devices (size of mgmtCount) */	
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

/* Response message lengths using Vadatech MCH (0s for those that can vary) */ 
#define IPMI_RPLY_CLOSE_SESSION_LENGTH_VT           22
#define IPMI_RPLY_SENSOR_READ_MAX_LENGTH_VT         48 /* length varies */
#define IPMI_RPLY_GET_SENSOR_THRESH_LENGTH_VT       51 /* need to determine this; using NAT length for now*/
#define IPMI_RPLY_GET_CHAS_STATUS_LENGTH_VT         47 /* does NOT include optional byte */
#define IPMI_RPLY_CHAS_CTRL_LENGTH_VT               44
#define IPMI_RPLY_GET_FRU_INFO_LENGTH_VT            47
#define IPMI_RPLY_READ_FRU_DATA_BASE_LENGTH_VT      45
#define IPMI_RPLY_WRITE_FRU_DATA_LENGTH_VT          45
#define IPMI_RPLY_GET_SDRREP_INFO_LENGTH_VT         58
#define IPMI_RPLY_RESERVE_SDRREP_LENGTH_VT          46
#define IPMI_RPLY_GET_SDR_BASE_LENGTH_VT            50 /* get sdr message length is this + remaining data bytes */
#define IPMI_RPLY_GET_SDR_LENGTH_VT                 0  /* varies */
#define IPMI_RPLY_GET_DEV_SDR_INFO_LENGTH_VT        50
#define IPMI_RPLY_GET_DEV_SDR_LENGTH_VT             0  /* varies */
#define IPMI_RPLY_COLD_RESET_LENGTH_VT              44
#define IPMI_RPLY_SET_FRU_POLICY_LENGTH_VT          44
#define IPMI_RPLY_GET_FRU_POLICY_LENGTH_VT          45
#define IPMI_RPLY_SET_FRU_ACT_LENGTH_VT             45
#define IPMI_RPLY_GET_DEVICE_ID_LENGTH_VT           59 /* includes optional bytes */
#define IPMI_RPLY_GET_FAN_PROP_LENGTH_VT            49
#define IPMI_RPLY_GET_FAN_LEVEL_LENGTH_VT           48
#define IPMI_RPLY_SET_FAN_LEVEL_LENGTH_VT           45

/* Response message lengths using NAT MCH (0s for those we haven't figured out yet) */
#define IPMI_RPLY_CLOSE_SESSION_LENGTH_NAT           22 /* UPDATE THIS */
#define IPMI_RPLY_SENSOR_READ_MAX_LENGTH_NAT         34 /* length varies */
#define IPMI_RPLY_GET_SENSOR_THRESH_LENGTH_NAT       51
#define IPMI_RPLY_GET_CHAS_STATUS_LENGTH_NAT         47 /* does NOT include optional byte */
#define IPMI_RPLY_CHAS_CTRL_LENGTH_NAT               44
#define IPMI_RPLY_GET_FRU_INFO_LENGTH_NAT            32
#define IPMI_RPLY_READ_FRU_DATA_BASE_LENGTH_NAT      30
#define IPMI_RPLY_WRITE_FRU_DATA_LENGTH_NAT          45
#define IPMI_RPLY_GET_SDRREP_INFO_LENGTH_NAT         43
#define IPMI_RPLY_RESERVE_SDRREP_LENGTH_NAT          31
#define IPMI_RPLY_GET_SDR_BASE_LENGTH_NAT            35 /* get sdr message length is this + remaining data bytes */
#define IPMI_RPLY_GET_SDR_LENGTH_NAT                 0  /* varies */
#define IPMI_RPLY_GET_DEV_SDR_INFO_LENGTH_NAT        50
#define IPMI_RPLY_GET_DEV_SDR_LENGTH_NAT             0  /* varies */
#define IPMI_RPLY_COLD_RESET_LENGTH_NAT              44
#define IPMI_RPLY_SET_FRU_POLICY_LENGTH_NAT          29
#define IPMI_RPLY_GET_FRU_POLICY_LENGTH_NAT          39
#define IPMI_RPLY_SET_FRU_ACT_LENGTH_NAT             45
#define IPMI_RPLY_GET_DEVICE_ID_LENGTH_NAT           44 /* includes optional bytes */
#define IPMI_RPLY_GET_FAN_PROP_LENGTH_NAT            43
#define IPMI_RPLY_GET_FAN_LEVEL_LENGTH_NAT           42
#define IPMI_RPLY_SET_FAN_LEVEL_LENGTH_NAT           41
#define IPMI_RPLY_OFFSET_NAT              -15
#define IPMI_RPLY_2ND_BRIDGED_OFFSET_NAT  -8

/* Response message data; offsets from beginning of message; 
 * These defaults work for single-bridged replies from Vadatech MCH.
 * Non-bridged or NAT replies require different offsets
 */
#define IPMI_RPLY_TEMP_ID_OFFSET             21     /* Temporary session ID */
#define IPMI_RPLY_TEMP_ID_LENGTH             4
#define IPMI_RPLY_STR_OFFSET                 25     /* Challenge string */
#define IPMI_RPLY_STR_LENGTH                 16
#define IPMI_RPLY_AUTH_TYPE_OFFSET           21     /* Authentication type */
#define IPMI_RPLY_AUTH_TYPE_LENGTH           1
#define IPMI_RPLY_ID_OFFSET                  22     /* Session ID for remainder of session */
#define IPMI_RPLY_ID_LENGTH                  4
#define IPMI_RPLY_INIT_SEND_SEQ_OFFSET       26     /* Start sequence number for messages to MCH */
#define IPMI_RPLY_INIT_SEND_SEQ_LENGTH       4
#define IPMI_RPLY_MAX_PRIV_OFFSET            30     /* Maximum privilege level allowed for session */
#define IPMI_RPLY_MAX_PRIV_LENGTH            1
#define IPMI_RPLY_SEQ_OFFSET                 5      /* From-MCH sequence number (in reply to non-bridged message) */
#define IPMI_RPLY_SEQ_LENGTH                 4
#define IPMI_RPLY_SEQLUN_OFFSET              18     /* MS 6 bits: IPMI sequence, LS 2 bits: LUN */
#define IPMI_RPLY_COMPLETION_CODE_OFFSET     42
#define IPMI_RPLY_IPMI_VERS_OFFSET           47     /* Returned from Get Device ID Command */
#define IPMI_RPLY_MANUF_ID_OFFSET            49     /* Returned from Get Device ID Command */
#define IPMI_RPLY_MANUF_ID_LENGTH            3    
#define IPMI_RPLY_SENSOR_READING_OFFSET      43     /* Sensor reading (1 byte) */
#define IPMI_RPLY_SENSOR_ENABLE_BITS_OFFSET  44     /* Sensor enable bits: event msgs; scanning; reading/state */
#define IPMI_RPLY_HS_SENSOR_READING_OFFSET   45     /* Hot swap sensor reading (1 byte) */
#define IPMI_RPLY_SENSOR_THRESH_MASK_OFFSET  43     /* Mask of readable thresholds */
#define IPMI_RPLY_SENSOR_THRESH_LNC_OFFSET   44     /* Lower non-critical threshold */
#define IPMI_RPLY_SENSOR_THRESH_LC_OFFSET    45     /* Lower critical threshold */
#define IPMI_RPLY_SENSOR_THRESH_LNR_OFFSET   46     /* Lower non-recoverable threshold */
#define IPMI_RPLY_SENSOR_THRESH_UNC_OFFSET   47     /* Upper non-critical threshold */
#define IPMI_RPLY_SENSOR_THRESH_UC_OFFSET    48     /* Upper critical threshold */
#define IPMI_RPLY_SENSOR_THRESH_UNR_OFFSET   49     /* Upper non-recoverable threshold */
#define IPMI_RPLY_FRU_AREA_SIZE_LSB_OFFSET   43     /* FRU inventory area size in bytes, LSB */
#define IPMI_RPLY_FRU_AREA_SIZE_MSB_OFFSET   44     /* FRU inventory area size in bytes, MSB */
#define IPMI_RPLY_FRU_AREA_ACCESS_OFFSET     45     /* Bit 0 indicates if device is accessed by bytes (0) or words (1) */
#define IPMI_RPLY_FRU_DATA_READ_OFFSET       44     /* FRU data */
#define IPMI_RPLY_SDRREP_VER_OFFSET          43     /* SDR Repository Info */    
#define IPMI_RPLY_SDRREP_CNT_LSB_OFFSET      44     /* Number of records in repository, LSB */
#define IPMI_RPLY_SDRREP_CNT_MSB_OFFSET      45     /* Number of records in repository, MSB */
#define IPMI_RPLY_SDRREP_ADD_TS_OFFSET       48     /* Timestamp of most recent addition, LSB first */
#define IPMI_RPLY_SDRREP_DEL_TS_OFFSET       52     /* Timestamp of most recent deletion, LSB first */
#define IPMI_RPLY_DEV_SDR_CNT_OFFSET         43     /* Number of SDRs for device */
#define IPMI_RPLY_DEV_SDR_FLAGS_OFFSET       44     /* Number of SDRs for device */
#define IPMI_RPLY_GET_SDR_NEXT_ID_LSB_OFFSET 43     /* ID of next sensor in repository, LSB */
#define IPMI_RPLY_GET_SDR_NEXT_ID_MSB_OFFSET 44     /* ID of next sensor in repository, MSB */
#define IPMI_RPLY_GET_SDR_DATA_OFFSET        45     /* ID of next sensor in repository, MSB */
#define IPMI_RPLY_GET_FAN_PROP_MIN_OFFSET    44     /* Fan tray minimum fan level */
#define IPMI_RPLY_GET_FAN_PROP_MAX_OFFSET    45     /* Fan tray maximum fan level */
#define IPMI_RPLY_GET_FAN_PROP_NOM_OFFSET    46     /* Fan tray nominal fan level */
#define IPMI_RPLY_GET_FAN_PROP_PROP_OFFSET   47     /* Fan tray properties */
#define IPMI_RPLY_GET_FAN_LEVEL_OFFSET       44     /* Current fan level */
#define IPMI_RPLY_GET_POWER_LEVEL_PROP_OFFSET   44    /* FRU power properties */
#define IPMI_RPLY_GET_POWER_LEVEL_DELAY_OFFSET  45    /* Delay to stable power */
#define IPMI_RPLY_GET_POWER_LEVEL_MULT_OFFSET   46    /* Power multiplier */
#define IPMI_RPLY_GET_POWER_LEVEL_DRAW_OFFSET   47    /* Draw of first level (optional: next levels follow this, up to 25 total) */
#define IPMI_RPLY_GET_FRU_POLICY_PROP_OFFSET    44    /* FRU activation policy properties */

#define IPMI_RPLY_COMPLETION_CODE_UNBRIDGED_OFFSET 20

#define SENSOR_TYPE_HOTSWAP        0xF0
#define SENSOR_TYPE_HOTSWAP_NAT    0xF2

/* MCH vendors */
#define MCH_TYPE_UNKNOWN  0
#define MCH_TYPE_VT       1
#define MCH_TYPE_NAT      2
#define MCH_IS_VT(x)   (x == MCH_TYPE_VT)
#define MCH_IS_NAT(x)  (x == MCH_TYPE_NAT)

/* Manufacturer ID */
#define MCH_MANUF_ID_VT   0x5D32
#define MCH_MANUF_ID_NAT  0x6C78

/* Vadatech */
#define VT_ENTITY_ID_MCH      0xC2 
#define VT_ENTITY_ID_AMC      0xC1
#define VT_ENTITY_ID_CU       0x1E
#define VT_ENTITY_ID_PM       0x0A
#define VT_ENTITY_ID_RTM      0xC0  /* asked Vivek to verify */

/* PICMG */
#define FRU_PWR_DYNAMIC(x) x & (1 << 7)
#define FRU_PWR_LEVEL(x)   x & 0xF
#define FRU_PWR_STEADY_STATE     0
#define FRU_PWR_STEADY_STATE_DES 1
#define FRU_PWR_EARLY            2
#define FRU_PWR_EARLY_DES        3
#define FRU_PWR_MSG_CMPTBL(x) ((x>=3 && x<=16) || (x>=40 && x<=41) || (x>=50 && x<=53)) /* See Vadatech CLI Manual (5.7.9) */

#define PICMG_ENTITY_ID_FRONT_BOARD          0xA0
#define PICMG_ENTITY_ID_RTM                  0xC0
#define PICMG_ENTITY_ID_AMC                  0xC1
#define PICMG_ENTITY_ID_SHM                  0xF0 /* Shelf management controller */
#define PICMG_ENTITY_ID_FILTRATION_UNIT      0xF1
#define PICMG_ENTITY_ID_SHELF_FRU_INFO       0xF2
#define PICMG_ENTITY_ID_ALARM_PANEL          0xF3
#define PICMG_ENTITY_ID_POWER_FILTERING      0x15

/* MicroTCA */
#define UTCA_FRU_TYPE_CARRIER                0
#define UTCA_FRU_TYPE_SHELF_MIN              1
#define UTCA_FRU_TYPE_SHELF_MAX              2
#define UTCA_FRU_TYPE_MCH_MIN                3
#define UTCA_FRU_TYPE_MCH_MAX                4
#define UTCA_FRU_TYPE_AMC_MIN                5
#define UTCA_FRU_TYPE_AMC_MAX                39
#define UTCA_FRU_TYPE_CU_MIN                 40
#define UTCA_FRU_TYPE_CU_MAX                 49
#define UTCA_FRU_TYPE_PM_MIN                 50
#define UTCA_FRU_TYPE_PM_MAX                 59
#define UTCA_FRU_TYPE_RTM_MIN                90
#define UTCA_FRU_TYPE_RTM_MAX                124
#define UTCA_FRU_TYPE_LOG_CARRIER            253


#ifdef __cplusplus
};
#endif

#endif
