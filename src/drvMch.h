#ifndef DRV_MCH_H
#define DRV_MCH_H

#include <epicsThread.h>
#include <asynDriver.h>
#include <devMch.h>
#include <ipmiDef.h>

#define MAX_FRU             255
#define SDR_FRU_MAX_LENGTH  33
#define MAX_MCH             255

#define MCH_INIT_NOT_DONE    0
#define MCH_INIT_IN_PROGRESS 1
#define MCH_INIT_DONE        2

#ifdef __cplusplus
extern "C" {
#endif

extern int mchIsAlive[MAX_MCH];
extern int mchInitDone[MAX_MCH];

typedef struct FruFieldRec_ {
	uint8_t     type;         /* Type of data */
	uint8_t     length;       /* Length of data (bytes) */
	uint8_t    *data;
} FruFieldRec, *FruField;

typedef struct FruBoardRec_ {
	uint8_t      offset;       /* Offset into FRU storage */
	uint8_t      version;      /* Format version (0x01) */
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
	uint8_t      version;      /* Format version (0x01) */
	uint8_t      length;       /* Length of FRU area */
        uint8_t      lang;         /* Language code */
	FruFieldRec  manuf;        /* Manufacturer */
	FruFieldRec  prod;         /* Product name */
	FruFieldRec  part;         /* Part number */
	FruFieldRec  ver;          /* Version number */
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
        uint8_t      length;        /* Number of remaining record bytes (we only use the first few) */
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
 * Handle for FRU Device Locator Record, IPMI Table 43-7
 */
typedef struct SdrFruRec_ {
	uint8_t      id[2];         /* Sensor ID (can change, so key bytes must also be used to identify a sensor) */
	uint8_t      ver;           /* SDR version (0x51) */
	uint8_t      recType;       /* Record type, 0x01 for Full Sensor Record */
        uint8_t      length;        /* Number of remaining record bytes (we only use the first few) */
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
	

/* Handle for Sensor Data Record Repository and Dynamic Sensor Device */
typedef struct SdrRepRec_ {
	uint8_t      ver;           /* SDR Version (0x51) */
        uint8_t      size[2];       /* Number of sensor data records; LS byte stored first */
	uint8_t      free[2];       /* Free space in bytes; LS byte stored first */
	uint8_t      addTs[4];      /* Timestamp of most recent addition; LS byte stored first */
        uint8_t      delTs[4];      /* Timestamp of most recent erase; LS byte stored first */
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
	uint8_t      instance;      /* Instance of this FRU type, set in drvMchUtil.c */
	uint8_t      size[2];       /* FRU Inventory Area size, LS byte stored first */	
	uint8_t      access;        /* Access FRU data by words or bytes */
	uint8_t      readOffset[2];  
	uint16_t     read;          /* Read number */
	FruBoardRec  board;
	FruProdRec   prod;
	SdrFruRec    sdr;
	char         parm[10];      /* Describes FRU type, used to load EPICS records */
	int          hotswap;       /* Index into sensor array for FRU's hotswap sensor */
	int          tempCnt;       /* Number of temperature sensors associated with this FRU */         
	int          fanCnt;        /* Number of fan sensors associated with this FRU */         
	int          vCnt;          /* Number of v sensors associated with this FRU */ 
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
} SensorRec, *Sensor;

/* Struct for MCH session information */
typedef struct MchSessRec_ {
	const char   *name;          /* MCH port name used by asyn */
	int           instance;      /* MCH instance number; assigned at init */
	epicsThreadId pingThreadId;  /* Thread ID for task that periodically pings MCH */
	uint8_t       id[4];         /* Session ID */
	uint8_t       str[16];       /* Session challenge string; used in establishing a session */
        uint8_t       seqSend[4];    /* Message sequence number for messages to MCH, null until session activated; chosen by MCH; rolls over at 0xFFFFFFFF */
        uint8_t       seqRply[4];    /* Message sequence number for messages from MCH; rolls over at 0xFFFFFFFF */
	uint8_t       seq;           /* IPMI sequence (6-bit number); used to confirm reply is for correct request */
	double        timeout;       /* Asyn read timeout */
	int           session;       /* Enable session with MCH */
	int           err;           /* Count of sequential message errors */
} MchSessRec, *MchSess;

/* Struct for MCH system information */
typedef struct MchSysRec_ {
	const char   *name;          /* MCH port name used by asyn */
	SdrRepRec     sdrRep;
	uint8_t       sensCount;     /* Sensor count */
	SensorRec    *sens;          /* Array of sensors (size of senscount) */	
	uint8_t       fruCount;      /* FRU count */
	FruRec       *fru;           /* Array of FRUs (size of MAX_FRU) */
} MchSysRec, *MchSys;

/* Struct for MCH system information */
typedef struct MchDataRec_ {
	MchSess        mchSess;       /* MCH session info */
} MchDataRec, *MchData;

extern MchSys mchSysData[MAX_MCH];

#ifdef __cplusplus
};
#endif

#endif
