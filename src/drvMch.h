//////////////////////////////////////////////////////////////////////////////
// This file is part of 'ipmiComm'.
// It is subject to the license terms in the LICENSE.txt file found in the
// top-level directory of this distribution and at:
//    https://confluence.slac.stanford.edu/display/ppareg/LICENSE.html.
// No part of 'ipmiComm', including this file,
// may be copied, modified, propagated, or distributed except according to
// the terms contained in the LICENSE.txt file.
//////////////////////////////////////////////////////////////////////////////
#ifndef DRV_MCH_H
#define DRV_MCH_H

#include <epicsThread.h>
#include <devMch.h>
#include <ipmiDef.h>

#define MAX_FRU       255 // 0xFF reserved, according to IPMI 2.0 spec
#define MAX_MGMT      32  // management controller device, arbitrary limit, may need adjusting
#define MAX_FRU_MGMT  MAX_FRU + MAX_MGMT
#define MAX_MCH       255
#define MAX_SENS_INST 32 /* Max instances of one sensor type on one FRU or Management Controller entity */

extern uint32_t mchStat[MAX_MCH];

/* Sensor scan period [seconds] */
extern volatile uint8_t mchSensorScanPeriod;

extern epicsMutexId mchStatMtx[MAX_MCH];

/* Used for sensor scanning; one list per MCH */
extern IOSCANPVT drvSensorScan[MAX_MCH];

/* Vadatech typically sends 2 replies; NAT sends 1 */
#define RPLY_TIMEOUT_SENDMSG_RPLY 0.50
#define RPLY_TIMEOUT_DEFAULT      0.25

/* Mask of MCH status
 * INIT uses 2 lowest bits:
 * must AND stat with MCH_MASK_INIT and
 * then test for value, for example:
 * if ( (mchStat & MCH_MASK_INIT) == MCH_MASK_INIT_DONE )
 * Similarly for DBG
 */
#define MCH_MASK_INIT             (0x3)
#define MCH_MASK_INIT_NOT_DONE    0
#define MCH_MASK_INIT_IN_PROGRESS 1
#define MCH_MASK_INIT_DONE        2
#define MCH_MASK_INIT_FAILED      3
#define MCH_MASK_ONLN             (1 << 2) /* 1 = MCH responds to pings */
#define MCH_MASK_CNFG_CHK         (1 << 3) /* 1 = time to check config up-to-date */
#define MCH_MASK_DBG              (0x30)
#define MCH_INIT_DONE(x)          ((x & MCH_MASK_INIT) == MCH_MASK_INIT_DONE)
#define MCH_INIT_NOT_DONE(x)      ((x & MCH_MASK_INIT) == MCH_MASK_INIT_NOT_DONE)
#define MCH_ONLN(x)               ((x & MCH_MASK_ONLN) >> 2)
#define MCH_CNFG_CHK(x)           ((x & MCH_MASK_CNFG_CHK) >> 3)
#define MCH_DBG(x)                ((x & MCH_MASK_DBG) >> 4)
#define MCH_DBG_SET(x)            (x << 4) /* Select debug bits in mask */

#define MCH_DBG_OFF  0
#define MCH_DBG_LOW  1
#define MCH_DBG_MED  2
#define MCH_DBG_HIGH 3

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct MchCbRec_ MchCbRec;

    typedef struct EntAssocRec_ {
        SdrEntAssocRec sdr;
    } EntAssocRec, *EntAssoc;

    typedef struct DevEntAssocRec_ {
        SdrDevEntAssocRec sdr;
        uint8_t           ownerAddr; /* Slave address of owner */
        uint8_t           ownerChan; /* Channel of owner */
    } DevEntAssocRec, *DevEntAssoc;

    /* Data structure to uniquely identify an entity */
    typedef struct EntityRec_ {
        uint8_t entityId;
        uint8_t entityInst;
        /* Following only used for device-relative entities */
        uint8_t addr; /* For device-relative, this is set to the owner's address if the container address is not defined
                         in the entity assoc record */
        uint8_t chan; /* For device-relative, this is set to the owner's chan if the container chan is not defined in
                         the entity assoc record */
    } EntityRec, *Entity;

    /* Handle for each Management Controller Device
     * Management controllers that also provide FRU data will
     * additionally have an associated FRU data structure
     */
    typedef struct MgmtRec_ {
        uint8_t    instance; /* Instance of management controller set in drvMch.c - needed? */
        SdrMgmtRec sdr;
        SdrRepRec  sdrRep;
        EntityRec*
            entity; /* Array of associated entities that should be considered subsets of this management controller */
        int entityAlloc; /* Flag indicating entity array memory has been allocated and can be freed during configuration
                            update*/
        int entityCount; /* Count of entities contained by this management controller */
    } MgmtRec, *Mgmt;

    /* Handle for each FRU and Management Controller that provides FRU data */
    typedef struct FruRec_ {
        uint8_t id;             /* FRU ID: for MicroTCA matches FRU index and defined by MicroTCA spec;
                                 * for ATCA set to chosen values to identify associated hardware*/
        uint8_t       type;     /* FRU type */
        uint8_t       instance; /* Instance of this FRU type, set in drvMch.c - still in use? */
        uint8_t       size[2];  /* FRU Inventory Area size, LS byte stored first */
        uint8_t       access;   /* Access FRU data by words or bytes */
        uint8_t       readOffset[2];
        uint16_t      read; /* Read number */
        FruChassisRec chassis;
        FruBoardRec   board;
        FruProdRec    prod;
        SdrFruRec     sdr;
        char          parm[10]; /* Describes FRU type, used to load EPICS records */
        uint8_t       pwrDyn;   /* 1 if FRU supports dynamic reconfiguration of power, otherwise 0 */
        uint8_t       pwrDly;   /* Delay to stable power */
                                /* Next section used only by cooling unit FRUs */
        uint8_t    fanMin;      /* Fan minimum level */
        uint8_t    fanMax;      /* Fan maximum level */
        uint8_t    fanNom;      /* Fan nominal level */
        uint8_t    fanProp;     /* [7] 1 if fan try supports automatic fan speed adjustment */
        int        mgmtIndex;   /* If associated with management controller, index into Mgmt array; else -1 */
        EntityRec* entity;      /* Array of associated entities that should be considered subsets of this FRU */
        int entityAlloc; /* Flag indicating entity array memory has been allocated and can be freed during configuration
                            update*/
        int entityCount; /* Count of entities contained by this FRU */
                         /* Next section used only for PICMG systems */
        uint8_t siteNumber; /* Maps to physical location, slot number */
        uint8_t siteType;   /* Maps to physical location, slot number */
    } FruRec, *Fru;

    /* Handle for each SDR */
    typedef struct SensorRec_ {
        SdrFullRec sdr;      /* Full Sensor SDR */
        uint8_t    val;      /* Most recent sensor reading */
        int        fruId;    /* FRU ID for associated FRU ( -1 if no associated FRU ) */
        int        fruIndex; /* Index into FRU array for associated FRU ( -1 if no associated FRU ) */
        int     mgmtIndex; /* Index into Mgmt array for associated Management Controller ( -1 if no associated MGMT ) */
        uint8_t instance;  /* Instance of this sensor type on this entity (usually a FRU) */
        int     unavail;   /* 1 if sensor reading returns 'Requested Sensor, data, or record not present' */
        char    parm[10];  /* Describes signal type, used by device support */
        size_t  readMsgLength; /* Get Sensor Reading message response length */
        int     cnfg;          /* 0 if needs record fields need to be updated */
        uint8_t tmask;         /* Mask of which thresholds are readable */
        uint8_t tlnc;          /* Threshold lower non-critical */
        uint8_t tlc;           /* Threshold lower critical */
        uint8_t tlnr;          /* Threshold lower non-recoverable */
        uint8_t tunc;          /* Threshold upper non-critical */
        uint8_t tuc;           /* Threshold upper critical */
        uint8_t tunr;          /* Threshold upper non-recoverable */
    } SensorRec, *Sensor;

    /* Struct for MCH session information */
    typedef struct MchSessRec_ {
        char          name[MAX_NAME_LENGTH]; /* MCH port name used by asyn */
        int           instance;              /* MCH instance number; assigned at init */
        epicsThreadId pingThreadId;          /* Thread ID for task that periodically pings MCH */
        double        timeout;               /* Asyn read timeout */
        int           session;               /* Enable session with MCH */
        int           err;                   /* Count of sequential message errors */
        int type; /* MCH vendor, Vadatech, NAT, etc. - need to clean this up, perhaps merge with vendor and/or add
                     'features' */
    } MchSessRec, *MchSess;

    /* Struct for MCH system information */
    typedef struct MchSysRec_ {
        char      name[MAX_NAME_LENGTH]; /* MCH port name used by asyn */
        SdrRepRec sdrRep;
        uint32_t  sdrCount;
        uint8_t   fruCount; /* FRU device count */
        size_t
            fruCountMax; /* Max number of supported FRUs. Architecture-dependent. Implemented to reduce memory usage */
        size_t mgmtCountMax; /* Max number of supported MGMTs. Architecture-dependent. Implemented to reduce memory
                                usage */
        size_t sensCountMax; /* Max number of supported sensors. Architecture-dependent. Implemented to reduce memory
                                usage */
        FruRec* fru;         /* Array of FRUs (size of MAX_FRU) */
        int fruLkup[MAX_FRU_MGMT]; /* Element values of fruLkup are indices into data arrays for FRUs (and Management
                                    * Controllers that provide FRU data) Indices of fruLkup are 'ids' of FRU/MGMT, which
                                    * may be chosen arbitrarily for each platform in order to provide consistent IDs in
                                    * device support addresses 0-MAX_FRU used for FRUs, higher indices used for
                                    * Management Controllers; value of -1 if not used
                                    */
        int sensLkup[MAX_FRU_MGMT][MAX_SENSOR_TYPE]
                    [MAX_SENS_INST]; /* Index into sens struct array, used by devsup, -1 if not used */
                                     /* First index is FRU index (not FRU ID) */
        int sensCount;   /* Sensor count, data type must be larger than MAX_FRU_MGMT*MAX_SENSOR_TYPE*MAX_SENS_INST */
        SensorRec* sens; /* Array of sensors (size of sensCount) */
        int sensAlloc;   /* Flag indicating sensor array memory has been allocated and can be freed during configuration
                            update*/
        uint8_t        mgmtCount; /* Management controller device count */
        MgmtRec*       mgmt;      /* Array of management controller devices (size of mgmtCount) */
        MchCbRec*      mchcb;     /* Callbacks for architecture-specific functionality */
        EntAssocRec    entAssoc[10];
        int            entAssocCount;
        DevEntAssocRec devEntAssoc[10];
        int            devEntAssocCount;
    } MchSysRec, *MchSys;
    /* Struct for MCH system information */
    typedef struct MchDataRec_ {
        IpmiSess ipmiSess; /* IPMI session information; defined in ipmiDef.h */
        MchSess  mchSess;  /* Additional MCH session info */
        MchSys   mchSys;   /* MCH system info */
    } MchDataRec, *MchData;

    /*extern MchSys mchSysData[MAX_MCH];*/

    int  mchCnfgChk(MchData mchData);
    void mchStatSet(int inst, uint32_t clear, uint32_t set);
    int  mchNewSession(MchSess mchSess, IpmiSess ipmiSess);
    int  mchGetSensorReadingStat(MchData mchData, uint8_t* response, Sensor sens);
    int  mchGetFruIdFromIndex(MchData mchData, int index);

#define IPMI_RPLY_CLOSE_SESSION_LENGTH_VT 22

/* IPMI device types */
#define MCH_TYPE_MAX         10 /* Number possible MCH types; increase as needed */
#define MCH_TYPE_UNKNOWN     0
#define MCH_TYPE_VT          1
#define MCH_TYPE_NAT         2
#define MCH_TYPE_SUPERMICRO  3
#define MCH_TYPE_PENTAIR     4
#define MCH_TYPE_ARTESYN     5
#define MCH_TYPE_ADVANTECH   6
#define MCH_IS_VT(x)         (x == MCH_TYPE_VT)
#define MCH_IS_NAT(x)        (x == MCH_TYPE_NAT)
#define MCH_IS_SUPERMICRO(x) (x == MCH_TYPE_SUPERMICRO)
#define MCH_IS_ADVANTECH(x)  (x == MCH_TYPE_ADVANTECH)
#define MCH_IS_MICROTCA(x)   ((x == MCH_TYPE_NAT) || (x == MCH_TYPE_VT))
#define MCH_IS_ATCA(x)       ((x == MCH_TYPE_PENTAIR) || (x == MCH_TYPE_ARTESYN))
#define MCH_IS_PICMG(x)      (MCH_IS_MICROTCA(x) || MCH_IS_ATCA(x))

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
#define MCH_MANUF_ID_SUPERMICRO 0x2A7C /* Supermicro PC */
#define MCH_MANUF_ID_PENTAIR    0x400A /* Formerly Pigeon Point; ATCA system */
#define MCH_MANUF_ID_ARTESYN    0x65CD /* Formerly Emerson; ATCA system */
#define MCH_MANUF_ID_ADVANTECH  0x2839 /* Advantech PC */

/* Vadatech */
#define VT_ENTITY_ID_MCH 0xC2
#define VT_ENTITY_ID_AMC 0xC1
#define VT_ENTITY_ID_CU  0x1E
#define VT_ENTITY_ID_PM  0x0A
#define VT_ENTITY_ID_RTM 0xC0 /* asked Vivek to verify */

    extern const void* mchCbRegistryId;
    extern struct MchCbRec_ {
        void (*assign_sys_sizes)(MchData mchData);
        void (*assign_site_info)(MchData mchData);
        void (*assign_fru_lkup)(MchData mchData);
        int (*fru_data_suppl)(MchData mchData, int index);
        void (*sensor_get_fru)(MchData mchData, Sensor sens);
        int (*get_chassis_status)(MchData mchData, uint8_t* data);
        /* Following are PICMG only */
        int (*set_fru_act)(MchData mchData, uint8_t* data, uint8_t fruIndex, uint8_t parm);
        int (*get_fan_prop)(MchData mchData, uint8_t* data, uint8_t fruIndex);
        int (*get_fan_level)(MchData mchData, uint8_t* data, uint8_t fruIndex, uint8_t* level);
        int (*set_fan_level)(MchData mchData, uint8_t* data, uint8_t fruIndex, uint8_t level);
        int (*get_power_level)(MchData mchData, uint8_t* data, uint8_t fruIndex, uint8_t parm);
    }* MchCb;

#ifdef __cplusplus
};
#endif

#endif
