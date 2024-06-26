//////////////////////////////////////////////////////////////////////////////
// This file is part of 'ipmiComm'.
// It is subject to the license terms in the LICENSE.txt file found in the
// top-level directory of this distribution and at:
//    https://confluence.slac.stanford.edu/display/ppareg/LICENSE.html.
// No part of 'ipmiComm', including this file,
// may be copied, modified, propagated, or distributed except according to
// the terms contained in the LICENSE.txt file.
//////////////////////////////////////////////////////////////////////////////
#ifndef IPMI_DEF_H
#define IPMI_DEF_H

/***************************

TODO:

UPDATE ALL MSG ROUTINES TO USE FRU INDEX INSTEAD OF ID

Test MicroTCA remote on/off commands with new privilege level of operator

Test everything with Vadatech MCH

For username/password, find other solution than hard-coded in code/headers

Need to reconsider how to handle alarms if sensor scanning disabled
--Continue to enforce alarms? Instead of INVALID, 'grey' out alarms on display?

Change language/names everywhere to no longer use 'MCH' to describe a general system

***************************/

#include <math.h> /* pow */

#define MSG_MAX_LENGTH 200 /* arbitrarily chosen, not in the specs */

/* BMC features; currently named MCH, which will eventually have to be changed everywhere to be more generic */
#define MCH_FEAT_SENDMSG_RPLY                                                                                          \
    (1 << 0) /* BMC responds separately to send-message before responding to embedded message; also requires longer    \
                message timeout */
#define MCH_FEAT_PLL_MON  (1 << 1) /* Monitor additional voltage/temp of PLL */
#define MCH_FEAT_FPGA_MON (1 << 2) /* Monitor Xilinx */
#define MCH_FEAT_FOT_MON  (1 << 3) /* Monitor transceiver */
#define MCH_FEAT_SYS_INFO (1 << 4) /* Monitor system info */

typedef struct IpmiSessRec_* IpmiSess;

/*   Callback helper function to write/read IPMI messages
 *
 *   device   - Pointer to device data structure (see driver)
 *   sess     - Pointer to IPMI data structure (see ipmiDef.h)
 *   msg      - Pointer to outgoing message array
 *   msg_n    - Size of outgoing message
 *   rply     - Pointer to response array
 *   rply_n   - Expected response size, 0 if unknown
 *   cmd      - IPMI command code
 *   netfn    - IPMI network function
 *   offs     - Offset from IPMI_RPLY_COMPLETION_CODE_OFFSET to read comp code (see ipmiDef.h)
 *   outsess  - 1 if outside a session; 0 if in a session
 */
typedef int (*IpmiWriteReadHelper)(void* device, IpmiSess sess, uint8_t* msg, size_t msg_n, uint8_t* rply,
                                   size_t* rply_n, uint8_t cmd, uint8_t netfn, int offs, int outsess);

/* Struct for IPMI session information */
typedef struct IpmiSessRec_ {
    uint8_t ivers;      /* IPMI version number */
    uint8_t id[4];      /* Session ID */
    uint8_t str[16];    /* Session challenge string; used in establishing a session */
    uint8_t seqSend[4]; /* Message sequence number for messages to device, null until session activated; chosen by
                           system software (us); rolls over at 0xFFFFFFFF */
    uint8_t seqRply[4]; /* Message sequence number for messages from device; rolls over at 0xFFFFFFFF */
    uint8_t seq;        /* IPMI sequence (6-bit number); used to confirm reply is for correct request */
    uint8_t chan;       /* Channel number, returned by Get Channel Authentication Capabilities response */
    uint8_t authSup;    /* Channel authentication types supported, returned by Get Channel Authentication Capabilities
                           response */
    uint8_t authReq; /* Authentication type requested, used in Get Session Challenge and Activate Session requests (and
                        any other authenticated requests) */
    uint8_t             features; /* Mask to describe vendor-specific behavior */
    double              timeout;  /* Asyn read timeout */
    IpmiWriteReadHelper wrf;      /* Callback to driver write/read function */
} IpmiSessRec;

/* Data structure for Sensor Data Record (one per sensor) IPMI v2.0 Section 43.1
 * Used for both Full and Compact SDRs
 * In this implementation, we don't support all of the Full SDR fields
 */
typedef struct SdrFullRec_ {
    uint8_t id[2];      /* Sensor ID (can change, so key bytes must also be used to identify a sensor) */
    uint8_t ver;        /* SDR version (0x51) */
    uint8_t recType;    /* Record type, 0x01 for Full Sensor Record */
    uint8_t length;     /* Number of remaining record bytes */
    uint8_t owner;      /* Sensor owner ID, [7:1] slave address or software ID; [0] 0 = IPMB, 1 = system software */
    uint8_t lun;        /* Sensor owner LUN, [7:4] channel number; [3:2] reserved; [1:0] sensor owner LUN */
    uint8_t number;     /* Sensor number, unique behind given address and LUN, 0xFF reserved */
    uint8_t entityId;   /* Entity ID (physical entity that sensor is associated with) */
    uint8_t entityInst; /* Entity instance, [7] 0 = physical entity, 1 = logical; [6:0] instance number 0x00-0x5F
                           system-relative, 0x60-0x7F device-relative */
    uint8_t init;       /* Sensor initialization (see IPMI spec page 472 for details) */
    uint8_t cap;        /* Sensor capabilities (see IPMI spec page 473 for details) */
    uint8_t sensType;   /* Sensor type code (see IPMI spec table 42-3) */
    uint8_t readType;   /* Event/reading type code (see IPMI spec table 42-1) */
    /* Following fields only applicable to Full SDR */
    uint8_t units1; /* [7:6] analog data format, [5:3] rate unit, [2:1] modifier unit, [0] percentage */
    uint8_t units2; /* Units type code, IPMI 43-15 */
    uint8_t units3; /* Units type code, 0x00 if unused */
    uint8_t linear; /* Linearization [7] reserved, [6:0] formula type */
    uint8_t M;      /* M in conversion, LS 8 bits (2's complement signed 10-bit 'M' value) */
    uint8_t MTol;   /* [7:6] MS 2 bits of M, [5:0] tolerance 6 bits unsigned (in +/- 1/2 raw counts) */
    uint8_t B;      /* B in conversion, LS 8 bits (2's complement signed 10-bit 'B' value) */
    uint8_t BAcc;   /* [7:6] MS 2 bits of B, [5:0] accuracy LW 6 bits (unsigned 10-bit Basic Sensor Accuracy in 1/100 %
                       scaled up by accuracy exponent */
    uint8_t acc;    /* [7:4] MS 4 bits of accuracy, [3:2] accuracy exponent 2 bits unsigned, [1:0] sensor direction */
    uint8_t RexpBexp; /* [7:4] R (result) exponent 4 bits 2's complement signed, [3:0] B exponent 4 bits, 2's complement
                         signed */
    uint8_t strLength; /* Sensor ID string type/length */
    uint8_t anlgChar;  /* Analog characteristics [7:3] reserved, [2] normal min specified, [1] normal max specified [0],
                          nominal reading specified */
    uint8_t nominal;   /* Nominal reading (raw units) */
    uint8_t normMax;   /* Normal maximum (raw units) */
    uint8_t normMin;   /* Normal minimum (raw units) */
    char    str[17];   /* Sensor ID string, 16 bytes maximum */
    /* calculated values, only supported for Full SDR */
    int m;
    int b;
    int rexp;
    int bexp;

} SdrFullRec, *SdrFull;

/*
 * Data structure for FRU Device Locator Record, IPMI v2.0 Table 43-7
 */
typedef struct SdrFruRec_ {
    uint8_t id[2];   /* Sensor ID (can change, so key bytes must also be used to identify a sensor) */
    uint8_t ver;     /* SDR version (0x51) */
    uint8_t recType; /* Record type, 0x11 for FRU Device Locator Record */
    uint8_t length;  /* Number of remaining record bytes */
    uint8_t addr;    /* [7:1] slave address of controller, all 0 if dev on IPMB, [0] reserved */
    uint8_t fruId; /* FRU Device ID/Slave address. For logical FRU device, [7:0] FRU dev ID, for non-intelligent, [7:1]
                      slave address, [0] reserved */
    uint8_t lun; /* [7] logical/physical FRU device, [6:5] reserved, [4:3] LUN for FRU commands, [2:0] private bus ID */
    uint8_t chan;       /* [7:4] Channel number to access device, 0 if on IPMB (MS bit in next byte?); [3:0] reserved */
    uint8_t devType;    /* Device type code Table 43-12 */
    uint8_t devMod;     /* Device type modifier Table 43-12 */
    uint8_t entityId;   /* Entity ID (physical entity that sensor is associated with) */
    uint8_t entityInst; /* Entity instance, [7] 0 = physical entity, 1 = logical; [6:0] instance number 0x00-0x5F
                           system-relative, 0x60-0x7F device-relative */
    uint8_t strLength;  /* Device ID string type/length Section 43.15*/
    char    str[17];    /* Device ID string, 16 bytes maximum */
} SdrFruRec, *SdrFru;

/*
 * Data structure for Management Controller Device Locator Record, IPMI v2.0 Table 43-8
 */
typedef struct SdrMgmtRec_ {
    uint8_t id[2];   /* Record ID */
    uint8_t ver;     /* SDR version */
    uint8_t recType; /* Record type, 0x12 for Management Controller Locator */
    uint8_t length;  /* Number of remaining record bytes */
    uint8_t addr;    /* [7:1] I2C slave address of device on channel, [0] reserved */
    uint8_t chan;    /* [7:4] reserved; [3:0] Channel number for the channel that the management controller is on, use 0
                        for primary BMC */
    uint8_t pwr;     /* Power state notification and Global initialization, see IPMI spec for meaning of each bit */
    uint8_t cap;     /* Device capabilities, see IPMI spec for meaning of each bit */
    /* next 3 bytes reserved */
    uint8_t entityId;   /* Entity ID for the FRU associated with this device, 0 if not specified */
    uint8_t entityInst; /* Entity instance */
    uint8_t strLength;  /* Device ID string type/length Section 43.15*/
    char    str[17];    /* Device ID string, 16 bytes maximum */
} SdrMgmtRec, *SdrMgmt;

/*
 * Data structure for Entity Association Record, IPMI v2.0 Table 43-4
 */
typedef struct SdrEntAssocRec_ {
    uint8_t id[2];      /* Record ID */
    uint8_t ver;        /* SDR version */
    uint8_t recType;    /* Record type, 0x08 for Entity Association Record */
    uint8_t length;     /* Number of remaining record bytes */
    uint8_t cntnrId;    /* Entity ID for container entity */
    uint8_t cntnrInst;  /* Entity instance for container entity */
    uint8_t flags;      /* [7] 0=contained entities specified as list, 1=specified as range;
                         * [6] 0=no linked Entity Assoc records, 1=linked Entity Assoc records exist;
                         * [5] 0=container and contained entities can be assumed absent if presence sensor for container
                         *       cannot be acessed, 1=presence sensor should always be available
                         * [4:0] reserved
                         */
    uint8_t cntndId1;   /* If list Entity Id for contained entity 1, if range Entity Id of entity for contained entity
                           range 1 */
    uint8_t cntndInst1; /* If list Entity Inst for contained entity 1, if range Entity Inst for *first* entity in
                           contained entity range 1 */
                        /* contained entity id/inst fields 2-4 are 0 if unspecified */
    uint8_t cntndId2;   /* If list Entity Id for contained entity 2, if range Entity Id of entity for contained entity
                           range 1 (must match cnndEntId1) */
    uint8_t cntndInst2; /* If list Entity Inst for contained entity 2, if range Entity Inst for *last* entity in
                           contained entity range 1 */
    uint8_t cntndId3;   /* If list Entity Id for contained entity 3, if range Entity Id of entity for contained entity
                           range 2 */
    uint8_t cntndInst3; /* If list Entity Inst for contained entity 13 if range Entity Inst for *first* entity in
                           contained entity range 2 */
    uint8_t cntndId4;   /* If list Entity Id for contained entity 4, if range Entity Id of entity for contained entity
                           range 2 */
    uint8_t cntndInst4; /* If list Entity Inst for contained entity 4, if range Entity Inst for *last* entity in
                           contained entity range 2 */
} SdrEntAssocRec, *SdrEntAssoc;

/*
 * Data structure for Device-relative Entity Association Record, IPMI v2.0 Table 43-4
 */
typedef struct SdrDevEntAssocRec_ {
    uint8_t id[2];      /* Record ID */
    uint8_t ver;        /* SDR version */
    uint8_t recType;    /* Record type, 0x08 for Entity Association Record */
    uint8_t length;     /* Number of remaining record bytes */
    uint8_t cntnrId;    /* Entity ID for container entity */
    uint8_t cntnrInst;  /* Entity instance for container entity */
    uint8_t cntnrAddr;  /* Device address for container entity, [7:1] slave address of mgmt cntrlr against which
                           device-relative instance is defined, [0] reserved */
    uint8_t cntnrChan;  /* [7:4] Channel number of channel that holds the mgmt cntrlr, 0 if Entity Inst is
                           device-relative, [3:0] reserved */
    uint8_t flags;      /* [7] 0=contained entities specified as list, 1=specified as range;
                         * [6] 0=no linked Entity Assoc records, 1=linked Entity Assoc records exist;
                         * [5] 0=container and contained entities can be assumed absent if presence sensor for container
                         *       cannot be acessed, 1=presence sensor should always be available
                         * [4:0] reserved
                         */
    uint8_t cntndAddr1; /* [7:1] Slave address of mgmt cntrlr against which device-relative Entity Inst for contained
                           entity 1 is defined, [0] reserved */
    uint8_t
        cntndChan1; /* [7:4] Channel number of channel that holds mgmt cntrlr against which device-relative Entity Inst
                     * for contained entity 1 is defined, 0 if instance values are device-relative; [3:0] reserved
                     */
    uint8_t cntndId1;   /* If list Entity Id for contained entity 1, if range Entity Id of entity for contained entity
                           range 1 */
    uint8_t cntndInst1; /* If list Entity Inst for contained entity 1, if range Entity Inst for *first* entity in
                           contained entity range 1 */
                        /* contained entity id/inst fields 2-4 are 0 if unspecified */
    uint8_t cntndAddr2; /* [7:1] Slave address of mgmt cntrlr against which device-relative Entity Inst for contained
                           entity 1 is defined, [0] reserved */
    uint8_t
        cntndChan2; /* [7:4] Channel number of channel that holds mgmt cntrlr against which device-relative Entity Inst
                     * for contained entity 1 is defined, 0 if instance values are device-relative; [3:0] reserved
                     */
    uint8_t cntndId2;   /* If list Entity Id for contained entity 2, if range Entity Id of entity for contained entity
                           range 2 (must match containedEntityId1) */
    uint8_t cntndInst2; /* If list Entity Inst for contained entity 2, if range Entity Inst for *last* entity in
                           contained entity range 2 */

    uint8_t cntndAddr3; /* [7:1] Slave address of mgmt cntrlr against which device-relative Entity Inst for contained
                           entity 3 is defined, [0] reserved */
    uint8_t
        cntndChan3; /* [7:4] Channel number of channel that holds mgmt cntrlr against which device-relative Entity Inst
                     * for contained entity 3 is defined, 0 if instance values are device-relative; [3:0] reserved
                     */
    uint8_t cntndId3;   /* If list Entity Id for contained entity 3, if range Entity Id of entity for contained entity
                           range 2 */
    uint8_t cntndInst3; /* If list Entity Inst for contained entity 13 if range Entity Inst for *first* entity in
                           contained entity range 2 */

    uint8_t cntndAddr4; /* [7:1] Slave address of mgmt cntrlr against which device-relative Entity Inst for contained
                           entity 4 is defined, [0] reserved */
    uint8_t
        cntndChan4; /* [7:4] Channel number of channel that holds mgmt cntrlr against which device-relative Entity Inst
                     * for contained entity 4 is defined, 0 if instance values are device-relative; [3:0] reserved
                     */
    uint8_t cntndId4;   /* If list Entity Id for contained entity 4, if range Entity Id of entity for contained entity
                           range 2 */
    uint8_t cntndInst4; /* If list Entity Inst for contained entity 4, if range Entity Inst for *last* entity in
                           contained entity range 2 */

} SdrDevEntAssocRec, *SdrDevEntAssoc;

/* Data structure for Sensor Data Record Repository and Dynamic Sensor Device */
typedef struct SdrRepRec_ {
    uint8_t  ver;       /* SDR Version (0x51) */
    uint8_t  size[2];   /* Number of sensor data records; LS byte stored first */
    uint8_t  free[2];   /* Free space in bytes; LS byte stored first */
    uint32_t addTs;     /* Timestamp of most recent addition - remove: ; LS byte stored first */
    uint32_t delTs;     /* Timestamp of most recent erase - remove: ; LS byte stored first */
    uint8_t  config[1]; /* 8 bits to describe Respository configuration, mostly supported commands */
    uint8_t  resId;     /* Reservation ID assigned when reserving SDR */
                        /* remaining members used by dynamic sensor device only */
    uint8_t devSdrSize; /* Number of sensor data records in chassis dynamic Device SDR */
    uint8_t devSdrDyn;  /* Sensor population dynamic, 1 is dynamic, 0 is static */
    uint8_t lun0;       /* Sensors in LUN 0, 1 yes, 0 no */
    uint8_t lun1;       /* Sensors in LUN 1, 1 yes, 0 no */
    uint8_t lun2;       /* Sensors in LUN 2, 1 yes, 0 no */
    uint8_t lun3;       /* Sensors in LUN 3, 1 yes, 0 no */
} SdrRepRec, *SdrRep;

// Find a way to not hard-code array sizes below
extern uint8_t RMCP_HEADER[4];
extern uint8_t ASF_MSG[8];
extern uint8_t IPMI_WRAPPER[10];
extern uint8_t IPMI_WRAPPER_PWD_KEY[26];
extern uint8_t IPMI_MSG1[3];
extern uint8_t GET_AUTH_MSG[6];
extern uint8_t GET_SESS_MSG[21];
extern uint8_t GET_SESS_MSG_PWD_KEY[21];
extern uint8_t ACT_SESS_MSG[26];
extern uint8_t SET_PRIV_MSG[5];
extern uint8_t SEND_MSG_MSG[5];
extern uint8_t SENS_READ_MSG[5];
extern uint8_t GET_SENSOR_THRESH_MSG[5];
extern uint8_t FRU_READ_MSG[8];
extern uint8_t GET_SDR_MSG[10];
extern uint8_t CLOSE_SESS_MSG[8];
extern uint8_t CHAS_CTRL_MSG[5];
extern uint8_t GET_DEV_SDR_INFO_MSG[5];
extern uint8_t BASIC_MSG[4];

/* RMCP message header */
#define RMCP_MSG_VER           0x06 /* RMCP message version */
#define RMCP_MSG_SEQ           0xFF /* RMCP message header; 0xFF for no ack */
#define RMCP_MSG_CLASS_ASF     0x06 /* Message class, ASF  */
#define RMCP_MSG_CLASS_IPMI    0x07 /* Message class, IPMI */
#define RMCP_MSG_AUTH_NONE     0x00 /* Message authentication type, 0 for none */
#define RMCP_MSG_HEADER_LENGTH sizeof(RMCP_HEADER)
#define RMCP_MSG_CLASS_OFFSET  3 /* Message class */

/* ASF message */
#define ASF_MSG_OFFSET               RMCP_MSG_HEADER_LENGTH
#define ASF_MSG_HEADER_LENGTH        8
#define ASF_RPLY_PONG_PAYLOAD_LENGTH 16

/* IPMI message wrapper */
extern size_t IPMI_WRAPPER_LENGTH;
extern size_t IPMI_WRAPPER_AUTH_LENGTH;
#define IPMI_WRAPPER_AUTH_TYPE_OFFSET 0  /* Authentication type */
#define IPMI_WRAPPER_SEQ_INITIAL      1  /* Our starting sequence number (arbitrary) */
#define IPMI_WRAPPER_SEQ_OFFSET       1  /* Session sequence number */
#define IPMI_WRAPPER_SEQ_LENGTH       4  /* Session sequence number length */
#define IPMI_WRAPPER_ID_OFFSET        5  /* Session ID */
#define IPMI_WRAPPER_ID_LENGTH        4  /* Session ID length */
#define IPMI_WRAPPER_AUTH_CODE_OFFSET 9  /* Message authentication code offset (used in authenticated messages only) */
#define IPMI_WRAPPER_AUTH_CODE_LENGTH 16 /* Message authentication code length */
#define IPMI_WRAPPER_NBYTES_OFFSET    9  /* Number of bytes in IPMI message (includes bridged message) */
#define IPMI_WRAPPER_AUTH_NBYTES_OFFSET                                                                                \
    9 + IPMI_WRAPPER_AUTH_CODE_LENGTH /* Number of bytes in IPMI message (includes bridged message) */

/* IPMI message bytes after wrapper but before payload
 * Includes IMSG1 and first 3 bytes of IMSG2 (up to but not including completion code)
 * Just convenient offsets for extracting payload data from replies
 */
#define IPMI_MSG_HEADER_LENGTH       IPMI_MSG1_LENGTH + 3
#define IPMI_RPLY_HEADER_LENGTH      RMCP_MSG_HEADER_LENGTH + IPMI_WRAPPER_LENGTH + IPMI_MSG_HEADER_LENGTH
#define IPMI_RPLY_HEADER_AUTH_LENGTH RMCP_MSG_HEADER_LENGTH + IPMI_WRAPPER_AUTH_LENGTH + IPMI_MSG_HEADER_LENGTH

/* IPMI message 1 */
extern size_t IPMI_MSG1_LENGTH;

/* IPMI message 2 common */
#define IPMI_MSG2_SEQLUN_OFFSET    1
#define IPMI_SEQLUN_EXTRACT_SEQ(x) ((x & 0xFC) >> 2)

/* Authentication types */
#define IPMI_AUTH_TYPE_SUPPORT(x)  (x & 0x37)
#define IPMI_MSG_AUTH_TYPE_NONE    0
#define IPMI_MSG_AUTH_TYPE_MD2     1
#define IPMI_MSG_AUTH_TYPE_MD5     2
#define IPMI_MSG_AUTH_TYPE_PWD_KEY 4
#define IPMI_MSG_AUTH_TYPE_OEM     5

/* Privilege levels */
#define IPMI_MSG_PRIV_LEVEL_NONE  0x00 /* Privilege level: none  */
#define IPMI_MSG_PRIV_LEVEL_CB    0x01 /* Privilege level: callback  */
#define IPMI_MSG_PRIV_LEVEL_USER  0x02 /* Privilege level: user  */
#define IPMI_MSG_PRIV_LEVEL_OPER  0x03 /* Privilege level: operator  */
#define IPMI_MSG_PRIV_LEVEL_ADMIN 0x04 /* Privilege level: administrator  */
#define IPMI_MSG_PRIV_LEVEL_OEM   0x05 /* Privilege level: OEM proprietary  */

/* Channel numbers and request options */
#define IPMI_MSG_CHAN_IPMB0 0
#define IPMI_MSG_CHAN_IPMBL 7
#define IPMI_MSG_NOTRACKING 0
#define IPMI_MSG_TRACKING   (1 << 6)
#define IPMI_MSG_RAW        (1 << 7)
#define IPMI_MSG_CURR_CHAN  0x0E /* Channel number currently in use */

/* IPMI responder and requester addresses */
#define IPMI_MSG_ADDR_BMC                    0x20 /* UTC001 MCH BMC address */
#define IPMI_MSG_ADDR_CM                     0x82 /* UTC001 MCH carrier manager responder address */
#define IPMI_MSG_ADDR_SW                     0x81 /* Our address (can be 0x81, 0x83, 0x85, 0x87, 0x89) */
#define IPMI_MSG1_RSADDR_OFFSET              0
#define IPMI_MSG1_NETFNLUN_OFFSET            1
#define IPMI_MSG2_RQADDR_OFFSET              0 /* Requester's address (used in bridged IPMI msg 2) */
#define IPMI_MSG2_ID_OFFSET                  3 /* Session ID */
#define IPMI_MSG2_ID_LENGTH                  4
#define IPMI_MSG2_STR_OFFSET                 5 /* Challenge string */
#define IPMI_MSG2_STR_LENGTH                 16
#define IPMI_MSG2_SEQLUN_OFFSET              1 /* MS 6 bits: IPMI sequence, LS 2 bits: LUN */
#define IPMI_MSG2_CMD_OFFSET                 2 /* Command code */
#define IPMI_MSG2_CHAN_OFFSET                3 /* Channel to send message over (0 for IPMB) */
#define IPMI_MSG2_SENSOR_OFFSET              3 /* Sensor number */
#define IPMI_MSG2_AUTH_TYPE_OFFSET           3 /* For Get Session Challenge and Activate Session */
#define IPMI_MSG2_PRIV_LEVEL_OFFSET          3
#define IPMI_MSG2_READ_FRU_ID_OFFSET         3
#define IPMI_MSG2_READ_FRU_LSB_OFFSET        4 /* FRU Inventory Offset to read, LS Byte */
#define IPMI_MSG2_READ_FRU_MSB_OFFSET        5 /* FRU Inventory Offset to read, MS Byte */
#define IPMI_MSG2_READ_FRU_CNT_OFFSET        6 /* Count to read in bytes, 1-based */
#define IPMI_MSG2_GET_SDR_RES_LSB_OFFSET     3 /* SDR Reservation ID, LS Byte */
#define IPMI_MSG2_GET_SDR_RES_MSB_OFFSET     4 /* SDR Reservation ID, LS Byte */
#define IPMI_MSG2_GET_SDR_ID_LSB_OFFSET      5 /* SDR Record ID, LS Byte */
#define IPMI_MSG2_GET_SDR_ID_MSB_OFFSET      6 /* SDR Record ID, MS Byte */
#define IPMI_MSG2_GET_SDR_OFFSET_OFFSET      7 /* Offset into record to start read */
#define IPMI_MSG2_GET_SDR_CNT_OFFSET         8 /* Count to read in bytes (0xFF means entire record) */
#define IPMI_MSG2_GET_DEV_SDR_INFO_OP_OFFSET 3 /* 1 get SDR count, 0 get sensor count */

/* IPMI message command codes (cmd) */
#define IPMI_MSG_CMD_GET_CHAN_AUTH         0x38
#define IPMI_MSG_CMD_GET_SESSION_CHALLENGE 0x39
#define IPMI_MSG_CMD_ACTIVATE_SESSION      0x3A
#define IPMI_MSG_CMD_SET_PRIV_LEVEL        0x3B
#define IPMI_MSG_CMD_CLOSE_SESSION         0x3C
#define IPMI_MSG_CMD_SEND_MSG              0x34
#define IPMI_MSG_CMD_SENSOR_READ           0x2D
#define IPMI_MSG_CMD_SET_SENSOR_THRESH     0x26
#define IPMI_MSG_CMD_GET_SENSOR_THRESH     0x27
#define IPMI_MSG_CMD_GET_CHAS_STATUS       0x01
#define IPMI_MSG_CMD_CHAS_CTRL             0x02
#define IPMI_MSG_CMD_GET_FRU_INFO          0x10
#define IPMI_MSG_CMD_READ_FRU_DATA         0x11
#define IPMI_MSG_CMD_WRITE_FRU_DATA        0x12
#define IPMI_MSG_CMD_GET_SDRREP_INFO       0x20
#define IPMI_MSG_CMD_RESERVE_SDRREP        0x22
#define IPMI_MSG_CMD_GET_SDR               0x23
#define IPMI_MSG_CMD_GET_DEV_SDR_INFO      0x20
#define IPMI_MSG_CMD_GET_DEV_SDR           0x21
#define IPMI_MSG_CMD_RESERVE_DEV_SDRREP    0x22
#define IPMI_MSG_CMD_COLD_RESET            0x02
#define IPMI_MSG_CMD_SET_FRU_POLICY        0x0A
#define IPMI_MSG_CMD_GET_FRU_POLICY        0x0B
#define IPMI_MSG_CMD_SET_FRU_ACT           0x0C
#define IPMI_MSG_CMD_GET_DEVICE_ID         0x01

/* IPMI message request network function codes */
#define IPMI_MSG_NETFN_CHASSIS      0x00
#define IPMI_MSG_NETFN_SENSOR_EVENT 0x04
#define IPMI_MSG_NETFN_APP_REQUEST  0x06
#define IPMI_MSG_NETFN_STORAGE      0x0A
#define IPMI_MSG_NETFN_PICMG        0x2C

/* Read offset: bridged request, reply is in 1 message packet (NAT) */
#define IPMI_RPLY_BRIDGED_1REPLY_OFFSET 7
/* Read offset: bridged request, reply is in 2 message packet2 (VT) */
#define IPMI_RPLY_BRIDGED_2REPLY_OFFSET 22
/* Read offset: double bridged request, reply is in 1 message packet (NAT), not sure about this one */
#define IPMI_RPLY_2BRIDGED_1REPLY_OFFSET 0

#define FOOTER_LENGTH 1 /* checksum */

/* Response payload (aka IMSG2) message lengths
 * Includes completion code, but not final checksum)
 * (set to 0 for those that can vary)
 */
#define IPMI_RPLY_IMSG2_GET_CHAN_AUTH_LENGTH         9
#define IPMI_RPLY_IMSG2_GET_SESSION_CHALLENGE_LENGTH 21
#define IPMI_RPLY_IMSG2_ACTIVATE_SESSION_LENGTH      11
#define IPMI_RPLY_IMSG2_SET_PRIV_LEVEL_LENGTH        2
#define IPMI_RPLY_IMSG2_CLOSE_SESSION_LENGTH         1
#define IPMI_RPLY_IMSG2_SENSOR_READ_MAX_LENGTH       5 /* length varies */
#define IPMI_RPLY_IMSG2_GET_SENSOR_THRESH_LENGTH     8 /* need to determine this; using NAT length for now*/
#define IPMI_RPLY_IMSG2_GET_CHAS_STATUS_LENGTH       4 /* does NOT include optional byte */
#define IPMI_RPLY_IMSG2_CHAS_CTRL_LENGTH             1
#define IPMI_RPLY_IMSG2_GET_FRU_INFO_LENGTH          4 /* get fru message length is this + remaining data bytes */
#define IPMI_RPLY_IMSG2_READ_FRU_DATA_BASE_LENGTH    2
#define IPMI_RPLY_IMSG2_WRITE_FRU_DATA_LENGTH        2
#define IPMI_RPLY_IMSG2_GET_SDRREP_INFO_LENGTH       15
#define IPMI_RPLY_IMSG2_RESERVE_SDRREP_LENGTH        3
#define IPMI_RPLY_IMSG2_GET_SDR_BASE_LENGTH          3 /* get sdr message length is this + remaining data bytes */
#define IPMI_RPLY_IMSG2_GET_SDR_LENGTH               0 /* varies */
#define IPMI_RPLY_IMSG2_GET_DEV_SDR_INFO_LENGTH      7
#define IPMI_RPLY_IMSG2_GET_DEV_SDR_LENGTH           0 /* varies; base length is 3 */
#define IPMI_RPLY_IMSG2_COLD_RESET_LENGTH            1
#define IPMI_RPLY_IMSG2_SEND_MSG_LENGTH              4
#define IPMI_RPLY_IMSG2_CLOSE_SESSION_LENGTH         1
#define IPMI_RPLY_IMSG2_SENSOR_READ_MAX_LENGTH       5 /* length varies */
#define IPMI_RPLY_IMSG2_GET_SENSOR_THRESH_LENGTH     8 /* need to determine this; using NAT length for now*/
#define IPMI_RPLY_IMSG2_GET_CHAS_STATUS_LENGTH       4 /* does NOT include optional byte */
#define IPMI_RPLY_IMSG2_CHAS_CTRL_LENGTH             1
#define IPMI_RPLY_IMSG2_GET_FRU_INFO_LENGTH          4 /* get fru message length is this + remaining data bytes */
#define IPMI_RPLY_IMSG2_READ_FRU_DATA_BASE_LENGTH    2
#define IPMI_RPLY_IMSG2_GET_FRU_INV_INFO_LENGTH      4
#define IPMI_RPLY_IMSG2_WRITE_FRU_DATA_LENGTH        2
#define IPMI_RPLY_IMSG2_GET_DEVICE_ID_LENGTH         12
#define IPMI_RPLY_IMSG2_GET_SDRREP_INFO_LENGTH       15
#define IPMI_RPLY_IMSG2_RESERVE_SDRREP_LENGTH        3
#define IPMI_RPLY_IMSG2_GET_SDR_BASE_LENGTH          3 /* get sdr message length is this + remaining data bytes */
#define IPMI_RPLY_IMSG2_GET_SDR_LENGTH               0 /* varies */
#define IPMI_RPLY_IMSG2_GET_DEV_SDR_INFO_LENGTH      7
#define IPMI_RPLY_IMSG2_GET_DEV_SDR_LENGTH           0 /* varies; base length is 3 */
#define IPMI_RPLY_IMSG2_COLD_RESET_LENGTH            1

/* Get Authentication Capabilities message */
#define IPMI_RPLY_IMSG2_AUTH_CAP_CHAN_OFFSET 1 /* Get Authentication Capabilities response - channel number */
#define IPMI_RPLY_IMSG2_AUTH_CAP_AUTH_OFFSET                                                                           \
    2 /* Get Authentication Capabilities response - authentication type support */

/* Get Session Challenge message */
#define IPMI_RPLY_IMSG2_GET_SESS_TEMP_ID_OFFSET 1 /* Temporary session ID */
#define IPMI_RPLY_IMSG2_SESSION_ID_LENGTH                                                                              \
    4 /* For both permanent and temporary session ID (sess chall and act sess message replies) */
#define IPMI_RPLY_IMSG2_GET_SESS_CHALLENGE_STR_OFFSET 5 /* Challenge string */
#define IPMI_RPLY_CHALLENGE_STR_LENGTH                16

/* Activate Session message */
#define IPMI_RPLY_IMSG2_ACT_SESS_AUTH_TYPE_OFFSET     1 /* Authentication type */
#define IPMI_RPLY_IMSG2_ACT_SESS_ID_OFFSET            2 /* Session ID for remainder of session */
#define IPMI_RPLY_IMSG2_ACT_SESS_INIT_SEND_SEQ_OFFSET 6 /* Start sequence number for messages to MCH */
#define IPMI_RPLY_INIT_SEND_SEQ_LENGTH                4
#define IPMI_RPLY_IMSG2_ACT_SESS_MAX_PRIV_OFFSET      10 /* Maximum privilege level allowed for session */
#define IPMI_RPLY_HDR_SESS_SEQ_OFFSET                 1 /* From-MCH session sequence number (in reply to non-bridged message) */
#define IPMI_RPLY_SEQ_LENGTH                          4

/* Get Chassis Status message */
#define IPMI_RPLY_IMSG2_GET_CHAS_POWER_STATE_OFFSET 1 /* Current power state */
#define IPMI_RPLY_IMSG2_GET_CHAS_LAST_EVENT_OFFSET  2 /* Last power event */
#define IPMI_RPLY_IMSG2_GET_CHAS_MISC_STATE_OFFSET  3 /* Misc chassis state */
#define IPMI_GET_CHAS_POWER_STATE(x)                (x & ~0x80)
#define IPMI_GET_CHAS_LAST_EVENT(x)                 (x & ~0xE0)
#define IPMI_GET_CHAS_MISC_STATE(x)                 (x & ~0xF0)

/* Get Device ID message */
#define IPMI_RPLY_IMSG2_GET_DEVICE_ID_ID_OFFSET 1 /* Device ID, 0 = unspecified */
#define IPMI_RPLY_IMSG2_GET_DEVICE_ID_DEVICE_VERS_OFFSET                                                               \
    2 /* Device revision, [7] 1=provides device SDR;[6:4] reserved; [3:0] device revision, binary encoded */
#define IPMI_RPLY_IMSG2_GET_DEVICE_ID_FW_VERS1_OFFSET                                                                  \
    3 /* FW revision 1, [7] device avail (see ipmi spec for details), [6:0] major fw revision, binary encoded */
#define IPMI_RPLY_IMSG2_GET_DEVICE_ID_FW_VERS2_OFFSET 4 /* FW revision 2, minor firmware revision, BCD encoded */
#define IPMI_RPLY_IMSG2_GET_DEVICE_ID_IPMI_VERS_OFFSET                                                                 \
    5 /* IPMI version, BCD encoded, 0x00 reserved, [7:4] LS digit, [3:0] MS digit */
#define IPMI_RPLY_IMSG2_GET_DEVICE_ID_SUPPORT_OFFSET                                                                   \
    6 /* Additional support, see device capabilities in Management Controller record below*/
#define IPMI_RPLY_IMSG2_GET_DEVICE_ID_MANUF_ID_OFFSET  7 /* Returned from Get Device ID Command */
#define IPMI_RPLY_IPMI_VERS_OFFSET                     8 /* Returned from Get Device ID Command */
#define IPMI_RPLY_IMSG2_GET_DEVICE_ID_IPMI_VERS_OFFSET 5 /* Returned from Get Device ID Command */
#define IPMI_RPLY_MANUF_ID_LENGTH                      3
#define IPMI_VER_LSD(x)                                (x & 0xF0) >> 4 /* Extract IPMI version, least significant digit, eg 0x51 = version 1.5 */
#define IPMI_VER_MSD(x)                                x & 0x0F     /* Extract IPMI version, most significant digit */
#define IPMI_MANUF_ID(x)                               x & 0x0FFFFF /* Extract manufacturer ID */
#define IPMI_DEVICE_PROVIDES_DEVICE_SDR(x)             ((x & 1 << 7) >> 7)
//#define IPMI_DEVICE_PROVIDES_FRU_INV_INFO(x) ((x&1<<3)>>3)

/* SDR commands */
#define IPMI_SDRREP_PARM_GET_SDR     0
#define IPMI_SDRREP_PARM_GET_DEV_SDR 1

/* Get Device SDR Info message */
#define IPMI_GET_DEV_SDR_INFO_LUN_SENS_COUNT 0
#define IPMI_GET_DEV_SDR_INFO_SDR_COUNT      1

/* Get SDR Repository Info message */
#define IPMI_RPLY_IMSG2_SDRREP_VER_OFFSET         1  /* SDR Repository Info */
#define IPMI_RPLY_IMSG2_SDRREP_CNT_LSB_OFFSET     2  /* Number of records in repository, LSB */
#define IPMI_RPLY_IMSG2_SDRREP_CNT_MSB_OFFSET     3  /* Number of records in repository, MSB */
#define IPMI_RPLY_IMSG2_SDRREP_ADD_TS_OFFSET      6  /* Timestamp of most recent addition, LSB first */
#define IPMI_RPLY_IMSG2_SDRREP_DEL_TS_OFFSET      10 /* Timestamp of most recent deletion, LSB first */
#define IPMI_RPLY_IMSG2_DEV_SDR_INFO_CNT_OFFSET   1  /* Number of SDRs for device */
#define IPMI_RPLY_IMSG2_DEV_SDR_INFO_FLAGS_OFFSET 2  /* SDR flags */
#define IPMI_TS_LENGTH                            4  /* Number of bytes in Timestamp */

/* Get SDR message */
#define IPMI_RPLY_IMSG2_GET_SDR_NEXT_ID_LSB_OFFSET 1 /* ID of next sensor in repository, LSB */
#define IPMI_RPLY_IMSG2_GET_SDR_NEXT_ID_MSB_OFFSET 2 /* ID of next sensor in repository, MSB */
#define IPMI_RPLY_IMSG2_GET_SDR_DATA_OFFSET        3 /* Requested SDR data */
#define SDR_MAX_READ_SIZE                                                                                              \
    22 /*not all systems can deliver full SDR in one message, ipmitool uses 22 for VT (and maybe others */

/* Reserve SDR message */
#define IPMI_RPLY_IMSG2_GET_SDR_RES_LSB_OFFSET 1 /* Reservation ID returned by Reserve SDR Rep */
#define IPMI_RPLY_IMSG2_GET_SDR_RES_MSB_OFFSET 2 /* Reservation ID returned by Reserve SDR Rep */

/* Get FRU Inventory Area Info message */
#define IPMI_RPLY_IMSG2_FRU_AREA_SIZE_LSB_OFFSET 1 /* FRU inventory area size in bytes, LSB */
#define IPMI_RPLY_IMSG2_FRU_AREA_SIZE_MSB_OFFSET 2 /* FRU inventory area size in bytes, MSB */
#define IPMI_RPLY_IMSG2_FRU_AREA_ACCESS_OFFSET   3 /* Bit 0 indicates if device is accessed by bytes (0) or words (1) */
#define IPMI_RPLY_IMSG2_FRU_DATA_READ_OFFSET     2 /* FRU data */

/* Get Sensor Reading message */
#define IPMI_RPLY_IMSG2_SENSOR_READING_OFFSET 1 /* Sensor reading (1 byte) */
#define IPMI_RPLY_IMSG2_DISCRETE_SENSOR_READING_OFFSET                                                                 \
    3                                               /* Discrete multi-state sensor value in threshold/assertions byte */
#define IPMI_RPLY_IMSG2_SENSOR_ENABLE_BITS_OFFSET 2 /* Sensor enable bits: event msgs; scanning; reading/state */
#define IPMI_RPLY_IMSG2_SENSOR_THRESH_MASK_OFFSET 1 /* Mask of readable thresholds */
#define IPMI_RPLY_IMSG2_SENSOR_THRESH_LNC_OFFSET  2 /* Lower non-critical threshold */
#define IPMI_RPLY_IMSG2_SENSOR_THRESH_LC_OFFSET   3 /* Lower critical threshold */
#define IPMI_RPLY_IMSG2_SENSOR_THRESH_LNR_OFFSET  4 /* Lower non-recoverable threshold */
#define IPMI_RPLY_IMSG2_SENSOR_THRESH_UNC_OFFSET  5 /* Upper non-critical threshold */
#define IPMI_RPLY_IMSG2_SENSOR_THRESH_UC_OFFSET   6 /* Upper critical threshold */
#define IPMI_RPLY_IMSG2_SENSOR_THRESH_UNR_OFFSET  7 /* Upper non-recoverable threshold */

#define IPMI_DATA_LANG_CODE_ENGLISH1 0
#define IPMI_DATA_LANG_CODE_ENGLISH2 25

#define IPMI_DATA_TYPE(x)         (x & 0xC0) >> 6
#define IPMI_DATA_LENGTH(x)       x & 0x3F
#define IPMI_DATA_LANG_ENGLISH(x) (x == IPMI_DATA_LANG_CODE_ENGLISH1) || (x == IPMI_DATA_LANG_CODE_ENGLISH2)

/* Completion codes, IPMI v2.0 Table 5-2 */
#define IPMI_COMP_CODE_NORMAL                  0x00
#define IPMI_COMP_CODE_NODE_BUSY               0xC0
#define IPMI_COMP_CODE_INVALID_COMMAND         0xC1
#define IPMI_COMP_CODE_INVALID_COMMAND_FOR_LUN 0xC2
#define IPMI_COMP_CODE_TIMEOUT                 0xC3
#define IPMI_COMP_CODE_OUT_OF_SPACE            0xC4
#define IPMI_COMP_CODE_RESERVATION             0xC5
#define IPMI_COMP_CODE_REQUEST_TRUNCATED       0xC6
#define IPMI_COMP_CODE_REQUEST_LENGTH_INVALID  0xC7
#define IPMI_COMP_CODE_REQUEST_LENGTH_LIMIT    0xC8
#define IPMI_COMP_CODE_PARAMETER_RANGE         0xC9
#define IPMI_COMP_CODE_REQUESTED_BYTES         0xCA
#define IPMI_COMP_CODE_REQUESTED_DATA          0xCB
#define IPMI_COMP_CODE_INVALID_FIELD           0xCC
#define IPMI_COMP_CODE_COMMAND_ILLEGAL         0xCD
#define IPMI_COMP_CODE_COMMAND_RESPONSE        0xCE
#define IPMI_COMP_CODE_DUPLICATED_REQUEST      0xCF
#define IPMI_COMP_CODE_SDR_REP_UPDATE          0xD0
#define IPMI_COMP_CODE_DEVICE_FW_UPDATE        0xD1
#define IPMI_COMP_CODE_BMC_INIT                0xD2
#define IPMI_COMP_CODE_DESTINATION_UNAVAIL     0xD3
#define IPMI_COMP_CODE_PRIVILEGE               0xD4
#define IPMI_COMP_CODE_NOT_SUPPORTED           0xD5
#define IPMI_COMP_CODE_SUBFUNCTION_UNAVAIL     0xD6
#define IPMI_COMP_CODE_DEVICE_SPECIFIC_MIN     0x01
#define IPMI_COMP_CODE_DEVICE_SPECIFIC_MAX     0x7E
#define IPMI_COMP_CODE_COMMAND_SPECIFIC_MIN    0x80
#define IPMI_COMP_CODE_COMMAND_SPECIFIC_MAX    0xBE
#define IPMI_COMP_CODE_UNSPECIFIED             0xFF

/* Command-specific completion codes */
#define IPMI_COMP_CODE_GET_SESS_INVALID_USER 0x81
#define IPMI_COMP_CODE_GET_SESS_NULL_USER    0x82

/* Sensor definitions and constants */
#define IPMI_SENSOR_READING_DISABLED(x)          x & 1 << 5
#define IPMI_SENSOR_SCANNING_DISABLED(x)         ~x & 1 << 6
#define IPMI_SDR_SENSOR_THRESH_ACCESS(x)         (x & 0xC) >> 2
#define IPMI_SDR_SENSOR_THRESH_NONE              0
#define IPMI_SDR_SENSOR_THRESH_READABLE          1
#define IPMI_SDR_SENSOR_THRESH_READABLE_SETTABLE 2
#define IPMI_SDR_SENSOR_THRESH_FIXED_UNREADABLE  3
#define IPMI_SENSOR_THRESH_IS_READABLE(x)                                                                              \
    ((x == IPMI_SDR_SENSOR_THRESH_READABLE) || (x == IPMI_SDR_SENSOR_THRESH_READABLE_SETTABLE))
#define IPMI_SENSOR_THRESH_LNC_READABLE(x) x & 1 << 0
#define IPMI_SENSOR_THRESH_LC_READABLE(x)  x & 1 << 1
#define IPMI_SENSOR_THRESH_LNR_READABLE(x) x & 1 << 2
#define IPMI_SENSOR_THRESH_UNC_READABLE(x) x & 1 << 3
#define IPMI_SENSOR_THRESH_UC_READABLE(x)  x & 1 << 4
#define IPMI_SENSOR_THRESH_UNR_READABLE(x) x & 1 << 5

/* Sensor Data Record (SDR) types*/
#define SDR_TYPE_FULL_SENSOR      0x01
#define SDR_TYPE_COMPACT_SENSOR   0x02
#define SDR_TYPE_EVENT_ONLY       0x03
#define SDR_TYPE_ENTITY_ASSOC     0x08
#define SDR_TYPE_DEV_ENTITY_ASSOC 0x09
#define SDR_TYPE_GENERIC_DEV      0x10
#define SDR_TYPE_FRU_DEV          0x11
#define SDR_TYPE_MGMT_CTRL_DEV    0x12
#define SDR_TYPE_MGMT_CTRL_CONF   0x13
#define SDR_TYPE_BMC_MSG_CHAN     0x14
#define SDR_TYPE_OEM              0xC0

/* SDR data */
#define SDR_ID_LAST_SENSOR 0xFFFF
#define SDR_MAX_LENGTH     64
#define SDR_FRU_MAX_LENGTH 33

/* Common to all SDRs */
#define SDR_HEADER_LENGTH   5
#define SDR_ID_LSB_OFFSET   0 /* SDR Header */
#define SDR_ID_MSB_OFFSET   1
#define SDR_VER_OFFSET      2
#define SDR_REC_TYPE_OFFSET 3
#define SDR_LENGTH_OFFSET   4

/* SDR Full Sensor contents */
#define SDR_OWNER_OFFSET        5 /* SDR Key Fields */
#define SDR_LUN_OFFSET          6
#define SDR_NUMBER_OFFSET       7
#define SDR_ENTITY_ID_OFFSET    8 /* SDR Body */
#define SDR_ENTITY_INST_OFFSET  9
#define SDR_INIT_OFFSET         10
#define SDR_CAP_OFFSET          11
#define SDR_SENS_TYPE_OFFSET    12
#define SDR_READ_TYPE_OFFSET    13
#define SDR_ASSERT_MASK_OFFSET  14
#define SDR_ASSERT_MASK_SIZE    2
#define SDR_DEASSER_MAXK_OFFSET 16
#define SDR_DEASSERT_MASK_SIZE  2
#define SDR_READ_MASK_OFFSET    18
#define SDR_READ_MASK_SIZE      2
#define SDR_UNITS1_OFFSET       20
#define SDR_UNITS2_OFFSET       21
#define SDR_UNITS3_OFFSET       22
#define SDR_LINEAR_OFFSET       23
#define SDR_M_OFFSET            24 /* LS 8 bits, 2's complement signed 10-bit 'M' value */
#define SDR_M_TOL_OFFSET        25 /* [7:6] MS 2 bits, [5:0] Tolerance 6 bits, unsigned (in +/- 1/2 raw counts) */
#define SDR_B_OFFSET            26 /* LS 8 bits, 2's complement signed 10-bit 'B' value */
#define SDR_B_ACC_OFFSET                                                                                               \
    27 /* [7:6] MS 2 bits unsigned, 10-bit acc in 1/100 % scaled up by: [5:0] Accuracy LS 6 bits                       \
        */
#define SDR_ACC_OFFSET        28 /* [7:4] Accuracy MS 4 bits, [3:2] Accuracy exp: 2 bits unsigned, [1:0] sensor direction */
#define SDR_EXP_OFFSET        29
#define SDR_ANLG_CHAR_OFFSET  30
#define SDR_NOMINAL_OFFSET    31
#define SDR_NORM_MAX_OFFSET   32
#define SDR_NORM_MIN_OFFSET   33
#define SDR_STR_LENGTH_OFFSET 47
#define SDR_STR_OFFSET        48
/*...more...*/

/* SDR Compact Sensor */
#define SDR_COMPACT_STR_LENGTH_OFFSET 31
#define SDR_COMPACT_STR_OFFSET        32

#define SENSOR_LINEAR(x)      (x & 0x7F)
#define SENSOR_CONV_M_B(x, y) (x + ((y & 0xC0) << 2))
#define TWOS_COMP_SIGNED_NBIT(x, n)                                                                                    \
    (x > (pow(2, n) / 2 - 1)) ? x - pow(2, n) : x /* Extract n-bit signed number stored as two's complement */
#define ONES_COMP_SIGNED_NBIT(x, n)                                                                                    \
    (x > (pow(2, n) / 2 - 1)) ? x - pow(2, n) + 1 : x /* Extract n-bit signed number stored as one's complement */
#define SENSOR_CONV_REXP(x)      ((x & 0xF0) >> 4)
#define SENSOR_CONV_BEXP(x)      (x & 0xF)
#define SENSOR_NOMINAL_GIVEN(x)  (x & (1 << 0))
#define SENSOR_NORM_MAX_GIVEN(x) (x & (1 << 1))
#define SENSOR_NORM_MIN_GIVEN(x) (x & (1 << 2))
#define SENSOR_NUMERIC_FORMAT(x) ((x & 0xC0) >> 6)
#define SDR_ENTITY_LOGICAL(x)    ((x & (1 << 7)) >> 7) /* 1 if entity is logical entity; 0 if is physical entity */

#define SENSOR_UNITS_UNSPEC   0
#define SENSOR_UNITS_DEGC     1
#define SENSOR_UNITS_DEGF     2
#define SENSOR_UNITS_DEGK     3
#define SENSOR_UNITS_VOLTS    4
#define SENSOR_UNITS_AMPS     5
#define SENSOR_UNITS_WATTS    6
#define SENSOR_UNITS_JOULES   7
#define SENSOR_UNITS_COULOMBS 8
#define SENSOR_UNITS_RPM      18
/* more... */

#define SENSOR_CONV_LINEAR    0
#define SENSOR_CONV_LN        1
#define SENSOR_CONV_LOG10     2
#define SENSOR_CONV_LOG2      3
#define SENSOR_CONV_E         4
#define SENSOR_CONV_EXP10     5
#define SENSOR_CONV_EXP2      6
#define SENSOR_CONV_1_X       7
#define SENSOR_CONV_SQR       8
#define SENSOR_CONV_CUBE      9
#define SENSOR_CONV_SQRT      10
#define SENSOR_CONV_CUBE_NEG1 11

#define SENSOR_NUMERIC_FORMAT_UNSIGNED   0
#define SENSOR_NUMERIC_FORMAT_ONES_COMP  1
#define SENSOR_NUMERIC_FORMAT_TWOS_COMP  2
#define SENSOR_NUMERIC_FORMAT_NONNUMERIC 3

/* SDR FRU Device contents */
#define SDR_FRU_ADDR_OFFSET        5 /* SDR Body */
#define SDR_FRU_ID_OFFSET          6
#define SDR_FRU_LUN_OFFSET         7
#define SDR_FRU_CHAN_OFFSET        8
#define SDR_FRU_TYPE_OFFSET        10
#define SDR_FRU_TYPE_MOD_OFFSET    11
#define SDR_FRU_ENTITY_ID_OFFSET   12
#define SDR_FRU_ENTITY_INST_OFFSET 13
#define SDR_FRU_STR_LENGTH_OFFSET  15
#define SDR_FRU_STR_OFFSET         16

/* SDR Management Device contents */
#define SDR_MGMT_ADDR_OFFSET        5 /* SDR Body */
#define SDR_MGMT_CHAN_OFFSET        6
#define SDR_MGMT_PWR_OFFSET         7
#define SDR_MGMT_CAP_OFFSET         8
#define SDR_MGMT_ENTITY_ID_OFFSET   12
#define SDR_MGMT_ENTITY_INST_OFFSET 13
#define SDR_MGMT_STR_LENGTH_OFFSET  14
#define SDR_MGMT_STR_OFFSET         15
/*...Device Capabilities */
#define IPMI_DEVICE_SLAVE_ADDR(x) ((x & 0xFE) >> 1)   /* 7-bit I2C slave address of device on channel */
#define IPMI_CHAN_NUMBER(x)       (x & 0xF)           /* Chan number mgmt controller is on; 0 for BMC */
#define IPMI_DEV_CAP_CHASSIS(x)   ((x & 1 << 7) >> 7) /* Device functions as chassis device, per ICMB spec) */
#define IPMI_DEV_CAP_BRIDGE(x)    ((x & 1 << 6) >> 6) /* Responds to Bridge NetFn commands */
#define IPMI_DEV_CAP_IPMB_EVG(x)  ((x & 1 << 5) >> 5) /* Generates event messages on IPMB */
#define IPMI_DEV_CAP_IPMB_EVR(x)  ((x & 1 << 4) >> 4) /* Accepts event messages on IPMB */
#define IPMI_DEV_CAP_FRU_INV(x)   ((x & 1 << 3) >> 3) /* Accepts FRU commands to FRU Device #0 at LUN 00b */
#define IPMI_DEV_CAP_SEL(x)       ((x & 1 << 2) >> 2) /* Provides interface to SEL */
#define IPMI_DEV_CAP_SDRREP(x)                                                                                         \
    ((x & 1 << 1) >> 1) /* For BMC, indicates provides interface to 1b = SDR Rep; else accepts Device SDR commands */
#define IPMI_DEV_CAP_SENSOR(x) (x & 1) /* Accepts sensor commands (see Table 37-11 IPMB/I2C Device Type Codes) */

#define SENSOR_OWNER_ID(x)      (x & 0xFE)
#define SENSOR_OWNER_ID_TYPE(x) (x & 0x1)
#define SENSOR_OWNER_CHAN(x)    (x & 0xF0)
#define SENSOR_OWNER_LUN(x)     (x & 0x3)

#define DEV_SENSOR_DYNAMIC(x) (x & 0x80)
#define DEV_SENSOR_LUN0(x)    (x & 0x1)
#define DEV_SENSOR_LUN1(x)    (x & 0x2)
#define DEV_SENSOR_LUN2(x)    (x & 0x4)
#define DEV_SENSOR_LUN3(x)    (x & 0x8)

/* SDR Entity Association and Device-relative Entity Association common contents */
#define SDR_CNTNR_ENTITY_ID_OFFSET   5 /* SDR Body */
#define SDR_CNTNR_ENTITY_INST_OFFSET 6

/* SDR Entity Assocation contents */
#define SDR_ENTITY_ASSOC_FLAGS_OFFSET 7
#define SDR_CNTND_ENTITY_ID1_OFFSET   8
#define SDR_CNTND_ENTITY_INST1_OFFSET 9
#define SDR_CNTND_ENTITY_ID2_OFFSET   10
#define SDR_CNTND_ENTITY_INST2_OFFSET 11
#define SDR_CNTND_ENTITY_ID3_OFFSET   12
#define SDR_CNTND_ENTITY_INST3_OFFSET 13
#define SDR_CNTND_ENTITY_ID4_OFFSET   14
#define SDR_CNTND_ENTITY_INST4_OFFSET 15

/* SDR Device-relative Entity Assocation contents */
#define SDR_DEV_CNTNR_ADDR_OFFSET         7
#define SDR_DEV_CNTNR_CHAN_OFFSET         8
#define SDR_DEV_ENTITY_ASSOC_FLAGS_OFFSET 9
#define SDR_DEV_CNTND_ADDR1_OFFSET        10
#define SDR_DEV_CNTND_CHAN1_OFFSET        11
#define SDR_DEV_CNTND_ENTITY_ID1_OFFSET   12
#define SDR_DEV_CNTND_ENTITY_INST1_OFFSET 13
#define SDR_DEV_CNTND_ADDR2_OFFSET        14
#define SDR_DEV_CNTND_CHAN2_OFFSET        15
#define SDR_DEV_CNTND_ENTITY_ID2_OFFSET   16
#define SDR_DEV_CNTND_ENTITY_INST2_OFFSET 17
#define SDR_DEV_CNTND_ADDR3_OFFSET        18
#define SDR_DEV_CNTND_CHAN3_OFFSET        19
#define SDR_DEV_CNTND_ENTITY_ID3_OFFSET   20
#define SDR_DEV_CNTND_ENTITY_INST3_OFFSET 21
#define SDR_DEV_CNTND_ADDR4_OFFSET        22
#define SDR_DEV_CNTND_CHAN4_OFFSET        23
#define SDR_DEV_CNTND_ENTITY_ID4_OFFSET   24
#define SDR_DEV_CNTND_ENTITY_INST4_OFFSET 25

/* Information stored in entity assocation records */
#define ENTITY_ASSOC_FLAGS_RANGE(x)                                                                                    \
    ((x & (1 << 7)) >> 7) /* If 1 contained entities provided as range; if 0 provided as list */
#define ENTITY_ASSOC_FLAGS_LINK(x) ((x & (1 << 6)) >> 6) /* If 1 linked assocation records exist */
#define ENTITY_ASSOC_FLAGS_PRESSENS(x)                                                                                 \
    ((x & (1 << 5)) >> 5) /* If 1 presence sensor should always be accessible;                                         \
 if 0 absense of sensor indicates container/contained entitites are absent */

/* Sensor types */
#define MAX_SENSOR_TYPE           0xFF
#define SENSOR_TYPE_TEMP          0x01
#define SENSOR_TYPE_VOLTAGE       0x02
#define SENSOR_TYPE_CURRENT       0x03
#define SENSOR_TYPE_FAN           0x04
#define SENSOR_TYPE_PHYS_SECURITY 0x05
#define SENSOR_TYPE_PLAT_SECURITY 0x06
#define SENSOR_TYPE_PROCESSOR     0x07
#define SENSOR_TYPE_POWER_SUPPPLY 0x08
#define SENSOR_TYPE_POWER_UNIT    0x09
#define SENSOR_TYPE_COOLING_DEV   0x0A
#define SENSOR_TYPE_OTHER         0x0B
#define SENSOR_TYPE_MEMORY        0x0C
#define SENSOR_TYPE_DRIVE_SLOT    0x0D
#define SENSOR_TYPE_POST_MEMORY   0x0E
#define SENSOR_TYPE_SYS_FW        0x0F
#define SENSOR_TYPE_FRU_STATE     0x2C
/* more... */

/* Entity instance ranges:
 * system-relative: the combination of entity ID and instance
 *                  must be unique within the system
 * device-relative: that combination must be unique relative
 *                  to the management controller providing access
 *                  to the entity
 */
#define ENTITY_INST_SYS_REL_MIN 0x00
#define ENTITY_INST_SYS_REL_MAX 0x5F
#define ENTITY_INST_DEV_REL_MIN 0x60
#define ENTITY_INST_DEV_REL_MAX 0x7F

/* Entity IDs */
#define ENTITY_ID_UNSPEC               0x00
#define ENTITY_ID_OTHER                0x01
#define ENTITY_ID_UNKNOWN              0x02
#define ENTITY_ID_PROCESSOR            0x03
#define ENTITY_ID_DISK_BAY_1           0x04
#define ENTITY_ID_PERIPH_BAY_1         0x05
#define ENTITY_ID_SYS_MGMT_MOD         0x06
#define ENTITY_ID_SYS_BOARD            0x07
#define ENTITY_ID_MEMORY_MOD           0x08
#define ENTITY_ID_PROCESSOR_MOD        0x09
#define ENTITY_ID_POWER_SUPPLY         0x0A
#define ENTITY_ID_ADD_IN_CARD          0x0B
#define ENTITY_ID_FRONT_BOARD          0x0C
#define ENTITY_ID_BACK_BOARD           0x0D
#define ENTITY_ID_POWER_BOARD          0x0E
#define ENTITY_ID_DRIVE_BCKPLN         0x0F
#define ENTITY_ID_EXPANSION_BOARD      0x10
#define ENTITY_ID_SYS_BOARD_OTHER      0x11
#define ENTITY_ID_PROCESSOR_BOARD      0x12
#define ENTITY_ID_POWER_UNIT           0x13
#define ENTITY_ID_POWER_MODULE         0x14
#define ENTITY_ID_POWER_MGMT           0x15
#define ENTITY_ID_CHASSIS_BACK_BOARD   0x16
#define ENTITY_ID_SYS_CHASSIS          0x17
#define ENTITY_ID_SUB_CHASSIS          0x18
#define ENTITY_ID_CHASSIS_OTHER        0x19
#define ENTITY_ID_DISK_BAY_2           0x1A
#define ENTITY_ID_PERIPH_BAY_2         0x1B
#define ENTITY_ID_DEVICE_BAY           0x1C
#define ENTITY_ID_FAN_COOLING          0x1D
#define ENTITY_ID_COOLING_UNIT         0x1E
#define ENTITY_ID_CABLE                0x1F
#define ENTITY_ID_MEMORY_DEV           0x20
#define ENTITY_ID_SYS_MGMT_SW          0x21
#define ENTITY_ID_BIOS                 0x22
#define ENTITY_ID_OS                   0x23
#define ENTITY_ID_SYS_BUS              0x24
#define ENTITY_ID_GROUP                0x25
#define ENTITY_ID_REMOTE_COMM_DEV      0x26
#define ENTITY_ID_EXT_ENVIRONMENT      0x27
#define ENTITY_ID_BATTERY              0x28
#define ENTITY_ID_PROCESSING_BLADE     0x29
#define ENTITY_ID_CONN_SWITCH          0x2A
#define ENTITY_ID_PROCESSOR_MEMORY_MOD 0x2B
#define ENTITY_ID_IO_MOD               0x2C
#define ENTITY_ID_PROCESSOR_IO_MOD     0x2D
#define ENTITY_ID_MGMT_CTRL_FW         0x2E
#define ENTITY_ID_IPMI_CHANNEL         0x2F
#define ENTITY_ID_PCI_BUS              0x30
#define ENTITY_ID_PCI_EXPRESS_BUS      0x31
#define ENTITY_ID_SCSI_BUS             0x32
#define ENTITY_ID_SATA_SAS_BUS         0x33
#define ENTITY_ID_PROCESSOR_FRONT_BUS  0x34

/* FRU data storage, specified in IPMI Platform Management FRU
 * Information Storage Definition V1.0, Document Revision 1.1
 */

/* Number of bytes stored in 6 bits, so 2^6 = 64 max bytes */
#define MAX_FRU_FIELD_RAW_LENGTH 64
/* Based on how FRU field data is stored, actual length may be up to 2 x raw length */
#define MAX_FRU_FIELD_LENGTH (MAX_FRU_FIELD_RAW_LENGTH * 2)

#define FRU_FIELD_LENGTH_TYPE_RAW       0
#define FRU_FIELD_LENGTH_TYPE_CONVERTED 1

typedef struct FruFieldRec_ {
    uint8_t type;                            /* Type of data */
    uint8_t rlength;                         /* Length of raw data (bytes) */
    uint8_t rdata[MAX_FRU_FIELD_RAW_LENGTH]; /* Raw data */
    uint8_t length;                          /* Length of data after converted to ASCII (bytes) */
    uint8_t data[MAX_FRU_FIELD_LENGTH];      /* Data */
} FruFieldRec, *FruField;

/* Chassis area doesn't have a language field
 * Serial number always encoded in English
 */
typedef struct FruChassisRec_ {
    uint8_t     offset; /* Offset into FRU storage */
    uint8_t     ver;    /* Format version (0x01) */
    uint8_t     length; /* Length of FRU area */
    uint8_t     type;   /* Chassis type */
    FruFieldRec part;   /* Part number */
    FruFieldRec sn;     /* Serial number */
                        /* optional custom fields */
} FruChassisRec, *FruChassis;

typedef struct FruBoardRec_ {
    uint8_t     offset; /* Offset into FRU storage */
    uint8_t     ver;    /* Format version (0x01) */
    uint8_t     length; /* Length of FRU area */
    uint8_t     lang;   /* Language code */
    uint8_t     date;   /* Manufacturer date/time (minutes from 00:00 1/1/96) */
    FruFieldRec manuf;  /* Manufacturer */
    FruFieldRec prod;   /* Product name */
    FruFieldRec sn;     /* Serial number */
    FruFieldRec part;   /* Part number */
                        /* more */
} FruBoardRec, *FruBoard;

typedef struct FruProdRec_ {
    uint8_t     offset;  /* Offset into FRU storage */
    uint8_t     ver;     /* Format version (0x01) */
    uint8_t     length;  /* Length of FRU area */
    uint8_t     lang;    /* Language code */
    FruFieldRec manuf;   /* Manufacturer */
    FruFieldRec prod;    /* Product name */
    FruFieldRec part;    /* Part number */
    FruFieldRec version; /* Version number */
    FruFieldRec sn;      /* Serial number */
                         /* more */
} FruProdRec, *FruProd;

/* FRU data */
#define MSG_FRU_DATA_READ_SIZE 0x10

/* Common header (mandatory) */
#define FRU_DATA_COMMON_HEADER_OFFSET               0
#define FRU_DATA_COMMON_HEADER_INTERNAL_AREA_OFFSET 1
#define FRU_DATA_COMMON_HEADER_CHASSIS_AREA_OFFSET  2
#define FRU_DATA_COMMON_HEADER_BOARD_AREA_OFFSET    3
#define FRU_DATA_COMMON_HEADER_PROD_AREA_OFFSET     4
#define FRU_DATA_COMMON_HEADER_MULTIREC_AREA_OFFSET 5
#define FRU_DATA_COMMON_HEADER_PAD                  6
#define FRU_DATA_COMMON_HEADER_CS                   7

/* Internal use area (we don't use it) */
#define FRU_DATA_INTERNAL_AREA_VERSION_OFFSET 0
#define FRU_DATA_INTERNAL_DATA_OFFSET         1

/* Chassis area (optional); language always english */
#define FRU_DATA_CHASSIS_AREA_VERSION_OFFSET     0
#define FRU_DATA_CHASSIS_AREA_LENGTH_OFFSET      1
#define FRU_DATA_CHASSIS_TYPE_OFFSET             2
#define FRU_DATA_CHASSIS_AREA_PART_LENGTH_OFFSET 3
#define FRU_DATA_CHASSIS_AREA_PART_OFFSET        4
/* Offsets vary:
 * serial number length
 * serial number
 * custom fields
 * byte to indicate if more fields
 * remaining space ( 0x00 )
 * checksum
 */

/* Board area (optional) */
#define FRU_DATA_BOARD_AREA_VERSION_OFFSET      0
#define FRU_DATA_BOARD_AREA_LENGTH_OFFSET       1
#define FRU_DATA_BOARD_AREA_LANG_OFFSET         2
#define FRU_DATA_BOARD_AREA_MANUF_TIME_OFFSET   3
#define FRU_DATA_BOARD_AREA_MANUF_LENGTH_OFFSET 6
#define FRU_DATA_BOARD_AREA_MANUF_OFFSET        7
/* Offsets vary:
 * product name length
 * product name
 * serial number length
 * serial number
 * fru file id length
 * fru file id
 * custom fields
 * byte to indicate if more fields
 * remaining space ( 0x00 )
 * checksum
 */

/* Product area (optional) */
#define FRU_DATA_PROD_AREA_VERSION_OFFSET      0
#define FRU_DATA_PROD_AREA_LENGTH_OFFSET       1
#define FRU_DATA_PROD_AREA_LANG_OFFSET         2
#define FRU_DATA_PROD_AREA_MANUF_LENGTH_OFFSET 3
#define FRU_DATA_PROD_AREA_MANUF_OFFSET        4
/* Offsets vary:
 * product name length
 * product name
 * product part/model number length
 * product part/model number
 * product version length
 * product version
 * product serial number length
 * product serial number
 * asset tag length
 * asset tag
 * fru file id length
 * fru file id
 * custom fields
 * byte to indicate if more fields
 * remaining space ( 0x00 )
 * checksum
 */

/* MultiRecord area (optional) */
#define FRU_DATA_MULTIREC_AREA_HEADER_RECORD_TYPE_OFFSET   0
#define FRU_DATA_MULTIREC_AREA_HEADER_END_OF_LIST_OFFSET   1
#define FRU_DATA_MULTIREC_AREA_HEADER_RECORD_LENGTH_OFFSET 2
#define FRU_DATA_MULTIREC_AREA_HEADER_RECORD_CS_OFFSET     3
#define FRU_DATA_MULTIREC_AREA_HEADER_HEADER_CS_OFFSET     4
#define FRU_DATA_MULTIREC_END_OF_LIST(x)                   (x & 0x80) >> 7

/* FRU info field data types */
#define FRU_DATA_TYPE_BINARY    0 /* binary or unspecified */
#define FRU_DATA_TYPE_BCDPLUS   1
#define FRU_DATA_TYPE_6BITASCII 2
#define FRU_DATA_TYPE_LANG                                                                                             \
    3 /* 8-bit ASCII + Latin 1 if language is English;                                                                 \
       * otherwise 2-byte UNICODE with LS byte first.                                                                  \
       */
/* Chassis types */
#define FRU_DATA_CHASSIS_TYPE_OTHER          0x01
#define FRU_DATA_CHASSIS_TYPE_UNKNOWN        0x02
#define FRU_DATA_CHASSIS_TYPE_DESKTOP        0x03
#define FRU_DATA_CHASSIS_TYPE_LPDESKTOP      0x04
#define FRU_DATA_CHASSIS_TYPE_PIZZABOX       0x05
#define FRU_DATA_CHASSIS_TYPE_MINITOWER      0x06
#define FRU_DATA_CHASSIS_TYPE_TOWER          0x07
#define FRU_DATA_CHASSIS_TYPE_PORTABLE       0x08
#define FRU_DATA_CHASSIS_TYPE_LAPTOP         0x09
#define FRU_DATA_CHASSIS_TYPE_NOTEBOOK       0x0A
#define FRU_DATA_CHASSIS_TYPE_HANDHELD       0x0B
#define FRU_DATA_CHASSIS_TYPE_DOCKINGSTATION 0x0C
#define FRU_DATA_CHASSIS_TYPE_ALLINONE       0x0D
#define FRU_DATA_CHASSIS_TYPE_SUBNOTEBOOK    0x0E
#define FRU_DATA_CHASSIS_TYPE_SPACESAVING    0x0F
#define FRU_DATA_CHASSIS_TYPE_LUNCHBOX       0x10
#define FRU_DATA_CHASSIS_TYPE_MAINSERVER     0x11
#define FRU_DATA_CHASSIS_TYPE_EXPANSION      0x12
#define FRU_DATA_CHASSIS_TYPE_SUBCHASSIS     0x13
#define FRU_DATA_CHASSIS_TYPE_BUSEXPANSION   0x14
#define FRU_DATA_CHASSIS_TYPE_PERIPHERAL     0x15
#define FRU_DATA_CHASSIS_TYPE_RAID           0x16
#define FRU_DATA_CHASSIS_TYPE_RACKMOUNT      0x17
/* More to come... */

#endif
