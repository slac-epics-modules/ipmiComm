#ifndef IPMI_DEF_H
#define IPMI_DEF_H

#include <math.h> /* pow */

#define MSG_MAX_LENGTH  200

/* RMCP message header */ 
#define RMCP_MSG_VER        0x06  /* RMCP message version */
#define RMCP_MSG_SEQ        0xFF  /* RMCP message header; 0xFF for no ack */
#define RMCP_MSG_CLASS_ASF  0x06  /* Message class, ASF  */
#define RMCP_MSG_CLASS_IPMI 0x07  /* Message class, IPMI */
#define RMCP_MSG_AUTH_NONE  0x00  /* Message authentication type, 0 for none */
#define RMCP_MSG_HEADER_LENGTH 4
#define RMCP_MSG_CLASS_OFFSET  3  /* Message class */

/* ASF message */
#define ASF_MSG_OFFSET         4

/* IPMI message header */
#define IPMI_MSG_HEADER_SEQ_INITIAL          1

/* IPMI outgoing message */
#define IPMI_MSG1_LENGTH                     3
#define IPMI_MSG2_MAX_LENGTH                 50   /*update*/
#define IPMI_MSG_HEADER_LENGTH               10
#define IPMI_MSG_HEADER_OFFSET               4    /* IPMI message header offset */
#define IPMI_MSG_ADDR_BMC                    0x20 /* UTC001 MCH BMC address */
#define IPMI_MSG_ADDR_CM                     0x82 /* UTC001 MCH carrier manager responder address */
#define IPMI_MSG_ADDR_SW                     0x81 /* Our address (can be 0x81, 0x83, 0x85, 0x87, 0x89) */
#define IPMI_MSG_PRIV_LEVEL_NONE             0x00 /* Privilege level: none  */
#define IPMI_MSG_PRIV_LEVEL_CB               0x01 /* Privilege level: callback  */
#define IPMI_MSG_PRIV_LEVEL_USER             0x02 /* Privilege level: user  */
#define IPMI_MSG_PRIV_LEVEL_OPER             0x03 /* Privilege level: operator  */
#define IPMI_MSG_PRIV_LEVEL_ADMIN            0x04 /* Privilege level: administrator  */
#define IPMI_MSG_PRIV_LEVEL_OEM              0x05 /* Privilege level: OEM proprietary  */
#define IPMI_MSG_AUTH_TYPE_NONE              0
#define IPMI_MSG_CURR_CHAN                   0x0E /* Channel number currently in use */
#define IPMI_MSG_HDR_SEQ_OFFSET              1    /* Session sequence number */
#define IPMI_MSG_HDR_SEQ_LENGTH              4    
#define IPMI_MSG_HDR_ID_OFFSET               5    /* Session ID */
#define IPMI_MSG_HDR_ID_LENGTH               4    
#define IPMI_MSG_HDR_NBYTES_OFFSET           9    /* Number of bytes in IPMI message (includes bridged message) */
#define IPMI_MSG1_RSADDR_OFFSET              0
#define IPMI_MSG1_NETFNLUN_OFFSET            1
#define IPMI_MSG2_RSADDR_OFFSET              0    /* Requester's address (used in bridged IPMI msg 2) */
#define IPMI_MSG2_ID_OFFSET                  3    /* Session ID */
#define IPMI_MSG2_ID_LENGTH                  4    
#define IPMI_MSG2_STR_OFFSET                 5    /* Challenge string */
#define IPMI_MSG2_STR_LENGTH                 16   
#define IPMI_MSG2_SEQLUN_OFFSET              1    /* MS 6 bits: IPMI sequence, LS 2 bits: LUN */
#define IPMI_MSG2_CMD_OFFSET                 2    /* Command code */
#define IPMI_MSG2_CHAN_OFFSET                3    /* Channel to send message over (0 for IPMB) */
#define IPMI_MSG2_SENSOR_OFFSET              3    /* Sensor number */
#define IPMI_MSG2_CHAN_OFFSET                3    /* For channel number in bridged request */
#define IPMI_MSG2_PRIV_LEVEL_OFFSET          3
#define IPMI_MSG2_READ_FRU_ID_OFFSET         3    
#define IPMI_MSG2_READ_FRU_LSB_OFFSET        4    /* FRU Inventory Offset to read, LS Byte */
#define IPMI_MSG2_READ_FRU_MSB_OFFSET        5    /* FRU Inventory Offset to read, MS Byte */
#define IPMI_MSG2_READ_FRU_CNT_OFFSET        6    /* Count to read in bytes, 1-based */
#define IPMI_MSG2_GET_SDR_RES_LSB_OFFSET     3    /* SDR Reservation ID, LS Byte */
#define IPMI_MSG2_GET_SDR_RES_MSB_OFFSET     4    /* SDR Reservation ID, LS Byte */
#define IPMI_MSG2_GET_SDR_ID_LSB_OFFSET      5    /* SDR Record ID, LS Byte */
#define IPMI_MSG2_GET_SDR_ID_MSB_OFFSET      6    /* SDR Record ID, LS Byte */
#define IPMI_MSG2_GET_SDR_OFFSET_OFFSET      7    /* Offset into record to start read */
#define IPMI_MSG2_GET_SDR_CNT_OFFSET         8    /* Count to read in bytes (0xFF means entire record) */
#define IPMI_MSG2_GET_DEV_SDR_INFO_OP_OFFSET 3    /* 1 get SDR count, 0 get sensor count */
#define IPMI_MSG2_SET_FRU_ACT_DEACTIVATE     0    /* Command value in Set FRU Activate command */
#define IPMI_MSG2_SET_FRU_ACT_ACTIVATE       1
#define IPMI_MSG2_SET_FRU_ACT_FRU_OFFSET     4    /* FRU ID, used in many commands */
#define IPMI_MSG2_SET_FRU_ACT_CMD_OFFSET     5
#define IPMI_MSG2_SET_FRU_POLICY_MASK_OFFSET 5
#define IPMI_MSG2_SET_FRU_POLICY_BITS_OFFSET 6
#define IPMI_MSG2_SET_FAN_LEVEL_LEVEL_OFFSET 5    /* New fan level to set */
#define IPMI_MSG2_GET_POWER_LEVEL_TYPE_OFFSET 5

/* IPMI message command codes (cmd) */
#define IPMI_MSG_CMD_GET_CHAN_AUTH           0x38
#define IPMI_MSG_CMD_GET_SESSION_CHALLENGE   0x39
#define IPMI_MSG_CMD_ACTIVATE_SESSION        0x3A
#define IPMI_MSG_CMD_SET_PRIV_LEVEL          0x3B
#define IPMI_MSG_CMD_CLOSE_SESSION           0x3C
#define IPMI_MSG_CMD_SEND_MSG                0x34
#define IPMI_MSG_CMD_SENSOR_READ             0x2D
#define IPMI_MSG_CMD_GET_CHAS_STATUS         0x01
#define IPMI_MSG_CMD_CHAS_CTRL               0x02
#define IPMI_MSG_CMD_GET_FRU_INFO            0x10
#define IPMI_MSG_CMD_READ_FRU_DATA           0x11
#define IPMI_MSG_CMD_WRITE_FRU_DATA          0x12
#define IPMI_MSG_CMD_GET_SDRREP_INFO         0x20
#define IPMI_MSG_CMD_RESERVE_SDRREP          0x22
#define IPMI_MSG_CMD_GET_SDR                 0x23
#define IPMI_MSG_CMD_GET_DEV_SDR_INFO        0x20
#define IPMI_MSG_CMD_GET_DEV_SDR             0x21
#define IPMI_MSG_CMD_COLD_RESET              0x02
#define IPMI_MSG_CMD_SET_FRU_POLICY          0x0A
#define IPMI_MSG_CMD_SET_FRU_ACT             0x0C
#define IPMI_MSG_CMD_GET_DEVICE_ID           0x01
#define IPMI_MSG_CMD_GET_POWER_LEVEL         0x12
#define IPMI_MSG_CMD_GET_FAN_PROP            0x14
#define IPMI_MSG_CMD_GET_FAN_LEVEL           0x16
#define IPMI_MSG_CMD_SET_FAN_LEVEL           0x15

/* IPMI message request network function codes */
#define IPMI_MSG_NETFN_CHASSIS                0x00
#define IPMI_MSG_NETFN_SENSOR_EVENT           0x04
#define IPMI_MSG_NETFN_APP_REQUEST            0x06
#define IPMI_MSG_NETFN_STORAGE                0x0A
#define IPMI_MSG_NETFN_PICMG                  0x2C

/* Response message lengths (0s for those we haven't figured out yet) */
#define IPMI_RPLY_PONG_LENGTH                    28
#define IPMI_RPLY_GET_CHAN_AUTH_LENGTH           30
#define IPMI_RPLY_GET_SESSION_CHALLENGE_LENGTH   42
#define IPMI_RPLY_ACTIVATE_SESSION_LENGTH        32
#define IPMI_RPLY_SET_PRIV_LEVEL_LENGTH          23
#define IPMI_RPLY_CLOSE_SESSION_LENGTH           22
#define IPMI_RPLY_SENSOR_READ_LENGTH             0  /* varies */
#define IPMI_RPLY_GET_CHAS_STATUS_LENGTH         47 /* does NOT include optional byte */
#define IPMI_RPLY_CHAS_CTRL_LENGTH               44
#define IPMI_RPLY_GET_FRU_INFO_LENGTH            47
#define IPMI_RPLY_READ_FRU_DATA_BASE_LENGTH      45
#define IPMI_RPLY_WRITE_FRU_DATA_LENGTH          45
#define IPMI_RPLY_GET_SDRREP_INFO_LENGTH         58
#define IPMI_RPLY_RESERVE_SDRREP_LENGTH          46
#define IPMI_RPLY_GET_SDR_LENGTH                 0  /* varies */
#define IPMI_RPLY_GET_DEV_SDR_INFO_LENGTH        50
#define IPMI_RPLY_GET_DEV_SDR_LENGTH             0  /* varies */
#define IPMI_RPLY_COLD_RESET_LENGTH              44
#define IPMI_RPLY_SET_FRU_POLICY_LENGTH          44
#define IPMI_RPLY_SET_FRU_ACT_LENGTH             45
#define IPMI_RPLY_GET_DEVICE_ID_LENGTH           59 /* includes optional bytes */
#define IPMI_RPLY_GET_FAN_PROP_LENGTH            49
#define IPMI_RPLY_GET_FAN_LEVEL_LENGTH           48
#define IPMI_RPLY_SET_FAN_LEVEL_LENGTH           45

/* Response message data; offsets from beginning of message */
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
#define IPMI_RPLY_SENSOR_READING_OFFSET      43     /* Sensor reading (1 byte) */
#define IPMI_RPLY_SENSOR_ENABLE_BITS_OFFSET  44     /* Sensor enable bits: event msgs; scanning; reading/state */
#define IPMI_RPLY_HS_SENSOR_READING_OFFSET   45     /* Hot swap sensor reading (1 byte) */
#define IPMI_RPLY_FRU_AREA_SIZE_LSB_OFFSET   43     /* FRU inventory area size in bytes, LSB */
#define IPMI_RPLY_FRU_AREA_SIZE_MSB_OFFSET   44     /* FRU inventory area size in bytes, MSB */
#define IPMI_RPLY_FRU_AREA_ACCESS_OFFSET     45     /* Bit 0 indicates if device is accessed by bytes (0) or words (1) */
#define IPMI_RPLY_FRU_DATA_READ_OFFSET       44     /* FRU data */
#define IPMI_RPLY_SDRREP_VER_OFFSET          44     /* SDR Repository Info */    
#define IPMI_RPLY_SDRREP_CNT_LSB_OFFSET      45     /* Number of records in repository, LSB */
#define IPMI_RPLY_SDRREP_CNT_MSB_OFFSET      46     /* Number of records in repository, MSB */
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

/* For responses to bridged messages (returns 2 messages which we read as 1); offset from beginning of first message */
#define IPMI_RPLY_BRIDGED_SEQ_OFFSET         27     /* From-MCH sequence number */
#define IPMI_RPLY_BRIDGED_SEQLUN_OFFSET      41     /* MS 6 bits: IPMI sequence, LS 2 bits: LUN */

#define IPMI_SENSOR_READING_DISABLED(x)  x & 1<<5
#define IPMI_SENSOR_SCANNING_DISABLED(x) x & 1<<6

#define IPMI_DATA_TYPE(x)          x & 3<<6
#define IPMI_DATA_LENGTH(x)        x & 0x3F
#define IPMI_DATA_LANG_ENGLISH(x)  x==0 || x==25                 

/* Completion codes, IPMI spec Table 5-2 */
#define IPMI_COMP_CODE_NORMAL                   0x00
#define IPMI_COMP_CODE_NODE_BUSY                0xC0
#define IPMI_COMP_CODE_INVALID_COMMAND          0xC1
#define IPMI_COMP_CODE_INVALID_COMMAND_FOR_LUN  0xC2
#define IPMI_COMP_CODE_TIMEOUT                  0xC3
#define IPMI_COMP_CODE_OUT_OF_SPACE             0xC4
#define IPMI_COMP_CODE_RESERVATION              0xC5
#define IPMI_COMP_CODE_REQUEST_TRUNCATED        0xC6
#define IPMI_COMP_CODE_REQUEST_LENGTH_INVALID   0xC7
#define IPMI_COMP_CODE_REQUEST_LENGTH_LIMIT     0xC8
#define IPMI_COMP_CODE_PARAMETER_RANGE          0xC9
#define IPMI_COMP_CODE_REQUESTED_BYTES          0xCA
#define IPMI_COMP_CODE_REQUESTED_DATA           0xCB
#define IPMI_COMP_CODE_INVALID_FIELD            0xCC
#define IPMI_COMP_CODE_COMMAND_ILLEGAL          0xCD
#define IPMI_COMP_CODE_COMMAND_RESPONSE         0xCE
#define IPMI_COMP_CODE_DUPLICATED_REQUEST       0xCF
#define IPMI_COMP_CODE_SDR_REP_UPDATE           0xD0
#define IPMI_COMP_CODE_DEVICE_FW_UPDATE         0xD1
#define IPMI_COMP_CODE_BMC_INIT                 0xD2
#define IPMI_COMP_CODE_DESTINATION_UNAVAIL      0xD3
#define IPMI_COMP_CODE_PRIVILEGE                0xD4
#define IPMI_COMP_CODE_NOT_SUPPORTED            0xD5
#define IPMI_COMP_CODE_SUBFUNCTION_UNAVAIL      0xD6
#define IPMI_COMP_CODE_DEVICE_SPECIFIC_MIN      0x01
#define IPMI_COMP_CODE_DEVICE_SPECIFIC_MAX      0x7E
#define IPMI_COMP_CODE_COMMAND_SPECIFIC_MIN     0x80
#define IPMI_COMP_CODE_COMMAND_SPECIFIC_MAX     0xBE
#define IPMI_COMP_CODE_UNSPECIFIED              0xFF

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

/*SDR data*/
#define SDR_ID_LAST_SENSOR     0xFFFF
#define SDR_MAX_LENGTH         64
#define SDR_FRU_MAX_LENGTH     33

/* Common to all SDRs */
#define SDR_HEADER_LENGTH      5
#define SDR_ID_LSB_OFFSET      0  /* SDR Header */
#define SDR_ID_MSB_OFFSET      1
#define SDR_VER_OFFSET         2
#define SDR_REC_TYPE_OFFSET    3
#define SDR_LENGTH_OFFSET      4

/* SDR Full Sensor contents */
#define SDR_OWNER_OFFSET        5  /* SDR Key Fields */
#define SDR_LUN_OFFSET          6
#define SDR_NUMBER_OFFSET       7     
#define SDR_ENTITY_ID_OFFSET    8  /* SDR Body */
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
#define SDR_M_TOL_OFFSET        25 /* [7:2] MS 2 bits, [5:0] Tolerance 6 bits, unsigned (in +/- 1/2 raw counts) */ 
#define SDR_B_OFFSET            26 /* LS 8 bits, 2's complement signed 10-bit 'B' value */			   
#define SDR_B_ACC_OFFSET        27 /* [7:2] MS 2 bits unsigned, 10-bit acc in 1/100 % scaled up by: [5:0] Accuracy LS 6 bits */ 
#define SDR_ACC_OFFSET          28 /* [7:4] Accuracy MS 4 bits, [3:2] Accuracy exp: 2 bits unsigned, [1:0] sensor direction */
#define SDR_EXP_OFFSET          29
#define SDR_ANLG_CHAR_OFFSET    30
#define SDR_NOMINAL_OFFSET      31
#define SDR_NORM_MAX_OFFSET     32
#define SDR_NORM_MIN_OFFSET     33
#define SDR_STR_LENGTH_OFFSET   47
#define SDR_STR_OFFSET          48
/*...more...*/

#define SENSOR_LINEAR(x)           (x & 0x7F)
#define SENSOR_CONV_M_B(x,y)       (x + ( (y & 0xC0) << 2))
#define TWOS_COMP_SIGNED_NBIT(x,n) (x > (pow(2,n)/2 - 1)) ? x - pow(2,n)    : x /* Extract n-bit signed number stored as two's complement */
#define ONES_COMP_SIGNED_NBIT(x,n) (x > (pow(2,n)/2 - 1)) ? x - pow(2,n) + 1 : x /* Extract n-bit signed number stored as one's complement */
#define SENSOR_CONV_REXP(x)        ((x & 0xF0) >> 4)
#define SENSOR_CONV_BEXP(x)        (x & 0xF)
#define SENSOR_NOMINAL_GIVEN(x)    (x & (1<<0))
#define SENSOR_NORM_MAX_GIVEN(x)   (x & (1<<1))
#define SENSOR_NORM_MIN_GIVEN(x)   (x & (1<<2))
#define SENSOR_NOMINAL_FORMAT(x)   ((x & 0xC0) >> 6)

#define SENSOR_UNITS_UNSPEC        0
#define SENSOR_UNITS_DEGC          1
#define SENSOR_UNITS_DEGF          2
#define SENSOR_UNITS_DEGK          3
#define SENSOR_UNITS_VOLTS         4
#define SENSOR_UNITS_AMPS          5
#define SENSOR_UNITS_WATTS         6
#define SENSOR_UNITS_JOULES        7
#define SENSOR_UNITS_COULOMBS      8
#define SENSOR_UNITS_RPM           18
/* more... */

#define SENSOR_CONV_LINEAR         0
#define SENSOR_CONV_LN             1
#define SENSOR_CONV_LOG10          2
#define SENSOR_CONV_LOG2           3
#define SENSOR_CONV_E              4
#define SENSOR_CONV_EXP10          5
#define SENSOR_CONV_EXP2           6
#define SENSOR_CONV_1_X            7
#define SENSOR_CONV_SQR            8
#define SENSOR_CONV_CUBE           9
#define SENSOR_CONV_SQRT           10
#define SENSOR_CONV_CUBE_NEG1      11

#define SENSOR_NOMINAL_UNSIGNED    0
#define SENSOR_NOMINAL_ONES_COMP   1
#define SENSOR_NOMINAL_TWOS_COMP   2
#define SENSOR_NOMINAL_NONNUMERIC  3

/* SDR FRU Device contents */
#define SDR_FRU_ADDR_OFFSET        5  /* SDR Body */
#define SDR_FRU_ID_OFFSET          6
#define SDR_FRU_LUN_OFFSET         7     
#define SDR_FRU_CHAN_OFFSET        8 
#define SDR_FRU_TYPE_OFFSET        10
#define SDR_FRU_TYPE_MOD_OFFSET    11
#define SDR_FRU_ENTITY_ID_OFFSET   12
#define SDR_FRU_ENTITY_INST_OFFSET 13
#define SDR_FRU_STR_LENGTH_OFFSET  15
#define SDR_FRU_STR_OFFSET         16

#define SENSOR_OWNER_ID(x)        x & 0xFE
#define SENSOR_OWNER_ID_TYPE(x)   x & 0x1
#define SENSOR_OWNER_CHAN(x)      x & 0xF0
#define SENSOR_OWNER_LUN(x)       x & 0x3

#define DEV_SENSOR_DYNAMIC(x)     x & 0x80
#define DEV_SENSOR_LUN0(x)        x & 0x1
#define DEV_SENSOR_LUN1(x)        x & 0x2
#define DEV_SENSOR_LUN2(x)        x & 0x4
#define DEV_SENSOR_LUN3(x)        x & 0x8

/* Sensor types */
#define MAX_SENSOR_TYPE            0xFF

#define SENSOR_TYPE_TEMP           0x01
#define SENSOR_TYPE_VOLTAGE        0x02
#define SENSOR_TYPE_CURRENT        0x03
#define SENSOR_TYPE_FAN            0x04
#define SENSOR_TYPE_PHYS_SECURITY  0x05
#define SENSOR_TYPE_PLAT_SECURITY  0x06
#define SENSOR_TYPE_PROCESSOR      0x07
#define SENSOR_TYPE_POWER_SUPPPLY  0x08
#define SENSOR_TYPE_POWER_UNIT     0x09
#define SENSOR_TYPE_COOLING_DEV    0x0A
#define SENSOR_TYPE_OTHER          0x0B
#define SENSOR_TYPE_MEMORY         0x0C
#define SENSOR_TYPE_DRIVE_SLOT     0x0D
#define SENSOR_TYPE_POST_MEMORY    0x0E
#define SENSOR_TYPE_SYS_FW         0x0F
#define SENSOR_TYPE_FRU_STATE      0x2C
#define SENSOR_TYPE_HOT_SWAP       0xF0
/* more... */

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


/* FRU data */

#define MSG_FRU_DATA_READ_SIZE 0x10

/* Common header (mandatory) */
#define FRU_DATA_COMMON_HEADER_OFFSET 0
#define FRU_DATA_COMMON_HEADER_INTERNAL_AREA_OFFSET 1
#define FRU_DATA_COMMON_HEADER_CHASSIS_AREA_OFFSET  2 
#define FRU_DATA_COMMON_HEADER_BOARD_AREA_OFFSET    3 
#define FRU_DATA_COMMON_HEADER_PROD_AREA_OFFSET     4 
#define FRU_DATA_COMMON_HEADER_MULTIREC_AREA_OFFSET 5 
#define FRU_DATA_COMMON_HEADER_PAD                  6 
#define FRU_DATA_COMMON_HEADER_CS                   7 

/* Internal use area (we don't use it) */
#define FRU_DATA_INTERNAL_AREA_VERSION_OFFSET       0
#define FRU_DATA_INTERNAL_DATA_OFFSET               1

/* Chassis area (optional) */
#define FRU_DATA_CHASSIS_AREA_VERSION_OFFSET        0
#define FRU_DATA_CHASSIS_AREA_LENGTH_OFFSET         1
#define FRU_DATA_CHASSIS_TYPE_OFFSET                2
#define FRU_DATA_CHASSIS_AREA_PART_LENGTH_OFFSET    3
#define FRU_DATA_CHASSIS_AREA_PART_OFFSET           4
/* Offsets vary:
 * serial number length
 * serial number 
 * custom fields
 * byte to indicate if more fields
 * remaining space ( 0x00 )
 * checksum
 */

/* Board area (optional) */
#define FRU_DATA_BOARD_AREA_VERSION_OFFSET          0
#define FRU_DATA_BOARD_AREA_LENGTH_OFFSET           1
#define FRU_DATA_BOARD_AREA_LANG_OFFSET             2
#define FRU_DATA_BOARD_AREA_MANUF_TIME_OFFSET       3
#define FRU_DATA_BOARD_AREA_MANUF_LENGTH_OFFSET     6
#define FRU_DATA_BOARD_AREA_MANUF_OFFSET            7
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
#define FRU_DATA_PROD_AREA_VERSION_OFFSET           0
#define FRU_DATA_PROD_AREA_LENGTH_OFFSET            1
#define FRU_DATA_PROD_AREA_LANG_OFFSET              2
#define FRU_DATA_PROD_AREA_MANUF_LENGTH_OFFSET      3
#define FRU_DATA_PROD_AREA_MANUF_OFFSET             4
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

/* FRU info field data types */
#define FRU_DATA_TYPE_BINARY    0    /* binary of unspecified */
#define FRU_DATA_TYPE_BCDPLUS   1<<6
#define FRU_DATA_TYPE_6BITASCII 2<<6
#define FRU_DATA_TYPE_LANG      3<<6 /* 8-bit ASCII + Latin 1 if language is English;
                                      * otherwise 2-byte UNICODE with LS byte first. 
                                      */
/* Chassis types */
#define FRU_DATA_CHASSIS_TYPE_OTHER            0x01
#define FRU_DATA_CHASSIS_TYPE_UNKNOWN          0x02
#define FRU_DATA_CHASSIS_TYPE_DESKTOP          0x03
#define FRU_DATA_CHASSIS_TYPE_LPDESKTOP        0x04
#define FRU_DATA_CHASSIS_TYPE_PIZZABOX         0x05
#define FRU_DATA_CHASSIS_TYPE_MINITOWER        0x06
#define FRU_DATA_CHASSIS_TYPE_TOWER            0x07
#define FRU_DATA_CHASSIS_TYPE_PORTABLE         0x08
#define FRU_DATA_CHASSIS_TYPE_LAPTOP           0x09
#define FRU_DATA_CHASSIS_TYPE_NOTEBOOK         0x0A
#define FRU_DATA_CHASSIS_TYPE_HANDHELD         0x0B
#define FRU_DATA_CHASSIS_TYPE_DOCKINGSTATION   0x0C
#define FRU_DATA_CHASSIS_TYPE_ALLINONE         0x0D
#define FRU_DATA_CHASSIS_TYPE_SUBNOTEBOOK      0x0E
#define FRU_DATA_CHASSIS_TYPE_SPACESAVING      0x0F
#define FRU_DATA_CHASSIS_TYPE_LUNCHBOX         0x10
#define FRU_DATA_CHASSIS_TYPE_MAINSERVER       0x11
#define FRU_DATA_CHASSIS_TYPE_EXPANSION        0x12
#define FRU_DATA_CHASSIS_TYPE_SUBCHASSIS       0x13
#define FRU_DATA_CHASSIS_TYPE_BUSEXPANSION     0x14
#define FRU_DATA_CHASSIS_TYPE_PERIPHERAL       0x15
#define FRU_DATA_CHASSIS_TYPE_RAID             0x16
#define FRU_DATA_CHASSIS_TYPE_RACKMOUNT        0x17

/* More to come... */

/* PICMG */
#define FRU_PWR_DYNAMIC(x) x & (1 << 7)
#define FRU_PWR_LEVEL(x)   x & 0xF
#define FRU_PWR_STEADY_STATE     0
#define FRU_PWR_STEADY_STATE_DES 1
#define FRU_PWR_EARLY            2
#define FRU_PWR_EARLY_DES        3

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

/* Vadatech */
#define VT_ENTITY_ID_MCH      0xC2 
#define VT_ENTITY_ID_AMC      0xC1
#define VT_ENTITY_ID_CU       0x1E
#define VT_ENTITY_ID_PM       0x0A
#define VT_ENTITY_ID_RTM      0xC0  /* asked Vivek to verify */

#endif
