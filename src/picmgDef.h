#ifndef DRV_PICMG_H
#define DRV_PICMG_H

extern uint8_t SET_FRU_ACT_MSG[7];
extern uint8_t SET_FRU_POLICY_MSG[8];
extern uint8_t GET_FRU_POLICY_MSG[6];
extern uint8_t GET_FAN_PROP_MSG[6];
extern uint8_t GET_FAN_LEVEL_MSG[6];
extern uint8_t SET_FAN_LEVEL_MSG[8];
extern uint8_t GET_POWER_LEVEL_MSG[7];

extern uint8_t FRU_I2C_ADDR[102];

/*
 * Definitions and constants for
 * PICMG/MicroTCA/ATCA systems
 *
 */

/* PICMG */
#define PICMG_RPLY_IMSG2_GET_POWER_LEVEL_PROP_OFFSET   2    /* FRU power properties */
#define PICMG_RPLY_IMSG2_GET_POWER_LEVEL_DELAY_OFFSET  3    /* Delay to stable power */
#define PICMG_RPLY_IMSG2_GET_POWER_LEVEL_MULT_OFFSET   4    /* Power multiplier */
#define PICMG_RPLY_IMSG2_GET_POWER_LEVEL_DRAW_OFFSET   5    /* Draw of first level (optional: next levels follow this, up to 25 total) */
#define IPMI_RPLY_IMSG2_GET_FAN_PROP_MIN_OFFSET    2     /* Fan tray minimum fan level */
#define IPMI_RPLY_IMSG2_GET_FAN_PROP_MAX_OFFSET    3     /* Fan tray maximum fan level */
#define IPMI_RPLY_IMSG2_GET_FAN_PROP_NOM_OFFSET    4     /* Fan tray nominal fan level */
#define IPMI_RPLY_IMSG2_GET_FAN_PROP_PROP_OFFSET   5     /* Fan tray properties */

/* PICMG */
/* consider changing these names to include picmg  ? */
#define IPMI_MSG2_SET_FRU_ACT_DEACTIVATE      0    /* Command value in Set FRU Activate command */
#define IPMI_MSG2_SET_FRU_ACT_ACTIVATE        1
#define IPMI_MSG2_SET_FRU_ACT_FRU_OFFSET      4    /* FRU ID, used in many commands */
#define IPMI_MSG2_SET_FRU_ACT_CMD_OFFSET      5
#define IPMI_MSG2_SET_FRU_POLICY_MASK_OFFSET  5
#define IPMI_MSG2_SET_FRU_POLICY_BITS_OFFSET  6
#define IPMI_MSG2_SET_FAN_LEVEL_OFFSET        5    /* New fan level to set */
#define PICMG_RPLY_IMSG2_GET_POWER_LEVEL_TYPE_OFFSET    5
#define PICMG_RPLY_IMSG2_GET_FAN_OVERRIDE_LEVEL_OFFSET  2    /* Fan level set by shelf manager */
#define PICMG_RPLY_IMSG2_GET_FAN_LOCAL_LEVEL_OFFSET     3    /* Fan level set locally */
#define PICMG_RPLY_IMSG2_GET_FAN_LOCAL_ENABLED_OFFSET   4    /* 1 if fan is in local control, else 0 */

#define IPMI_MSG_CMD_GET_POWER_LEVEL         0x12
#define IPMI_MSG_CMD_GET_FAN_PROP            0x14
#define IPMI_MSG_CMD_GET_FAN_LEVEL           0x16
#define IPMI_MSG_CMD_SET_FAN_LEVEL           0x15

#define IPMI_RPLY_GET_FRU_POLICY_LENGTH   3
#define IPMI_RPLY_GET_FAN_PROP_LENGTH     6
#define IPMI_RPLY_GET_FAN_LEVEL_LENGTH    5  /* max length due to optional bytes */
#define IPMI_RPLY_GET_POWER_LEVEL_LENGTH  25 /* max length due to optional bytes */
#define IPMI_RPLY_SET_FAN_LEVEL_LENGTH    3  /* ignore optional byte for now */

/* need to update this */
#define IPMI_RPLY_GET_FRU_POLICY_PROP_OFFSET    44    /* FRU activation policy properties */

#define PICMG_FAN_LOCAL_CONTROL_SUPPORTED(x) (x&0x80)

#define PICMG_ENTITY_ID_FRONT_BOARD          0xA0
#define PICMG_ENTITY_ID_RTM                  0xC0
#define PICMG_ENTITY_ID_AMC                  0xC1
#define PICMG_ENTITY_ID_SHM                  0xF0 /* Shelf management controller */
#define PICMG_ENTITY_ID_FILTRATION_UNIT      0xF1
#define PICMG_ENTITY_ID_SHELF_FRU_INFO       0xF2
#define PICMG_ENTITY_ID_ALARM_PANEL          0xF3
#define PICMG_ENTITY_ID_POWER_FILTERING      0x15

/* PICMG */
#define FRU_PWR_DYNAMIC(x) x & (1 << 7)
#define FRU_PWR_LEVEL(x)   x & 0xF
#define FRU_PWR_STEADY_STATE     0
#define FRU_PWR_STEADY_STATE_DES 1
#define FRU_PWR_EARLY            2
#define FRU_PWR_EARLY_DES        3
#define FRU_PWR_MSG_CMPTBL(x) ((x>=3 && x<=16) || (x>=40 && x<=41) || (x>=50 && x<=53)) /* See Vadatech CLI Manual (5.7.9) */

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
#define IPMI_RPLY_2ND_BRIDGED_OFFSET_NAT   14

#ifdef __cplusplus
};
#endif

#endif
