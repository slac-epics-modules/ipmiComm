//////////////////////////////////////////////////////////////////////////////
// This file is part of 'ipmiComm'.
// It is subject to the license terms in the LICENSE.txt file found in the
// top-level directory of this distribution and at:
//    https://confluence.slac.stanford.edu/display/ppareg/LICENSE.html.
// No part of 'ipmiComm', including this file,
// may be copied, modified, propagated, or distributed except according to
// the terms contained in the LICENSE.txt file.
//////////////////////////////////////////////////////////////////////////////
#ifndef DRV_PICMG_H
#define DRV_PICMG_H

extern uint8_t GET_PICMG_PROP_MSG[5];
extern uint8_t SET_FRU_ACT_MSG[7];
extern uint8_t SET_FRU_POLICY_MSG[8];
extern uint8_t GET_FRU_POLICY_MSG[6];
extern uint8_t GET_FAN_PROP_MSG[6];
extern uint8_t GET_FAN_LEVEL_MSG[6];
extern uint8_t SET_FAN_LEVEL_MSG[8];
extern uint8_t GET_POWER_LEVEL_MSG[7];
extern uint8_t GET_ADDR_INFO_MSG[9];
extern uint8_t GET_ADDR_INFO_HWADDR_MSG[8];

extern uint8_t FRU_I2C_ADDR[102];

/*
 * Definitions and constants for
 * PICMG/MicroTCA/ATCA systems
 *
 */

/* PICMG */
#define PICMG_RPLY_IMSG2_GET_POWER_LEVEL_PROP_OFFSET  2 /* FRU power properties */
#define PICMG_RPLY_IMSG2_GET_POWER_LEVEL_DELAY_OFFSET 3 /* Delay to stable power */
#define PICMG_RPLY_IMSG2_GET_POWER_LEVEL_MULT_OFFSET  4 /* Power multiplier */
#define PICMG_RPLY_IMSG2_GET_POWER_LEVEL_DRAW_OFFSET                                                                   \
    5 /* Draw of first level (optional: next levels follow this, up to 25 total) */
#define IPMI_RPLY_IMSG2_GET_FAN_PROP_MIN_OFFSET  2 /* Fan tray minimum fan level */
#define IPMI_RPLY_IMSG2_GET_FAN_PROP_MAX_OFFSET  3 /* Fan tray maximum fan level */
#define IPMI_RPLY_IMSG2_GET_FAN_PROP_NOM_OFFSET  4 /* Fan tray nominal fan level */
#define IPMI_RPLY_IMSG2_GET_FAN_PROP_PROP_OFFSET 5 /* Fan tray properties */

/* PICMG */
/* consider changing these names to include picmg  ? */
#define IPMI_MSG2_SET_FRU_ACT_DEACTIVATE               0 /* Command value in Set FRU Activate command */
#define IPMI_MSG2_SET_FRU_ACT_ACTIVATE                 1
#define IPMI_MSG2_SET_FRU_ACT_FRU_OFFSET               4 /* FRU ID, used in many commands */
#define IPMI_MSG2_SET_FRU_ACT_CMD_OFFSET               5
#define IPMI_MSG2_SET_FRU_POLICY_MASK_OFFSET           5
#define IPMI_MSG2_SET_FRU_POLICY_BITS_OFFSET           6
#define IPMI_MSG2_SET_FAN_LEVEL_OFFSET                 5 /* New fan level to set */
#define PICMG_RPLY_IMSG2_GET_POWER_LEVEL_TYPE_OFFSET   5
#define PICMG_RPLY_IMSG2_GET_FAN_OVERRIDE_LEVEL_OFFSET 2 /* Fan level set by shelf manager */
#define PICMG_RPLY_IMSG2_GET_FAN_LOCAL_LEVEL_OFFSET    3 /* Fan level set locally */
#define PICMG_RPLY_IMSG2_GET_FAN_LOCAL_ENABLED_OFFSET  4 /* 1 if fan is in local control, else 0 */
#define PICMG_RPLY_IMSG2_PICMG_ID_OFFSET               1 /* Indicates PICMG-defined group extension; value of 0x00 is used */
#define PICMG_RPLY_IMSG2_GET_PICMG_PROP_PICMG_VERS_OFFSET                                                              \
    2 /* Version of PICMG extensions implemented by IPM controller */
#define PICMG_RPLY_IMSG2_GET_PICMG_PROP_MAX_FRU_ID_OFFSET                                                              \
    3 /* Maximum FRU device ID on system (excluding reserved 254, 255) */
#define PICMG_RPLY_IMSG2_GET_PICMG_PROP_IPMC_FRU_ID_OFFSET 4 /* FRU ID of IPM comtroller */

/* Get address info command */
#define PICMG_IMSG2_GET_ADDR_INFO_FRUID_OFFSET       4
#define PICMG_IMSG2_GET_ADDR_INFO_KEY_TYPE_OFFSET    5
#define PICMG_IMSG2_GET_ADDR_INFO_KEY_OFFSET         6
#define PICMG_IMSG2_GET_ADDR_INFO_SITE_TYPE_OFFSET   7
#define PICMG_RPLY_IMSG2_GET_ADDR_INFO_HWADDR_OFFSET 2 /* Hardware address */
#define PICMG_RPLY_IMSG2_GET_ADDR_INFO_IPMB0_ADDR_OFFSET                                                               \
    3 /* IPMB-0 address if implemented; for PICMG 2.9 value of 0xFF indicates IPMB-0 not implemented */
#define PICMG_RPLY_IMSG2_GET_ADDR_INFO_FRU_ID_OFFSET                                                                   \
    5 /* FRU device ID or 0xFF if Address Key Type = Physical Address and Site Type = RTM */
#define PICMG_RPLY_IMSG2_GET_ADDR_INFO_SITE_NUMBER_OFFSET 6 /* Site number */
#define PICMG_RPLY_IMSG2_GET_ADDR_INFO_SITE_TYPE_OFFSET   7 /* Site type */
#define PICMG_ADDR_KEY_TYPE_HWADDR                        0x00
#define PICMG_ADDR_KEY_TYPE_IPMB0                         0x01
#define PICMG_ADDR_KEY_TYPE_PICMG29                       0x02 /* Reserved for PICMG 2.9 */
#define PICMG_ADDR_KEY_TYPE_PHYSADDR                      0x03

#define IPMI_MSG_CMD_GET_PICMG_PROP  0x00
#define IPMI_MSG_CMD_GET_ADDR_INFO   0x01
#define IPMI_MSG_CMD_GET_POWER_LEVEL 0x12
#define IPMI_MSG_CMD_GET_FAN_PROP    0x14
#define IPMI_MSG_CMD_GET_FAN_LEVEL   0x16
#define IPMI_MSG_CMD_SET_FAN_LEVEL   0x15

#define IPMI_RPLY_GET_FRU_POLICY_LENGTH 3
#define IPMI_RPLY_GET_FAN_PROP_LENGTH   6
#define IPMI_RPLY_GET_FAN_LEVEL_LENGTH  5 /* max length due to optional bytes */
#define IPMI_RPLY_GET_POWER_LEVEL_LENGTH                                                                               \
    6 /* min length--currently only read lowest power level values, which is all SLAC ATCA systems provide */
#define IPMI_RPLY_SET_FAN_LEVEL_LENGTH        3 /* ignore optional byte for now */
#define IPMI_RPLY_IMSG2_GET_PICMG_PROP_LENGTH 4
#define PICMG_RPLY_GET_ADDR_INFO_LENGTH       8
#define PICMG_RPLY_SET_FRU_ACT_LENGTH         2

/* need to update this */
#define IPMI_RPLY_GET_FRU_POLICY_PROP_OFFSET 44 /* FRU activation policy properties */

#define PICMG_FAN_LOCAL_CONTROL_SUPPORTED(x) (x & 0x80)

/* PICMG */
#define FRU_PWR_DYNAMIC(x)       x & (1 << 7)
#define FRU_PWR_LEVEL(x)         x & 0xF
#define FRU_PWR_STEADY_STATE     0
#define FRU_PWR_STEADY_STATE_DES 1
#define FRU_PWR_EARLY            2
#define FRU_PWR_EARLY_DES        3
#define FRU_PWR_MSG_CMPTBL(x)                                                                                          \
    ((x >= 3 && x <= 16) || (x >= 40 && x <= 41) || (x >= 50 && x <= 53)) /* See Vadatech CLI Manual (5.7.9) */

/* Site type values */
#define PICMG_SITE_TYPE_FRONT_BOARD     0x00
#define PICMG_SITE_TYPE_POWER_ENTRY     0x01
#define PICMG_SITE_TYPE_SHELF_FRU_INFO  0x02
#define PICMG_SITE_TYPE_SHMC            0x03
#define PICMG_SITE_TYPE_FAN_TRAY        0x04
#define PICMG_SITE_TYPE_FAN_FILTER_TRAY 0x05
#define PICMG_SITE_TYPE_ALARM           0x06
#define PICMG_SITE_TYPE_AMC             0x07
#define PICMG_SITE_TYPE_PMC             0x08
#define PICMG_SITE_TYPE_RTM             0x09
#define PICMG_SITE_TYPE_OEM_MIN         0xC0
#define PICMG_SITE_TYPE_OEM_MAX         0xCF
/* Reserved site type ranges: 0x0A-0xBF and 0xD0-0xFE
 * Unknown site type 0xFF
 */

/* MicroTCA */
#define UTCA_FRU_TYPE_CARRIER     0
#define UTCA_FRU_TYPE_SHELF_MIN   1
#define UTCA_FRU_TYPE_SHELF_MAX   2
#define UTCA_FRU_TYPE_MCH_MIN     3
#define UTCA_FRU_TYPE_MCH_MAX     4
#define UTCA_FRU_TYPE_AMC_MIN     5
#define UTCA_FRU_TYPE_AMC_MAX     39
#define UTCA_FRU_TYPE_CU_MIN      40
#define UTCA_FRU_TYPE_CU_MAX      49
#define UTCA_FRU_TYPE_PM_MIN      50
#define UTCA_FRU_TYPE_PM_MAX      59
#define UTCA_FRU_TYPE_RTM_MIN     90
#define UTCA_FRU_TYPE_RTM_MAX     124
#define UTCA_FRU_TYPE_LOG_CARRIER 253

#define PICMG_FRU_TYPE_SHMC_MAX 254
#define PICMG_FRU_TYPE_SHMC_MIN 252

/* FRU index values for ATCA systems;
 * Values arbitrarily chosen by system software (us)
 * in order to have uniform FRU indices assigned to
 * hardware across systems. For example, slot 1 AMC will
 * use the same index across all systems. This is done to
 * be able to configure PVs & EDM screens in advance.
 */
#define ATCA_FRU_TYPE_CARRIER     0
#define ATCA_FRU_TYPE_SHELF_MIN   1
#define ATCA_FRU_TYPE_SHELF_MAX   2
#define ATCA_FRU_TYPE_MCH_MIN     3
#define ATCA_FRU_TYPE_MCH_MAX     4
#define ATCA_FRU_TYPE_AMC_MIN     5
#define ATCA_FRU_TYPE_AMC_MAX     39
#define ATCA_FRU_TYPE_CU_MIN      40
#define ATCA_FRU_TYPE_CU_MAX      49
#define ATCA_FRU_TYPE_PM_MIN      50
#define ATCA_FRU_TYPE_PM_MAX      59
#define ATCA_FRU_TYPE_RTM_MIN     90
#define ATCA_FRU_TYPE_RTM_MAX     124
#define ATCA_FRU_TYPE_LOG_CARRIER 253

/* Response message lengths using Vadatech MCH (0s for those that can vary) */
#define IPMI_RPLY_CLOSE_SESSION_LENGTH_VT      22
#define IPMI_RPLY_SENSOR_READ_MAX_LENGTH_VT    48 /* length varies */
#define IPMI_RPLY_GET_SENSOR_THRESH_LENGTH_VT  51 /* need to determine this; using NAT length for now*/
#define IPMI_RPLY_GET_CHAS_STATUS_LENGTH_VT    47 /* does NOT include optional byte */
#define IPMI_RPLY_CHAS_CTRL_LENGTH_VT          44
#define IPMI_RPLY_GET_FRU_INFO_LENGTH_VT       47
#define IPMI_RPLY_READ_FRU_DATA_BASE_LENGTH_VT 45
#define IPMI_RPLY_WRITE_FRU_DATA_LENGTH_VT     45
#define IPMI_RPLY_GET_SDRREP_INFO_LENGTH_VT    58
#define IPMI_RPLY_RESERVE_SDRREP_LENGTH_VT     46
#define IPMI_RPLY_GET_SDR_BASE_LENGTH_VT       50 /* get sdr message length is this + remaining data bytes */
#define IPMI_RPLY_GET_SDR_LENGTH_VT            0  /* varies */
#define IPMI_RPLY_GET_DEV_SDR_INFO_LENGTH_VT   50
#define IPMI_RPLY_GET_DEV_SDR_LENGTH_VT        0 /* varies */
#define IPMI_RPLY_COLD_RESET_LENGTH_VT         44
#define IPMI_RPLY_SET_FRU_POLICY_LENGTH_VT     44
#define IPMI_RPLY_GET_FRU_POLICY_LENGTH_VT     45
#define IPMI_RPLY_SET_FRU_ACT_LENGTH_VT        45
#define IPMI_RPLY_GET_DEVICE_ID_LENGTH_VT      59 /* includes optional bytes */
#define IPMI_RPLY_GET_FAN_PROP_LENGTH_VT       49
#define IPMI_RPLY_GET_FAN_LEVEL_LENGTH_VT      48
#define IPMI_RPLY_SET_FAN_LEVEL_LENGTH_VT      45

/* Response message lengths using NAT MCH (0s for those we haven't figured out yet) */
#define IPMI_RPLY_CLOSE_SESSION_LENGTH_NAT      22 /* UPDATE THIS */
#define IPMI_RPLY_SENSOR_READ_MAX_LENGTH_NAT    34 /* length varies */
#define IPMI_RPLY_GET_SENSOR_THRESH_LENGTH_NAT  51
#define IPMI_RPLY_GET_CHAS_STATUS_LENGTH_NAT    47 /* does NOT include optional byte */
#define IPMI_RPLY_CHAS_CTRL_LENGTH_NAT          44
#define IPMI_RPLY_GET_FRU_INFO_LENGTH_NAT       32
#define IPMI_RPLY_READ_FRU_DATA_BASE_LENGTH_NAT 30
#define IPMI_RPLY_WRITE_FRU_DATA_LENGTH_NAT     45
#define IPMI_RPLY_GET_SDRREP_INFO_LENGTH_NAT    43
#define IPMI_RPLY_RESERVE_SDRREP_LENGTH_NAT     31
#define IPMI_RPLY_GET_SDR_BASE_LENGTH_NAT       35 /* get sdr message length is this + remaining data bytes */
#define IPMI_RPLY_GET_SDR_LENGTH_NAT            0  /* varies */
#define IPMI_RPLY_GET_DEV_SDR_INFO_LENGTH_NAT   50
#define IPMI_RPLY_GET_DEV_SDR_LENGTH_NAT        0 /* varies */
#define IPMI_RPLY_COLD_RESET_LENGTH_NAT         44
#define IPMI_RPLY_SET_FRU_POLICY_LENGTH_NAT     29
#define IPMI_RPLY_GET_FRU_POLICY_LENGTH_NAT     39
#define IPMI_RPLY_SET_FRU_ACT_LENGTH_NAT        45
#define IPMI_RPLY_GET_DEVICE_ID_LENGTH_NAT      44 /* includes optional bytes */
#define IPMI_RPLY_GET_FAN_PROP_LENGTH_NAT       43
#define IPMI_RPLY_GET_FAN_LEVEL_LENGTH_NAT      42
#define IPMI_RPLY_SET_FAN_LEVEL_LENGTH_NAT      41
#define IPMI_RPLY_OFFSET_NAT                    -15
#define IPMI_RPLY_2ND_BRIDGED_OFFSET_NAT        14

/* PICMG-specific Entity IDs */
#define ENTITY_ID_PICMG_FRONT_BOARD    0xA0
#define ENTITY_ID_PICMG_RTM            0xC0
#define ENTITY_ID_PICMG_AMC            0xC1
#define ENTITY_ID_PICMG_SHMC           0xF0
#define ENTITY_ID_PICMG_FILT_UNIT      0xF1
#define ENTITY_ID_PICMG_SHELF_FRU_INFO 0xF2
#define ENTITY_ID_PICMG_ALARM_PANEL    0xF3
#define PICMG_ENTITY_ID_POWER_FILT     0x15

/*
#define PICMG_ENTITY_ID_FRONT_BOARD          0xA0
#define PICMG_ENTITY_ID_RTM                  0xC0
#define PICMG_ENTITY_ID_AMC                  0xC1
#define PICMG_ENTITY_ID_SHM                  0xF0 Shelf management controller
#define PICMG_ENTITY_ID_FILTRATION_UNIT      0xF1
#define PICMG_ENTITY_ID_SHELF_FRU_INFO       0xF2
#define PICMG_ENTITY_ID_ALARM_PANEL          0xF3
#define PICMG_ENTITY_ID_POWER_FILTERING      0x15
*/

/* PICMG hardware addresses */
#define PICMG_HWADDR_SHMC       0x10 /* Active shelf manager */
#define PICMG_HWADDR_ICMB       0x11 /* ICMB device */
#define PICMG_HWADDR_LOG_SLOT1  0x41 /* Logical slot 1 */
#define PICMG_HWADDR_LOG_SLOT2  0x42 /* Logical slot 2 */
#define PICMG_HWADDR_LOG_SLOT3  0x43 /* Logical slot 3 */
#define PICMG_HWADDR_LOG_SLOT4  0x44 /* Logical slot 4 */
#define PICMG_HWADDR_LOG_SLOT5  0x45 /* Logical slot 5 */
#define PICMG_HWADDR_LOG_SLOT6  0x46 /* Logical slot 6 */
#define PICMG_HWADDR_LOG_SLOT7  0x47 /* Logical slot 7 */
#define PICMG_HWADDR_LOG_SLOT8  0x48 /* Logical slot 8 */
#define PICMG_HWADDR_LOG_SLOT9  0x49 /* Logical slot 9 */
#define PICMG_HWADDR_LOG_SLOT10 0x4A /* Logical slot 10 */
#define PICMG_HWADDR_LOG_SLOT11 0x4B /* Logical slot 11 */
#define PICMG_HWADDR_LOG_SLOT12 0x4C /* Logical slot 12 */
#define PICMG_HWADDR_LOG_SLOT13 0x4D /* Logical slot 13 */
#define PICMG_HWADDR_LOG_SLOT14 0x4E /* Logical slot 14 */
#define PICMG_HWADDR_LOG_SLOT15 0x4F /* Logical slot 15 */
#define PICMG_HWADDR_LOG_SLOT16 0x50 /* Logical slot 16 */

/* PICMG IPMB addresses */
#define PICMG_IPMB_SHMC       0x20 /* Active shelf manager */
#define PICMG_IPMB_ICMB       0x22 /* ICMB device */
#define PICMG_IPMB_LOG_SLOT1  0x82 /* Logical slot 1 */
#define PICMG_IPMB_LOG_SLOT2  0x84 /* Logical slot 2 */
#define PICMG_IPMB_LOG_SLOT3  0x86 /* Logical slot 3 */
#define PICMG_IPMB_LOG_SLOT4  0x88 /* Logical slot 4 */
#define PICMG_IPMB_LOG_SLOT5  0x8A /* Logical slot 5 */
#define PICMG_IPMB_LOG_SLOT6  0x8C /* Logical slot 6 */
#define PICMG_IPMB_LOG_SLOT7  0x8E /* Logical slot 7 */
#define PICMG_IPMB_LOG_SLOT8  0x90 /* Logical slot 8 */
#define PICMG_IPMB_LOG_SLOT9  0x92 /* Logical slot 9 */
#define PICMG_IPMB_LOG_SLOT10 0x94 /* Logical slot 10 */
#define PICMG_IPMB_LOG_SLOT11 0x96 /* Logical slot 11 */
#define PICMG_IPMB_LOG_SLOT12 0x98 /* Logical slot 12 */
#define PICMG_IPMB_LOG_SLOT13 0x9A /* Logical slot 13 */
#define PICMG_IPMB_LOG_SLOT14 0x9C /* Logical slot 14 */
#define PICMG_IPMB_LOG_SLOT15 0x9E /* Logical slot 15 */
#define PICMG_IPMB_LOG_SLOT16 0xA0 /* Logical slot 16 */

/* PICMG sensor type codes */
#define SENSOR_TYPE_HOTSWAP     0xF0 /* Contains M-state */
#define SENSOR_TYPE_HOTSWAP_NAT 0xF2
#define SENSOR_TYPE_IPMB0       0xF1 /* IPMB-0 Physical Link */
#define SENSOR_TYPE_TELCO       0xF4 /* Telco Alarm Input */

#ifdef __cplusplus
}
;
#endif

#endif
