//////////////////////////////////////////////////////////////////////////////
// This file is part of 'ipmiComm'.
// It is subject to the license terms in the LICENSE.txt file found in the
// top-level directory of this distribution and at:
//    https://confluence.slac.stanford.edu/display/ppareg/LICENSE.html.
// No part of 'ipmiComm', including this file,
// may be copied, modified, propagated, or distributed except according to
// the terms contained in the LICENSE.txt file.
//////////////////////////////////////////////////////////////////////////////
#include <inttypes.h>
#include <ipmiMsg.h>
#include <picmgDef.h>

/* Get PICMG Properties */
uint8_t GET_PICMG_PROP_MSG[] = {IPMI_MSG_ADDR_BMC,           /* Requester's address */
                                0,                           /* Message sequence number */
                                IPMI_MSG_CMD_GET_PICMG_PROP, /* Command code */
                                0,                           /* PICMG Identifier, 0x00 is used */
                                0};                          /* For checksum */

/* Set FRU Activation */
uint8_t SET_FRU_ACT_MSG[] = {IPMI_MSG_ADDR_BMC,        /* Requester's address */
                             0,                        /* Message sequence number */
                             IPMI_MSG_CMD_SET_FRU_ACT, /* Command code */
                             0,                        /* PICMG Identifier, 0x00 is used */
                             0,                        /* FRU device ID */
                             0,                        /* Command value, 0x00 deactivate, 0x01 activate */
                             0};                       /* For checksum */

/* Set FRU Activation Policy */
uint8_t SET_FRU_POLICY_MSG[] = {IPMI_MSG_ADDR_BMC,           /* Requester's address */
                                0,                           /* Message sequence number */
                                IPMI_MSG_CMD_SET_FRU_POLICY, /* Command code */
                                0,                           /* PICMG Identifier, 0x00 is used */
                                0,                           /* FRU device ID */
                                0,                           /* FRU activation policy mask bits */
                                0,                           /* FRU activation policy set bits */
                                0};                          /* For checksum */

/* Get FRU Activation Policy */
uint8_t GET_FRU_POLICY_MSG[] = {IPMI_MSG_ADDR_BMC,           /* Requester's address */
                                0,                           /* Message sequence number */
                                IPMI_MSG_CMD_GET_FRU_POLICY, /* Command code */
                                0,                           /* PICMG Identifier, 0x00 is used */
                                0,                           /* FRU device ID */
                                0};                          /* For checksum */

/* Get Fan Speed Properties */
uint8_t GET_FAN_PROP_MSG[] = {IPMI_MSG_ADDR_BMC,         /* Requester's address */
                              0,                         /* Message sequence number */
                              IPMI_MSG_CMD_GET_FAN_PROP, /* Command code */
                              0,                         /* PICMG Identifier, 0x00 is used */
                              0,                         /* FRU ID */
                              0};                        /* For checksum */

/* Get Fan Level */
uint8_t GET_FAN_LEVEL_MSG[] = {IPMI_MSG_ADDR_BMC,          /* Requester's address */
                               0,                          /* Message sequence number */
                               IPMI_MSG_CMD_GET_FAN_LEVEL, /* Command code */
                               0,                          /* PICMG Identifier, 0x00 is used */
                               0,                          /* FRU ID */
                               0};                         /* For checksum */

/* Set Fan Level */
uint8_t SET_FAN_LEVEL_MSG[] = {IPMI_MSG_ADDR_BMC,          /* Requester's address */
                               0,                          /* Message sequence number */
                               IPMI_MSG_CMD_SET_FAN_LEVEL, /* Command code */
                               0,                          /* PICMG Identifier, 0x00 is used */
                               0,                          /* FRU ID */
                               0,                          /* Fan level */
                               0,  /* OPTIONAL local control enable state, 0 to disable, 1 to enable */
                               0}; /* For checksum */

/* Set Power Level */
uint8_t GET_POWER_LEVEL_MSG[] = {IPMI_MSG_ADDR_BMC,            /* Requester's address */
                                 0,                            /* Message sequence number */
                                 IPMI_MSG_CMD_GET_POWER_LEVEL, /* Command code */
                                 0,                            /* PICMG Identifier, 0x00 is used */
                                 0,                            /* FRU ID */
                                 0,  /* Power type: steady state, desired, early, desired early */
                                 0}; /* For checksum */

/* Get address info - implementing all optional fields */
uint8_t GET_ADDR_INFO_MSG[] = {IPMI_MSG_ADDR_BMC,          /* Requester's address */
                               0,                          /* Message sequence number */
                               IPMI_MSG_CMD_GET_ADDR_INFO, /* Command code */
                               0,                          /* PICMG Identifier, 0x00 is used */
                               0,                          /* FRU ID */
                               0,                          /* Address key type */
                               0,                          /* Address key */
                               0,                          /* Site type */
                               0};                         /* For checksum */

/* Get address info using HW address; does not implement optional last field */
uint8_t GET_ADDR_INFO_HWADDR_MSG[] = {IPMI_MSG_ADDR_BMC,          /* Requester's address */
                                      0,                          /* Message sequence number */
                                      IPMI_MSG_CMD_GET_ADDR_INFO, /* Command code */
                                      0,                          /* PICMG Identifier, 0x00 is used */
                                      0,                          /* FRU ID */
                                      0,                          /* Address key type */
                                      0,                          /* Address key */
                                      0};                         /* For checksum */

/* Get address info using HW address; does not implement optional last field */
uint8_t GET_ADDR_INFO_IPMB0_MSG[] = {IPMI_MSG_ADDR_BMC,          /* Requester's address */
                                     0,                          /* Message sequence number */
                                     IPMI_MSG_CMD_GET_ADDR_INFO, /* Command code */
                                     0,                          /* PICMG Identifier, 0x00 is used */
                                     0,                          /* FRU ID */
                                     PICMG_ADDR_KEY_TYPE_IPMB0,  /* Address key type */
                                     0,                          /* Address key */
                                     0};                         /* For checksum */

/* I2C addresses, compiled by NAT
 * Indexed by FRU ID
 * (last 150-ish addresses are unused,
 *  so left these out of the array)
 */

uint8_t FRU_I2C_ADDR[102] = {

    0,                                        /* carrier manager */
    0,                                        /* physical Shelf FRU Info 1 */
    0,                                        /* physical Shelf FRU Info 2 */
    0x10,                                     /* MCMC1 */
    0x12,                                     /* MCMC2 */
    0x72,                                     /* AMC1  */
    0x74,                                     /* AMC2  */
    0x76,                                     /* AMC3  */
    0x78,                                     /* AMC4	 */
    0x7A,                                     /* AMC5	 */
    0x7C,                                     /* AMC6	 */
    0x7E,                                     /* AMC7  */
    0x80,                                     /* AMC8	 */
    0x82,                                     /* AMC9	 */
    0x84,                                     /* AMC10 */
    0x86,                                     /* AMC11 */
    0x88,                                     /* AMC12 */
    0,    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,    /* 17-28 reserved for AMC */
    0xA2,                                     /* AMC13 */
    0xA4,                                     /* AMC13 */
    0,    0, 0, 0, 0, 0, 0, 0, 0,             /* 31-39 reserved for AMC */
    0xA8,                                     /* CU1   */
    0xAA,                                     /* CU2   */
    0,    0, 0, 0, 0, 0, 0, 0,                /* 42-49 reserved for CU  */
    0xC2,                                     /* PM1   */
    0xC4,                                     /* PM2   */
    0xC6,                                     /* PM3   */
    0xC8,                                     /* PM4   */
    0,    0, 0, 0, 0, 0,                      /* 54-59 reserved for PM  */
    0x14,                                     /* ClkMod1 */
    0x16,                                     /* HubMod1 */
    0x14,                                     /* ClkMod2 */
    0x16,                                     /* HubMod2 */
    0x1C,                                     /* MCH RTM 1 */
    0x1E,                                     /* MCH RTM 2 */
    0,    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, /* 66-78 reserved for OEM modules */
    0,                                        /* Telco Alarm Function */
    0,    0, 0, 0, 0, 0, 0, 0, 0, 0,          /* 80-89 implementation defined */
    0x72,                                     /* RTM1  */
    0x74,                                     /* RTM2  */
    0x76,                                     /* RTM3  */
    0x78,                                     /* RTM4  */
    0x7A,                                     /* RTM5  */
    0x7C,                                     /* RTM6  */
    0x7E,                                     /* RTM7  */
    0x80,                                     /* RTM8  */
    0x82,                                     /* RTM9  */
    0x84,                                     /* RTM10 */
    0x86,                                     /* RTM11 */
    0x88                                      /* RTM12 */
                                              /* 102-124 reserved for RTM
                                               * 125-127 reserved
                                               * 128 local ShM (blank)
                                               * 129-252 reserved (blank)
                                               * 253 logical CM (backplane FRU-Info) (blank)
                                               * 254 logical ShM (backplane FRU-Info) (blank)
                                               */
};
