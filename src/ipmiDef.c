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

uint8_t RMCP_HEADER[] = {RMCP_MSG_VER,         /* RMCP message version */
                         0,                    /* Reserved */
                         RMCP_MSG_SEQ,         /* RMCP message header */
                         RMCP_MSG_CLASS_IPMI}; /* Message class */

/* ASF message, used only for Presence Ping */
uint8_t ASF_MSG[] = {0,    0, 0x11, 0xBE, /* ASF IANA Enterprise number */
                     0x80,                /* Message type, Presence Ping */
                     0,                   /* Message tag */
                     0,                   /* Reserved */
                     0};                  /* Data length */

/* For messages with authentication type = NONE */
uint8_t IPMI_WRAPPER[] = {IPMI_MSG_AUTH_TYPE_NONE, /* Auth type none */
                          0,
                          0,
                          0,
                          0, /* 4-byte Session sequence number */
                          0,
                          0,
                          0,
                          0,  /* 4-byte Session ID */
                          0}; /* Number of bytes in message */

/* For messages with authentication type = STRAIGHT_PASSWORD_KEY */
uint8_t IPMI_WRAPPER_PWD_KEY[] = {IPMI_MSG_AUTH_TYPE_PWD_KEY, /* Auth type */
                                  0,
                                  0,
                                  0,
                                  0, /* 4-byte Session sequence number */
                                  0,
                                  0,
                                  0,
                                  0, /* 4-byte Session ID */
                                  'I',
                                  'p',
                                  'm',
                                  'i',
                                  '2',
                                  'A',
                                  'd',
                                  'm',
                                  'i',
                                  'n',
                                  0,
                                  0,
                                  0,
                                  0,
                                  0,
                                  0,  /* Message Authentication Code (password) */
                                  0}; /* Number of bytes in message */

/* IPMI message part 1 */
uint8_t IPMI_MSG1[] = {IPMI_MSG_ADDR_BMC,               /* Responder's address */
                       IPMI_MSG_NETFN_APP_REQUEST << 2, /* Network function code/LUN */
                       0};                              /* For checksum */

size_t IPMI_MSG1_LENGTH         = sizeof(IPMI_MSG1);
size_t IPMI_WRAPPER_LENGTH      = sizeof(IPMI_WRAPPER);
size_t IPMI_WRAPPER_AUTH_LENGTH = sizeof(IPMI_WRAPPER_PWD_KEY);

/*
 *  Below are IPMI messages part 2 - all variations we use
 */

/* Get Authentication Capabilities */
uint8_t GET_AUTH_MSG_ADMIN[] = {IPMI_MSG_ADDR_SW,           /* Requester's address */
                                0,                          /* Message sequence number */
                                IPMI_MSG_CMD_GET_CHAN_AUTH, /* Command code */
                                IPMI_MSG_CURR_CHAN,         /* Channel number */
                                IPMI_MSG_PRIV_LEVEL_ADMIN,  /* Privilege level */
                                0};                         /* For checksum */

/* Get Authentication Capabilities */
uint8_t GET_AUTH_MSG[] = {IPMI_MSG_ADDR_SW,           /* Requester's address */
                          0,                          /* Message sequence number */
                          IPMI_MSG_CMD_GET_CHAN_AUTH, /* Command code */
                          IPMI_MSG_CURR_CHAN,         /* Channel number */
                          IPMI_MSG_PRIV_LEVEL_OPER,   /* Privilege level */
                          0};                         /* For checksum */

/* Get Session Challenge, no-authentication version */
uint8_t GET_SESS_MSG[] = {IPMI_MSG_ADDR_SW,                   /* Requester's address */
                          0,                                  /* Message sequence number */
                          IPMI_MSG_CMD_GET_SESSION_CHALLENGE, /* Command code */
                          IPMI_MSG_AUTH_TYPE_NONE,            /* Authentication type */
                          0,
                          0,
                          0,
                          0,
                          0,
                          0,
                          0,
                          0,
                          0,
                          0,
                          0,
                          0,
                          0,
                          0,
                          0,
                          0,  /* Null user name */
                          0}; /* For checksum */

/* Get Session Challenge, with password/key authentication */
uint8_t GET_SESS_MSG_PWD_KEY[] = {IPMI_MSG_ADDR_SW,                   /* Requester's address */
                                  0,                                  /* Message sequence number */
                                  IPMI_MSG_CMD_GET_SESSION_CHALLENGE, /* Command code */
                                  IPMI_MSG_AUTH_TYPE_PWD_KEY,         /* Authentication type */
                                  'c',
                                  'o',
                                  'n',
                                  't',
                                  'r',
                                  'o',
                                  'l',
                                  's',
                                  0,
                                  0,
                                  0,
                                  0,
                                  0,
                                  0,
                                  0,
                                  0,  /* User name */
                                  0}; /* For checksum */

/* Activate Session */
uint8_t ACT_SESS_MSG[] = {IPMI_MSG_ADDR_SW,              /* Requester's address */
                          0,                             /* Message sequence number */
                          IPMI_MSG_CMD_ACTIVATE_SESSION, /* Command code */
                          IPMI_MSG_AUTH_TYPE_NONE,       /* Authentication type (must match that in GET_SESS_MSG) */
                          IPMI_MSG_PRIV_LEVEL_OPER,      /* Max priv level for session */
                          0,
                          0,
                          0,
                          0, /* For 16-byte challenge string */
                          0,
                          0,
                          0,
                          0,
                          0,
                          0,
                          0,
                          0,
                          0,
                          0,
                          0,
                          0,
                          IPMI_WRAPPER_SEQ_INITIAL,
                          0x00,
                          0x00,
                          0x00, /* Session sequence number; set to initial value */
                          0};   /* For checksum */

/* Set Privilege Level */
uint8_t SET_PRIV_MSG[] = {IPMI_MSG_ADDR_SW,            /* Requester's address */
                          0,                           /* Message sequence number */
                          IPMI_MSG_CMD_SET_PRIV_LEVEL, /* Command code */
                          IPMI_MSG_PRIV_LEVEL_OPER,    /* Priv level for session  */
                          0};                          /* For checksum */

/* Send Message to other channel */
uint8_t SEND_MSG_MSG[] = {IPMI_MSG_ADDR_SW,      /* Requester's address */
                          0,                     /* Message sequence number */
                          IPMI_MSG_CMD_SEND_MSG, /* Command code */
                          0,                     /* Channel (IMPB) */
                          0};                    /* For checksum */

/* Get Sensor Reading  */
uint8_t SENS_READ_MSG[] = {IPMI_MSG_ADDR_SW,         /* Requester's address */
                           0,                        /* Message sequence number */
                           IPMI_MSG_CMD_SENSOR_READ, /* Command code */
                           0,                        /* For sensor number */
                           0};                       /* For checksum */

/* Get Sensor Thresholds  */
uint8_t GET_SENSOR_THRESH_MSG[] = {IPMI_MSG_ADDR_SW,               /* Requester's address */
                                   0,                              /* Message sequence number */
                                   IPMI_MSG_CMD_GET_SENSOR_THRESH, /* Command code */
                                   0,                              /* For sensor number */
                                   0};                             /* For checksum */

/* Read FRU Data  */
uint8_t FRU_READ_MSG[] = {IPMI_MSG_ADDR_SW,           /* Requester's address */
                          0,                          /* Message sequence number */
                          IPMI_MSG_CMD_READ_FRU_DATA, /* Command code */
                          0,                          /* FRU device ID */
                          0,                          /* FRU inventory offset to read, LS byte */
                          0,                          /* FRU inventory offset to read, MS byte */
                          0,                          /* Count to read (1-based) */
                          0};                         /* For checksum */

/*  Reserve SDR Repository */
uint8_t RESERVE_SDR_MSG[] = {IPMI_MSG_ADDR_SW, /* Requester's address */
                             0,                /* Message sequence number */
                             0,                /* Command code */
                             0};               /* For checksum */

/* Get Sensor Data Record (SDR) and Get Device SDR */
uint8_t GET_SDR_MSG[] = {IPMI_MSG_ADDR_SW, /* Requester's address */
                         0,                /* Message sequence number */
                         0,                /* Command code */
                         0,
                         0, /* Reservation ID, LS byte first */
                         0,
                         0,  /* Record ID, LS byte first */
                         0,  /* Offset into record to begin read */
                         0,  /* Bytes to read (0xFF for entire record) */
                         0}; /* For checksum */

/* Get Device SDR Info   */
uint8_t GET_DEV_SDR_INFO_MSG[] = {
    IPMI_MSG_ADDR_SW,                /* Requester's address */
    0,                               /* Message sequence number */
    IPMI_MSG_CMD_GET_DEV_SDR_INFO,   /* Command code */
    IPMI_GET_DEV_SDR_INFO_SDR_COUNT, /* Request data, get SDR count or sensor count for specified LUN */
    0};                              /* For checksum */

/* Close Session */
uint8_t CLOSE_SESS_MSG[] = {IPMI_MSG_ADDR_SW,           /* Requester's address */
                            0,                          /* Message sequence number */
                            IPMI_MSG_CMD_CLOSE_SESSION, /* Command code */
                            0,
                            0,
                            0,
                            0,  /* For 4-byte session ID */
                            0}; /* For checksum */

/* Chassis Control */
uint8_t CHAS_CTRL_MSG[] = {IPMI_MSG_ADDR_SW,       /* Requester's address */
                           0,                      /* Message sequence number */
                           IPMI_MSG_CMD_CHAS_CTRL, /* Command code */
                           0,                      /* Command value (on, off, reset) */
                           0};                     /* For checksum */

/* Default message (no data) */
uint8_t BASIC_MSG[] = {IPMI_MSG_ADDR_SW, /* Requester's address */
                       0,                /* Message sequence number */
                       0,                /* Command code */
                       0};               /* For checksum */
