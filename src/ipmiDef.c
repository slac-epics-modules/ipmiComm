#include <inttypes.h>
#include <ipmiMsg.h>

uint8_t RMCP_HEADER[] = { RMCP_MSG_VER,           /* RMCP message version */	     
                          0,		               /* Reserved */		     
                          RMCP_MSG_SEQ,	       /* RMCP message header */	     
                          RMCP_MSG_CLASS_IPMI };  /* Message class */		     

/* ASF message, used only for Presence Ping */
uint8_t ASF_MSG[]     = { 0, 0, 0x11, 0xBE,       /* ASF IANA Enterprise number */	     
                          0x80,		          /* Message type, Presence Ping */		     
                          0,	                  /* Message tag */	     
                          0,                      /* Reserved */
			  0 };                    /* Data length */		     

uint8_t IPMI_HEADER[] = { IPMI_MSG_AUTH_TYPE_NONE,/* Auth type none */
                          0, 0, 0, 0,             /* 4-byte Session sequence number */
                          0, 0, 0, 0,             /* 4-byte Session ID */
                          0 };                    /* Number of bytes in message */

/* IPMI message part 1 */
uint8_t IPMI_MSG1[]   = { IPMI_MSG_ADDR_BMC,	     /* Responder's address */
		          IPMI_MSG_NETFN_APP_REQUEST,/* Network function code/LUN */
		          0 };                       /* For checksum */

/*
 *  Below are IPMI messages part 2 - all variations we use
 */

/* Get Authentication Capabilities */
uint8_t GET_AUTH_MSG[] = { IPMI_MSG_ADDR_SW,	       /* Requester's address */		   
		           0,			       /* Message sequence number */	    
		           IPMI_MSG_CMD_GET_CHAN_AUTH, /* Command code */	   
		           IPMI_MSG_CURR_CHAN,         /* Channel number */   
		           IPMI_MSG_PRIV_LEVEL_ADMIN,  /* Privilege level */
                           0 };                        /* For checksum */

/* Get Session Challenge */
uint8_t GET_SESS_MSG[] = { IPMI_MSG_ADDR_SW,	               /* Requester's address */		   
			   4,			               /* Message sequence number */	    
			   IPMI_MSG_CMD_GET_SESSION_CHALLENGE, /* Command code */	   
			   IPMI_MSG_AUTH_TYPE_NONE,            /* Authentication type */   
			   'a', 'd', 'm', 'i', 'n', 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, /* User name */
                           0 };                                /* For checksum */

/* Activate Session */
uint8_t ACT_SESS_MSG[] = { IPMI_MSG_ADDR_SW,	         /* Requester's address */		   
			   8,			         /* Message sequence number */	    
			   IPMI_MSG_CMD_ACTIVATE_SESSION,/* Command code */
		           IPMI_MSG_AUTH_TYPE_NONE,      /* Authentication type  */
			   IPMI_MSG_PRIV_LEVEL_ADMIN,    /* Max priv level for session */   
			   0, 0, 0, 0,                   /* For 16-byte challenge string */
			   0, 0, 0, 0,
			   0, 0, 0, 0,
			   0, 0, 0, 0,
                           IPMI_MSG_HEADER_SEQ_INITIAL, 0x00, 0x00, 0x00 , /* Session sequence number; set to initial value */
                           0 };                          /* For checksum */

/* Set Privilege Level */
uint8_t SET_PRIV_MSG[] = { IPMI_MSG_ADDR_SW,	         /* Requester's address */		   
			   0,			         /* Message sequence number */	    
			   IPMI_MSG_CMD_SET_PRIV_LEVEL,  /* Command code */
			   IPMI_MSG_PRIV_LEVEL_ADMIN,    /* Priv level for session  */   
                           0 };                          /* For checksum */

/* Send Message to other channel */
uint8_t SEND_MSG_MSG[] = { IPMI_MSG_ADDR_SW,             /* Requester's address */            
                           0,                            /* Message sequence number */         
                           IPMI_MSG_CMD_SEND_MSG,        /* Command code */         
			   0,                            /* Channel (IMPB) */
                           0 };                          /* For checksum */

/* Get Sensor Reading  */
uint8_t SENS_READ_MSG[] = { IPMI_MSG_ADDR_BMC,           /* Requester's address (BMC) */            
                            0,                           /* Message sequence number */         
                            IPMI_MSG_CMD_SENSOR_READ,    /* Command code */         
			    0,                           /* For sensor number */
                            0 };                         /* For checksum */

/* Read FRU Data  */
uint8_t FRU_READ_MSG[]  = { IPMI_MSG_ADDR_BMC,           /* Requester's address (BMC) */            
                            0,                           /* Message sequence number */         
                            IPMI_MSG_CMD_READ_FRU_DATA,  /* Command code */         
			    0,                           /* FRU device ID */
			    0,                           /* FRU inventory offset to read, LS byte */
			    0,                           /* FRU inventory offset to read, MS byte */
			    0,                           /* Count to read (1-based) */
                            0 };                         /* For checksum */

/* Get Sensor Data Record (SDR) and Get Device SDR */
uint8_t GET_SDR_MSG[]  =  { IPMI_MSG_ADDR_BMC,           /* Requester's address (BMC) */            
                            0,                           /* Message sequence number */         
                            0,                           /* Command code */         
			    0, 0,                        /* Reservation ID, LS byte first */
			    0, 0,                        /* Record ID, LS byte first */
			    0,                           /* Offset into record to begin read */
                            0,                           /* Bytes to read (0xFF for entire record) */
                            0 };                         /* For checksum */

/* Get Device SDR Info   */
uint8_t GET_DEV_SDR_INFO_MSG[] = { IPMI_MSG_ADDR_BMC,      /* Requester's address (BMC) */            
                            0,                             /* Message sequence number */         
                            IPMI_MSG_CMD_GET_DEV_SDR_INFO, /* Command code */         
			    0,                             /* Request data, 1 get SDR count, 0 get sensor count */
                            0 };                           /* For checksum */

/* Close Session */
uint8_t CLOSE_SESS_MSG[]= { IPMI_MSG_ADDR_SW,            /* Requester's address */            
                            0,                           /* Message sequence number */         
                            IPMI_MSG_CMD_CLOSE_SESSION,  /* Command code */         
			    0, 0, 0, 0,                  /* For 4-byte session ID */
                            0 };                         /* For checksum */

/* Chassis Control */
uint8_t CHAS_CTRL_MSG[] = { IPMI_MSG_ADDR_BMC,           /* Requester's address */            
                            0,                           /* Message sequence number */         
                            IPMI_MSG_CMD_CHAS_CTRL,      /* Command code */         
                            0,                           /* Command value (on, off, reset) */         
                            0 };                         /* For checksum */

/* Default message (no data) */
uint8_t BASIC_MSG[]=    { IPMI_MSG_ADDR_SW,              /* Requester's address */            
                            0,                           /* Message sequence number */         
                            0,                           /* Command code */         
                            0 };                         /* For checksum */


/* Messages below are not part of the base IPMI spec; they were added by ATCA */

/* Set FRU Activation */
uint8_t SET_FRU_ACT_MSG[] = { IPMI_MSG_ADDR_BMC,           /* Requester's address */            
                              0,                           /* Message sequence number */         
                              IPMI_MSG_CMD_SET_FRU_ACT,    /* Command code */         
			      0,                           /* PICMG Identifier, 0x00 is used */
                              0,                           /* FRU ID */	
                              0,                           /* Command value, 0x00 deactivate, 0x01 activate */         
                              0 };                         /* For checksum */

/* Set FRU Activation Policy */
uint8_t SET_FRU_POLICY_MSG[] = { IPMI_MSG_ADDR_BMC,           /* Requester's address */            
                                 0,                           /* Message sequence number */         
                                 IPMI_MSG_CMD_SET_FRU_POLICY, /* Command code */         
			         0,                           /* PICMG Identifier, 0x00 is used */
                                 0,                           /* FRU ID */	
                                 0,                           /* FRU activation policy mask bits */
                                 0,                           /* FRU activation policy set bits */         
                                 0 };                         /* For checksum */

/* Get Fan Speed Properties */
uint8_t GET_FAN_PROP_MSG[] = {   IPMI_MSG_ADDR_BMC,           /* Requester's address */            
                                 0,                           /* Message sequence number */         
                                 IPMI_MSG_CMD_GET_FAN_PROP,   /* Command code */         
			         0,                           /* PICMG Identifier, 0x00 is used */
                                 0,                           /* FRU ID */	
                                 0 };                         /* For checksum */

/* Get Fan Level */
uint8_t GET_FAN_LEVEL_MSG[] = {  IPMI_MSG_ADDR_BMC,           /* Requester's address */            
                                 0,                           /* Message sequence number */         
                                 IPMI_MSG_CMD_GET_FAN_LEVEL,  /* Command code */         
			         0,                           /* PICMG Identifier, 0x00 is used */
                                 0,                           /* FRU ID */	
                                 0 };                         /* For checksum */

/* Set Fan Level */
uint8_t SET_FAN_LEVEL_MSG[] = {  IPMI_MSG_ADDR_BMC,           /* Requester's address */            
                                 0,                           /* Message sequence number */         
                                 IPMI_MSG_CMD_SET_FAN_LEVEL,  /* Command code */         
			         0,                           /* PICMG Identifier, 0x00 is used */
                                 0,                           /* FRU ID */	
                                 0,                           /* Fan level */	
                                 0,                           /* OPTIONAL local control enable state, 0 to disable, 1 to enable */	
                                 0 };                         /* For checksum */
