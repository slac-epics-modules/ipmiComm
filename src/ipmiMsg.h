#ifndef DRV_IPMI_MSG_H
#define DRV_IPMI_MSG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <ipmiDef.h>
#include <drvMch.h>

/* Options for no response to message */
#define MSG_NO_RESEND  0  /* Start new session, but don't re-send original message */
#define MSG_RESEND     1  /* Start new session, then re-send original message */
#define MSG_NO_RETRY   2  /* If no response, just exit */

#define RPLY_TIMEOUT 2.0

extern volatile int IPMICOMM_DEBUG; 

/*
 * To-MCH IPMI message structure:
 *
 * {RMCP header}{IPMI header}{IPMI msg 1(checksum)}{IPMI msg 2 [Bridged msg 1(checksum)][Bridged msg 2(checksum)](checksum)}
 *                                                              |_____________________________________________|      |
 *                                                                                     |                             |
 *                                                                      IPMI Bridged message is optional          Checksum for
 *                                                                                                                IPMI msg 2
 *                                                                                                               (does not include
 *                                                                                                                bridged msg)
 * RMCP uses MSB first
 * IPMI uses LSB first - (this does not match the standard 'network order') 
 * so all multi-byte values below store LSB first
 *
 * 
 *   RMCP header - fixed for all IPMI messages (RMCP_HEADER below)
 *
 *   IPMI Message Session Header  - fixed length (IPMI_HEADER below)
 *
 *   IPMI Message Part 1 - fixed length (IMSG1 below)
 *       rsAddr    (1 byte) - Responder's address (BMC address)
 *       netFn/LUN (1 byte) - Function code 
 *
 *   IPMI Message Part 1 Checksum - 2's complement checksum (1 byte)
 *
 *   IPMI Message Part 2 - variable length depending on msg type (variations below)
 *       rqAddr (1 byte) - Requester's address
 *       rqSeq           - Message sequence number
 *       cmd             - Command code
 *       data (variable) - Depends on message type
 *
 * --- begin optional "bridged" message - embedded IPMI message to be sent to other channel. Used with IPMI Message cmd = MSG_IPMI_MCD_SEND_MSG
 *
 *        IPMI Message Part 1 - rsAddr = Bridged channel address
 *   
 *        IPMI Message Part 1 Checksum - 2's complement checksum (1 byte)
 *  
 *        IPMI Message Part 2 
 *
 *        IPMI Message Part 2 Checksum - 2's complement checksum (1 byte)
 *
 * --- end optional "bridged" message
 *   
 *   IPMI Message Part 2 Checksum - 2's complement checksum (1 byte)
 */

extern uint8_t RMCP_HEADER[4];
extern uint8_t ASF_MSG[8];
extern uint8_t IPMI_HEADER[10];
extern uint8_t IPMI_MSG1[3];
extern uint8_t GET_AUTH_MSG[6];
extern uint8_t GET_SESS_MSG[21];
extern uint8_t ACT_SESS_MSG[26];
extern uint8_t SET_PRIV_MSG[5];
extern uint8_t SEND_MSG_MSG[5];
extern uint8_t SENS_READ_MSG[5];
extern uint8_t FRU_READ_MSG[8];
extern uint8_t GET_SDR_MSG[10];
extern uint8_t CLOSE_SESS_MSG[8];
extern uint8_t CHAS_CTRL_MSG[5];
extern uint8_t GET_DEV_SDR_INFO_MSG[5];            
extern uint8_t BASIC_MSG[4];
extern uint8_t SET_FRU_ACT_MSG[7];
extern uint8_t SET_FRU_POLICY_MSG[8];
extern uint8_t GET_FAN_PROP_MSG[6];
extern uint8_t GET_FAN_LEVEL_MSG[6];
extern uint8_t SET_FAN_LEVEL_MSG[8];

/* Convert 2-element array (which stores LS byte first) to integer */
uint16_t arrayToUint16(uint8_t *data );

/* Increment value of 2-element uint8_t array (which stores LS byte first)
 * Roll over to 0 when max possible value is reached.
 */
void incr2Uint8Array(uint8_t *data, int incr); 

/* IMPORTANT: For all routines below, caller must perform locking */

void ipmiMsgSetSeqId(MchData mchData, uint8_t *message, uint8_t cmd);

int ipmiMsgWriteRead(const char *name, asynUser *pasynUser, uint8_t *message, size_t messageSize, uint8_t *response, size_t *responseSize, double timeout);

int ipmiMsgWriteReadHelper(MchData mchData, uint8_t *message, size_t messageSize, uint8_t *response, size_t *responseSize);

void ipmiMsgGetChanAuth(MchData mchData, uint8_t *data);

void ipmiMsgGetSess(MchData mchData, uint8_t *data);

void ipmiMsgActSess(MchData mchData, uint8_t *data);

void ipmiMsgSetPriv(MchData mchData, uint8_t *data, uint8_t level);

int ipmiMsgColdReset(MchData mchData, uint8_t *data);

int ipmiMsgChassisControl(MchData mchData, uint8_t *data, uint8_t parm);

int ipmiMsgGetFruInfo(MchData mchData, uint8_t *data, uint8_t id);

int ipmiMsgReadFru(MchData mchData, uint8_t *data, uint8_t id, uint8_t *readOffset, uint8_t readSize);

int ipmiMsgGetSdrRepInfo(MchData mchData, uint8_t *data);

int ipmiMsgGetDevSdrInfo(MchData mchData, uint8_t *data, uint8_t parm);

int ipmiMsgGetSdr(MchData mchData, uint8_t *data, uint8_t *id, uint8_t *res, uint8_t offset, uint8_t readSize, uint8_t parm);
				
int ipmiMsgReadSensor(MchData mchData, uint8_t *data, uint8_t sens, uint16_t addr, size_t *responseSize);

int ipmiMsgSetFruActPolicyHelper(MchData mchData, uint8_t *data, uint8_t fru, int parm);

int ipmiMsgGetFanProp(MchData mchData, uint8_t *data, uint8_t fru);

int ipmiMsgGetFanLevel(MchData mchData, uint8_t *data, uint8_t fru);

int ipmiMsgSetFanLevel(MchData mchData, uint8_t *data, uint8_t fru, uint8_t level);

#ifdef __cplusplus
};
#endif

#endif
