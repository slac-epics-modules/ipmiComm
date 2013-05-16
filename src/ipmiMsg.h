#ifndef DRV_IPMI_MSG_H
#define DRV_IPMI_MSG_H

#ifdef __cplusplus
extern "C" {
#endif

/*#include <ipmiDef.h>*/
#include <drvMch.h>

#define RPLY_TIMEOUT   1.0

extern volatile int IPMICOMM_DEBUG; 

/*
 * To-MCH IPMI message structure:
 *
 * {RMCP header}{IPMI header}{IPMI msg 1(cs)}{IPMI msg 2 [Bridged msg part1(cs)][Bridged msg part2 [Bridged msg part1(cs)][Bridged msg part2(cs)](checksum)] (checksum)}      		 
 *                                                        |_______________________________________||____________________________________________|      |	 |		  	 
 *                                                                            |                                          |                             |	 |		  	 
 *                                                            First optional bridged message              Second optional bridged message            cs for	 cs for IPMI msg part2	  	 
 *                                                                                                                                                   optional	     (does not include 	 
 *                                                                                                                                                   first bridged   bridged msgs)
 *                                                                                                                                                   msg part2
 *                                                                                                                                                  (does not         
 *                                                                                                                                                   include second 
 *                                                                                                                                                   bridged msg)
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
 * --- begin first optional "bridged" message - embedded IPMI message to be sent to other channel. Used with IPMI Message cmd = MSG_IPMI_MCD_SEND_MSG
 *
 *        IPMI Bridged Message Part 1 - rsAddr = Bridged channel address
 *   
 *        IPMI Bridged Message Part 1 Checksum - 2's complement checksum (1 byte)
 *  
 *        IPMI Bridged Message Part 2 
 *
 * --- begin second optional "bridged" message - embedded IPMI message to be sent to other channel. Used with IPMI Message cmd = MSG_IPMI_MCD_SEND_MSG
 *
 *        IPMI 2nd Bridged Message Part 1 - rsAddr = Bridged channel address
 *   
 *        IPMI 2nd Bridged Message Part 1 Checksum - 2's complement checksum (1 byte)
 *  
 *        IPMI 2nd Bridged Message Part 2 
 *
 *        IPMI 2nd Bridged Message Part 2 Checksum - 2's complement checksum (1 byte)
 *
 * --- end second optional "bridged" message
 *
 *        IPMI Bridged Message Part 2 Checksum - 2's complement checksum (1 byte)
 *
 * --- end first optional "bridged" message
 *   
 *   IPMI Message Part 2 Checksum - 2's complement checksum (1 byte)
 */

/* Convert 2-element array (which stores LS byte first) to integer */
uint16_t arrayToUint16(uint8_t *data );

/* Increment value of 2-element uint8_t array (which stores LS byte first)
 * Roll over to 0 when max possible value is reached.
 */
void incr2Uint8Array(uint8_t *data, int incr); 

/* IMPORTANT: For all routines below, caller must perform locking */

void ipmiMsgSetSeqId(MchSess mchSess, uint8_t *message, uint8_t cmd);

int ipmiMsgWriteRead(const char *name, uint8_t *message, size_t messageSize, uint8_t *response, size_t *responseSize, double timeout);

int ipmiMsgGetChanAuth(MchSess mchSess, uint8_t *data);

int ipmiMsgGetSess(MchSess mchSess, uint8_t *data);

int ipmiMsgActSess(MchSess mchSess, uint8_t *data);

int ipmiMsgSetPriv(MchSess mchSess, uint8_t *data, uint8_t level);

int ipmiMsgCloseSess(MchSess mchSess, uint8_t *data);

int ipmiMsgColdReset(MchSess mchSess, uint8_t *data);

int ipmiMsgChassisControl(MchSess mchSess, uint8_t *data, uint8_t parm);

int ipmiMsgGetFruInfo(MchSess mchSess, uint8_t *data, uint8_t id);

int ipmiMsgReadFru(MchSess mchSess, uint8_t *data, uint8_t id, uint8_t *readOffset, uint8_t readSize);

int ipmiMsgGetSdrRepInfo(MchSess mchSess, uint8_t *data);

int ipmiMsgGetDevSdrInfo(MchSess mchSess, uint8_t *data, uint8_t parm);

int ipmiMsgGetSdr(MchSess mchSess, uint8_t *data, uint8_t *id, uint8_t *res, uint8_t offset, uint8_t readSize, uint8_t parm);
				
int ipmiMsgReadSensor(MchSess mchSess, uint8_t *data, uint8_t sens, uint8_t lun, size_t *responseSize);

int ipmiMsgSetFruActHelper(MchSess mchSess, uint8_t *data, uint8_t fru, int parm);

int ipmiMsgGetFruActPolicyHelper(MchSess mchSess, uint8_t *data, uint8_t fru);

int ipmiMsgGetFanPropHelper(MchSess mchSess, uint8_t *data, uint8_t fru);

int ipmiMsgGetFanLevelHelper(MchSess mchSess, uint8_t *data, uint8_t fru);

int ipmiMsgSetFanLevelHelper(MchSess mchSess, uint8_t *data, uint8_t fru, uint8_t level);

int ipmiMsgGetPowerLevel(MchSess mchSess, uint8_t *data, uint8_t fru, uint8_t parm);

#ifdef __cplusplus
};
#endif

#endif
