#ifndef DRV_IPMI_MSG_H
#define DRV_IPMI_MSG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <drvMch.h>

/* Call realloc to allocate memory block; Use memset to set block to all zeros */
void *ipmiReallocZeros(void *dest, size_t size);

/* Convert 2-element array (which stores LS byte first) to integer */
uint16_t arrayToUint16(uint8_t *data );

/* Convert 4-element array (which stores LS byte first) to integer */
uint32_t arrayToUint32(uint8_t *data);

/* Increment value of 2-element uint8_t array (which stores LS byte first)
 * Roll over to 0 when max possible value is reached.
 */
void incr2Uint8Array(uint8_t *data, int incr); 

/* Increment value of 4-element uint8_t array (which stores LS byte first)
 * Roll over to 0 when max possible value is reached.
 */

void  incr4Uint8Array(uint8_t *data, int incr); 

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
 *   IPMI Message Session Header  - fixed length (IPMI_WRAPPER below)
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

/* IMPORTANT: For all routines below, caller must perform locking */

void ipmiBuildSendMsg(IpmiSess sess, uint8_t *message, size_t *messageSize, uint8_t cmd, uint8_t netfn, uint8_t rsAddr, uint8_t rqAddr, uint8_t *msg2, size_t msg2Size, uint8_t lun);

void ipmiCompletionCode(const char *name, uint8_t code, uint8_t cmd, uint8_t netfn);

int ipmiMsgBuild(IpmiSess sess, uint8_t *message, uint8_t cmd, uint8_t imsg1netfn, uint8_t *imsg2, size_t imsg2Size, uint8_t *b1msg1, uint8_t *b1msg2, size_t b1msg2Size, uint8_t *b2msg1, uint8_t *b2msg2, size_t b2msg2Size);

int ipmiMsgWriteRead(const char *name, uint8_t *message, size_t messageSize, uint8_t *response, size_t *responseSize, double timeout, size_t *responseLen);

int ipmiMsgGetDeviceId(void *device, IpmiSess sess, uint8_t *data, int bridged, uint8_t rsAddr, uint8_t rqAddr, size_t *responseSize, int offs);

int ipmiMsgGetChanAuth(void *device, IpmiSess sess, uint8_t *data, size_t *responseSize, size_t roffs);

int ipmiMsgGetSess(void *device, IpmiSess sess, uint8_t *data, size_t *responseSize, uint8_t *msg, size_t roffs);

int ipmiMsgActSess(void *device, IpmiSess sess, uint8_t *data, size_t *responseSize, size_t roffs);

int ipmiMsgSetPriv(void *device, IpmiSess sess, uint8_t *data, size_t *responseSize, uint8_t level, size_t roffs);

int ipmiMsgCloseSess(void *device, IpmiSess sess, uint8_t *data, size_t *responseSize);

int ipmiMsgColdReset(void *device, IpmiSess sess, uint8_t *data);

int ipmiMsgChassisControl(void *device, IpmiSess sess,  uint8_t *data, int bridged, uint8_t rsAddr, uint8_t rqAddr, uint8_t parm, size_t *responseSize, int offs);

int ipmiMsgGetChassisStatus(void *device, IpmiSess sess,  uint8_t *data, int bridged, uint8_t rsAddr, uint8_t rqAddr, size_t *responseSize, int roffs);

int ipmiMsgGetFruInvInfo(void *device, IpmiSess sess, uint8_t *data, int bridged, uint8_t rsAddr, uint8_t rqAddr, uint8_t id, size_t *responseSize, int offs);

int ipmiMsgReadFru(void *device, IpmiSess sess, uint8_t *data, int bridged, uint8_t rsAddr, uint8_t rqAddr, uint8_t id, uint8_t *readOffset, uint8_t readSize, size_t *responseSize, int offs);

int ipmiMsgGetSdrRepInfo(void *device, IpmiSess sess, uint8_t *data, int bridged, uint8_t rsAddr, uint8_t rqAddr, size_t *responseSize, int offs);

int ipmiMsgReserveSdrRep(void *device, IpmiSess sess, uint8_t *data, int bridged, uint8_t rsAddr, uint8_t rqAddr, size_t *responseSize, int offs);

int ipmiMsgGetDevSdrInfo(void *device, IpmiSess sess, uint8_t *data, int bridged, uint8_t rsAddr, uint8_t rqAddr, uint8_t parm, size_t *responseSize, int offs);

int ipmiMsgGetSdr(void *device, IpmiSess sess, uint8_t *data, int bridged, uint8_t rsAddr, uint8_t rqAddr,  uint8_t *id, uint8_t *res, uint8_t offset, uint8_t readSize, uint8_t parm, uint8_t recordSize, size_t *responseSize, int offs);
				
int ipmiMsgReadSensor(void *device, IpmiSess sess, uint8_t *data, uint8_t bridged, uint8_t rsAddr, uint8_t rqAddr, uint8_t sens, uint8_t lun, size_t *responseSize, int offs);

int ipmiMsgGetSensorThresholds(void *device, IpmiSess sess, uint8_t *data, uint8_t bridged, uint8_t rsAddr, uint8_t rqAddr, uint8_t sens, uint8_t lun, size_t *responseSize, int offs);

#ifdef __cplusplus
};
#endif

#endif
