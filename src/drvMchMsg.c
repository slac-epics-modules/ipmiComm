#include <errlog.h>
#include <epicsMutex.h>
#include <asynDriver.h>
#include <asynOctetSyncIO.h>

#include <stdint.h>
#include <string.h>

#include <ipmiMsg.h>
#include <ipmiDef.h>
#include <drvMchMsg.h>

/* change this to be callback mchWriteRead, move to drvMch.c
 *
 * Call ipmiMsgWriteRead. 
 * If error, increment error count. Else, set error count back to zero.
 * If MCH is alive, but there is no response or error count reaches 10,
 * start a new session and return error. 
 * Check message sequence, session sequence, completion code.
 *
 *   RETURNS: 0 if error-free response
 *            non-zero if error, no response, or failed to start new session
 *
 *   Arguments:
 *                mchSess      - Pointer to session data structure
 *                ipmiSess     - Pointer to IPMI data structure
 *                message      - Pointer to outgoing message array
 *                messageSize  - Size of outgoing message
 *                response     - Pointer to response array
 *                responseSize - Response size; if 0, allow for max response length
 *                cmd          - IPMI command code
 *                netfn        - IPMI network function
 *                codeOffs     - Additional offset used to read completion code
 *                outSess      - 1 if outside a session; 0 if in a session
 */
int
mchMsgWriteReadHelper(MchSess mchSess, IpmiSess ipmiSess, uint8_t *message, size_t messageSize, uint8_t *response, size_t *responseSize, uint8_t cmd, uint8_t netfn, int codeOffs, int outSess)
{
int      i, status, inst = mchSess->instance;
uint8_t  ipmiSeq = 0, code;
int      ipmiSeqOffs = IPMI_RPLY_SEQLUN_OFFSET;
uint8_t  seq[4];
uint32_t seqInt;
uint32_t seqRplyInt;
size_t   responseLen;

	/* seems we check this twice for sensor reads */
       	if ( !MCH_ONLN( mchStat[inst] ) )
		return -1;

       	status = ipmiMsgWriteRead( mchSess->name, message, messageSize, response, responseSize, mchSess->timeout, &responseLen );

	if ( MCH_DBG( mchStat[inst] ) > 1 ) {

		if ( status )
			printf("%s Message status %i, received %i, expected %i\n", mchSess->name, status, responseLen, *responseSize);

		if ( MCH_DBG( mchStat[inst] ) > 2 ) {
			printf("%s Message status %i, received %i, expected %i, raw data:\n", mchSess->name, status, responseLen, *responseSize );
			for ( i = 0; i < MSG_MAX_LENGTH; i++ )
				printf("%02x ", response[i]);
			printf("\n");
		}
	}

	if ( outSess ) {

		if ( (code = response[IPMI_RPLY_COMPLETION_CODE_UNBRIDGED_OFFSET]) && MCH_DBG( mchStat[inst] ) )
			ipmiCompletionCode( mchSess->name, code, cmd, netfn );
	       	return code;
	}

       	if ( (*responseSize == 0) || (mchSess->err > 9) ) {

       		if ( MCH_DBG( mchStat[inst] ) )
       			printf("%s start new session; err count is %i\n", mchSess->name, mchSess->err);

       		/* Reset error count to 0 */
       		mchSess->err = 0;

       		/* Close current session, start new one, and return error */
       		mchMsgCloseSess( mchSess, ipmiSess, response );
       		mchNewSession( mchSess, ipmiSess );
       	       	return -1;
       	}

       	/* Verify IPMI message sequence number. If incorrect, increment error count and return error */
       	ipmiSeq = (response[ipmiSeqOffs] & 0xFC) >> 2;
       	if ( ipmiSeq != ipmiSess->seq ) {
	       	if ( MCH_DBG( mchStat[inst] ) )
	       		printf("%s Incorrect IPMI sequence; got %i but expected %i\n", mchSess->name, ipmiSeq, ipmiSess->seq );
	       	mchSess->err++;
	       	return -1;
       	}

	/* Extract session sequence number from reply */
	for ( i = 0; i < IPMI_RPLY_SEQ_LENGTH ; i++)
       	       	seq[i] = response[IPMI_RPLY_SEQ_OFFSET + i]; /* bridged offset ? */
	       
	seqInt     = arrayToUint32( seq );
	seqRplyInt = arrayToUint32( ipmiSess->seqRply );

	/* Check session sequence number. If it is not increasing or more than 
	 * 7 counts higher than last time, increment our stored seq number and set error. 
	 * Else store sequence number.
	 *
	 * If seq number error or completion code non-zero, increment error count and return error.
         * Else reset error count to 0 and return success.
	 */
	if ( (seqInt <= seqRplyInt) || (seqInt - seqRplyInt > 7) ) {
		if ( MCH_DBG( mchStat[inst] ) > 1 )
	       		printf("%s sequence number %i, previous %i\n", mchSess->name, seqInt, seqRplyInt);
	       	incr4Uint8Array( ipmiSess->seqRply, 1 );
		mchSess->err++;
		return -1;
       	}
       	else {
		for ( i = 0; i < IPMI_RPLY_SEQ_LENGTH ; i++)
			ipmiSess->seqRply[i] = seq[i];

		if ( (code = response[IPMI_RPLY_COMPLETION_CODE_OFFSET + codeOffs]) ) {
			if ( MCH_DBG( mchStat[inst] ) )
				ipmiCompletionCode( mchSess->name, code, cmd, netfn );
			mchSess->err++;
			return code;
		}
	}

	mchSess->err = 0;
	return 0;
}

/* Close Session - test for NAT and determine reply offset*/
int
mchMsgCloseSess(MchSess mchSess, IpmiSess ipmiSess, uint8_t *data)
{
size_t responseSize = MCH_IS_NAT( mchSess->type ) ? IPMI_RPLY_CLOSE_SESSION_LENGTH_NAT : IPMI_RPLY_CLOSE_SESSION_LENGTH_VT;

	return ipmiMsgCloseSess( mchSess, ipmiSess, data, &responseSize );
}

/* Chassis Control 
 *
 *   RETURNS: status from ipmiMsgChassisControl
 *            0 on success
 *            non-zero for error
 */
int
mchMsgChassisControl(MchData mchData, uint8_t *data, uint8_t parm)
{
size_t  responseSize = 0; /* determine length */
int     offs  = MCH_IS_NAT( mchData->mchSess->type ) ? IPMI_RPLY_OFFSET_NAT : 0;

	return ipmiMsgChassisControl( mchData->mchSess, mchData->ipmiSess, data, parm, &responseSize, offs );
}

/* Get FRU Inventory Info 
 *
 *   RETURNS: status from ipmiMsgGetFruInfo
 *            0 on success
 *            non-zero for error
 */
int
mchMsgGetFruInfo(MchData mchData, uint8_t *data, uint8_t id)
{
size_t responseSize = MCH_IS_NAT( mchData->mchSess->type ) ? IPMI_RPLY_GET_FRU_INFO_LENGTH_NAT : IPMI_RPLY_GET_FRU_INFO_LENGTH_VT;
int    offs         = MCH_IS_NAT( mchData->mchSess->type ) ? IPMI_RPLY_OFFSET_NAT : 0;

	return ipmiMsgGetFruInfo( mchData->mchSess, mchData->ipmiSess, data, id, &responseSize, offs );
}

/* Read FRU data 
 *
 *   RETURNS: status from mchMsgWriteReadHelper
 *            0 on success
 *            non-zero for error
 */
int
mchMsgReadFru(MchData mchData, uint8_t *data, uint8_t id, uint8_t *readOffset, uint8_t readSize)
{
size_t   responseSize = MCH_IS_NAT( mchData->mchSess->type ) ? IPMI_RPLY_READ_FRU_DATA_BASE_LENGTH_NAT : IPMI_RPLY_READ_FRU_DATA_BASE_LENGTH_VT;
int      offs         = MCH_IS_NAT( mchData->mchSess->type ) ? IPMI_RPLY_OFFSET_NAT : 0;

	return ipmiMsgReadFru( mchData->mchSess, mchData->ipmiSess, data, id, readOffset, readSize, &responseSize, offs );
}

/* Get SDR (Sensor Data Record) Repository Info 
 *
 *   RETURNS: status from mchMsgWriteReadHelper
 *            0 on success
 *            non-zero for error
 */
int
mchMsgGetSdrRepInfo(MchData mchData, uint8_t *data)
{
size_t responseSize = MCH_IS_NAT( mchData->mchSess->type ) ? IPMI_RPLY_GET_SDRREP_INFO_LENGTH_NAT : IPMI_RPLY_GET_SDRREP_INFO_LENGTH_VT;
int    offs         = MCH_IS_NAT( mchData->mchSess->type ) ? IPMI_RPLY_OFFSET_NAT : 0;

	return ipmiMsgGetSdrRepInfo( mchData->mchSess, mchData->ipmiSess, data, &responseSize, offs );
}

/* 
 * Get SDR (Sensor Data Record)
 * 
 * Offset and reservation ID are 0 unless doing partial read that begins at an 
 * offset into the SDR other than 0.
 *
 * Used for both Get SDR (parm = 0) and Get Device SDR (parm = 1). 
 * Only differences in the message are the network function and command code.
 *
 *   RETURNS: status from mchMsgWriteReadHelper
 *            0 on success
 *            non-zero for error
 */
int
mchMsgGetSdr(MchData mchData, uint8_t *data, uint8_t *id, uint8_t *res, uint8_t offset, uint8_t readSize, uint8_t parm, uint8_t recordSize)
{
size_t responseSize = ( MCH_IS_NAT( mchData->mchSess->type ) ? IPMI_RPLY_GET_SDR_BASE_LENGTH_NAT : IPMI_RPLY_GET_SDR_BASE_LENGTH_VT) + recordSize;
int    offs         =   MCH_IS_NAT( mchData->mchSess->type ) ? IPMI_RPLY_OFFSET_NAT : 0;

	return ipmiMsgGetSdr( mchData->mchSess, mchData->ipmiSess, data, id, res, offset, readSize, parm, recordSize, &responseSize, offs );
}

/* Get Device Sensor Data Record (SDR) Info 
 *
 *   RETURNS: status from mchMsgWriteReadHelper
 *            0 on success
 *            non-zero for error
 */
int
mchMsgGetDevSdrInfo(MchData mchData, uint8_t *data, uint8_t parm)
{
size_t responseSize = 0;
int    offs         = MCH_IS_NAT( mchData->mchSess->type ) ? IPMI_RPLY_OFFSET_NAT : 0;

	return ipmiMsgGetDevSdrInfo( mchData->mchSess, mchData->ipmiSess, data, parm, &responseSize, offs );
}
	
/* Get Sensor Reading. Caller specifies expected message response length. 
 *
 *   RETURNS: status from mchMsgWriteReadHelper
 *            0 on success
 *            non-zero for error
 */
int
mchMsgReadSensor(MchData mchData, uint8_t *data, uint8_t sens, uint8_t lun, size_t *responseSize)
{
int offs = 0; /* doesn't seem to be used; look into this */

	return ipmiMsgReadSensor( mchData->mchSess, mchData->ipmiSess, data, sens, lun, responseSize, offs );
}

/* Set FRU Activation using Vadatech MCH - not used at this time
 *
 *   RETURNS: status from mchMsgWriteReadHelper
 *            0 on success
 *            non-zero for error
 */
int
mchMsgSetFruActVt(MchData mchData, uint8_t *data, uint8_t fru, uint8_t parm)
{
uint8_t  message[MSG_MAX_LENGTH] = { 0 };
size_t   imsg2Size   = sizeof( SEND_MSG_MSG );
size_t   b1msg1Size   = sizeof( IPMI_MSG1 );
size_t   b1msg2Size   = sizeof( SET_FRU_ACT_MSG );
uint8_t  imsg2[imsg2Size];
uint8_t  b1msg1[b1msg1Size];
uint8_t  b1msg2[b1msg2Size];
size_t   messageSize;
size_t   responseSize = IPMI_RPLY_SET_FRU_POLICY_LENGTH_VT;
uint8_t  cmd   = IPMI_MSG_CMD_SET_FRU_ACT;
uint8_t  netfn = IPMI_MSG_NETFN_PICMG;

	memcpy( imsg2, SEND_MSG_MSG   , imsg2Size );
	memcpy( b1msg1, IPMI_MSG1      , b1msg1Size );
	memcpy( b1msg2, SET_FRU_ACT_MSG, b1msg2Size );

	imsg2[IPMI_MSG2_CHAN_OFFSET] = IPMI_MSG_CHAN_IPMB0 + IPMI_MSG_TRACKING;

	b1msg1[IPMI_MSG1_RSADDR_OFFSET]   = IPMI_MSG_ADDR_CM;
	b1msg1[IPMI_MSG1_NETFNLUN_OFFSET] = netfn << 2;

	b1msg2[IPMI_MSG2_SET_FRU_ACT_FRU_OFFSET] = fru;
	b1msg2[IPMI_MSG2_SET_FRU_ACT_CMD_OFFSET] = parm;

	messageSize = ipmiMsgBuild( mchData->ipmiSess, message, cmd, 0, imsg2, imsg2Size, b1msg1, b1msg2, b1msg2Size, 0, 0, 0 );

	return mchMsgWriteReadHelper( mchData->mchSess, mchData->ipmiSess, message, messageSize, data, &responseSize, cmd, netfn, 0, 0 );
}

/* Set FRU Activation - For NAT MCH, used to deactivate/activate FRU
 *
 *
 *   RETURNS: status from mchMsgWriteReadHelper
 *            0 on success
 *            non-zero for error
 */
int
mchMsgSetFruActNat(MchData mchData, uint8_t *data, uint8_t fru, uint8_t parm)
{
uint8_t  message[MSG_MAX_LENGTH] = { 0 };
size_t   imsg2Size    = sizeof( SET_FRU_ACT_MSG );
uint8_t  imsg2[imsg2Size];
size_t   messageSize;
size_t   responseSize = 0;/*IPMI_RPLY_SET_FRU_ACT_LENGTH_NAT; - determine length */
uint8_t  cmd   = IPMI_MSG_CMD_SET_FRU_ACT;
uint8_t  netfn = IPMI_MSG_NETFN_PICMG;
int      offs = 0; /* determine needed offset */

	memcpy( imsg2, SET_FRU_ACT_MSG, imsg2Size );

	imsg2[IPMI_MSG2_RQADDR_OFFSET] = IPMI_MSG_ADDR_SW;
	imsg2[IPMI_MSG2_SET_FRU_ACT_FRU_OFFSET] = fru;
	imsg2[IPMI_MSG2_SET_FRU_ACT_CMD_OFFSET] = parm;

       	messageSize = ipmiMsgBuild( mchData->ipmiSess, message, cmd, &netfn, imsg2, imsg2Size, 0, 0, 0, 0, 0, 0 );

	return mchMsgWriteReadHelper( mchData->mchSess, mchData->ipmiSess, message, messageSize, data, &responseSize, cmd, netfn, offs, 0 );
}

/* Set FRU Activation Policy - For Vadatech MCH, used to deactivate/activate FRU
 *
 * (See ATCA spec 3.2.4.2.2)
 *
 * Activate version causes FRU to transition from "Inactive" to "Activation Request"
 * Deactivate version causes FRU to transition from "Active" to "Deactivation Request"
 * 
 *   RETURNS: status from mchMsgWriteReadHelper
 *            0 on success
 *            non-zero for error
 */
int 
mchMsgSetFruActPolicyVt(MchData mchData, uint8_t *data, uint8_t fru, uint8_t mask, uint8_t bits)
{
uint8_t  message[MSG_MAX_LENGTH] = { 0 };
size_t   imsg2Size   = sizeof( SEND_MSG_MSG );
size_t   b1msg1Size   = sizeof( IPMI_MSG1 );
size_t   b1msg2Size   = sizeof( SET_FRU_POLICY_MSG );
uint8_t  imsg2[imsg2Size];
uint8_t  b1msg1[b1msg1Size];
uint8_t  b1msg2[b1msg2Size];
size_t   messageSize;
size_t   responseSize = IPMI_RPLY_SET_FRU_POLICY_LENGTH_VT;
uint8_t  cmd   = IPMI_MSG_CMD_SET_FRU_POLICY;
uint8_t  netfn = IPMI_MSG_NETFN_PICMG;

	memcpy( imsg2, SEND_MSG_MSG   , imsg2Size );
	memcpy( b1msg1, IPMI_MSG1      , b1msg1Size );
	memcpy( b1msg2, SET_FRU_POLICY_MSG, b1msg2Size );

	imsg2[IPMI_MSG2_CHAN_OFFSET] = IPMI_MSG_CHAN_IPMB0 + IPMI_MSG_TRACKING;

	b1msg1[IPMI_MSG1_RSADDR_OFFSET]   = IPMI_MSG_ADDR_CM;
	b1msg1[IPMI_MSG1_NETFNLUN_OFFSET] = netfn << 2;

	b1msg2[IPMI_MSG2_SET_FRU_ACT_FRU_OFFSET] = fru;
	b1msg2[IPMI_MSG2_SET_FRU_POLICY_MASK_OFFSET] = mask;
	b1msg2[IPMI_MSG2_SET_FRU_POLICY_BITS_OFFSET] = bits;

	messageSize = ipmiMsgBuild( mchData->ipmiSess, message, cmd, 0, imsg2, imsg2Size, b1msg1, b1msg2, b1msg2Size, 0, 0, 0 );

	return mchMsgWriteReadHelper( mchData->mchSess, mchData->ipmiSess, message, messageSize, data, &responseSize, cmd, netfn, 0, 0 );
}

/* Set FRU Activation Policy using NAT MCH - not yet tested; may be single message like SetFruActNat
 *
 *   RETURNS: status from mchMsgWriteReadHelper
 *            0 on success
 *            non-zero for error
 */
int
mchMsgSetFruActPolicyNat(MchData mchData, uint8_t *data, uint8_t fru, uint8_t mask, uint8_t bits)
{
uint8_t  message[MSG_MAX_LENGTH] = { 0 };
size_t   imsg2Size    = sizeof( SEND_MSG_MSG );
size_t   b1msg1Size   = sizeof( IPMI_MSG1 );
size_t   b1msg2Size   = sizeof( SEND_MSG_MSG );
size_t   b2msg1Size   = sizeof( IPMI_MSG1 );
size_t   b2msg2Size   = sizeof( SET_FRU_POLICY_MSG );
uint8_t  imsg2[imsg2Size];
uint8_t  b1msg1[b1msg1Size];
uint8_t  b1msg2[b1msg2Size];
uint8_t  b2msg1[b2msg1Size];
uint8_t  b2msg2[b2msg2Size];
size_t   messageSize;
size_t   responseSize = IPMI_RPLY_SET_FRU_POLICY_LENGTH_NAT;
uint8_t  cmd          = IPMI_MSG_CMD_SET_FRU_POLICY;
uint8_t  netfn        = IPMI_MSG_NETFN_PICMG;
int      offs         = IPMI_RPLY_2ND_BRIDGED_OFFSET_NAT;

	memcpy( imsg2,  SEND_MSG_MSG , imsg2Size  );
	memcpy( b1msg1, IPMI_MSG1    , b1msg1Size );

	imsg2[IPMI_MSG2_CHAN_OFFSET]    = IPMI_MSG_CHAN_IPMB0 + IPMI_MSG_TRACKING;
	b1msg1[IPMI_MSG1_RSADDR_OFFSET] = IPMI_MSG_ADDR_CM;

	memcpy( b1msg2, SEND_MSG_MSG      , b1msg2Size );
	memcpy( b2msg1, IPMI_MSG1         , b2msg1Size );
	memcpy( b2msg2, SET_FRU_POLICY_MSG, b2msg2Size );

	b1msg2[IPMI_MSG2_RQADDR_OFFSET]   = IPMI_MSG_ADDR_BMC;
	b2msg1[IPMI_MSG1_NETFNLUN_OFFSET] = netfn << 2;
	b1msg2[IPMI_MSG2_CHAN_OFFSET]     = IPMI_MSG_CHAN_IPMBL + IPMI_MSG_TRACKING;
	b2msg1[IPMI_MSG1_RSADDR_OFFSET]   = FRU_I2C_ADDR[fru];
	b2msg2[IPMI_MSG2_SET_FRU_ACT_FRU_OFFSET] = 0; /* 0 for front; 0x80 for rear */
	b2msg2[IPMI_MSG2_SET_FRU_POLICY_MASK_OFFSET] = mask;
	b2msg2[IPMI_MSG2_SET_FRU_POLICY_BITS_OFFSET] = bits;

       	messageSize = ipmiMsgBuild( mchData->ipmiSess, message, cmd, 0, imsg2, imsg2Size, b1msg1, b1msg2, b1msg2Size, b2msg1, b2msg2, b2msg2Size );

	return mchMsgWriteReadHelper( mchData->mchSess, mchData->ipmiSess, message, messageSize, data, &responseSize, cmd, netfn, offs, 0 );
}

/* 
 * Based on parm (0 for Deactivate, 1 for Activate),
 * set appropriate mask and bits and call:
 *  ipmiMsgSetFruActPolicyVt for Vadatech MCH or 
 *  ipmiMsgSetFruActNat      for NAT MCH
 *
 * RETURNS:
 *         -1 if illegal parm value 
 *         or retrun value of ipmiMsgSetFruActPolicy (0 on success)
 */
int
mchMsgSetFruActHelper(MchData mchData, uint8_t *data, uint8_t fru, int parm) 
{
uint8_t mask;
uint8_t bits;

	mchMsgGetFruActPolicyHelper( mchData, data, fru );

	if ( parm == 0 ) {
		mask = 1 << 1; /* make this #define */
		bits = 0;
	}
	else if ( parm == 1 ){
		mask = 1;
		bits = 0;
	} 
	else
		return -1;

	if ( MCH_IS_NAT( mchData->mchSess->type ) )
		return mchMsgSetFruActNat( mchData, data, fru, parm );
	else
		return mchMsgSetFruActPolicyVt( mchData, data, fru, mask, bits );

}

/* Get FRU Activation Policy using Vadatech MCH; message contains 1 bridged message
 *
 *   RETURNS: status from mchMsgWriteReadHelper
 *            0 on success
 *            non-zero for error
 */
int
mchMsgGetFruActPolicyVt(MchData mchData, uint8_t *data, uint8_t fru)
{
uint8_t  message[MSG_MAX_LENGTH] = { 0 };
size_t   imsg2Size   = sizeof( SEND_MSG_MSG );
size_t   b1msg1Size   = sizeof( IPMI_MSG1 );
size_t   b1msg2Size   = sizeof( GET_FAN_PROP_MSG );
uint8_t  imsg2[imsg2Size];
uint8_t  b1msg1[b1msg1Size];
uint8_t  b1msg2[b1msg2Size];
size_t   messageSize;
size_t   responseSize = IPMI_RPLY_GET_FRU_POLICY_LENGTH_VT;
uint8_t  cmd   = IPMI_MSG_CMD_GET_FRU_POLICY;
uint8_t  netfn = IPMI_MSG_NETFN_PICMG;

	memcpy( imsg2,  SEND_MSG_MSG, imsg2Size );
	memcpy( b1msg1, IPMI_MSG1   , b1msg1Size );
	memcpy( b1msg2, GET_FRU_POLICY_MSG, b1msg2Size );

	imsg2[IPMI_MSG2_CHAN_OFFSET] = IPMI_MSG_CHAN_IPMB0 + IPMI_MSG_TRACKING;

	b1msg1[IPMI_MSG1_RSADDR_OFFSET]   = IPMI_MSG_ADDR_CM;
	b1msg1[IPMI_MSG1_NETFNLUN_OFFSET] = netfn << 2;

	b1msg2[IPMI_MSG2_SET_FRU_ACT_FRU_OFFSET] = fru;

	messageSize = ipmiMsgBuild( mchData->ipmiSess, message, cmd, 0, imsg2, imsg2Size, b1msg1, b1msg2, b1msg2Size, 0, 0, 0 );

	return mchMsgWriteReadHelper( mchData->mchSess, mchData->ipmiSess, message, messageSize, data, &responseSize, cmd, netfn, 0, 0 );
}


/* Get FRU Activation Policy using NAT MCH - not yet tested; may be single message like SetFruActNat
 *
 *   RETURNS: status from mchMsgWriteReadHelper
 *            0 on success
 *            non-zero for error
 */
int
mchMsgGetFruActPolicyNat(MchData mchData, uint8_t *data, uint8_t fru)
{
uint8_t  message[MSG_MAX_LENGTH] = { 0 };
size_t   imsg2Size    = sizeof( SEND_MSG_MSG );
size_t   b1msg1Size   = sizeof( IPMI_MSG1 );
size_t   b1msg2Size   = sizeof( SEND_MSG_MSG );
size_t   b2msg1Size   = sizeof( IPMI_MSG1 );
size_t   b2msg2Size   = sizeof( GET_FRU_POLICY_MSG );
uint8_t  imsg2[imsg2Size];
uint8_t  b1msg1[b1msg1Size];
uint8_t  b1msg2[b1msg2Size];
uint8_t  b2msg1[b2msg1Size];
uint8_t  b2msg2[b2msg2Size];
size_t   messageSize;
size_t   responseSize = IPMI_RPLY_GET_FRU_POLICY_LENGTH_NAT;
uint8_t  cmd          = IPMI_MSG_CMD_GET_FRU_POLICY;
uint8_t  netfn        = IPMI_MSG_NETFN_PICMG;
int      offs         = IPMI_RPLY_2ND_BRIDGED_OFFSET_NAT;

	memcpy( imsg2,  SEND_MSG_MSG , imsg2Size  );
	memcpy( b1msg1, IPMI_MSG1    , b1msg1Size );

	imsg2[IPMI_MSG2_CHAN_OFFSET]    = IPMI_MSG_CHAN_IPMB0 + IPMI_MSG_TRACKING;
	b1msg1[IPMI_MSG1_RSADDR_OFFSET] = IPMI_MSG_ADDR_CM;

	memcpy( b1msg2, SEND_MSG_MSG     , b1msg2Size );
	memcpy( b2msg1, IPMI_MSG1        , b2msg1Size );
	memcpy( b2msg2, GET_FRU_POLICY_MSG , b2msg2Size );

	b1msg2[IPMI_MSG2_RQADDR_OFFSET]   = IPMI_MSG_ADDR_BMC;
	b2msg1[IPMI_MSG1_NETFNLUN_OFFSET] = netfn << 2;
	b1msg2[IPMI_MSG2_CHAN_OFFSET]     = IPMI_MSG_CHAN_IPMBL + IPMI_MSG_TRACKING;
	b2msg1[IPMI_MSG1_RSADDR_OFFSET]   = FRU_I2C_ADDR[fru];
	b2msg2[IPMI_MSG2_SET_FRU_ACT_FRU_OFFSET] = 0;

       	messageSize = ipmiMsgBuild( mchData->ipmiSess, message, cmd, 0, imsg2, imsg2Size, b1msg1, b1msg2, b1msg2Size, b2msg1, b2msg2, b2msg2Size );

	return mchMsgWriteReadHelper( mchData->mchSess, mchData->ipmiSess, message, messageSize, data, &responseSize, cmd, netfn, offs, 0);
}

int
mchMsgGetFruActPolicyHelper(MchData mchData, uint8_t *data, uint8_t fru)
{
	if ( !( MCH_IS_NAT( mchData->mchSess->type )) )
		return mchMsgGetFruActPolicyVt( mchData, data, fru );
	else
		return mchMsgGetFruActPolicyNat( mchData, data, fru );

}

/* Get Device ID 
 *
 *   RETURNS: status from mchMsgWriteReadHelper
 *            0 on success
 *            non-zero for error
 */
int
mchMsgGetDeviceId(MchData mchData, uint8_t *data, uint8_t rsAddr)
{
size_t   responseSize = MCH_IS_NAT( mchData->mchSess->type ) ? IPMI_RPLY_GET_DEVICE_ID_LENGTH_NAT : IPMI_RPLY_GET_DEVICE_ID_LENGTH_VT;
int      offs         = MCH_IS_NAT( mchData->mchSess->type ) ? IPMI_RPLY_OFFSET_NAT : 0;

	return ipmiMsgGetDeviceId( mchData->mchSess, mchData->ipmiSess, data, rsAddr, &responseSize, offs );
}

/* Get Fan Speed Properties using Vadatech MCH; message contains 1 bridged message
 *
 *   RETURNS: status from mchMsgWriteReadHelper
 *            0 on success
 *            non-zero for error
 */
int
mchMsgGetFanPropVt(MchData mchData, uint8_t *data, uint8_t fru)
{
uint8_t  message[MSG_MAX_LENGTH] = { 0 };
size_t   imsg2Size   = sizeof( SEND_MSG_MSG );
size_t   b1msg1Size   = sizeof( IPMI_MSG1 );
size_t   b1msg2Size   = sizeof( GET_FAN_PROP_MSG );
uint8_t  imsg2[imsg2Size];
uint8_t  b1msg1[b1msg1Size];
uint8_t  b1msg2[b1msg2Size];
size_t   messageSize;
size_t   responseSize = IPMI_RPLY_GET_FAN_PROP_LENGTH_VT;
uint8_t  cmd   = IPMI_MSG_CMD_GET_FAN_PROP;
uint8_t  netfn = IPMI_MSG_NETFN_PICMG;

	memcpy( imsg2, SEND_MSG_MSG    , imsg2Size );
	memcpy( b1msg1, IPMI_MSG1       , b1msg1Size );
	memcpy( b1msg2, GET_FAN_PROP_MSG, b1msg2Size );

	imsg2[IPMI_MSG2_CHAN_OFFSET] = IPMI_MSG_CHAN_IPMB0 + IPMI_MSG_TRACKING;

	b1msg1[IPMI_MSG1_RSADDR_OFFSET]   = IPMI_MSG_ADDR_CM;
	b1msg1[IPMI_MSG1_NETFNLUN_OFFSET] = netfn << 2;

	b1msg2[IPMI_MSG2_SET_FRU_ACT_FRU_OFFSET] = fru;

	messageSize = ipmiMsgBuild( mchData->ipmiSess, message, cmd, 0, imsg2, imsg2Size, b1msg1, b1msg2, b1msg2Size, 0, 0, 0 );

	return mchMsgWriteReadHelper( mchData->mchSess, mchData->ipmiSess, message, messageSize, data, &responseSize, cmd, netfn, 0, 0 );
}


/* Get Fan Speed Properties using NAT MCH; message contains 2 bridged messages
 *
 *   RETURNS: status from mchMsgWriteReadHelper
 *            0 on success
 *            non-zero for error
 */
int
mchMsgGetFanPropNat(MchData mchData, uint8_t *data, uint8_t fru)
{
uint8_t  message[MSG_MAX_LENGTH] = { 0 };
size_t   imsg2Size    = sizeof( SEND_MSG_MSG );
size_t   b1msg1Size   = sizeof( IPMI_MSG1 );
size_t   b1msg2Size   = sizeof( SEND_MSG_MSG );
size_t   b2msg1Size   = sizeof( IPMI_MSG1 );
size_t   b2msg2Size   = sizeof( GET_FAN_PROP_MSG );
uint8_t  imsg2[imsg2Size];
uint8_t  b1msg1[b1msg1Size];
uint8_t  b1msg2[b1msg2Size];
uint8_t  b2msg1[b2msg1Size];
uint8_t  b2msg2[b2msg2Size];
size_t   messageSize;
size_t   responseSize = IPMI_RPLY_GET_FAN_PROP_LENGTH_NAT;
uint8_t  cmd          = IPMI_MSG_CMD_GET_FAN_PROP;
uint8_t  netfn        = IPMI_MSG_NETFN_PICMG;
int      offs         = IPMI_RPLY_2ND_BRIDGED_OFFSET_NAT;

	memcpy( imsg2,  SEND_MSG_MSG , imsg2Size  );
	memcpy( b1msg1, IPMI_MSG1    , b1msg1Size );

	imsg2[IPMI_MSG2_CHAN_OFFSET]    = IPMI_MSG_CHAN_IPMB0 + IPMI_MSG_TRACKING;
	b1msg1[IPMI_MSG1_RSADDR_OFFSET] = IPMI_MSG_ADDR_CM;

	memcpy( b1msg2, SEND_MSG_MSG     , b1msg2Size );
	memcpy( b2msg1, IPMI_MSG1        , b2msg1Size );
	memcpy( b2msg2, GET_FAN_PROP_MSG , b2msg2Size );

	b1msg2[IPMI_MSG2_RQADDR_OFFSET]   = IPMI_MSG_ADDR_BMC;
	b2msg1[IPMI_MSG1_NETFNLUN_OFFSET] = netfn << 2;
	b1msg2[IPMI_MSG2_CHAN_OFFSET]     = IPMI_MSG_CHAN_IPMBL + IPMI_MSG_TRACKING;
	b2msg1[IPMI_MSG1_RSADDR_OFFSET]   = FRU_I2C_ADDR[fru];
	b2msg2[IPMI_MSG2_SET_FRU_ACT_FRU_OFFSET] = 0;

       	messageSize = ipmiMsgBuild( mchData->ipmiSess, message, cmd, 0, imsg2, imsg2Size, b1msg1, b1msg2, b1msg2Size, b2msg1, b2msg2, b2msg2Size );

	return mchMsgWriteReadHelper( mchData->mchSess, mchData->ipmiSess, message, messageSize, data, &responseSize, cmd, netfn, offs, 0);
}

int
mchMsgGetFanPropHelper(MchData mchData, uint8_t *data, uint8_t fru)
{
	if ( MCH_IS_NAT( mchData->mchSess->type ) )
		return mchMsgGetFanPropNat( mchData, data, fru );
	else
		return mchMsgGetFanPropVt( mchData, data, fru );

}


/* Get Fan Level using Vadatech MCH; message contains 1 bridged message
 *
 *   RETURNS: status from mchMsgWriteReadHelper
 *            0 on success
 *            non-zero for error
 */
int
mchMsgGetFanLevelVt(MchData mchData, uint8_t *data, uint8_t fru)
{
uint8_t  message[MSG_MAX_LENGTH] = { 0 };
size_t   imsg2Size   = sizeof( SEND_MSG_MSG );
size_t   b1msg1Size   = sizeof( IPMI_MSG1 );
size_t   b1msg2Size   = sizeof( GET_FAN_LEVEL_MSG );
uint8_t  imsg2[imsg2Size];
uint8_t  b1msg1[b1msg1Size];
uint8_t  b1msg2[b1msg2Size];
size_t   messageSize;
size_t   responseSize = IPMI_RPLY_GET_FAN_LEVEL_LENGTH_VT;
uint8_t  cmd = IPMI_MSG_CMD_GET_FAN_LEVEL;
uint8_t  netfn = IPMI_MSG_NETFN_PICMG;

	memcpy( imsg2, SEND_MSG_MSG     , imsg2Size );
	memcpy( b1msg1, IPMI_MSG1        , b1msg1Size );
	memcpy( b1msg2, GET_FAN_LEVEL_MSG, b1msg2Size );

	imsg2[IPMI_MSG2_CHAN_OFFSET] = IPMI_MSG_CHAN_IPMB0 + IPMI_MSG_TRACKING;

	b1msg1[IPMI_MSG1_RSADDR_OFFSET]   = IPMI_MSG_ADDR_CM;
	b1msg1[IPMI_MSG1_NETFNLUN_OFFSET] = netfn << 2;

	b1msg2[IPMI_MSG2_SET_FRU_ACT_FRU_OFFSET] = fru;

	messageSize = ipmiMsgBuild( mchData->ipmiSess, message, cmd, 0, imsg2, imsg2Size, b1msg1, b1msg2, b1msg2Size, 0, 0, 0 );

	return mchMsgWriteReadHelper( mchData->mchSess, mchData->ipmiSess, message, messageSize, data, &responseSize, cmd, netfn, 0, 0 );
}

/* Get Fan Level using NAT MCH; message contains 2 bridged messages
 *
 *   RETURNS: status from mchMsgWriteReadHelper
 *            0 on success
 *            non-zero for error
 */
int
mchMsgGetFanLevelNat(MchData mchData, uint8_t *data, uint8_t fru)
{
uint8_t  message[MSG_MAX_LENGTH] = { 0 };
size_t   imsg2Size    = sizeof( SEND_MSG_MSG );
size_t   b1msg1Size   = sizeof( IPMI_MSG1 );
size_t   b1msg2Size   = sizeof( SEND_MSG_MSG );
size_t   b2msg1Size   = sizeof( IPMI_MSG1 );
size_t   b2msg2Size   = sizeof( GET_FAN_LEVEL_MSG );
uint8_t  imsg2[imsg2Size];
uint8_t  b1msg1[b1msg1Size];
uint8_t  b1msg2[b1msg2Size];
uint8_t  b2msg1[b2msg1Size];
uint8_t  b2msg2[b2msg2Size];
size_t   messageSize;
size_t   responseSize = IPMI_RPLY_GET_FAN_LEVEL_LENGTH_NAT;
uint8_t  cmd          = IPMI_MSG_CMD_GET_FAN_LEVEL;
uint8_t  netfn        = IPMI_MSG_NETFN_PICMG;
int      offs         = IPMI_RPLY_2ND_BRIDGED_OFFSET_NAT;

	memcpy( imsg2,  SEND_MSG_MSG , imsg2Size  );
	memcpy( b1msg1, IPMI_MSG1    , b1msg1Size );

	imsg2[IPMI_MSG2_CHAN_OFFSET]    = IPMI_MSG_CHAN_IPMB0 + IPMI_MSG_TRACKING;
	b1msg1[IPMI_MSG1_RSADDR_OFFSET] = IPMI_MSG_ADDR_CM;

	memcpy( b1msg2, SEND_MSG_MSG      , b1msg2Size );
	memcpy( b2msg1, IPMI_MSG1         , b2msg1Size );
	memcpy( b2msg2, GET_FAN_LEVEL_MSG , b2msg2Size );

	b1msg2[IPMI_MSG2_RQADDR_OFFSET]   = IPMI_MSG_ADDR_BMC;
	b2msg1[IPMI_MSG1_NETFNLUN_OFFSET] = netfn << 2;
	b1msg2[IPMI_MSG2_CHAN_OFFSET]     = IPMI_MSG_CHAN_IPMBL + IPMI_MSG_TRACKING;
	b2msg1[IPMI_MSG1_RSADDR_OFFSET]   = FRU_I2C_ADDR[fru];
	b2msg2[IPMI_MSG2_SET_FRU_ACT_FRU_OFFSET] = 0;

       	messageSize = ipmiMsgBuild( mchData->ipmiSess, message, cmd, 0, imsg2, imsg2Size, b1msg1, b1msg2, b1msg2Size, b2msg1, b2msg2, b2msg2Size );

	return mchMsgWriteReadHelper( mchData->mchSess, mchData->ipmiSess, message, messageSize, data, &responseSize, cmd, netfn, offs, 0 );
}

int
mchMsgGetFanLevelHelper(MchData mchData, uint8_t *data, uint8_t fru)
{
	if ( MCH_IS_NAT( mchData->mchSess->type ) )
		return mchMsgGetFanLevelNat( mchData, data, fru );
	else
		return mchMsgGetFanLevelVt(  mchData, data, fru );

}

/* Set Fan Level using Vadatech MCH; message contains 1 bridged message 
 *
 *   RETURNS: status from mchMsgWriteReadHelper
 *            0 on success
 *            non-zero for error
 */
int
mchMsgSetFanLevelVt(MchData mchData, uint8_t *data, uint8_t fru, uint8_t level)
{
uint8_t  message[MSG_MAX_LENGTH] = { 0 };
size_t   imsg2Size   = sizeof( SEND_MSG_MSG );
size_t   b1msg1Size   = sizeof( IPMI_MSG1 );
size_t   b1msg2Size   = sizeof( SET_FAN_LEVEL_MSG );
uint8_t  imsg2[imsg2Size];
uint8_t  b1msg1[b1msg1Size];
uint8_t  b1msg2[b1msg2Size];
size_t   messageSize;
size_t   responseSize = IPMI_RPLY_SET_FAN_LEVEL_LENGTH_VT;
uint8_t  cmd   = IPMI_MSG_CMD_SET_FAN_LEVEL;
uint8_t  netfn = IPMI_MSG_NETFN_PICMG;

	memcpy( imsg2, SEND_MSG_MSG     , imsg2Size );
	memcpy( b1msg1, IPMI_MSG1        , b1msg1Size );
	memcpy( b1msg2, SET_FAN_LEVEL_MSG, b1msg2Size );

	imsg2[IPMI_MSG2_CHAN_OFFSET] = IPMI_MSG_CHAN_IPMB0 + IPMI_MSG_TRACKING;

	b1msg1[IPMI_MSG1_RSADDR_OFFSET]   = IPMI_MSG_ADDR_CM;
	b1msg1[IPMI_MSG1_NETFNLUN_OFFSET] = netfn << 2;

	b1msg2[IPMI_MSG2_SET_FRU_ACT_FRU_OFFSET] = fru;
	b1msg2[IPMI_MSG2_SET_FAN_LEVEL_LEVEL_OFFSET] = level;

	messageSize = ipmiMsgBuild( mchData->ipmiSess, message, cmd, 0, imsg2, imsg2Size, b1msg1, b1msg2, b1msg2Size, 0, 0, 0 );

	return mchMsgWriteReadHelper( mchData->mchSess, mchData->ipmiSess, message, messageSize, data, &responseSize, cmd, netfn, 0, 0 );
}

/* Set Fan Level using NAT MCH; message contains 2 bridged messages
 *
 *   RETURNS: status from mchMsgWriteReadHelper
 *            0 on success
 *            non-zero for error
 */
int
mchMsgSetFanLevelNat(MchData mchData, uint8_t *data, uint8_t fru, uint8_t level)
{
uint8_t  message[MSG_MAX_LENGTH] = { 0 };
size_t   imsg2Size    = sizeof( SEND_MSG_MSG );
size_t   b1msg1Size   = sizeof( IPMI_MSG1 );
size_t   b1msg2Size   = sizeof( SEND_MSG_MSG );
size_t   b2msg1Size   = sizeof( IPMI_MSG1 );
size_t   b2msg2Size   = sizeof( SET_FAN_LEVEL_MSG );
uint8_t  imsg2[imsg2Size];
uint8_t  b1msg1[b1msg1Size];
uint8_t  b1msg2[b1msg2Size];
uint8_t  b2msg1[b2msg1Size];
uint8_t  b2msg2[b2msg2Size];
size_t   messageSize;
size_t   responseSize = IPMI_RPLY_SET_FAN_LEVEL_LENGTH_NAT;
uint8_t  cmd          = IPMI_MSG_CMD_GET_FAN_LEVEL;
uint8_t  netfn        = IPMI_MSG_NETFN_PICMG;
int      offs         = IPMI_RPLY_2ND_BRIDGED_OFFSET_NAT;

	memcpy( imsg2,  SEND_MSG_MSG , imsg2Size  );
	memcpy( b1msg1, IPMI_MSG1    , b1msg1Size );

	imsg2[IPMI_MSG2_CHAN_OFFSET]    = IPMI_MSG_CHAN_IPMB0 + IPMI_MSG_TRACKING;
	b1msg1[IPMI_MSG1_RSADDR_OFFSET] = IPMI_MSG_ADDR_CM;

	memcpy( b1msg2, SEND_MSG_MSG      , b1msg2Size );
	memcpy( b2msg1, IPMI_MSG1         , b2msg1Size );
	memcpy( b2msg2, SET_FAN_LEVEL_MSG , b2msg2Size );

	b1msg2[IPMI_MSG2_RQADDR_OFFSET]   = IPMI_MSG_ADDR_BMC;
	b2msg1[IPMI_MSG1_NETFNLUN_OFFSET] = netfn << 2;
	b1msg2[IPMI_MSG2_CHAN_OFFSET]     = IPMI_MSG_CHAN_IPMBL + IPMI_MSG_TRACKING;
	b2msg1[IPMI_MSG1_RSADDR_OFFSET]   = FRU_I2C_ADDR[fru];
	b2msg2[IPMI_MSG2_SET_FRU_ACT_FRU_OFFSET] = 0;
	b2msg2[IPMI_MSG2_SET_FAN_LEVEL_LEVEL_OFFSET] = level;

	messageSize = ipmiMsgBuild( mchData->ipmiSess, message, cmd, 0, imsg2, imsg2Size, b1msg1, b1msg2, b1msg2Size, b2msg1, b2msg2, b2msg2Size );

	return mchMsgWriteReadHelper( mchData->mchSess, mchData->ipmiSess, message, messageSize, data, &responseSize, cmd, netfn, offs, 0 );
}

int
mchMsgSetFanLevelHelper(MchData mchData, uint8_t *data, uint8_t fru, uint8_t level)
{
	if ( MCH_IS_NAT( mchData->mchSess->type ) )
		return mchMsgSetFanLevelNat( mchData, data, fru, level );
	else
		return mchMsgSetFanLevelVt( mchData, data, fru, level );

}

/* Get FRU Power Level (NAT does not support this command)
 *
 *   RETURNS: status from mchMsgWriteReadHelper
 *            0 on success
 *            non-zero for error
 */
int
mchMsgGetPowerLevelVt(MchData mchData, uint8_t *data, uint8_t fru, uint8_t parm)
{
uint8_t  message[MSG_MAX_LENGTH] = { 0 };
size_t   imsg2Size   = sizeof( SEND_MSG_MSG );
size_t   b1msg1Size   = sizeof( IPMI_MSG1 );
size_t   b1msg2Size   = sizeof( GET_POWER_LEVEL_MSG );
uint8_t  imsg2[imsg2Size];
uint8_t  b1msg1[b1msg1Size];
uint8_t  b1msg2[b1msg2Size];
size_t   messageSize;
size_t   responseSize = 0; /* determine responseSize */
uint8_t  cmd = IPMI_MSG_CMD_GET_POWER_LEVEL;
uint8_t  netfn = IPMI_MSG_NETFN_PICMG;

	memcpy( imsg2,  SEND_MSG_MSG       , imsg2Size );
	memcpy( b1msg1, IPMI_MSG1          , b1msg1Size );
	memcpy( b1msg2, GET_POWER_LEVEL_MSG, b1msg2Size );

	imsg2[IPMI_MSG2_CHAN_OFFSET] = IPMI_MSG_CHAN_IPMB0 + IPMI_MSG_TRACKING;

	b1msg1[IPMI_MSG1_RSADDR_OFFSET]   = IPMI_MSG_ADDR_CM;
	b1msg1[IPMI_MSG1_NETFNLUN_OFFSET] = netfn << 2;

	b1msg2[IPMI_MSG2_SET_FRU_ACT_FRU_OFFSET] = fru;
	b1msg2[IPMI_MSG2_GET_POWER_LEVEL_TYPE_OFFSET] = parm;

	messageSize = ipmiMsgBuild( mchData->ipmiSess, message, cmd, 0, imsg2, imsg2Size, b1msg1, b1msg2, b1msg2Size, 0, 0, 0 );

	return mchMsgWriteReadHelper( mchData->mchSess, mchData->ipmiSess, message, messageSize, data, &responseSize, cmd, netfn, 0, 0 );
}
