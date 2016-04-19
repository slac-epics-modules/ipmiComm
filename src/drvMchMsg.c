#include <errlog.h>
#include <epicsMutex.h>
#include <asynDriver.h>
#include <asynOctetSyncIO.h>

#include <stdint.h>
#include <string.h>

#include <ipmiMsg.h>
#include <ipmiDef.h>
#include <picmgDef.h>
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
int      ipmiSeqOffs;
uint8_t  seq[4];
uint32_t seqInt;
uint32_t seqRplyInt;
size_t   responseLen;

	ipmiSeqOffs = ( IPMI_MSG_AUTH_TYPE_NONE == message[RMCP_MSG_HEADER_LENGTH+IPMI_WRAPPER_AUTH_TYPE_OFFSET] ) ?
		RMCP_MSG_HEADER_LENGTH + IPMI_WRAPPER_LENGTH + IPMI_MSG1_LENGTH + IPMI_MSG2_SEQLUN_OFFSET          :
		RMCP_MSG_HEADER_LENGTH + IPMI_WRAPPER_AUTH_LENGTH + IPMI_MSG1_LENGTH + IPMI_MSG2_SEQLUN_OFFSET;

	/* seems we check this twice for sensor reads -- look into this */       	
	if ( !MCH_ONLN( mchStat[inst] ) )
		return -1;

       	status = ipmiMsgWriteRead( mchSess->name, message, messageSize, response, responseSize, mchSess->timeout, &responseLen );

	if ( MCH_DBG( mchStat[inst] ) >= MCH_DBG_MED ) {

		if ( MCH_DBG( mchStat[inst] ) >= MCH_DBG_HIGH ) {
			printf("%s Message status %i, received %i, expected %i, raw data:\n", mchSess->name, status, (int)responseLen, *(int *)responseSize );
			for ( i = 0; i < responseLen; i++ )
				printf("%02x ", response[i]);
			printf("\n");
		}
		else if ( status )
			printf("%s Message status %i, received %i, expected %i\n", mchSess->name, status, (int)responseLen, *(int *)responseSize);
	}

	*responseSize = responseLen; /* Pass actual response length back to caller */

	if ( outSess ) {
		if ( (code = response[codeOffs]/*IPMI_RPLY_COMPLETION_CODE_OFFSET + auth_offset]*/) && MCH_DBG( mchStat[inst] ) )
			ipmiCompletionCode( mchSess->name, code, cmd, netfn );
	       	return code;
	}

       	if ( mchSess->err > 9 ) {
       		if ( MCH_DBG( mchStat[inst] ) )
       			printf("%s start new session; err count is %i\n", mchSess->name, mchSess->err);

       		/* Reset error count to 0 */
       		mchSess->err = 0;

       		/* Close current session, start new one, and return error */
       		mchMsgCloseSess( mchSess, ipmiSess, response );
       		mchNewSession( mchSess, ipmiSess );
       	       	return -1;
       	}

	if ( responseLen == 0 ) {
		mchSess->err++;
		return -1;
	}

       	/* Verify IPMI message sequence number. If incorrect, increment error count and return error */
       	ipmiSeq = IPMI_SEQLUN_EXTRACT_SEQ(response[ipmiSeqOffs]);
       	if ( ipmiSeq != ipmiSess->seq ) {
	       	if ( MCH_DBG( mchStat[inst] ) )
	       		printf("%s Incorrect IPMI sequence; got %i but expected %i\n", mchSess->name, ipmiSeq, ipmiSess->seq );
	       	mchSess->err++;
	       	return -1;
       	}

	/* Extract session sequence number from reply */
	for ( i = 0; i < IPMI_RPLY_SEQ_LENGTH ; i++)
       	       	seq[i] = response[RMCP_MSG_HEADER_LENGTH + IPMI_WRAPPER_SEQ_OFFSET + i]; /* bridged offset ? */
	       
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
		if ( MCH_DBG( mchStat[inst] ) >= MCH_DBG_MED )
	       		printf("%s sequence number %i, previous %i\n", mchSess->name, seqInt, seqRplyInt);
	       	incr4Uint8Array( ipmiSess->seqRply, 1 );
		mchSess->err++;
		return -1;
       	}
       	else {
		for ( i = 0; i < IPMI_RPLY_SEQ_LENGTH ; i++)
			ipmiSess->seqRply[i] = seq[i];

		if ( (code = response[/*IPMI_RPLY_COMPLETION_CODE_OFFSET*/ codeOffs]/*codeOffs + auth_offset]*/) ) {
			if ( MCH_DBG( mchStat[inst] ) )
				ipmiCompletionCode( mchSess->name, code, cmd, netfn );
			//mchSess->err++; // only increment error count for some errors ? --not parameter out of range, for example
			return code;
		}
	}

	mchSess->err = 0;
	return 0;
}

/* Only relevant for messages in a session. Do not call for messages preceding a session. Do this so we can use authReq to determine auth type. */
static void
mchSetSizeOffs(IpmiSess ipmiSess, size_t payloadSize, size_t *roffs, size_t *responseSize, int *bridged, uint8_t *rsAddr, uint8_t *rqAddr)
{
int authOffs, offs;

	/* This section needs to be completely figured out (Vadatech) */
	if ( ipmiSess->features & MCH_FEAT_SENDMSG_RPLY ) {
		// note that we used to only check comp code of send msg reply (not payload reply) which is different than other devices; may need to revert to that
		offs = IPMI_RPLY_BRIDGED_2REPLY_OFFSET;
		*responseSize += IPMI_RPLY_HEADER_LENGTH + offs;
		*bridged = 1;
		*rsAddr = IPMI_MSG_ADDR_CM;
		*rqAddr = IPMI_MSG_ADDR_BMC;
	}
	else {
		offs = 0;
		*responseSize += IPMI_RPLY_HEADER_LENGTH;
		*bridged = 0;
		*rsAddr  = 0;
		*rqAddr  = 0;
	}


	if ( ipmiSess->authReq != IPMI_MSG_AUTH_TYPE_NONE ) {
		authOffs = sizeof( IPMI_WRAPPER_PWD_KEY ) - sizeof( IPMI_WRAPPER );
		offs += authOffs;
		*responseSize += authOffs;
	}

	*roffs = *responseSize;  /* Offset into packet our section of interest */
	/* If payloadSize is 0, it means the response length is variable or unknown;
         * Set responseSize to 0 to indicate this to write/read function */         
	*responseSize += payloadSize + FOOTER_LENGTH;

	return;
}

int
mchMsgCheckSizes(size_t destSize, int offset, size_t srcSize)
{
	if ( 0 >= srcSize ) {
	  printf("mchMsgCheckSizes: message size %i less than zero\n", (int)srcSize);
		return -1;
	}
	else if ( (offset + srcSize) > destSize ) {
	  printf("mchMsgCheckSizes: copy size %i larger than destination %i \n", offset+(int)srcSize, (int)destSize);
		return -1;
	}
	
	return 0;
}	

/* Somewhere need to state that all calls that pass in message buffer must use buffer size of MSG_MAX_LENGTH
-- or just make caller specify buffer size? */

/* Get Channel Authentication Capabilities
 *
 *   RETURNS: status from ipmiMsgGetChanAuth
 *            0 on success
 *            non-zero for error
 */
int
mchMsgGetChanAuth(MchSess mchSess, IpmiSess ipmiSess, uint8_t *data)
{
uint8_t  response[MSG_MAX_LENGTH] = { 0 }; 
size_t   roffs, responseSize, payloadSize = IPMI_RPLY_IMSG2_GET_CHAN_AUTH_LENGTH; 
int      rval;

	roffs = IPMI_RPLY_HEADER_LENGTH;
	responseSize = (payloadSize==0) ? 0 : roffs + payloadSize + FOOTER_LENGTH;

	if ( (rval = ipmiMsgGetChanAuth( mchSess, ipmiSess, response, &responseSize, roffs )) )
		goto bail;

	if ( (rval = mchMsgCheckSizes( sizeof( response ), roffs, payloadSize )) ) {
		printf("mchMsgGetChanAuth size error\n");
		goto bail;
	}

	memcpy( data, response + roffs, payloadSize );

bail:
	return rval;
}

/* Temporary workaround for using different usernames across different devices */
int
mchMsgGetSess(MchSess mchSess, IpmiSess ipmiSess, uint8_t *data)
{
uint8_t response[MSG_MAX_LENGTH] = { 0 }; 
size_t  size = sizeof( GET_SESS_MSG );
uint8_t msg[size]; 
size_t  roffs, responseSize, payloadSize = IPMI_RPLY_IMSG2_GET_SESSION_CHALLENGE_LENGTH; 
int     rval;

	roffs = IPMI_RPLY_HEADER_LENGTH;
	/* If payloadSize is 0, it means the response length is variable or unknown;
         * Set responseSize to 0 to indicate this to write/read function */         
	responseSize = (payloadSize==0) ? 0 : roffs + payloadSize + FOOTER_LENGTH;

	switch ( ipmiSess->authReq ) {

		default:
			printf("mchMsgGetSess: unsupported authentication type %i\n", ipmiSess->authReq );
			return 0;

		case IPMI_MSG_AUTH_TYPE_NONE:
			memcpy( msg, GET_SESS_MSG, sizeof( GET_SESS_MSG ) );
			break;

		case IPMI_MSG_AUTH_TYPE_PWD_KEY:
			memcpy( msg, GET_SESS_MSG_PWD_KEY, sizeof( GET_SESS_MSG_PWD_KEY )  );
			break;
	}

	if ( (rval = ipmiMsgGetSess( mchSess, ipmiSess, response, &responseSize, msg, roffs )) )
		goto bail;

	if ( (rval = mchMsgCheckSizes( sizeof( response ), roffs, payloadSize )) ) {
		printf("mchMsgGetSess size error\n");
		goto bail;
	}
	memcpy( data, response + roffs, payloadSize );

bail:
	return rval;
}

/* Activate Session
 *
 *   RETURNS: status from ipmiMsgActSess
 *            0 on success
 *            non-zero for error
 */
int
mchMsgActSess(MchSess mchSess, IpmiSess ipmiSess, uint8_t *data)
{
uint8_t  response[MSG_MAX_LENGTH] = { 0 }; 
size_t   roffs, responseSize = 0, payloadSize = IPMI_RPLY_IMSG2_ACTIVATE_SESSION_LENGTH; 
int      rval, bridged;
uint8_t  rsAddr, rqAddr;

	mchSetSizeOffs( ipmiSess, payloadSize, &roffs, &responseSize, &bridged, &rsAddr, &rqAddr );

	if ( (rval = ipmiMsgActSess( mchSess, ipmiSess, response, &responseSize, roffs )) )
		goto bail;

	if ( (rval = mchMsgCheckSizes( sizeof( response ), roffs, payloadSize )) ) {
		printf("mchMsgActSess size error\n");
		goto bail;
	}
	memcpy( data, response + roffs, payloadSize );

bail:
	return rval;
}

/* Set Session Privilege level
 *
 *   RETURNS: status from ipmiMsgSetPriv
 *            0 on success
 *            non-zero for error
 */
int
mchMsgSetPriv(MchSess mchSess, IpmiSess ipmiSess, uint8_t *data, uint8_t level)
{
uint8_t  response[MSG_MAX_LENGTH] = { 0 }; 
size_t   roffs, responseSize = 0, payloadSize = IPMI_RPLY_IMSG2_SET_PRIV_LEVEL_LENGTH; 
int      rval, bridged;
uint8_t  rsAddr, rqAddr;

	mchSetSizeOffs( ipmiSess, payloadSize, &roffs, &responseSize, &bridged, &rsAddr, &rqAddr );

	if ( (rval = ipmiMsgSetPriv( mchSess, ipmiSess, response, &responseSize, level, roffs )) )
		goto bail;

	if ( (rval = mchMsgCheckSizes( sizeof( response ), roffs, payloadSize )) ) {
		printf("mchMsgSetPriv size error\n");
		goto bail;
	}
	memcpy( data, response + roffs, payloadSize );

bail:
	return rval;
}

/* Get Device ID -- this is called before we know device type -- need to update this
 *
 *   RETURNS: status from mchMsgWriteReadHelper
 *            0 on success
 *            non-zero for error
 */
int
mchMsgGetDeviceId(MchData mchData, uint8_t *data, int tmp, uint8_t tmp1 ) //int bridged, uint8_t rsAddr) // change this to either never be bridged or to implement device checking and offsets; handle rqAddr too
{
uint8_t  response[MSG_MAX_LENGTH] = { 0 }; 
size_t   roffs, responseSize = 0, payloadSize = IPMI_RPLY_IMSG2_GET_DEVICE_ID_LENGTH; 
int      rval, bridged;
uint8_t  rsAddr, rqAddr;

	mchSetSizeOffs( mchData->ipmiSess, payloadSize, &roffs, &responseSize, &bridged, &rsAddr, &rqAddr);

	if ( (rval = ipmiMsgGetDeviceId( mchData->mchSess, mchData->ipmiSess, response, bridged, rsAddr, rqAddr, &responseSize, roffs )) )
		goto bail;

	if ( (rval = mchMsgCheckSizes( sizeof( response ), roffs, payloadSize )) ) {
		printf("mchMsgGetDeviceId size error\n");
		goto bail;
	}
	memcpy( data, response + roffs, payloadSize );

bail:
	return rval;
}

/* Close Session - test for NAT and determine reply offset*/
int
mchMsgCloseSess(MchSess mchSess, IpmiSess ipmiSess, uint8_t *data)
{
size_t responseSize = MCH_IS_NAT( mchSess->type ) ? IPMI_RPLY_CLOSE_SESSION_LENGTH_NAT : IPMI_RPLY_CLOSE_SESSION_LENGTH_VT;

	return ipmiMsgCloseSess( mchSess, ipmiSess, data, &responseSize /* need to add roffs */ );
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
uint8_t  response[MSG_MAX_LENGTH] = { 0 }; 
size_t   roffs, responseSize = 0, payloadSize = IPMI_RPLY_IMSG2_CHAS_CTRL_LENGTH; 
int      rval, bridged;
uint8_t  rsAddr, rqAddr;

	mchSetSizeOffs( mchData->ipmiSess, payloadSize, &roffs, &responseSize, &bridged, &rsAddr, &rqAddr );

	if ( (rval = ipmiMsgChassisControl( mchData->mchSess, mchData->ipmiSess, response, bridged, rsAddr, rqAddr, parm, &responseSize, roffs )) )
		goto bail;

	if ( (rval = mchMsgCheckSizes( sizeof( response ), roffs, payloadSize )) ) {
		printf("mchMsgChassisControl size error\n");
		goto bail;
	}
	memcpy( data, response + roffs, payloadSize );

bail:
	return rval;
}

/* Get Chassis Status
 *
 *   RETURNS: status from ipmiMsgGetChassisStatus
 *            0 on success
 *            non-zero for error
 */
int
mchMsgGetChassisStatus(MchData mchData, uint8_t *data)
{
uint8_t  response[MSG_MAX_LENGTH] = { 0 }; 
size_t   roffs, responseSize = 0, payloadSize = IPMI_RPLY_IMSG2_GET_CHAS_STATUS_LENGTH; 
int      rval, bridged;
uint8_t  rsAddr, rqAddr;

/* Not supported by all devices (of course); workaround for now: */

	if ( 0 == MCH_IS_SUPERMICRO( mchData->mchSess->type ) )
		return -1;


	mchSetSizeOffs( mchData->ipmiSess, payloadSize, &roffs, &responseSize, &bridged, &rsAddr, &rqAddr );

	if ( (rval = ipmiMsgGetChassisStatus( mchData->mchSess, mchData->ipmiSess, response, bridged, rsAddr, rqAddr, &responseSize, roffs )) )
		goto bail;

	if ( (rval = mchMsgCheckSizes( sizeof( response ), roffs, payloadSize )) ) {
		printf("mchMsgGetChassisStatus size error\n");
		goto bail;
	}
	memcpy( data, response + roffs, payloadSize );

bail:
	return rval;
}

/* Get FRU Inventory Info 
 *
 *   RETURNS: status from ipmiMsgGetFruInfo
 *            0 on success
 *            non-zero for error
 */
int
mchMsgGetFruInvInfo(MchData mchData, uint8_t *data, uint8_t id)
{
uint8_t  response[MSG_MAX_LENGTH] = { 0 }; 
size_t   roffs, responseSize = 0, payloadSize = IPMI_RPLY_IMSG2_GET_FRU_INV_INFO_LENGTH; 
int      rval, bridged;
uint8_t  rsAddr, rqAddr;

	mchSetSizeOffs( mchData->ipmiSess, payloadSize, &roffs, &responseSize, &bridged, &rsAddr, &rqAddr );

	if ( (rval = ipmiMsgGetFruInvInfo( mchData->mchSess, mchData->ipmiSess, response, bridged, rsAddr, rqAddr, id, &responseSize, roffs )) )
		goto bail;

	if ( (rval = mchMsgCheckSizes( sizeof( response ), roffs, payloadSize )) ) {
		printf("mchMsgGetFruInvInfo size error\n");
		goto bail;
	}

	memcpy( data, response + roffs, payloadSize );

bail:
	return rval;
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
uint8_t  response[MSG_MAX_LENGTH] = { 0 }; 
size_t   roffs, responseSize = 0, payloadSize = IPMI_RPLY_IMSG2_READ_FRU_DATA_BASE_LENGTH + readSize; 
int      rval, bridged;
uint8_t  rsAddr, rqAddr;

	mchSetSizeOffs( mchData->ipmiSess, payloadSize, &roffs, &responseSize, &bridged, &rsAddr, &rqAddr );

	if ( (rval = ipmiMsgReadFru( mchData->mchSess, mchData->ipmiSess, response, bridged, rsAddr, rqAddr, id, readOffset, readSize, &responseSize, roffs )) )
		goto bail;

	if ( (rval = mchMsgCheckSizes( sizeof( response ), roffs, payloadSize )) ) {
		printf("mchMsgReadFru size error\n");
		goto bail;
	}

	memcpy( data, response + roffs, payloadSize );

bail:
	return rval;
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
uint8_t  response[MSG_MAX_LENGTH] = { 0 }; 
size_t   roffs, responseSize = 0, payloadSize = IPMI_RPLY_IMSG2_GET_SDRREP_INFO_LENGTH; 
int      rval, bridged;
uint8_t  rsAddr, rqAddr;

	mchSetSizeOffs( mchData->ipmiSess, payloadSize, &roffs, &responseSize, &bridged, &rsAddr, &rqAddr );

	if ( (rval = ipmiMsgGetSdrRepInfo( mchData->mchSess, mchData->ipmiSess, response, bridged, rsAddr, rqAddr, &responseSize, roffs )) )
		goto bail;

	if ( (rval = mchMsgCheckSizes( sizeof( response ), roffs, payloadSize )) ) {
	  printf("mchMsgGetSdrRepInfo size error roffs %i payloadSize %i responseSize %i\n", (int)roffs, (int)payloadSize, (int)responseSize);
		goto bail;
	}

	memcpy( data, response + roffs, payloadSize );

bail:
	return rval;
}

/* Reserve SDR (Sensor Data Record) Repository
 *
 *   RETURNS: status from mchMsgWriteReadHelper
 *            0 on success
 *            non-zero for error
 */
int
mchMsgReserveSdrRep(MchData mchData, uint8_t *data)
{
uint8_t  response[MSG_MAX_LENGTH] = { 0 }; 
size_t   roffs, responseSize = 0, payloadSize = IPMI_RPLY_IMSG2_RESERVE_SDRREP_LENGTH; 
int      rval, bridged;
uint8_t  rsAddr, rqAddr;

	mchSetSizeOffs( mchData->ipmiSess, payloadSize, &roffs, &responseSize, &bridged, &rsAddr, &rqAddr );

	if ( (rval = ipmiMsgReserveSdrRep( mchData->mchSess, mchData->ipmiSess, response, bridged, rsAddr, rqAddr, &responseSize, roffs )) )
		goto bail;

	if ( (rval = mchMsgCheckSizes( sizeof( response ), roffs, payloadSize )) ) {
		printf("mchMsgReserveSdrRep size error\n");
		goto bail;
	}

	memcpy( data, response + roffs, payloadSize );

bail:
	return rval;
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
uint8_t  response[MSG_MAX_LENGTH] = { 0 }; 
uint8_t  sdrDataSize = ( readSize == 0xFF ) ? SDR_MAX_LENGTH : readSize;
size_t   roffs, responseSize = 0, payloadSize = IPMI_RPLY_IMSG2_GET_SDR_BASE_LENGTH + sdrDataSize; 
int      rval, bridged;
uint8_t  rsAddr, rqAddr;
    
	mchSetSizeOffs( mchData->ipmiSess, payloadSize, &roffs, &responseSize, &bridged, &rsAddr, &rqAddr );

	if ( (rval = ipmiMsgGetSdr( mchData->mchSess, mchData->ipmiSess, response, bridged, rsAddr, rqAddr, id, res, offset, readSize, parm, recordSize, &responseSize, roffs )) )
		goto bail;

	if ( (rval = mchMsgCheckSizes( sizeof( response ), roffs, payloadSize )) ) {
		printf("mchMsgGetSdr size error\n");
		goto bail;
	}
 
	memcpy( data, response + roffs, payloadSize );

bail:
	return rval;
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
uint8_t  response[MSG_MAX_LENGTH] = { 0 }; 
size_t   roffs, responseSize = 0, payloadSize = IPMI_RPLY_IMSG2_GET_DEV_SDR_INFO_LENGTH; 
int      rval, bridged;
uint8_t  rsAddr, rqAddr;

	mchSetSizeOffs( mchData->ipmiSess, payloadSize, &roffs, &responseSize, &bridged, &rsAddr, &rqAddr );

	if ( (rval = ipmiMsgGetDevSdrInfo( mchData->mchSess, mchData->ipmiSess, response, bridged, rsAddr, rqAddr, parm, &responseSize, roffs )) )
		goto bail;

	if ( (rval = mchMsgCheckSizes( sizeof( response ), roffs, payloadSize )) ) {
		printf("mchMsgGetDevSdrInfo size error\n");
		goto bail;
	}

	memcpy( data, response + roffs, payloadSize );

bail:
	return rval;
}

/* Get Sensor Reading. Caller specifies expected message response length. 
 *
 *   RETURNS: status from mchMsgWriteReadHelper
 *            0 on success
 *            non-zero for error
 */
int
mchMsgReadSensor(MchData mchData, uint8_t *data, uint8_t sens, uint8_t lun, size_t *sensReadMsgSize)
{
uint8_t  response[MSG_MAX_LENGTH] = { 0 }; 
size_t   roffs, responseSize = 0, payloadSize = *sensReadMsgSize; 
int      rval, bridged;
uint8_t  rsAddr, rqAddr;

	mchSetSizeOffs( mchData->ipmiSess, payloadSize, &roffs, &responseSize, &bridged, &rsAddr, &rqAddr );

	if ( (rval = ipmiMsgReadSensor( mchData->mchSess, mchData->ipmiSess, response, bridged, rsAddr, rqAddr, sens, lun, &responseSize, roffs )) )
		goto bail;

	payloadSize = *sensReadMsgSize = responseSize - roffs - FOOTER_LENGTH;
	if ( (rval = mchMsgCheckSizes( sizeof( response ), roffs, payloadSize )) ) {
		printf("mchMsgReadSensor size error\n");
		goto bail;
	}

	memcpy( data, response + roffs, payloadSize );

bail:
	return rval;
}

int
mchMsgGetSensorThresholds(MchData mchData, uint8_t *data, uint8_t sens, uint8_t lun)
{
uint8_t  response[MSG_MAX_LENGTH] = { 0 }; 
size_t   roffs, responseSize = 0, payloadSize = IPMI_RPLY_IMSG2_GET_SENSOR_THRESH_LENGTH; 
int      rval, bridged;
uint8_t  rsAddr, rqAddr;

	mchSetSizeOffs( mchData->ipmiSess, payloadSize, &roffs, &responseSize, &bridged, &rsAddr, &rqAddr);

	if ( (rval = ipmiMsgGetSensorThresholds( mchData->mchSess, mchData->ipmiSess, response, bridged, rsAddr, rqAddr, sens, lun, &responseSize, roffs )) )
		goto bail;

	if ( (rval = mchMsgCheckSizes( sizeof( response ), roffs, payloadSize )) ) {
		printf("mchMsgGetSensorThresholds size error\n");
		goto bail;
	}

	memcpy( data, response + roffs, payloadSize );

bail:
	return rval;
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

	messageSize = ipmiMsgBuild( mchData->ipmiSess, message, cmd, IPMI_MSG_NETFN_APP_REQUEST, imsg2, imsg2Size, b1msg1, b1msg2, b1msg2Size, 0, 0, 0 );

	return mchMsgWriteReadHelper( mchData->mchSess, mchData->ipmiSess, message, messageSize, data, &responseSize, cmd, netfn, 0/*need to set codeoffs*/, 0 );
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

       	messageSize = ipmiMsgBuild( mchData->ipmiSess, message, cmd, netfn, imsg2, imsg2Size, 0, 0, 0, 0, 0, 0 );

	return mchMsgWriteReadHelper( mchData->mchSess, mchData->ipmiSess, message, messageSize, data, &responseSize, cmd, netfn, offs/*need to set codeoffs*/, 0 );
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

	messageSize = ipmiMsgBuild( mchData->ipmiSess, message, cmd, IPMI_MSG_NETFN_APP_REQUEST, imsg2, imsg2Size, b1msg1, b1msg2, b1msg2Size, 0, 0, 0 );

	return mchMsgWriteReadHelper( mchData->mchSess, mchData->ipmiSess, message, messageSize, data, &responseSize, cmd, netfn, 0/*need to set codeoffs*/, 0 );
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

       	messageSize = ipmiMsgBuild( mchData->ipmiSess, message, cmd, IPMI_MSG_NETFN_APP_REQUEST, imsg2, imsg2Size, b1msg1, b1msg2, b1msg2Size, b2msg1, b2msg2, b2msg2Size );

	return mchMsgWriteReadHelper( mchData->mchSess, mchData->ipmiSess, message, messageSize, data, &responseSize, cmd, netfn, offs/*need to set codeoffs*/, 0 );
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

/* Get FRU Activation Policy using Vadatech MCH; message contains 1 bridged message -> needs updating 3/23/16
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
//size_t   responseSize = IPMI_RPLY_GET_FRU_POLICY_LENGTH_VT;
uint8_t  cmd   = IPMI_MSG_CMD_GET_FRU_POLICY;
uint8_t  netfn = IPMI_MSG_NETFN_PICMG;

	memcpy( imsg2,  SEND_MSG_MSG, imsg2Size );
	memcpy( b1msg1, IPMI_MSG1   , b1msg1Size );
	memcpy( b1msg2, GET_FRU_POLICY_MSG, b1msg2Size );

	imsg2[IPMI_MSG2_CHAN_OFFSET] = IPMI_MSG_CHAN_IPMB0 + IPMI_MSG_TRACKING;

	b1msg1[IPMI_MSG1_RSADDR_OFFSET]   = IPMI_MSG_ADDR_CM;
	b1msg1[IPMI_MSG1_NETFNLUN_OFFSET] = netfn << 2;

	b1msg2[IPMI_MSG2_SET_FRU_ACT_FRU_OFFSET] = fru;

	messageSize = ipmiMsgBuild( mchData->ipmiSess, message, cmd, IPMI_MSG_NETFN_APP_REQUEST, imsg2, imsg2Size, b1msg1, b1msg2, b1msg2Size, 0, 0, 0 );

	return mchMsgWriteReadHelper( mchData->mchSess, mchData->ipmiSess, message, messageSize, data, &responseSize, cmd, netfn, 0/*need to set codeoffs*/, 0 );
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
size_t   roffs;
size_t   payloadSize  = IPMI_RPLY_GET_FRU_POLICY_LENGTH;
size_t   responseSize = 2*IPMI_MSG1_LENGTH + 2*IPMI_RPLY_IMSG2_SEND_MSG_LENGTH;
uint8_t  cmd          = IPMI_MSG_CMD_GET_FRU_POLICY;
uint8_t  netfn        = IPMI_MSG_NETFN_PICMG;
int      bridged;
uint8_t  rsAddr, rqAddr;

	mchSetSizeOffs( mchData->ipmiSess, payloadSize, &roffs, &responseSize, &bridged, &rsAddr, &rqAddr );

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

       	messageSize = ipmiMsgBuild( mchData->ipmiSess, message, cmd, IPMI_MSG_NETFN_APP_REQUEST, imsg2, imsg2Size, b1msg1, b1msg2, b1msg2Size, b2msg1, b2msg2, b2msg2Size );

	return mchMsgWriteReadHelper( mchData->mchSess, mchData->ipmiSess, message, messageSize, data, &responseSize, cmd, netfn, roffs, 0);
}

int
mchMsgGetFruActPolicyHelper(MchData mchData, uint8_t *data, uint8_t fru)
{
	if ( !( MCH_IS_NAT( mchData->mchSess->type )) )
		return mchMsgGetFruActPolicyVt( mchData, data, fru );
	else
		return mchMsgGetFruActPolicyNat( mchData, data, fru );

}

/* Get Fan Speed Properties using Vadatech MCH; message contains 1 bridged message -> needs updating 3/23/16
 *
 *   RETURNS: status from mchMsgWriteReadHelper
 *            0 on success
 *            non-zero for error
 */
int
mchMsgGetFanPropVt(MchData mchData, uint8_t *data, uint8_t fru)
{
uint8_t  message[MSG_MAX_LENGTH] = { 0 };
uint8_t  imsg2Size = sizeof( GET_FAN_PROP_MSG );
uint8_t  imsg2[imsg2Size];
size_t   messageSize;

uint8_t  response[MSG_MAX_LENGTH] = { 0 }; 
size_t   roffs, responseSize, payloadSize = IPMI_RPLY_GET_FAN_PROP_LENGTH; 
int      rval, offs=0, bridged;
uint8_t  rsAddr, rqAddr;
uint8_t  cmd   = IPMI_MSG_CMD_GET_FAN_PROP;
uint8_t  netfn = IPMI_MSG_NETFN_PICMG;

	mchSetSizeOffs( mchData->ipmiSess, payloadSize, &roffs, &responseSize, &bridged, &rsAddr, &rqAddr);

	memcpy( imsg2, GET_FAN_PROP_MSG, imsg2Size );

	imsg2[IPMI_MSG2_SET_FRU_ACT_FRU_OFFSET] = fru;

	if ( bridged ) // may need to distinguish between once and twice-bridged messages
		ipmiBuildSendMsg( mchData->ipmiSess, message, &messageSize, cmd, netfn, rsAddr, rqAddr, imsg2, imsg2Size, 0 );
	else
		messageSize = ipmiMsgBuild( mchData->ipmiSess, message, cmd, netfn, imsg2, imsg2Size, 0, 0, 0, 0, 0, 0 );

	if ( (rval = mchMsgWriteReadHelper( mchData->mchSess, mchData->ipmiSess, message, messageSize, response, &responseSize, cmd, netfn, offs/*need to set codeoffs*/, 0 )) )
		goto bail;

	if ( (rval = mchMsgCheckSizes( sizeof( response ), roffs, payloadSize )) ) {
		printf("mchMsgGetFanPropVt size error\n");
		goto bail;
	}

	memcpy( data, response + roffs, payloadSize );

bail:
	return rval;
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
uint8_t  message[MSG_MAX_LENGTH]  = { 0 };
uint8_t  response[MSG_MAX_LENGTH] = { 0 };
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
size_t   roffs;
size_t   payloadSize  = IPMI_RPLY_GET_FAN_PROP_LENGTH;
size_t   responseSize = 2*IPMI_MSG1_LENGTH + 2*IPMI_RPLY_IMSG2_SEND_MSG_LENGTH;
uint8_t  cmd          = IPMI_MSG_CMD_GET_FAN_PROP;
uint8_t  netfn        = IPMI_MSG_NETFN_PICMG;
int      rval, bridged;
uint8_t  rsAddr, rqAddr;

	mchSetSizeOffs( mchData->ipmiSess, payloadSize, &roffs, &responseSize, &bridged, &rsAddr, &rqAddr );

	memcpy( imsg2,  SEND_MSG_MSG , imsg2Size  );
	memcpy( b1msg1, IPMI_MSG1    , b1msg1Size );

	imsg2[IPMI_MSG2_CHAN_OFFSET]    = IPMI_MSG_CHAN_IPMB0 + IPMI_MSG_TRACKING;
	b1msg1[IPMI_MSG1_RSADDR_OFFSET] = IPMI_MSG_ADDR_CM;

	memcpy( b1msg2, SEND_MSG_MSG      , b1msg2Size );
	memcpy( b2msg1, IPMI_MSG1         , b2msg1Size );
	memcpy( b2msg2, GET_FAN_PROP_MSG , b2msg2Size );

	b1msg2[IPMI_MSG2_RQADDR_OFFSET]   = IPMI_MSG_ADDR_BMC;
	b2msg1[IPMI_MSG1_NETFNLUN_OFFSET] = netfn << 2;
	b1msg2[IPMI_MSG2_CHAN_OFFSET]     = IPMI_MSG_CHAN_IPMBL + IPMI_MSG_TRACKING;
	b2msg1[IPMI_MSG1_RSADDR_OFFSET]   = FRU_I2C_ADDR[fru];
	b2msg2[IPMI_MSG2_SET_FRU_ACT_FRU_OFFSET] = 0;

       	messageSize = ipmiMsgBuild( mchData->ipmiSess, message, cmd, IPMI_MSG_NETFN_APP_REQUEST, imsg2, imsg2Size, b1msg1, b1msg2, b1msg2Size, b2msg1, b2msg2, b2msg2Size );

	if ( (rval = mchMsgWriteReadHelper( mchData->mchSess, mchData->ipmiSess, message, messageSize, response, &responseSize, cmd, netfn, roffs, 0 )) )
		goto bail;

	if ( (rval = mchMsgCheckSizes( sizeof( response ), roffs, payloadSize )) ) {
		printf("mchMsgGetFanPropNat size error\n");
		goto bail;
	}

	memcpy( data, response + roffs, payloadSize );
    
bail:
	return rval;
}

int
mchMsgGetFanPropHelper(MchData mchData, uint8_t *data, uint8_t fru)
{
	if ( MCH_IS_NAT( mchData->mchSess->type ) )
		return mchMsgGetFanPropNat( mchData, data, fru );
	else
		return mchMsgGetFanPropVt( mchData, data, fru );

}

/* -> needs updating 3/23/16 */
int
mchMsgGetFanLevelVt(MchData mchData, uint8_t *data, uint8_t fru)
{
uint8_t  message[MSG_MAX_LENGTH] = { 0 };
uint8_t  imsg2Size = sizeof( GET_FAN_LEVEL_MSG );
uint8_t  imsg2[imsg2Size];
size_t   messageSize;

uint8_t  response[MSG_MAX_LENGTH] = { 0 }; 
size_t   roffs, responseSize, payloadSize = IPMI_RPLY_GET_FAN_LEVEL_LENGTH; 
int      rval, offs=0, bridged;
uint8_t  rsAddr, rqAddr;
uint8_t  cmd   = IPMI_MSG_CMD_GET_FAN_LEVEL;
uint8_t  netfn = IPMI_MSG_NETFN_PICMG;

	mchSetSizeOffs( mchData->ipmiSess, payloadSize, &roffs, &responseSize, &bridged, &rsAddr, &rqAddr);

	memcpy( imsg2, GET_FAN_LEVEL_MSG, imsg2Size );

	imsg2[IPMI_MSG2_SET_FRU_ACT_FRU_OFFSET]      = fru;

	if ( bridged ) // may need to distinguish between once and twice-bridged messages
		ipmiBuildSendMsg( mchData->ipmiSess, message, &messageSize, cmd, netfn, rsAddr, rqAddr, imsg2, imsg2Size, 0 );
	else
		messageSize = ipmiMsgBuild( mchData->ipmiSess, message, cmd, netfn, imsg2, imsg2Size, 0, 0, 0, 0, 0, 0 );

	if ( (rval = mchMsgWriteReadHelper( mchData->mchSess, mchData->ipmiSess, message, messageSize, response, &responseSize, cmd, netfn, offs/*need to set codeoffs*/, 0 )) )
		goto bail;

	if ( (rval = mchMsgCheckSizes( sizeof( response ), roffs, payloadSize )) ) {
		printf("mchMsgGetFanLevelVt size error\n");
		goto bail;
	}

	memcpy( data, response + roffs, payloadSize );

bail:
	return rval;
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
uint8_t  message[MSG_MAX_LENGTH]  = { 0 };
uint8_t  response[MSG_MAX_LENGTH] = { 0 };
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
size_t   roffs;
size_t   payloadSize  = IPMI_RPLY_GET_FAN_LEVEL_LENGTH;
size_t   responseSize = 2*IPMI_MSG1_LENGTH + 2*IPMI_RPLY_IMSG2_SEND_MSG_LENGTH;
uint8_t  cmd          = IPMI_MSG_CMD_GET_FAN_LEVEL;
uint8_t  netfn        = IPMI_MSG_NETFN_PICMG;
int      rval, bridged;
uint8_t  rsAddr, rqAddr;

	mchSetSizeOffs( mchData->ipmiSess, payloadSize, &roffs, &responseSize, &bridged, &rsAddr, &rqAddr );

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

       	messageSize = ipmiMsgBuild( mchData->ipmiSess, message, cmd, IPMI_MSG_NETFN_APP_REQUEST, imsg2, imsg2Size, b1msg1, b1msg2, b1msg2Size, b2msg1, b2msg2, b2msg2Size );

	if ( (rval = mchMsgWriteReadHelper( mchData->mchSess, mchData->ipmiSess, message, messageSize, response, &responseSize, cmd, netfn, roffs, 0 )) )
		goto bail;

	if ( (rval = mchMsgCheckSizes( sizeof( response ), roffs, payloadSize )) ) {
		printf("mchMsgGetFanLevelNat size error\n");
		goto bail;
	}

	memcpy( data, response + roffs, payloadSize );

bail:    
	return rval;
}

/* Details of this algorithm specified in PICMG 3.0 Rev 3.0 ATCA Base Spec, Table 3-87 */
int
mchMsgGetFanLevelHelper(MchData mchData, uint8_t *data, uint8_t fruId, uint8_t fanProp, uint8_t *level)
{
int rval;
uint8_t llevel, olevel, lenabled;

	if ( MCH_IS_NAT( mchData->mchSess->type ) )
		rval = mchMsgGetFanLevelNat( mchData, data, fruId );
	else
		rval = mchMsgGetFanLevelVt(  mchData, data, fruId );

	if ( rval )
		return rval;

	olevel = data[PICMG_RPLY_IMSG2_GET_FAN_OVERRIDE_LEVEL_OFFSET];

	if ( PICMG_FAN_LOCAL_CONTROL_SUPPORTED(fanProp) ) {

		llevel   = data[PICMG_RPLY_IMSG2_GET_FAN_LOCAL_LEVEL_OFFSET];
		lenabled = data[ PICMG_RPLY_IMSG2_GET_FAN_LOCAL_ENABLED_OFFSET];

		if ( olevel == 0xFF )
			*level = llevel;

		else if ( lenabled )
			*level = (llevel > olevel) ? llevel : olevel;

		else if ( (0xFE==llevel) || (0xFE==olevel) )
			*level = -1; /* Shut down state; arbitrarily choose -1 for this state for now */

		else
			*level = olevel;
	}
	else
		*level = olevel;

	return rval;
}

/* Set Fan Level using Vadatech MCH; message contains 1 bridged message  -> needs updating 3/23/16
 *
 *   RETURNS: status from mchMsgWriteReadHelper
 *            0 on success
 *            non-zero for error
 */
int
mchMsgSetFanLevelVt(MchData mchData, uint8_t *data, uint8_t fru, uint8_t level)
{
uint8_t  message[MSG_MAX_LENGTH] = { 0 };
uint8_t  imsg2Size = sizeof( SET_FAN_LEVEL_MSG );
uint8_t  imsg2[imsg2Size];
size_t   messageSize;

uint8_t  response[MSG_MAX_LENGTH] = { 0 }; 
size_t   roffs, responseSize, payloadSize = IPMI_RPLY_SET_FAN_LEVEL_LENGTH; 
int      rval, offs=0, bridged;
uint8_t  rsAddr, rqAddr;
uint8_t  cmd   = IPMI_MSG_CMD_SET_FAN_LEVEL;
uint8_t  netfn = IPMI_MSG_NETFN_PICMG;

	mchSetSizeOffs( mchData->ipmiSess, payloadSize, &roffs, &responseSize, &bridged, &rsAddr, &rqAddr);

	memcpy( imsg2, SET_FAN_LEVEL_MSG, imsg2Size );

	imsg2[IPMI_MSG2_SET_FRU_ACT_FRU_OFFSET] = fru;
	imsg2[IPMI_MSG2_SET_FAN_LEVEL_OFFSET]   = level;

	if ( bridged ) // may need to distinguish between once and twice-bridged messages
		ipmiBuildSendMsg( mchData->ipmiSess, message, &messageSize, cmd, netfn, rsAddr, rqAddr, imsg2, imsg2Size, 0 );
	else
		messageSize = ipmiMsgBuild( mchData->ipmiSess, message, cmd, netfn, imsg2, imsg2Size, 0, 0, 0, 0, 0, 0 );

	if ( (rval = mchMsgWriteReadHelper( mchData->mchSess, mchData->ipmiSess, message, messageSize, response, &responseSize, cmd, netfn, offs/*need to set codeoffs*/, 0 )) )
		goto bail;

	if ( (rval = mchMsgCheckSizes( sizeof( response ), roffs, payloadSize )) ) {
		printf("mchMsgSetFanLevelVt size error\n");
		goto bail;
	}
	memcpy( data, response + roffs, payloadSize );

bail:
	return rval;
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
uint8_t  message[MSG_MAX_LENGTH]  = { 0 };
uint8_t  response[MSG_MAX_LENGTH] = { 0 };
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
size_t   offs         = IPMI_RPLY_2ND_BRIDGED_OFFSET_NAT;
size_t   roffs        = IPMI_RPLY_HEADER_LENGTH + 3 + offs;
size_t   payloadSize  = IPMI_RPLY_SET_FAN_LEVEL_LENGTH;
size_t   responseSize = roffs + payloadSize + FOOTER_LENGTH;
uint8_t  cmd          = IPMI_MSG_CMD_SET_FAN_LEVEL;
uint8_t  netfn        = IPMI_MSG_NETFN_PICMG;
int      rval;

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
	b2msg2[IPMI_MSG2_SET_FAN_LEVEL_OFFSET] = level;

       	messageSize = ipmiMsgBuild( mchData->ipmiSess, message, cmd, IPMI_MSG_NETFN_APP_REQUEST, imsg2, imsg2Size, b1msg1, b1msg2, b1msg2Size, b2msg1, b2msg2, b2msg2Size );

	if ( (rval = mchMsgWriteReadHelper( mchData->mchSess, mchData->ipmiSess, message, messageSize, response, &responseSize, cmd, netfn, offs/*need to set codeoffs*/, 0 )) )
		goto bail;

	if ( (rval = mchMsgCheckSizes( sizeof( response ), roffs, payloadSize )) ) {
		printf("mchMsgSetFanLevelNat size error\n");
		goto bail;
	}
	memcpy( data, response + roffs, payloadSize );
    
bail:
	return rval;
}

int
mchMsgSetFanLevelHelper(MchData mchData, uint8_t *data, uint8_t fru, uint8_t level)
{
	if ( MCH_IS_NAT( mchData->mchSess->type ) )
		return mchMsgSetFanLevelNat( mchData, data, fru, level );
	else
		return mchMsgSetFanLevelVt( mchData, data, fru, level );
}

/*  -> needs updating 3/23/16 */
int
mchMsgGetPowerLevelVt(MchData mchData, uint8_t *data, uint8_t fru, uint8_t parm)
{
uint8_t  message[MSG_MAX_LENGTH] = { 0 };
uint8_t  imsg2Size = sizeof( GET_POWER_LEVEL_MSG );
uint8_t  imsg2[imsg2Size];
size_t   messageSize;

uint8_t  response[MSG_MAX_LENGTH] = { 0 }; 
size_t   roffs, responseSize, payloadSize = IPMI_RPLY_GET_POWER_LEVEL_LENGTH; 
int      rval, offs=0, bridged;
uint8_t  rsAddr, rqAddr;
uint8_t  cmd   = IPMI_MSG_CMD_GET_POWER_LEVEL;
uint8_t  netfn = IPMI_MSG_NETFN_PICMG;

	mchSetSizeOffs( mchData->ipmiSess, payloadSize, &roffs, &responseSize, &bridged, &rsAddr, &rqAddr);

	memcpy( imsg2, GET_POWER_LEVEL_MSG, imsg2Size );

	imsg2[IPMI_MSG2_SET_FRU_ACT_FRU_OFFSET]      = fru;
	imsg2[PICMG_RPLY_IMSG2_GET_POWER_LEVEL_TYPE_OFFSET] = parm;

	if ( bridged ) // may need to distinguish between once and twice-bridged messages
		ipmiBuildSendMsg( mchData->ipmiSess, message, &messageSize, cmd, netfn, rsAddr, rqAddr, imsg2, imsg2Size, 0 );
	else
		messageSize = ipmiMsgBuild( mchData->ipmiSess, message, cmd, netfn, imsg2, imsg2Size, 0, 0, 0, 0, 0, 0 );

	if ( (rval = mchMsgWriteReadHelper( mchData->mchSess, mchData->ipmiSess, message, messageSize, response, &responseSize, cmd, netfn, offs/*need to set codeoffs*/, 0 )) )
		goto bail;

	if ( (rval = mchMsgCheckSizes( sizeof( response ), roffs, payloadSize )) ) {
		printf("mchMsgGetPowerLevelVt size error\n");
		goto bail;
	}
	memcpy( data, response + roffs, payloadSize );

bail:
	return rval;
}
