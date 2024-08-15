//////////////////////////////////////////////////////////////////////////////
// This file is part of 'ipmiComm'.
// It is subject to the license terms in the LICENSE.txt file found in the
// top-level directory of this distribution and at:
//    https://confluence.slac.stanford.edu/display/ppareg/LICENSE.html.
// No part of 'ipmiComm', including this file,
// may be copied, modified, propagated, or distributed except according to
// the terms contained in the LICENSE.txt file.
//////////////////////////////////////////////////////////////////////////////
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
int mchMsgWriteReadHelper(MchSess mchSess, IpmiSess ipmiSess, uint8_t* message, size_t messageSize, uint8_t* response,
                          size_t* responseSize, uint8_t cmd, uint8_t netfn, int codeOffs, int outSess) {
    int      i, status, inst = mchSess->instance;
    uint8_t  ipmiSeq = 0, code;
    int      ipmiSeqOffs;
    uint8_t  seq[4];
    uint32_t seqInt, seqRplyInt, seqDiff;
    size_t   responseLen;

    ipmiSeqOffs = (IPMI_MSG_AUTH_TYPE_NONE == message[RMCP_MSG_HEADER_LENGTH + IPMI_WRAPPER_AUTH_TYPE_OFFSET])
                      ? RMCP_MSG_HEADER_LENGTH + IPMI_WRAPPER_LENGTH + IPMI_MSG1_LENGTH + IPMI_MSG2_SEQLUN_OFFSET
                      : RMCP_MSG_HEADER_LENGTH + IPMI_WRAPPER_AUTH_LENGTH + IPMI_MSG1_LENGTH + IPMI_MSG2_SEQLUN_OFFSET;

    /* seems we check this twice for sensor reads -- look into this */
    if (!MCH_ONLN(mchStat[inst])) {
        return -1;
    }

    status =
        ipmiMsgWriteRead(mchSess->name, message, messageSize, response, responseSize, mchSess->timeout, &responseLen);

    if (MCH_DBG(mchStat[inst]) >= MCH_DBG_MED) {

        if (MCH_DBG(mchStat[inst]) >= MCH_DBG_HIGH) {
            printf("%s Message status %i, received %i, expected %i, raw data:\n", mchSess->name, status,
                   (int)responseLen, *(int*)responseSize);
            for (i = 0; i < responseLen; i++) {
                printf("%02x ", response[i]);
            }
            printf("\n");
        } else if (status) {
            printf("%s Message status %i, received %i, expected %i\n", mchSess->name, status, (int)responseLen,
                   *(int*)responseSize);
        }
    }

    *responseSize = responseLen; /* Pass actual response length back to caller */

    if (outSess) {
        if (status) {
            return status;
        }

        if ((code = response[codeOffs]) && MCH_DBG(mchStat[inst])) {
            ipmiCompletionCode(mchSess->name, code, cmd, netfn);
        }
        return code;
    }

    if (mchSess->err > 9) {
        if (MCH_DBG(mchStat[inst])) {
            printf("%s start new session; err count is %i\n", mchSess->name, mchSess->err);
        }

        /* Reset error count to 0 */
        mchSess->err = 0;

        /* Close current session, start new one, and return error */
        mchMsgCloseSess(mchSess, ipmiSess, response);
        mchNewSession(mchSess, ipmiSess);
        return -1;
    }

    if (responseLen == 0) {
        mchSess->err++;
        return -1;
    }

    /* Verify IPMI message sequence number. If incorrect, increment error count and return error */
    ipmiSeq = IPMI_SEQLUN_EXTRACT_SEQ(response[ipmiSeqOffs]);
    if (ipmiSeq != ipmiSess->seq) {
        if (MCH_DBG(mchStat[inst])) {
            printf("%s Incorrect IPMI sequence; got %i but expected %i\n", mchSess->name, ipmiSeq, ipmiSess->seq);
        }
        mchSess->err++;
        return -1;
    }

    /* Extract session sequence number from reply */
    for (i = 0; i < IPMI_RPLY_SEQ_LENGTH; i++) {
        seq[i] = response[RMCP_MSG_HEADER_LENGTH + IPMI_WRAPPER_SEQ_OFFSET + i]; /* bridged offset ? */
    }

    seqInt     = arrayToUint32(seq);
    seqRplyInt = arrayToUint32(ipmiSess->seqRply);
    seqDiff    = seqInt - seqRplyInt;

    if (MCH_DBG(mchStat[inst]) >= MCH_DBG_HIGH) {
        printf("%s new sequence number %i, stored %i\n", mchSess->name, seqInt, seqRplyInt);
    }

    /* Check session sequence number. If it is not increasing or more than
     * 7 counts higher than last time, discard message and set error.
     * If it is more than 7 counts higher, store new sequence number.
     *
     * If neither above errors are encountered, sotre new sequence number
     *
     * If seq number error or completion code non-zero, increment error count and return error.
     * Else reset error count to 0 and return success.
     */
    if ((seqInt <= seqRplyInt) || (seqDiff > 7)) {
        if (MCH_DBG(mchStat[inst]) >= MCH_DBG_MED) {
            printf("%s sequence number %i, previous %i\n", mchSess->name, seqInt, seqRplyInt);
        }
        if ((seqDiff > 7)) {
            for (i = 0; i < IPMI_RPLY_SEQ_LENGTH; i++) {
                ipmiSess->seqRply[i] = seq[i];
            }
        } else {
            incr4Uint8Array(ipmiSess->seqRply, 1);
        }

        /* Dumb workaround for Advantech non-increasing numbers */
        if (!(mchSess->type == MCH_TYPE_ADVANTECH)) {
            mchSess->err++;
            return -1;
        }
    } else {
        for (i = 0; i < IPMI_RPLY_SEQ_LENGTH; i++) {
            ipmiSess->seqRply[i] = seq[i];
        }

        if ((code = response[codeOffs])) {
            if (MCH_DBG(mchStat[inst])) {
                ipmiCompletionCode(mchSess->name, code, cmd, netfn);
            }
            // mchSess->err++; // only increment error count for some errors ? --not parameter out of range, for example
            return code;
        }
    }

    mchSess->err = 0;
    return 0;
}

/* Only relevant for messages in a session. Do not call for messages preceding a session. Do this so we can use authReq
 * to determine auth type. Some devices do both bridged/non-bridged messages, so cannot use ipmiSess->features alone to
 * determine this. For now, also check *bridged, which should be set to non-zero value for a bridged message. Also this
 * is only for once-bridged messages. Twice-bridged messages handled separately
 */
void mchSetSizeOffs(IpmiSess ipmiSess, size_t payloadSize, size_t* roffs, size_t* responseSize, int* bridged,
                    uint8_t* rsAddr, uint8_t* rqAddr) {
    int authOffs, offs;

    /* This section needs to be completely figured out (Vadatech) */
    if (ipmiSess->features & MCH_FEAT_SENDMSG_RPLY) {
        // note that we used to only check comp code of send msg reply (not payload reply) which is different than other
        // devices; may need to revert to that
        offs = IPMI_RPLY_BRIDGED_2REPLY_OFFSET;
        *responseSize += IPMI_RPLY_HEADER_LENGTH + offs;
        *bridged = 1;
        *rsAddr  = IPMI_MSG_ADDR_CM;
        *rqAddr  = IPMI_MSG_ADDR_BMC;
    } else if (*bridged) {
        offs = IPMI_RPLY_BRIDGED_2REPLY_OFFSET;
        *responseSize += IPMI_RPLY_HEADER_LENGTH + offs;
        *rqAddr = IPMI_MSG_ADDR_BMC;
    } else {
        offs = 0;
        *responseSize += IPMI_RPLY_HEADER_LENGTH;
        *bridged = 0;
        *rqAddr  = IPMI_MSG_ADDR_SW;
    }

    if (ipmiSess->authReq != IPMI_MSG_AUTH_TYPE_NONE) {
        authOffs = sizeof(IPMI_WRAPPER_PWD_KEY) - sizeof(IPMI_WRAPPER);
        offs += authOffs;
        *responseSize += authOffs;
    }

    *roffs = *responseSize; /* Offset into packet our section of interest */
    /* If payloadSize is 0, it means the response length is variable or unknown;
     * Set responseSize to 0 to indicate this to write/read function */
    *responseSize += payloadSize + FOOTER_LENGTH;

    return;
}

int mchMsgCheckSizes(size_t destSize, int offset, size_t srcSize) {
    if (0 >= (int)srcSize) {
        /* printf("mchMsgCheckSizes: message size %i less than zero\n", (int)srcSize); */
        return -1;
    } else if ((offset + (int)srcSize) > (int)destSize) {
        /* printf("mchMsgCheckSizes: copy size %i larger than destination %i \n", offset+(int)srcSize, (int)destSize);
         */
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
int mchMsgGetChanAuth(MchSess mchSess, IpmiSess ipmiSess, uint8_t* data) {
    uint8_t response[MSG_MAX_LENGTH] = {0};
    size_t  roffs, responseSize, payloadSize = IPMI_RPLY_IMSG2_GET_CHAN_AUTH_LENGTH;
    int     rval;

    roffs        = IPMI_RPLY_HEADER_LENGTH;
    responseSize = (payloadSize == 0) ? 0 : roffs + payloadSize + FOOTER_LENGTH;

    if ((rval = ipmiMsgGetChanAuth(mchSess, ipmiSess, response, &responseSize, roffs))) {
        goto bail;
    }

    if ((rval = mchMsgCheckSizes(sizeof(response), roffs, payloadSize))) {
        printf("mchMsgGetChanAuth size error\n");
        goto bail;
    }

    memcpy(data, response + roffs, payloadSize);

bail:
    return rval;
}

/* Temporary workaround for using different usernames across different devices */
int mchMsgGetSess(MchSess mchSess, IpmiSess ipmiSess, uint8_t* data) {
    uint8_t response[MSG_MAX_LENGTH] = {0};
    size_t  size                     = sizeof(GET_SESS_MSG);
    uint8_t msg[size];
    size_t  roffs, responseSize, payloadSize = IPMI_RPLY_IMSG2_GET_SESSION_CHALLENGE_LENGTH;
    int     rval;

    roffs = IPMI_RPLY_HEADER_LENGTH;
    /* If payloadSize is 0, it means the response length is variable or unknown;
     * Set responseSize to 0 to indicate this to write/read function */
    responseSize = (payloadSize == 0) ? 0 : roffs + payloadSize + FOOTER_LENGTH;

    switch (ipmiSess->authReq) {

        default:
            printf("mchMsgGetSess: unsupported authentication type %i\n", ipmiSess->authReq);
            return 0;

        case IPMI_MSG_AUTH_TYPE_NONE:
            memcpy(msg, GET_SESS_MSG, sizeof(GET_SESS_MSG));
            break;

        case IPMI_MSG_AUTH_TYPE_PWD_KEY:
            memcpy(msg, GET_SESS_MSG_PWD_KEY, sizeof(GET_SESS_MSG_PWD_KEY));
            break;
    }

    if ((rval = ipmiMsgGetSess(mchSess, ipmiSess, response, &responseSize, msg, roffs))) {
        goto bail;
    }

    if ((rval = mchMsgCheckSizes(sizeof(response), roffs, payloadSize))) {
        printf("mchMsgGetSess size error\n");
        goto bail;
    }
    memcpy(data, response + roffs, payloadSize);

bail:
    return rval;
}

/* Activate Session
 *
 *   RETURNS: status from ipmiMsgActSess
 *            0 on success
 *            non-zero for error
 */
int mchMsgActSess(MchSess mchSess, IpmiSess ipmiSess, uint8_t* data) {
    uint8_t response[MSG_MAX_LENGTH] = {0};
    size_t  roffs, responseSize = 0, payloadSize = IPMI_RPLY_IMSG2_ACTIVATE_SESSION_LENGTH;
    int     rval, bridged = 0;
    uint8_t rsAddr = IPMI_MSG_ADDR_BMC, rqAddr;

    mchSetSizeOffs(ipmiSess, payloadSize, &roffs, &responseSize, &bridged, &rsAddr, &rqAddr);

    if ((rval = ipmiMsgActSess(mchSess, ipmiSess, response, &responseSize, roffs))) {
        goto bail;
    }

    if ((rval = mchMsgCheckSizes(sizeof(response), roffs, payloadSize))) {
        printf("mchMsgActSess size error\n");
        goto bail;
    }
    memcpy(data, response + roffs, payloadSize);

bail:
    return rval;
}

/* Set Session Privilege level
 *
 *   RETURNS: status from ipmiMsgSetPriv
 *            0 on success
 *            non-zero for error
 */
int mchMsgSetPriv(MchSess mchSess, IpmiSess ipmiSess, uint8_t* data, uint8_t level) {
    uint8_t response[MSG_MAX_LENGTH] = {0};
    size_t  roffs, responseSize = 0, payloadSize = IPMI_RPLY_IMSG2_SET_PRIV_LEVEL_LENGTH;
    int     rval, bridged = 0;
    uint8_t rsAddr = IPMI_MSG_ADDR_BMC, rqAddr;

    mchSetSizeOffs(ipmiSess, payloadSize, &roffs, &responseSize, &bridged, &rsAddr, &rqAddr);

    if ((rval = ipmiMsgSetPriv(mchSess, ipmiSess, response, &responseSize, level, roffs))) {
        goto bail;
    }

    if ((rval = mchMsgCheckSizes(sizeof(response), roffs, payloadSize))) {
        printf("mchMsgSetPriv size error\n");
        goto bail;
    }
    memcpy(data, response + roffs, payloadSize);

bail:
    return rval;
}

/* Get Device ID -- this is called before we know device type -- need to update this
 *
 *   RETURNS: status from mchMsgWriteReadHelper
 *            0 on success
 *            non-zero for error
 */
static int mchMsgGetDeviceId(MchData mchData, uint8_t* data, int bridged,
                             uint8_t rsAddr) // change this to either never be bridged or to implement device checking
                                             // and offsets; handle rqAddr too
{
    uint8_t response[MSG_MAX_LENGTH] = {0};
    size_t  roffs, responseSize = 0, payloadSize = IPMI_RPLY_IMSG2_GET_DEVICE_ID_LENGTH;
    int     rval;
    uint8_t rqAddr;

    mchSetSizeOffs(mchData->ipmiSess, payloadSize, &roffs, &responseSize, &bridged, &rsAddr, &rqAddr);

    if ((rval = ipmiMsgGetDeviceId(mchData->mchSess, mchData->ipmiSess, response, bridged, rsAddr, rqAddr,
                                   &responseSize, roffs))) {
        goto bail;
    }

    if ((rval = mchMsgCheckSizes(sizeof(response), roffs, payloadSize))) {
        printf("mchMsgGetDeviceId size error\n");
        goto bail;
    }
    memcpy(data, response + roffs, payloadSize);

bail:
    return rval;
}

int mchMsgGetDeviceIdWrapper(MchData mchData, uint8_t* data, uint8_t rsAddr) {
    int bridged = 0;

    if (rsAddr != IPMI_MSG_ADDR_BMC) {
        bridged = 1;
    }
    return mchMsgGetDeviceId(mchData, data, bridged, rsAddr);
}

/* Broadcast Get Device ID -- this is called before we know device type -- need to update this
 *
 *   RETURNS: status from mchMsgWriteReadHelper
 *            0 on success
 *            non-zero for error
 */
int mchMsgBroadcastGetDeviceId(MchData mchData, uint8_t* data, int tmp,
                               uint8_t tmp1) // int bridged, uint8_t rsAddr) // change this $
{
    uint8_t response[MSG_MAX_LENGTH] = {0};
    size_t  roffs, responseSize = 0, payloadSize = IPMI_RPLY_IMSG2_GET_DEVICE_ID_LENGTH;
    int     rval, bridged = 0;
    uint8_t rsAddr = IPMI_MSG_ADDR_BMC, rqAddr;

    mchSetSizeOffs(mchData->ipmiSess, payloadSize, &roffs, &responseSize, &bridged, &rsAddr, &rqAddr);

    if ((rval = ipmiMsgBroadcastGetDeviceId(mchData->mchSess, mchData->ipmiSess, response, bridged, rsAddr, rqAddr,
                                            &responseSize, roffs))) {
        goto bail;
    }

    if ((rval = mchMsgCheckSizes(sizeof(response), roffs, payloadSize))) {
        printf("mchMsgBroadcastGetDeviceId size error\n");
        goto bail;
    }
    memcpy(data, response + roffs, payloadSize);
bail:
    return rval;
}

/* Close Session - test for NAT and determine reply offset*/
int mchMsgCloseSess(MchSess mchSess, IpmiSess ipmiSess, uint8_t* data) {
    size_t responseSize =
        MCH_IS_NAT(mchSess->type) ? IPMI_RPLY_CLOSE_SESSION_LENGTH_NAT : IPMI_RPLY_CLOSE_SESSION_LENGTH_VT;

    return ipmiMsgCloseSess(mchSess, ipmiSess, data, &responseSize /* need to add roffs */);
}

/* Chassis Control
 *
 *   RETURNS: status from ipmiMsgChassisControl
 *            0 on success
 *            non-zero for error
 */
int mchMsgChassisControl(MchData mchData, uint8_t* data, uint8_t parm) {
    uint8_t response[MSG_MAX_LENGTH] = {0};
    size_t  roffs, responseSize = 0, payloadSize = IPMI_RPLY_IMSG2_CHAS_CTRL_LENGTH;
    int     rval, bridged = 0;
    uint8_t rsAddr = IPMI_MSG_ADDR_BMC, rqAddr;

    mchSetSizeOffs(mchData->ipmiSess, payloadSize, &roffs, &responseSize, &bridged, &rsAddr, &rqAddr);

    if ((rval = ipmiMsgChassisControl(mchData->mchSess, mchData->ipmiSess, response, bridged, rsAddr, rqAddr, parm,
                                      &responseSize, roffs))) {
        goto bail;
    }

    if ((rval = mchMsgCheckSizes(sizeof(response), roffs, payloadSize))) {
        printf("mchMsgChassisControl size error\n");
        goto bail;
    }
    memcpy(data, response + roffs, payloadSize);

bail:
    return rval;
}

/* Get Chassis Status, supported by Supermicro and ATCA systems
 *
 *   RETURNS: status from ipmiMsgGetChassisStatus
 *            0 on success
 *            non-zero for error
 */
int mchMsgGetChassisStatus(MchData mchData, uint8_t* data) {
    uint8_t response[MSG_MAX_LENGTH] = {0};
    size_t  roffs, responseSize = 0, payloadSize = IPMI_RPLY_IMSG2_GET_CHAS_STATUS_LENGTH;
    int     rval, bridged = 0;
    uint8_t rsAddr = IPMI_MSG_ADDR_BMC, rqAddr;

    mchSetSizeOffs(mchData->ipmiSess, payloadSize, &roffs, &responseSize, &bridged, &rsAddr, &rqAddr);

    if ((rval = ipmiMsgGetChassisStatus(mchData->mchSess, mchData->ipmiSess, response, bridged, rsAddr, rqAddr,
                                        &responseSize, roffs))) {
        goto bail;
    }

    if ((rval = mchMsgCheckSizes(sizeof(response), roffs, payloadSize))) {
        printf("mchMsgGetChassisStatus size error\n");
        goto bail;
    }
    memcpy(data, response + roffs, payloadSize);

bail:
    return rval;
}

/* Get FRU Inventory Info, optionally bridged message. If not bridged, caller should set bridged arg to 0.
 *
 *   RETURNS: status from ipmiMsgGetFruInfo
 *            0 on success
 *            non-zero for error
 */
static int mchMsgGetFruInvInfo(MchData mchData, uint8_t* data, uint8_t id, int bridged, uint8_t rsAddr) {
    uint8_t response[MSG_MAX_LENGTH] = {0};
    size_t  roffs, responseSize = 0, payloadSize = IPMI_RPLY_IMSG2_GET_FRU_INV_INFO_LENGTH;
    int     rval;
    uint8_t rqAddr;

    mchSetSizeOffs(mchData->ipmiSess, payloadSize, &roffs, &responseSize, &bridged, &rsAddr, &rqAddr);

    if ((rval = ipmiMsgGetFruInvInfo(mchData->mchSess, mchData->ipmiSess, response, bridged, rsAddr, rqAddr, id,
                                     &responseSize, roffs))) {
        goto bail;
    }

    if ((rval = mchMsgCheckSizes(sizeof(response), roffs, payloadSize))) {
        printf("mchMsgGetFruInvInfo size error\n");
        goto bail;
    }

    memcpy(data, response + roffs, payloadSize);

bail:
    return rval;
}

int mchMsgGetFruInvInfoWrapper(MchData mchData, uint8_t* data, Fru fru) {
    int     bridged = 0;
    uint8_t rsAddr  = fru->sdr.addr;

    if (rsAddr != IPMI_MSG_ADDR_BMC) {
        bridged = 1;
    }
    return mchMsgGetFruInvInfo(mchData, data, fru->sdr.fruId, bridged, rsAddr);
}

/* Read FRU data
 *
 *   RETURNS: status from mchMsgWriteReadHelper
 *            0 on success
 *            non-zero for error
 */
static int mchMsgReadFru(MchData mchData, uint8_t* data, uint8_t id, uint8_t* readOffset, uint8_t readSize, int bridged,
                         uint8_t rsAddr) {
    uint8_t response[MSG_MAX_LENGTH] = {0};
    size_t  roffs, responseSize = 0, payloadSize = IPMI_RPLY_IMSG2_READ_FRU_DATA_BASE_LENGTH + readSize;
    int     rval;
    uint8_t rqAddr;

    mchSetSizeOffs(mchData->ipmiSess, payloadSize, &roffs, &responseSize, &bridged, &rsAddr, &rqAddr);

    if ((rval = ipmiMsgReadFru(mchData->mchSess, mchData->ipmiSess, response, bridged, rsAddr, rqAddr, id, readOffset,
                               readSize, &responseSize, roffs))) {
        goto bail;
    }

    if ((rval = mchMsgCheckSizes(sizeof(response), roffs, payloadSize))) {
        printf("mchMsgReadFru size error\n");
        goto bail;
    }

    memcpy(data, response + roffs, payloadSize);

bail:
    return rval;
}

int mchMsgReadFruWrapper(MchData mchData, uint8_t* data, Fru fru, uint8_t* readOffset, uint8_t readSize) {
    int     bridged = 0;
    uint8_t rsAddr  = fru->sdr.addr;

    if (rsAddr != IPMI_MSG_ADDR_BMC) {
        bridged = 1;
    }
    return mchMsgReadFru(mchData, data, fru->sdr.fruId, readOffset, readSize, bridged, rsAddr);
}

/* Get SDR (Sensor Data Record) Repository Info
 *
 *   RETURNS: status from mchMsgWriteReadHelper
 *            0 on success
 *            non-zero for error
 */
static int mchMsgGetSdrRepInfo(MchData mchData, uint8_t* data, uint8_t parm, int bridged, uint8_t rsAddr) {
    uint8_t response[MSG_MAX_LENGTH] = {0};
    size_t  roffs, responseSize = 0;
    size_t  payloadSize = (parm == IPMI_SDRREP_PARM_GET_DEV_SDR) ? IPMI_RPLY_IMSG2_GET_DEV_SDR_INFO_LENGTH
                                                                 : IPMI_RPLY_IMSG2_GET_SDRREP_INFO_LENGTH;
    int     rval;
    uint8_t rqAddr;

    mchSetSizeOffs(mchData->ipmiSess, payloadSize, &roffs, &responseSize, &bridged, &rsAddr, &rqAddr);

    /* Set channel ? */
    if ((rval = ipmiMsgGetSdrRepInfo(mchData->mchSess, mchData->ipmiSess, response, bridged, rsAddr, rqAddr,
                                     &responseSize, roffs, parm))) {
        goto bail;
    }

    if ((rval = mchMsgCheckSizes(sizeof(response), roffs, payloadSize))) {
        printf("mchMsgGetSdrRepInfo size error roffs %i payloadSize %i responseSize %i\n", (int)roffs, (int)payloadSize,
               (int)responseSize);
        goto bail;
    }

    memcpy(data, response + roffs, payloadSize);

bail:
    return rval;
}

int mchMsgGetSdrRepInfoWrapper(MchData mchData, uint8_t* data, uint8_t parm, uint8_t rsAddr) {
    int bridged = 0;

    if (rsAddr != IPMI_MSG_ADDR_BMC) {
        bridged = 1;
    }

    return mchMsgGetSdrRepInfo(mchData, data, parm, bridged, rsAddr);
}

/* Get Device Sensor Data Record (SDR) Info
 *
 *   RETURNS: status from mchMsgWriteReadHelper
 *            0 on success
 *            non-zero for error
 *
int
mchMsgGetDevSdrInfo(MchData mchData, uint8_t *data, int bridged, uint8_t rsAddr, uint8_t parm)
{
uint8_t  response[MSG_MAX_LENGTH] = { 0 };
size_t   roffs, responseSize = 0, payloadSize = IPMI_RPLY_IMSG2_GET_DEV_SDR_INFO_LENGTH;
int      rval;
uint8_t  rqAddr;

    mchSetSizeOffs( mchData->ipmiSess, payloadSize, &roffs, &responseSize, &bridged, &rsAddr, &rqAddr );

    if ( (rval = ipmiMsgGetDevSdrInfo( mchData->mchSess, mchData->ipmiSess, response, bridged, rsAddr, rqAddr, parm,
&responseSize, roffs )) ) goto bail;

    if ( (rval = mchMsgCheckSizes( sizeof( response ), roffs, payloadSize )) ) {
        printf("mchMsgGetDevSdrInfo size error\n");
        goto bail;
    }

    memcpy( data, response + roffs, payloadSize );

bail:
    return rval;
}
*/

/* Reserve SDR (Sensor Data Record) Repository
 *
 *   RETURNS: status from mchMsgWriteReadHelper
 *            0 on success
 *            non-zero for error
 */
static int mchMsgReserveSdrRep(MchData mchData, uint8_t* data, uint8_t parm, int bridged, uint8_t rsAddr) {
    uint8_t response[MSG_MAX_LENGTH] = {0};
    size_t  roffs, responseSize = 0, payloadSize = IPMI_RPLY_IMSG2_RESERVE_SDRREP_LENGTH;
    int     rval;
    uint8_t rqAddr;

    mchSetSizeOffs(mchData->ipmiSess, payloadSize, &roffs, &responseSize, &bridged, &rsAddr, &rqAddr);

    if ((rval = ipmiMsgReserveSdrRep(mchData->mchSess, mchData->ipmiSess, response, bridged, rsAddr, rqAddr,
                                     &responseSize, roffs, parm))) {
        goto bail;
    }

    if ((rval = mchMsgCheckSizes(sizeof(response), roffs, payloadSize))) {
        printf("mchMsgReserveSdrRep size error\n");
        goto bail;
    }

    memcpy(data, response + roffs, payloadSize);

bail:
    return rval;
}

int mchMsgReserveSdrRepWrapper(MchData mchData, uint8_t* data, uint8_t parm, uint8_t rsAddr) {
    int bridged = 0;

    if (rsAddr != IPMI_MSG_ADDR_BMC) {
        bridged = 1;
    }
    return mchMsgReserveSdrRep(mchData, data, parm, bridged, rsAddr);
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
static int mchMsgGetSdr(MchData mchData, uint8_t* data, uint8_t* id, uint8_t* res, uint8_t offset, uint8_t readSize,
                        uint8_t parm, int bridged, uint8_t rsAddr) {
    uint8_t response[MSG_MAX_LENGTH] = {0};
    uint8_t sdrDataSize              = (readSize == 0xFF) ? SDR_MAX_LENGTH : readSize;
    size_t  roffs, responseSize = 0, payloadSize = IPMI_RPLY_IMSG2_GET_SDR_BASE_LENGTH + sdrDataSize;
    int     rval;
    uint8_t rqAddr;

    mchSetSizeOffs(mchData->ipmiSess, payloadSize, &roffs, &responseSize, &bridged, &rsAddr, &rqAddr);

    if ((rval = ipmiMsgGetSdr(mchData->mchSess, mchData->ipmiSess, response, bridged, rsAddr, rqAddr, id, res, offset,
                              readSize, &responseSize, roffs, parm))) {
        goto bail;
    }

    if ((rval = mchMsgCheckSizes(sizeof(response), roffs, payloadSize))) {
        printf("mchMsgGetSdr size error\n");
        goto bail;
    }

    memcpy(data, response + roffs, payloadSize);

bail:
    return rval;
}

int mchMsgGetSdrWrapper(MchData mchData, uint8_t* data, uint8_t* id, uint8_t* res, uint8_t offset, uint8_t readSize,
                        uint8_t parm, uint8_t rsAddr) {
    int bridged = 0;

    if (rsAddr != IPMI_MSG_ADDR_BMC) {
        bridged = 1;
    }
    return mchMsgGetSdr(mchData, data, id, res, offset, readSize, parm, bridged, rsAddr);
}

/* Get Sensor Reading. Caller specifies expected message response length.
 *
 *   RETURNS: status from mchMsgWriteReadHelper
 *            0 on success
 *            non-zero for error
 */
int mchMsgReadSensor(MchData mchData, uint8_t* data, uint8_t sens, uint8_t lun, size_t* sensReadMsgSize, int bridged,
                     uint8_t rsAddr) {
    uint8_t response[MSG_MAX_LENGTH] = {0};
    size_t  roffs, responseSize = 0, payloadSize = *sensReadMsgSize;
    int     rval;
    uint8_t rqAddr;

    mchSetSizeOffs(mchData->ipmiSess, payloadSize, &roffs, &responseSize, &bridged, &rsAddr, &rqAddr);

    if ((rval = ipmiMsgReadSensor(mchData->mchSess, mchData->ipmiSess, response, bridged, rsAddr, rqAddr, sens, lun,
                                  &responseSize, roffs))) {
        goto bail;
    }

    payloadSize = *sensReadMsgSize = responseSize - roffs - FOOTER_LENGTH;
    if ((rval = mchMsgCheckSizes(sizeof(response), roffs, payloadSize))) {
        /* printf("mchMsgReadSensor size error\n"); */
        goto bail;
    }

    // printf("mchMsgReadSensor: responseSize %i roffs %i response+roffs %i payloadSize %i\n", (int)responseSize,
    // (int)roffs, (int)(response + roffs), (int)payloadSize);

    memcpy(data, response + roffs, payloadSize);

bail:
    return rval;
}

int mchMsgReadSensorWrapper(MchData mchData, uint8_t* data, Sensor sens, size_t* sensReadMsgSize) {
    int     bridged = 0;
    uint8_t rsAddr  = sens->sdr.owner;

    if (rsAddr != IPMI_MSG_ADDR_BMC) {
        bridged = 1;
    }
    return mchMsgReadSensor(mchData, data, sens->sdr.number, (sens->sdr.lun & 0x3), sensReadMsgSize, bridged, rsAddr);
}

static int mchMsgGetSensorThresholds(MchData mchData, uint8_t* data, uint8_t sens, uint8_t lun, int bridged,
                                     uint8_t rsAddr) {
    uint8_t response[MSG_MAX_LENGTH] = {0};
    size_t  roffs, responseSize = 0, payloadSize = IPMI_RPLY_IMSG2_GET_SENSOR_THRESH_LENGTH;
    int     rval;
    uint8_t rqAddr;

    mchSetSizeOffs(mchData->ipmiSess, payloadSize, &roffs, &responseSize, &bridged, &rsAddr, &rqAddr);

    if ((rval = ipmiMsgGetSensorThresholds(mchData->mchSess, mchData->ipmiSess, response, bridged, rsAddr, rqAddr, sens,
                                           lun, &responseSize, roffs))) {
        goto bail;
    }

    if ((rval = mchMsgCheckSizes(sizeof(response), roffs, payloadSize))) {
        printf("mchMsgGetSensorThresholds size error\n");
        goto bail;
    }

    memcpy(data, response + roffs, payloadSize);

bail:
    return rval;
}

int mchMsgGetSensorThresholdsWrapper(MchData mchData, uint8_t* data, Sensor sens) {

    int     bridged = 0;
    uint8_t rsAddr  = sens->sdr.owner;

    if (rsAddr != IPMI_MSG_ADDR_BMC) {
        bridged = 1;
    }
    return mchMsgGetSensorThresholds(mchData, data, sens->sdr.number, (sens->sdr.lun & 0x3), bridged, rsAddr);
}
