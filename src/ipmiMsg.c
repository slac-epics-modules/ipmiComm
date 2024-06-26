//////////////////////////////////////////////////////////////////////////////
// This file is part of 'ipmiComm'.
// It is subject to the license terms in the LICENSE.txt file found in the
// top-level directory of this distribution and at:
//    https://confluence.slac.stanford.edu/display/ppareg/LICENSE.html.
// No part of 'ipmiComm', including this file,
// may be copied, modified, propagated, or distributed except according to
// the terms contained in the LICENSE.txt file.
//////////////////////////////////////////////////////////////////////////////
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h> /* pow */

#include <errlog.h>
#include <epicsMutex.h>
#include <asynDriver.h>
#include <asynOctetSyncIO.h>
#include <iocsh.h>
#include <epicsExport.h>

#include <ipmiMsg.h>
#include <ipmiDef.h>
#include <picmgDef.h>

/*
 * Convert 2-element uint8_t array (which stores LS byte first) to 16-bit integer
 */
uint16_t arrayToUint16(uint8_t* data) {
    unsigned i;
    uint16_t value = 0;

    for (i = 0; i < 2; i++)
        value |= *data++ << i * 8;

    return value;
}

/*
 * Increment value of 2-element uint8_t array (which stores LS byte first)
 * Roll over to 0 when max possible value is reached.
 */
void incr2Uint8Array(uint8_t* data, int incr) {
    unsigned i, nBytes = 2;
    uint16_t value = 0;

    value = arrayToUint16(data);

    if (value >= (pow(2, 8 * nBytes) - incr))
        value = 0;
    else
        value += incr;
    /* Copy incremented value to array */
    for (i = 0; i < nBytes; i++)
        *data++ = (value >> i * 8) & 0xFF;
}

/*
 * Convert 4-element uint8_t array (which stores LS byte first) to 32-bit integer
 */
uint32_t arrayToUint32(uint8_t* data) {
    unsigned i;
    uint32_t value = 0;

    for (i = 0; i < 4; i++)
        value |= *data++ << i * 8;

    return value;
}

/*
 * Increment value of 4-element uint8_t array (which stores LS byte first)
 * Roll over to 0 when max possible value is reached.
 */
void incr4Uint8Array(uint8_t* data, int incr) {
    unsigned i, nBytes = 4;
    uint32_t value = 0;

    value = arrayToUint32(data);

    /* test rollover */
    if (value >= (pow(2, 8 * nBytes) - incr))
        value = 0;
    else
        value += incr;

    /* Copy incremented value to array */
    for (i = 0; i < nBytes; i++)
        *data++ = (value >> i * 8) & 0xFF;
}

/*
 * Given pointer to uint8_t data array and size of array,
 * calculate and return the two's complement checksum
 */
static uint8_t calcTwosComplementChecksum(uint8_t* data, unsigned n) {
    unsigned i;
    uint32_t cs = 0;

    n--;

    for (i = 0; i < n; i++) {
        cs += *data++;
    }

    return (0x100 - (cs & 0xFF));
}

/* Print command description and response completion code */
void ipmiCompletionCode(const char* name, uint8_t code, uint8_t cmd, uint8_t netfn) {
    char cmdStr[50];
    char codeStr[100];

    switch (code) {

        default:
            sprintf(codeStr, "Unspecified completion code");
            goto codeDone;

        case IPMI_COMP_CODE_NORMAL:
            sprintf(codeStr, "Command completed normally");
            goto codeDone;

        case IPMI_COMP_CODE_NODE_BUSY:
            sprintf(codeStr, "Node busy");
            goto codeDone;

        case IPMI_COMP_CODE_INVALID_COMMAND:
            sprintf(codeStr, "Invalid command");
            goto codeDone;

        case IPMI_COMP_CODE_INVALID_COMMAND_FOR_LUN:
            sprintf(codeStr, "Invalid command for given LUN");
            goto codeDone;

        case IPMI_COMP_CODE_TIMEOUT:
            sprintf(codeStr, "Timeout while processing command");
            goto codeDone;

        case IPMI_COMP_CODE_OUT_OF_SPACE:
            sprintf(codeStr, "Out of space");
            goto codeDone;

        case IPMI_COMP_CODE_RESERVATION:
            sprintf(codeStr, "Reservation cancelled or invalid reservation ID");
            goto codeDone;

        case IPMI_COMP_CODE_REQUEST_TRUNCATED:
            sprintf(codeStr, "Request data truncated");
            goto codeDone;

        case IPMI_COMP_CODE_REQUEST_LENGTH_INVALID:
            sprintf(codeStr, "Request data length invalid");
            goto codeDone;

        case IPMI_COMP_CODE_REQUEST_LENGTH_LIMIT:
            sprintf(codeStr, "Request data field length exceeded");
            goto codeDone;

        case IPMI_COMP_CODE_PARAMETER_RANGE:
            sprintf(codeStr, "Parameter out of range");
            goto codeDone;

        case IPMI_COMP_CODE_REQUESTED_BYTES:
            sprintf(codeStr, "Cannot return number of requested data bytes");
            goto codeDone;

        case IPMI_COMP_CODE_REQUESTED_DATA:
            sprintf(codeStr, "Requested sensor, data, or record not present");
            goto codeDone;

        case IPMI_COMP_CODE_INVALID_FIELD:
            sprintf(codeStr, "Invalid data field in request");
            goto codeDone;

        case IPMI_COMP_CODE_COMMAND_ILLEGAL:
            sprintf(codeStr, "Command illegal for specified sensor or record type");
            goto codeDone;

        case IPMI_COMP_CODE_COMMAND_RESPONSE:
            sprintf(codeStr, "Command response could not be provided");
            goto codeDone;

        case IPMI_COMP_CODE_DUPLICATED_REQUEST:
            sprintf(codeStr, "Cannot execute duplicated request");
            goto codeDone;

        case IPMI_COMP_CODE_SDR_REP_UPDATE:
            sprintf(codeStr, "Response could not be provided. SDR repository in update mode");
            goto codeDone;

        case IPMI_COMP_CODE_DEVICE_FW_UPDATE:
            sprintf(codeStr, "Response could not be provided. Device in firmware update mode");
            goto codeDone;

        case IPMI_COMP_CODE_BMC_INIT:
            sprintf(codeStr, "Response could not be provided. BMC initialization in progress");
            goto codeDone;

        case IPMI_COMP_CODE_DESTINATION_UNAVAIL:
            sprintf(codeStr, "Destination unavailable");
            goto codeDone;

        case IPMI_COMP_CODE_PRIVILEGE:
            sprintf(codeStr, "Insufficient privilege level or other security-based restriction");
            goto codeDone;

        case IPMI_COMP_CODE_NOT_SUPPORTED:
            sprintf(codeStr, "Command or request parameter not supported in present state");
            goto codeDone;

        case IPMI_COMP_CODE_SUBFUNCTION_UNAVAIL:
            sprintf(codeStr, "Parameter is illegal because sub-function is unavailable");
            goto codeDone;
    }

    if ((code >= IPMI_COMP_CODE_DEVICE_SPECIFIC_MIN) && (code <= IPMI_COMP_CODE_DEVICE_SPECIFIC_MAX))
        sprintf(codeStr, "Device-specific completion code %02x", code);

    else if ((code >= IPMI_COMP_CODE_COMMAND_SPECIFIC_MIN) && (code <= IPMI_COMP_CODE_DEVICE_SPECIFIC_MAX)) {

        switch (cmd) {

            default:
                sprintf(codeStr, "Command-specific completion code %02x", code);
                goto codeDone;

            case IPMI_MSG_CMD_SEND_MSG:

                switch (code) {

                    default:
                        sprintf(codeStr, "Send Message completion code %02x", code);
                        goto codeDone;

                    case 0x80:
                        sprintf(codeStr, "Invalid session handle");
                        goto codeDone;
                }
        }
    }

codeDone:

    switch (netfn) {

        default:
            break;

        case IPMI_MSG_NETFN_CHASSIS:

            switch (cmd) {

                default:
                    sprintf(cmdStr, "NetFn Chassis command %02x", cmd);
                    break;

                case IPMI_MSG_CMD_GET_CHAS_STATUS:
                    sprintf(cmdStr, "Get Chassis Status");
                    break;

                case IPMI_MSG_CMD_CHAS_CTRL:
                    sprintf(cmdStr, "Chassis Control");
                    break;
            }
            break;

        case IPMI_MSG_NETFN_APP_REQUEST:

            switch (cmd) {

                default:
                    sprintf(cmdStr, "NetFn App Request command %02x", cmd);
                    break;

                case IPMI_MSG_CMD_GET_CHAN_AUTH:
                    sprintf(cmdStr, "Get Channel Authentication Capabilities");
                    break;

                case IPMI_MSG_CMD_GET_SESSION_CHALLENGE:
                    sprintf(cmdStr, "Get Session Challenge");

                    switch (code) {

                        default:
                            break;
                        case IPMI_COMP_CODE_GET_SESS_INVALID_USER:
                            sprintf(codeStr, "Invalid user name");
                            break;
                        case IPMI_COMP_CODE_GET_SESS_NULL_USER:
                            sprintf(codeStr, "Null user name");
                            break;
                    }
                    break;

                case IPMI_MSG_CMD_ACTIVATE_SESSION:
                    sprintf(cmdStr, "Activate Session");

                    switch (code) {

                        default:
                            break;
                        case 0x81:
                            sprintf(codeStr, "No session slot available");
                            break;
                        case 0x82:
                            sprintf(codeStr, "No slot available for user");
                            break;
                        case 0x83:
                            sprintf(codeStr, "No slot available for privilege");
                            break;
                        case 0x84:
                            sprintf(codeStr, "Session sequence number out of range");
                            break;
                        case 0x85:
                            sprintf(codeStr, "Invalid session ID in request");
                            break;
                        case 0x86:
                            sprintf(codeStr, "Requested privilege exceeds user/channel limit");
                            break;
                    }
                    break;

                case IPMI_MSG_CMD_SET_PRIV_LEVEL:
                    sprintf(cmdStr, "Set Privilege Level");
                    break;

                case IPMI_MSG_CMD_CLOSE_SESSION:
                    sprintf(cmdStr, "Close Session");
                    break;

                case IPMI_MSG_CMD_SEND_MSG:
                    sprintf(cmdStr, "Send Message");
                    break;

                case IPMI_MSG_CMD_COLD_RESET:
                    sprintf(cmdStr, "Cold Reset");
                    break;
                case IPMI_MSG_CMD_GET_DEVICE_ID:
                    sprintf(cmdStr, "Get Device ID");
                    break;
            }
            break;

        case IPMI_MSG_NETFN_SENSOR_EVENT:

            switch (cmd) {

                default:
                    sprintf(cmdStr, "NetFn Sensor/Event command %02x", cmd);
                    break;

                case IPMI_MSG_CMD_SENSOR_READ:
                    sprintf(cmdStr, "Get Sensor Reading");
                    break;

                case IPMI_MSG_CMD_GET_DEV_SDR_INFO:
                    sprintf(cmdStr, "Get Device SDR Info");
                    break;

                case IPMI_MSG_CMD_GET_DEV_SDR:
                    sprintf(cmdStr, "Get Device SDR");
                    break;
            }
            break;

        case IPMI_MSG_NETFN_STORAGE:

            switch (cmd) {

                default:
                    sprintf(cmdStr, "NetFn Storage command %02x", cmd);
                    break;

                case IPMI_MSG_CMD_GET_FRU_INFO:
                    sprintf(cmdStr, "Get FRU Inventory Area Info");
                    break;

                case IPMI_MSG_CMD_READ_FRU_DATA:
                    sprintf(cmdStr, "Read FRU Data");
                    break;

                case IPMI_MSG_CMD_WRITE_FRU_DATA:
                    sprintf(cmdStr, "Write FRU Data");
                    break;

                case IPMI_MSG_CMD_GET_SDRREP_INFO:
                    sprintf(cmdStr, "Get SDR Repository Info");
                    break;

                case IPMI_MSG_CMD_RESERVE_SDRREP:
                    sprintf(cmdStr, "Reserve SDR Repository");
                    break;

                case IPMI_MSG_CMD_GET_SDR:
                    sprintf(cmdStr, "Get SDR");
                    break;
            }
            break;

        case IPMI_MSG_NETFN_PICMG:

            switch (cmd) {

                default:
                    sprintf(cmdStr, "NetFn PICMG command %02x", cmd);
                    break;

                case IPMI_MSG_CMD_SET_FRU_POLICY:
                    sprintf(cmdStr, "Set FRU Activation Policy");
                    break;

                case IPMI_MSG_CMD_SET_FRU_ACT:
                    sprintf(cmdStr, "Set FRU Activation");
                    break;

                case IPMI_MSG_CMD_GET_ADDR_INFO:
                    sprintf(cmdStr, "Get Address Info");
                    break;

                case IPMI_MSG_CMD_GET_POWER_LEVEL:
                    sprintf(cmdStr, "Get FRU Power Level");
                    break;

                case IPMI_MSG_CMD_GET_FAN_PROP:
                    sprintf(cmdStr, "Get Fan Properties");
                    break;

                case IPMI_MSG_CMD_GET_FAN_LEVEL:
                    sprintf(cmdStr, "Get Fan Level");
                    break;

                case IPMI_MSG_CMD_SET_FAN_LEVEL:
                    sprintf(cmdStr, "Set Fan Level");
                    break;
            }
    }

    printf("%s %s: %s\n", name, cmdStr, codeStr);
}

/*
 * Set authentication type based on message type and set
 * some message parameters based on auth type
 */
static void ipmiMsgSetImsg1Auth(IpmiSess sess, uint8_t cmd, uint8_t* auth, size_t* iwrapperSize, int* offset_nbytes) {
    /* Get Channel Authentication and Get Session Challenge messages use
     * authentication type none in message header even if session will use
     * another type
     */
    if ((cmd == IPMI_MSG_CMD_GET_CHAN_AUTH) || (cmd == IPMI_MSG_CMD_GET_SESSION_CHALLENGE) ||
        (sess->authReq == IPMI_MSG_AUTH_TYPE_NONE)) {
        *auth          = IPMI_MSG_AUTH_TYPE_NONE;
        *iwrapperSize  = IPMI_WRAPPER_LENGTH;
        *offset_nbytes = /*RMCP_MSG_HEADER_LENGTH +*/ IPMI_WRAPPER_NBYTES_OFFSET;
    } else {
        *auth          = sess->authReq;
        *iwrapperSize  = IPMI_WRAPPER_AUTH_LENGTH;
        *offset_nbytes = /*RMCP_MSG_HEADER_LENGTH +*/ IPMI_WRAPPER_AUTH_NBYTES_OFFSET;
    }
}

/*
 * Set session sequence number and session ID for IPMI message,
 * depending on message type. Copy to header.
 */
static void ipmiMsgSetSeqId(IpmiSess sess, uint8_t* message, uint8_t cmd) {
    int i;
    /* Session sequence number is not incremented (or used) for messages outside of a session */
    if (cmd != IPMI_MSG_CMD_GET_CHAN_AUTH && cmd != IPMI_MSG_CMD_GET_SESSION_CHALLENGE &&
        cmd != IPMI_MSG_CMD_ACTIVATE_SESSION && cmd != IPMI_MSG_CMD_SET_PRIV_LEVEL && cmd != 0)
        incr4Uint8Array(sess->seqSend, 1);

    /* Close Session command contains the session ID */
    if (cmd == IPMI_MSG_CMD_CLOSE_SESSION) {
        for (i = 0; i < IPMI_MSG2_ID_LENGTH; i++)
            message[IPMI_MSG2_ID_OFFSET + i] = sess->id[i];
    }

    /* Copy data to IPMI wrapper */
    for (i = 0; i < IPMI_WRAPPER_SEQ_LENGTH; i++)
        message[IPMI_WRAPPER_SEQ_OFFSET + i] = sess->seqSend[i];

    for (i = 0; i < IPMI_WRAPPER_ID_LENGTH; i++)
        message[IPMI_WRAPPER_ID_OFFSET + i] = sess->id[i];
}

/*
 *
 * Build IPMI outgoing message.
 *
 * Message structure (see ipmiMsg.h for more details):
 *
 * cs = checksum
 *
 * {RMCP header}{IPMI wrapper}{IPMI msg 1(cs)}{IPMI msg 2 [Bridged msg part1(cs)][Bridged msg part2 [Bridged msg
 * part1(cs)][Bridged msg part2(cs)](checksum)] (checksum)}
 *                                                        |_______________________________________||____________________________________________|
 * |	 | |                                          |                             |	 | First optional bridged
 * message              Second optional bridged message            cs for	 cs for IPMI msg part2 optional	     (does
 * not include first bridged   bridged msgs) msg part2 (does not include second bridged msg) RETURNS: number of bytes in
 * message
 *
 *
 *   Arguments:
 *                sess       - Pointer to IPMI session data structure
 *                message    - Pointer to message array
 *                cmd        - Command code
 *                imsg1netfn - Pointer to NETFN code used in IPMI msg 1; 0 if default is used
 *                imsg2      - Pointer to IPMI message 2
 *                imsg2Size  - Size of IPMI message 2
 *                b1msg1     - Pointer to IPMI message 1 of optional first bridged message;  0 if not used
 *                b1msg2     - Pointer to IPMI message 2 of optional first bridged message;  0 if not used
 *                b1msg2Size - Size    of IPMI message 2 of optional first bridged message;  0 if not used
 *                b2msg1     - Pointer to IPMI message 1 of optional second bridged message; 0 if not used
 *                b2msg2     - Pointer to IPMI message 2 of optional second bridged message; 0 if not used
 *                b2msg2Size - Size    of IPMI message 2 of optional second bridged message; 0 if not used
 *                auth       - Authentication type to use for message wrapper (depending on msg type, may be different
 * from the type specified in imsg2)
 */
int ipmiMsgBuild(IpmiSess sess, uint8_t* message, uint8_t cmd, uint8_t imsg1netfn, uint8_t* imsg2, size_t imsg2Size,
                 uint8_t* b1msg1, uint8_t* b1msg2, size_t b1msg2Size, uint8_t* b2msg1, uint8_t* b2msg2,
                 size_t b2msg2Size) {
    size_t  iwrapperSize;
    int     offset_nbytes;
    uint8_t auth;
    size_t  imsg1Size = sizeof(IPMI_MSG1);
    uint8_t imsg1[imsg1Size];
    int     n, i, offset = 0;
    uint8_t cs2, b1cs2; /* imsg2 and b1msg2 checksums */

    ipmiMsgSetImsg1Auth(sess, cmd, &auth, &iwrapperSize, &offset_nbytes);

    /* Must be called after ipmiMsgSetImsg1Auth unless we change this to malloc later */
    uint8_t iwrapper[iwrapperSize];

    switch (auth) {

        default:
            printf("ipmiMsgBuild: unsupported authentication type %i\n", auth);
            return 0;

        case IPMI_MSG_AUTH_TYPE_NONE:
            memcpy(iwrapper, IPMI_WRAPPER, iwrapperSize);
            break;

        case IPMI_MSG_AUTH_TYPE_PWD_KEY:
            memcpy(iwrapper, IPMI_WRAPPER_PWD_KEY, iwrapperSize);
            break;
    }

    memcpy(imsg1, IPMI_MSG1, imsg1Size);

    imsg1[IPMI_MSG1_NETFNLUN_OFFSET] = imsg1netfn << 2;

    ipmiMsgSetSeqId(sess, iwrapper, cmd);

    /* Activate Session command echoes the challenge string */
    if (cmd == IPMI_MSG_CMD_ACTIVATE_SESSION) {
        for (i = 0; i < IPMI_MSG2_STR_LENGTH; i++)
            imsg2[IPMI_MSG2_STR_OFFSET + i] = sess->str[i];
    }

    /* Number of bytes in message */
    iwrapper[offset_nbytes] = (b1msg1 && b2msg1) ? 3 * imsg1Size + imsg2Size + b1msg2Size + b2msg2Size
                              : (b1msg1)         ? 2 * imsg1Size + imsg2Size + b1msg2Size
                                                 : imsg1Size + imsg2Size;

    n = sizeof(RMCP_HEADER) + iwrapperSize + iwrapper[offset_nbytes];

    /* Build message */
    memcpy(message, RMCP_HEADER, sizeof(RMCP_HEADER));

    offset += sizeof(RMCP_HEADER);

    memcpy(message + offset, iwrapper, iwrapperSize);

    /* Set IPMI sequence number */
    if (sess->seq >= 0x3F)
        sess->seq = 1;
    else
        sess->seq++;

    imsg2[IPMI_MSG2_SEQLUN_OFFSET] |= (sess->seq << 2);

    /* Calculate checksums */
    imsg1[imsg1Size - 1] = calcTwosComplementChecksum((uint8_t*)imsg1, imsg1Size);
    cs2 = imsg2[imsg2Size - 1] = calcTwosComplementChecksum((uint8_t*)imsg2, imsg2Size);

    offset += iwrapperSize;
    memcpy(message + offset, imsg1, imsg1Size);

    offset += imsg1Size;

    /* Copy optional bridged messages and checksums to message array */
    if (b1msg1) {

        b1msg1[imsg1Size - 1] = calcTwosComplementChecksum((uint8_t*)b1msg1, imsg1Size);
        b1cs2 = b1msg2[b1msg2Size - 1] = calcTwosComplementChecksum((uint8_t*)b1msg2, b1msg2Size);

        memcpy(message + offset, imsg2, imsg2Size - 1);

        offset += imsg2Size - 1;
        memcpy(message + offset, b1msg1, imsg1Size);

        offset += imsg1Size;

        if (b2msg1) {

            b2msg1[imsg1Size - 1]  = calcTwosComplementChecksum((uint8_t*)b2msg1, imsg1Size);
            b2msg2[b2msg2Size - 1] = calcTwosComplementChecksum((uint8_t*)b2msg2, b2msg2Size);

            memcpy(message + offset, b1msg2, b1msg2Size - 1);

            offset += b1msg2Size - 1;
            memcpy(message + offset, b2msg1, imsg1Size);

            offset += imsg1Size;
            memcpy(message + offset, b2msg2, b2msg2Size);

            offset += b2msg2Size;
            memcpy(message + offset, &b1cs2, 1);
            offset++;
        } else {
            memcpy(message + offset, b1msg2, b1msg2Size);
            offset += b1msg2Size;
        }

        memcpy(message + offset, &cs2, 1);
    } else
        memcpy(message + offset, imsg2, imsg2Size);

    return n;
}

/*
 * Send message and read response
 *
 * responseSize is the expected response length. It is 0 if the response length is not known.
 * In that case, set it to MSG_MAX_LENGTH.  When we don't know the response length,
 * asyn may return status 'timeout', which we ignore.
 *
 *   RETURNS: asyn status from write/read
 */
int ipmiMsgWriteRead(const char* name, uint8_t* message, size_t messageSize, uint8_t* response, size_t* responseSize,
                     double timeout, size_t* responseLen) {
    size_t     numSent;
    int        eomReason;
    asynStatus status;
    asynUser*  pasynUser;

    if ((status = pasynOctetSyncIO->connect(name, 0, &pasynUser, NULL))) {
        return status;
    }

    if (*responseSize == 0)
        *responseSize = MSG_MAX_LENGTH;

    memset(response, 0, *responseSize); /* Initialize response to 0s in order to detect empty bytes ? */

    status = pasynOctetSyncIO->writeRead(pasynUser, (const char*)message, messageSize, (char*)response, *responseSize,
                                         timeout, &numSent, responseLen, &eomReason);

    pasynOctetSyncIO->disconnect(pasynUser);

    return status;
}

/*
 * To start a session, perform the following sequence of messages:
 *
 *    Get Channel Authentication Capabilities
 *    Get Session Challenge
 *    Activate Session
 *    Set Session Privilege
 *
 * Messages sent outside of a session or to close a session
 * use ipmiMsgWriteRead directly, which does not try to start a new session.
 */

/* Get Channel Authentication Capabilities */
int ipmiMsgGetChanAuth(void* device, IpmiSess sess, uint8_t* data, size_t* responseSize, size_t roffs) {
    uint8_t  message[MSG_MAX_LENGTH] = {0};
    uint8_t* imsg2                   = GET_AUTH_MSG;
    size_t   imsg2Size               = sizeof(GET_AUTH_MSG);
    uint8_t  cmd                     = IPMI_MSG_CMD_GET_CHAN_AUTH;
    size_t   messageSize =
        ipmiMsgBuild(sess, message, cmd, IPMI_MSG_NETFN_APP_REQUEST, imsg2, imsg2Size, 0, 0, 0, 0, 0, 0);

    return sess->wrf(device, sess, message, messageSize, data, responseSize, cmd, IPMI_MSG_NETFN_APP_REQUEST, roffs, 1);
}

/* Get Session Challenge */
int ipmiMsgGetSess(void* device, IpmiSess sess, uint8_t* data, size_t* responseSize, uint8_t* msg, size_t roffs) {
    uint8_t  message[MSG_MAX_LENGTH] = {0};
    uint8_t* imsg2                   = msg;    // temporary - GET_SESS_MSG;
    size_t   imsg2Size = sizeof(GET_SESS_MSG); /* If we keep drvMchGetSess, then it should pass message size here */
    uint8_t  cmd       = IPMI_MSG_CMD_GET_SESSION_CHALLENGE;
    size_t   messageSize =
        ipmiMsgBuild(sess, message, cmd, IPMI_MSG_NETFN_APP_REQUEST, imsg2, imsg2Size, 0, 0, 0, 0, 0, 0);

    /* Set authentication method */
    imsg2[IPMI_MSG2_AUTH_TYPE_OFFSET] = sess->authReq;

    return sess->wrf(device, sess, message, messageSize, data, responseSize, cmd, IPMI_MSG_NETFN_APP_REQUEST, roffs, 1);
}

/* Activate Session */
int ipmiMsgActSess(void* device, IpmiSess sess, uint8_t* data, size_t* responseSize, size_t roffs) {
    uint8_t  message[MSG_MAX_LENGTH] = {0};
    uint8_t* imsg2                   = ACT_SESS_MSG;
    size_t   imsg2Size               = sizeof(ACT_SESS_MSG);
    uint8_t  cmd                     = IPMI_MSG_CMD_ACTIVATE_SESSION;
    size_t   messageSize;
    int      i;

    /* Copy challenge string */
    for (i = 0; i < IPMI_MSG2_STR_LENGTH; i++)
        imsg2[IPMI_MSG2_STR_OFFSET + i] = sess->str[i];

    /* Set authentication method */
    imsg2[IPMI_MSG2_AUTH_TYPE_OFFSET] = sess->authReq;

    messageSize = ipmiMsgBuild(sess, message, cmd, IPMI_MSG_NETFN_APP_REQUEST, imsg2, imsg2Size, 0, 0, 0, 0, 0, 0);

    return sess->wrf(device, sess, message, messageSize, data, responseSize, cmd, IPMI_MSG_NETFN_APP_REQUEST, roffs, 1);
}

/* Set Privilege Level */
int ipmiMsgSetPriv(void* device, IpmiSess sess, uint8_t* data, size_t* responseSize, uint8_t level, size_t roffs) {
    uint8_t  message[MSG_MAX_LENGTH] = {0};
    uint8_t* imsg2                   = SET_PRIV_MSG;
    size_t   imsg2Size               = sizeof(SET_PRIV_MSG);
    size_t   messageSize;
    uint8_t  cmd = IPMI_MSG_CMD_SET_PRIV_LEVEL;

    imsg2[IPMI_MSG2_PRIV_LEVEL_OFFSET] = IPMI_MSG_PRIV_LEVEL_OPER; // temporary override ... level;

    messageSize = ipmiMsgBuild(sess, message, cmd, IPMI_MSG_NETFN_APP_REQUEST, imsg2, imsg2Size, 0, 0, 0, 0, 0, 0);

    return sess->wrf(device, sess, message, messageSize, data, responseSize, cmd, IPMI_MSG_NETFN_APP_REQUEST, roffs, 1);
}

/* Close Session - test for NAT and determine reply offset*/
int ipmiMsgCloseSess(void* device, IpmiSess sess, uint8_t* data, size_t* responseSize) {
    uint8_t  message[MSG_MAX_LENGTH];
    uint8_t* imsg2     = CLOSE_SESS_MSG;
    size_t   imsg2Size = sizeof(CLOSE_SESS_MSG);
    uint8_t  cmd       = IPMI_MSG_CMD_CLOSE_SESSION;
    size_t   messageSize =
        ipmiMsgBuild(sess, message, cmd, IPMI_MSG_NETFN_APP_REQUEST, imsg2, imsg2Size, 0, 0, 0, 0, 0, 0);

    return sess->wrf(device, sess, message, messageSize, data, responseSize, cmd, IPMI_MSG_NETFN_APP_REQUEST,
                     0 /*roffs*/, 1);
}

/*
 * Cold Reset. Do not reconnect; next request message will handle reconnection
 *
 *   RETURNS: status from sess->wrf
 *            0 on success
 *            non-zero for error
 */
int ipmiMsgColdReset(void* device, IpmiSess sess, uint8_t* data) {
    uint8_t  message[MSG_MAX_LENGTH] = {0};
    uint8_t* imsg2                   = BASIC_MSG;
    size_t   imsg2Size               = sizeof(BASIC_MSG);
    size_t   messageSize;
    size_t   responseSize = 0;
    uint8_t  cmd          = IPMI_MSG_CMD_COLD_RESET;

    imsg2[IPMI_MSG2_CMD_OFFSET] = cmd;

    messageSize = ipmiMsgBuild(sess, message, cmd, IPMI_MSG_NETFN_APP_REQUEST, imsg2, imsg2Size, 0, 0, 0, 0, 0, 0);

    return sess->wrf(device, sess, message, messageSize, data, /* need to update this */ &responseSize, cmd,
                     IPMI_MSG_NETFN_APP_REQUEST, 0, 0);
}

/* Chassis Control
 *
 *   RETURNS: status from sess->wrf
 *            0 on success
 *            non-zero for error
 */
int ipmiMsgChassisControl(void* device, IpmiSess sess, uint8_t* data, int bridged, uint8_t rsAddr, uint8_t rqAddr,
                          uint8_t parm, size_t* responseSize, int roffs) {
    uint8_t message[MSG_MAX_LENGTH] = {0};
    uint8_t imsg2Size               = sizeof(CHAS_CTRL_MSG);
    uint8_t imsg2[imsg2Size];
    size_t  messageSize;
    uint8_t cmd   = IPMI_MSG_CMD_CHAS_CTRL;
    uint8_t netfn = IPMI_MSG_NETFN_CHASSIS;

    memcpy(imsg2, CHAS_CTRL_MSG, imsg2Size);
    imsg2[IPMI_MSG2_CMD_OFFSET] = cmd;

    imsg2[IPMI_MSG2_SENSOR_OFFSET] = parm;

    if (bridged) // may need to distinguish between once and twice-bridged messages
        ipmiBuildSendMsg(sess, message, &messageSize, cmd, netfn, rsAddr, rqAddr, imsg2, imsg2Size, 0);
    else
        messageSize = ipmiMsgBuild(sess, message, cmd, netfn, imsg2, imsg2Size, 0, 0, 0, 0, 0, 0);

    return sess->wrf(device, sess, message, messageSize, data, responseSize, cmd, netfn, roffs, 0);
}

/* Get Chassis Status
 *
 *   RETURNS: status from sess->wrf
 *            0 on success
 *            non-zero for error
 */
int ipmiMsgGetChassisStatus(void* device, IpmiSess sess, uint8_t* data, int bridged, uint8_t rsAddr, uint8_t rqAddr,
                            size_t* responseSize, int roffs) {
    uint8_t message[MSG_MAX_LENGTH] = {0};
    uint8_t imsg2Size               = sizeof(BASIC_MSG);
    uint8_t imsg2[imsg2Size];
    size_t  messageSize;
    uint8_t cmd   = IPMI_MSG_CMD_GET_CHAS_STATUS;
    uint8_t netfn = IPMI_MSG_NETFN_CHASSIS;

    memcpy(imsg2, BASIC_MSG, imsg2Size);
    imsg2[IPMI_MSG2_CMD_OFFSET] = cmd;

    if (bridged) // may need to distinguish between once and twice-bridged messages
        ipmiBuildSendMsg(sess, message, &messageSize, cmd, netfn, rsAddr, rqAddr, imsg2, imsg2Size, 0);
    else
        messageSize = ipmiMsgBuild(sess, message, cmd, netfn, imsg2, imsg2Size, 0, 0, 0, 0, 0, 0);

    return sess->wrf(device, sess, message, messageSize, data, responseSize, cmd, netfn, roffs, 0);
}

/* Get FRU Inventory Info
 *
 *   RETURNS: status from sess->wrf
 *            0 on success
 *            non-zero for error
 */
int ipmiMsgGetFruInvInfo(void* device, IpmiSess sess, uint8_t* data, int bridged, uint8_t rsAddr, uint8_t rqAddr,
                         uint8_t id, size_t* responseSize, int roffs) {
    uint8_t message[MSG_MAX_LENGTH] = {0};
    uint8_t imsg2Size               = sizeof(SENS_READ_MSG);
    uint8_t imsg2[imsg2Size];
    size_t  messageSize;
    uint8_t cmd   = IPMI_MSG_CMD_GET_FRU_INFO;
    uint8_t netfn = IPMI_MSG_NETFN_STORAGE;

    memcpy(imsg2, SENS_READ_MSG, imsg2Size);
    imsg2[IPMI_MSG2_CMD_OFFSET] = cmd;

    imsg2[IPMI_MSG2_SENSOR_OFFSET] = id;

    if (bridged) // may need to distinguish between once and twice-bridged messages
        ipmiBuildSendMsg(sess, message, &messageSize, cmd, netfn, rsAddr, rqAddr, imsg2, imsg2Size, 0);
    else
        messageSize = ipmiMsgBuild(sess, message, cmd, netfn, imsg2, imsg2Size, 0, 0, 0, 0, 0, 0);

    return sess->wrf(device, sess, message, messageSize, data, responseSize, cmd, netfn, roffs, 0);
}

/* Read FRU data
 *
 *   RETURNS: status from sess->wrf
 *            0 on success
 *            non-zero for error
 */
int ipmiMsgReadFru(void* device, IpmiSess sess, uint8_t* data, int bridged, uint8_t rsAddr, uint8_t rqAddr, uint8_t id,
                   uint8_t* readOffset, uint8_t readSize, size_t* responseSize, int roffs) {
    uint8_t message[MSG_MAX_LENGTH] = {0};
    uint8_t imsg2Size               = sizeof(FRU_READ_MSG);
    uint8_t imsg2[imsg2Size];
    size_t  messageSize;
    uint8_t cmd   = IPMI_MSG_CMD_READ_FRU_DATA;
    uint8_t netfn = IPMI_MSG_NETFN_STORAGE;

    memcpy(imsg2, FRU_READ_MSG, imsg2Size);
    imsg2[IPMI_MSG2_CMD_OFFSET] = cmd;

    imsg2[IPMI_MSG2_CMD_OFFSET]          = cmd;
    imsg2[IPMI_MSG2_READ_FRU_ID_OFFSET]  = id;
    imsg2[IPMI_MSG2_READ_FRU_LSB_OFFSET] = readOffset[0];
    imsg2[IPMI_MSG2_READ_FRU_MSB_OFFSET] = readOffset[1];
    imsg2[IPMI_MSG2_READ_FRU_CNT_OFFSET] = readSize;

    if (bridged) // may need to distinguish between once and twice-bridged messages
        ipmiBuildSendMsg(sess, message, &messageSize, cmd, netfn, rsAddr, rqAddr, imsg2, imsg2Size, 0);
    else
        messageSize = ipmiMsgBuild(sess, message, cmd, netfn, imsg2, imsg2Size, 0, 0, 0, 0, 0, 0);

    return sess->wrf(device, sess, message, messageSize, data, responseSize, cmd, netfn, roffs, 0);
}

/* Get device, sess Sensor Data Record (SDR) Info
 *
 *   RETURNS: status from sess->wrf
 *            0 on success
 *            non-zero for error
 *
int
ipmiMsgGetDevSdrInfo(void *device, IpmiSess sess, uint8_t *data, int bridged, uint8_t rsAddr, uint8_t rqAddr, uint8_t
parm, size_t *responseSize, int roffs)
{
uint8_t  message[MSG_MAX_LENGTH] = { 0 };
uint8_t  imsg2Size = sizeof( GET_DEV_SDR_INFO_MSG );
uint8_t  imsg2[imsg2Size];
size_t   messageSize;
uint8_t  cmd   = IPMI_MSG_CMD_GET_DEV_SDR_INFO;
uint8_t  netfn = IPMI_MSG_NETFN_SENSOR_EVENT;

    memcpy( imsg2, GET_DEV_SDR_INFO_MSG, imsg2Size );
    imsg2[IPMI_MSG2_CMD_OFFSET] = cmd;

    if ( bridged ) / may need to distinguish between once and twice-bridged messages
        ipmiBuildSendMsg( sess, message, &messageSize, cmd, netfn, rsAddr, rqAddr, imsg2, imsg2Size, 0 );
    else
        messageSize = ipmiMsgBuild( sess, message, cmd, netfn, imsg2, imsg2Size, 0, 0, 0, 0, 0, 0 );

    return sess->wrf( device, sess, message, messageSize, data, responseSize, cmd, netfn, roffs, 0 );
}
*/

/* Get SDR (Sensor Data Record) Repository Info
 *
 * Used for both Get SDR (parm = 0) and Get device, sess SDR (parm = 1).
 * Only differences in the message are the network function and command code.
 *
 *   RETURNS: status from sess->wrf
 *            0 on success
 *            non-zero for error
 */
int ipmiMsgGetSdrRepInfo(void* device, IpmiSess sess, uint8_t* data, int bridged, uint8_t rsAddr, uint8_t rqAddr,
                         size_t* responseSize, int roffs, uint8_t parm) {
    uint8_t message[MSG_MAX_LENGTH] = {0};
    uint8_t imsg2Size = (parm == IPMI_SDRREP_PARM_GET_DEV_SDR) ? sizeof(GET_DEV_SDR_INFO_MSG) : sizeof(BASIC_MSG);
    uint8_t imsg2[imsg2Size];
    size_t  messageSize;
    uint8_t cmd = (parm == IPMI_SDRREP_PARM_GET_DEV_SDR) ? IPMI_MSG_CMD_GET_DEV_SDR_INFO : IPMI_MSG_CMD_GET_SDRREP_INFO;
    uint8_t netfn = (parm == IPMI_SDRREP_PARM_GET_DEV_SDR) ? IPMI_MSG_NETFN_SENSOR_EVENT : IPMI_MSG_NETFN_STORAGE;

    if (parm == IPMI_SDRREP_PARM_GET_DEV_SDR)
        memcpy(imsg2, GET_DEV_SDR_INFO_MSG, imsg2Size);
    else
        memcpy(imsg2, BASIC_MSG, imsg2Size);
    imsg2[IPMI_MSG2_CMD_OFFSET] = cmd;

    if (bridged) // may need to distinguish between once and twice-bridged messages
        ipmiBuildSendMsg(sess, message, &messageSize, cmd, netfn, rsAddr, rqAddr, imsg2, imsg2Size, 0);
    else
        messageSize = ipmiMsgBuild(sess, message, cmd, netfn, imsg2, imsg2Size, 0, 0, 0, 0, 0, 0);

    return sess->wrf(device, sess, message, messageSize, data, responseSize, cmd, netfn, roffs, 0);
}

/* Reserve SDR (Sensor Data Record) Repository
 *
 * Used for both Get SDR (parm = 0) and Get device, sess SDR (parm = 1).
 * Only differences in the message are the network function and command code.
 *
 *   RETURNS: status from sess->wrf
 *            0 on success
 *            non-zero for error
 */
int ipmiMsgReserveSdrRep(void* device, IpmiSess sess, uint8_t* data, int bridged, uint8_t rsAddr, uint8_t rqAddr,
                         size_t* responseSize, int roffs, uint8_t parm) {
    uint8_t message[MSG_MAX_LENGTH] = {0};
    uint8_t imsg2Size               = sizeof(BASIC_MSG);
    uint8_t imsg2[imsg2Size];
    size_t  messageSize;
    uint8_t cmd =
        (parm == IPMI_SDRREP_PARM_GET_DEV_SDR) ? IPMI_MSG_CMD_RESERVE_DEV_SDRREP : IPMI_MSG_CMD_RESERVE_SDRREP;
    uint8_t netfn = (parm == IPMI_SDRREP_PARM_GET_DEV_SDR) ? IPMI_MSG_NETFN_SENSOR_EVENT : IPMI_MSG_NETFN_STORAGE;

    memcpy(imsg2, BASIC_MSG, imsg2Size);
    imsg2[IPMI_MSG2_CMD_OFFSET] = cmd;

    if (bridged) // may need to distinguish between once and twice-bridged messages
        ipmiBuildSendMsg(sess, message, &messageSize, cmd, netfn, rsAddr, rqAddr, imsg2, imsg2Size, 0);
    else
        messageSize = ipmiMsgBuild(sess, message, cmd, netfn, imsg2, imsg2Size, 0, 0, 0, 0, 0, 0);

    return sess->wrf(device, sess, message, messageSize, data, responseSize, cmd, netfn, roffs, 0);
}

/*
 * Get SDR (Sensor Data Record)
 *
 * Offset and reservation ID are 0 unless doing partial read that begins at an
 * offset into the SDR other than 0.
 *
 * Used for both Get SDR (parm = 0) and Get device, sess SDR (parm = 1).
 * Only differences in the message are the network function and command code.
 *
 *   RETURNS: status from sess->wrf
 *            0 on success
 *            non-zero for error
 */
int ipmiMsgGetSdr(void* device, IpmiSess sess, uint8_t* data, int bridged, uint8_t rsAddr, uint8_t rqAddr, uint8_t* id,
                  uint8_t* res, uint8_t offset, uint8_t readSize, size_t* responseSize, int roffs, uint8_t parm) {
    uint8_t message[MSG_MAX_LENGTH] = {0};
    uint8_t imsg2Size               = sizeof(GET_SDR_MSG);
    uint8_t imsg2[imsg2Size];
    size_t  messageSize;
    uint8_t cmd   = (parm == IPMI_SDRREP_PARM_GET_DEV_SDR) ? IPMI_MSG_CMD_GET_DEV_SDR : IPMI_MSG_CMD_GET_SDR;
    uint8_t netfn = (parm == IPMI_SDRREP_PARM_GET_DEV_SDR) ? IPMI_MSG_NETFN_SENSOR_EVENT : IPMI_MSG_NETFN_STORAGE;

    memcpy(imsg2, GET_SDR_MSG, imsg2Size);
    imsg2[IPMI_MSG2_CMD_OFFSET] = cmd;

    imsg2[IPMI_MSG2_CMD_OFFSET]             = cmd;
    imsg2[IPMI_MSG2_GET_SDR_RES_LSB_OFFSET] = res[0];
    imsg2[IPMI_MSG2_GET_SDR_RES_MSB_OFFSET] = res[1];
    imsg2[IPMI_MSG2_GET_SDR_ID_LSB_OFFSET]  = id[0];
    imsg2[IPMI_MSG2_GET_SDR_ID_MSB_OFFSET]  = id[1];
    imsg2[IPMI_MSG2_GET_SDR_OFFSET_OFFSET]  = offset;
    imsg2[IPMI_MSG2_GET_SDR_CNT_OFFSET]     = readSize;

    if (bridged) // may need to distinguish between once and twice-bridged messages
        ipmiBuildSendMsg(sess, message, &messageSize, cmd, netfn, rsAddr, rqAddr, imsg2, imsg2Size, 0);
    else
        messageSize = ipmiMsgBuild(sess, message, cmd, netfn, imsg2, imsg2Size, 0, 0, 0, 0, 0, 0);

    return sess->wrf(device, sess, message, messageSize, data, responseSize, cmd, netfn, roffs, 0);
}

/* Get Sensor Reading. Caller specifies expected message response length.
 *
 *   RETURNS: status from sess->wrf
 *            0 on success
 *            non-zero for error
 */
int ipmiMsgReadSensor(void* device, IpmiSess sess, uint8_t* data, uint8_t bridged, uint8_t rsAddr, uint8_t rqAddr,
                      uint8_t sens, uint8_t lun, size_t* responseSize, int roffs) {
    uint8_t message[MSG_MAX_LENGTH] = {0};
    uint8_t imsg2Size               = sizeof(SENS_READ_MSG);
    uint8_t imsg2[imsg2Size];
    size_t  messageSize;
    uint8_t cmd   = IPMI_MSG_CMD_SENSOR_READ;
    uint8_t netfn = IPMI_MSG_NETFN_SENSOR_EVENT;

    memcpy(imsg2, SENS_READ_MSG, imsg2Size);
    imsg2[IPMI_MSG2_CMD_OFFSET] = cmd;

    imsg2[IPMI_MSG2_SENSOR_OFFSET] = sens;

    if (bridged) // may need to distinguish between once and twice-bridged messages
        ipmiBuildSendMsg(sess, message, &messageSize, cmd, netfn, rsAddr, rqAddr, imsg2, imsg2Size, lun);
    else
        messageSize = ipmiMsgBuild(sess, message, cmd, netfn, imsg2, imsg2Size, 0, 0, 0, 0, 0, 0);

    return sess->wrf(device, sess, message, messageSize, data, responseSize, cmd, netfn, roffs, 0);
}

/* Get Sensor Thresholds
 *
 *   RETURNS: status from sess->wrf
 *            0 on success
 *            non-zero for error
 */
int ipmiMsgGetSensorThresholds(void* device, IpmiSess sess, uint8_t* data, uint8_t bridged, uint8_t rsAddr,
                               uint8_t rqAddr, uint8_t sens, uint8_t lun, size_t* responseSize, int roffs) {
    uint8_t message[MSG_MAX_LENGTH] = {0};
    uint8_t imsg2Size               = sizeof(GET_SENSOR_THRESH_MSG);
    uint8_t imsg2[imsg2Size];
    size_t  messageSize;
    uint8_t cmd   = IPMI_MSG_CMD_GET_SENSOR_THRESH;
    uint8_t netfn = IPMI_MSG_NETFN_SENSOR_EVENT;

    memcpy(imsg2, GET_SENSOR_THRESH_MSG, imsg2Size);
    imsg2[IPMI_MSG2_CMD_OFFSET] = cmd;

    imsg2[IPMI_MSG2_SENSOR_OFFSET] = sens;

    if (bridged) // may need to distinguish between once and twice-bridged messages
        ipmiBuildSendMsg(sess, message, &messageSize, cmd, netfn, rsAddr, rqAddr, imsg2, imsg2Size, lun);
    else
        messageSize = ipmiMsgBuild(sess, message, cmd, netfn, imsg2, imsg2Size, 0, 0, 0, 0, 0, 0);

    return sess->wrf(device, sess, message, messageSize, data, responseSize, cmd, netfn, roffs, 0);
}

/* Get device, sess ID
 *
 *   RETURNS: status from sess->wrf
 *            0 on success
 *            non-zero for error
 */
int ipmiMsgGetDeviceId(void* device, IpmiSess sess, uint8_t* data, int bridged, uint8_t rsAddr, uint8_t rqAddr,
                       size_t* responseSize, int roffs) {
    uint8_t message[MSG_MAX_LENGTH] = {0};
    uint8_t imsg2Size               = sizeof(BASIC_MSG);
    uint8_t imsg2[imsg2Size];
    size_t  messageSize;
    uint8_t cmd   = IPMI_MSG_CMD_GET_DEVICE_ID;
    uint8_t netfn = IPMI_MSG_NETFN_APP_REQUEST;

    memcpy(imsg2, BASIC_MSG, imsg2Size);
    imsg2[IPMI_MSG2_CMD_OFFSET] = cmd;

    if (bridged) // may need to distinguish between once and twice-bridged messages
        ipmiBuildSendMsg(sess, message, &messageSize, cmd, netfn, rsAddr, rqAddr, imsg2, imsg2Size, 0);
    else
        messageSize = ipmiMsgBuild(sess, message, cmd, netfn, imsg2, imsg2Size, 0, 0, 0, 0, 0, 0);

    return sess->wrf(device, sess, message, messageSize, data, responseSize, cmd, netfn, roffs, 0);
}
/* Get device, sess ID
 *
 *   RETURNS: status from sess->wrf
 *            0 on success
 *            non-zero for error
 */
int ipmiMsgBroadcastGetDeviceId(void* device, IpmiSess sess, uint8_t* data, int bridged, uint8_t rsAddr, uint8_t rqAddr,
                                size_t* responseSize, int roffs) {
    uint8_t message[MSG_MAX_LENGTH] = {0};
    uint8_t imsg2Size               = sizeof(BASIC_MSG);
    uint8_t imsg2[imsg2Size];
    size_t  messageSize;
    uint8_t cmd   = IPMI_MSG_CMD_GET_DEVICE_ID;
    uint8_t netfn = IPMI_MSG_NETFN_APP_REQUEST;

    memcpy(imsg2, BASIC_MSG, imsg2Size);
    imsg2[IPMI_MSG2_CMD_OFFSET] = cmd;

    if (bridged) // may need to distinguish between once and twice-bridged messages
        ipmiBuildSendMsg(sess, message + 1, &messageSize, cmd, netfn, rsAddr, rqAddr, imsg2, imsg2Size, 0);
    else
        messageSize = ipmiMsgBuild(sess, message, cmd, netfn, imsg2, imsg2Size, 0, 0, 0, 0, 0, 0);

    return sess->wrf(device, sess, message, messageSize, data, responseSize, cmd, netfn, roffs, 0);
}

/* Name this PICMG? */
/* Get Address Info -  this is version that attempts to get physical location
 *                     information for a specific device using IPMB-0 address
 *
 *   RETURNS: status from sess->wrf
 *            0 on success
 *            non-zero for error
 */
int ipmiMsgGetAddressInfoIpmb0(void* device, IpmiSess sess, uint8_t* data, int bridged, uint8_t rsAddr, uint8_t rqAddr,
                               size_t* responseSize, int roffs, uint8_t fru, uint8_t key) {
    uint8_t message[MSG_MAX_LENGTH] = {0};
    uint8_t imsg2Size               = sizeof(GET_ADDR_INFO_MSG);
    uint8_t imsg2[imsg2Size];
    size_t  messageSize;
    uint8_t cmd   = IPMI_MSG_CMD_GET_ADDR_INFO;
    uint8_t netfn = IPMI_MSG_NETFN_PICMG;

    memcpy(imsg2, GET_ADDR_INFO_MSG, imsg2Size);
    imsg2[IPMI_MSG2_RQADDR_OFFSET] = IPMI_MSG_ADDR_SW;
    imsg2[IPMI_MSG2_CMD_OFFSET]    = cmd;

    imsg2[PICMG_IMSG2_GET_ADDR_INFO_FRUID_OFFSET]    = fru;
    imsg2[PICMG_IMSG2_GET_ADDR_INFO_KEY_TYPE_OFFSET] = PICMG_ADDR_KEY_TYPE_IPMB0;
    imsg2[PICMG_IMSG2_GET_ADDR_INFO_KEY_OFFSET]      = key;
    // if using different routines for different types of get hw address requests, may not need to pass these
    // values as arguments; can be hard-coded in routine

    if (bridged) // may need to distinguish between once and twice-bridged messages
        ipmiBuildSendMsg(sess, message, &messageSize, cmd, netfn, rsAddr, rqAddr, imsg2, imsg2Size, 0);
    else
        messageSize = ipmiMsgBuild(sess, message, cmd, netfn, imsg2, imsg2Size, 0, 0, 0, 0, 0, 0);

    return sess->wrf(device, sess, message, messageSize, data, responseSize, cmd, netfn, roffs, 0);
}

/* Name this PICMG? */
/* Get Address Info -  this is version that attempts to get physical location
 *                     information for a specific device
 *
 *   RETURNS: status from sess->wrf
 *            0 on success
 *            non-zero for error
 */
int ipmiMsgGetAddressInfoHwAddr(void* device, IpmiSess sess, uint8_t* data, int bridged, uint8_t rsAddr, uint8_t rqAddr,
                                size_t* responseSize, int roffs, uint8_t fru, uint8_t keytype, uint8_t key,
                                uint8_t sitetype) {
    uint8_t message[MSG_MAX_LENGTH] = {0};
    uint8_t imsg2Size               = sizeof(GET_ADDR_INFO_MSG);
    uint8_t imsg2[imsg2Size];
    size_t  messageSize;
    uint8_t cmd   = IPMI_MSG_CMD_GET_ADDR_INFO;
    uint8_t netfn = IPMI_MSG_NETFN_PICMG;

    memcpy(imsg2, GET_ADDR_INFO_MSG, imsg2Size);
    imsg2[IPMI_MSG2_RQADDR_OFFSET] = IPMI_MSG_ADDR_SW;
    imsg2[IPMI_MSG2_CMD_OFFSET]    = cmd;

    imsg2[PICMG_IMSG2_GET_ADDR_INFO_FRUID_OFFSET]     = fru;
    imsg2[PICMG_IMSG2_GET_ADDR_INFO_KEY_TYPE_OFFSET]  = keytype;
    imsg2[PICMG_IMSG2_GET_ADDR_INFO_KEY_OFFSET]       = key;
    imsg2[PICMG_IMSG2_GET_ADDR_INFO_SITE_TYPE_OFFSET] = sitetype;
    // if using different routines for different types of get hw address requests, may not need to pass these
    // values as arguments; can be hard-coded in routine

    if (bridged) // may need to distinguish between once and twice-bridged messages
        ipmiBuildSendMsg(sess, message, &messageSize, cmd, netfn, rsAddr, rqAddr, imsg2, imsg2Size, 0);
    else
        messageSize = ipmiMsgBuild(sess, message, cmd, netfn, imsg2, imsg2Size, 0, 0, 0, 0, 0, 0);

    return sess->wrf(device, sess, message, messageSize, data, responseSize, cmd, netfn, roffs, 0);
}

/* Name this PICMG? */
/* Get Address Info - this is version that does not seek information
 *                    about a specific FRU/hardware, and will receive info
 *                    about the IPM Controller that implements the command
 *   RETURNS: status from sess->wrf
 *            0 on success
 *            non-zero for error
 */
int ipmiMsgGetAddressInfoIpmc(void* device, IpmiSess sess, uint8_t* data, int bridged, uint8_t rsAddr, uint8_t rqAddr,
                              size_t* responseSize, int roffs) {
    uint8_t message[MSG_MAX_LENGTH] = {0};
    uint8_t imsg2Size               = sizeof(GET_PICMG_PROP_MSG);
    uint8_t imsg2[imsg2Size];
    size_t  messageSize;
    uint8_t cmd   = IPMI_MSG_CMD_GET_ADDR_INFO;
    uint8_t netfn = IPMI_MSG_NETFN_PICMG;

    memcpy(imsg2, GET_PICMG_PROP_MSG, imsg2Size);
    imsg2[IPMI_MSG2_RQADDR_OFFSET] = IPMI_MSG_ADDR_SW;
    imsg2[IPMI_MSG2_CMD_OFFSET]    = cmd;

    if (bridged) // may need to distinguish between once and twice-bridged messages
        ipmiBuildSendMsg(sess, message, &messageSize, cmd, netfn, rsAddr, rqAddr, imsg2, imsg2Size, 0);
    else
        messageSize = ipmiMsgBuild(sess, message, cmd, netfn, imsg2, imsg2Size, 0, 0, 0, 0, 0, 0);

    return sess->wrf(device, sess, message, messageSize, data, responseSize, cmd, netfn, roffs, 0);
}

/* Get PICMG properties
 *
 *   RETURNS: status from sess->wrf
 *            0 on success
 *            non-zero for error
 */
int ipmiMsgGetPicmgProp(void* device, IpmiSess sess, uint8_t* data, int bridged, uint8_t rsAddr, uint8_t rqAddr,
                        size_t* responseSize, int roffs) {
    uint8_t message[MSG_MAX_LENGTH] = {0};
    uint8_t imsg2Size               = sizeof(GET_PICMG_PROP_MSG);
    uint8_t imsg2[imsg2Size];
    size_t  messageSize;
    uint8_t cmd   = IPMI_MSG_CMD_GET_PICMG_PROP;
    uint8_t netfn = IPMI_MSG_NETFN_PICMG;

    memcpy(imsg2, GET_PICMG_PROP_MSG, imsg2Size);
    imsg2[IPMI_MSG2_RQADDR_OFFSET] = IPMI_MSG_ADDR_SW;
    imsg2[IPMI_MSG2_CMD_OFFSET]    = cmd;

    if (bridged) // may need to distinguish between once and twice-bridged messages
        ipmiBuildSendMsg(sess, message, &messageSize, cmd, netfn, rsAddr, rqAddr, imsg2, imsg2Size, 0);
    else
        messageSize = ipmiMsgBuild(sess, message, cmd, netfn, imsg2, imsg2Size, 0, 0, 0, 0, 0, 0);

    return sess->wrf(device, sess, message, messageSize, data, responseSize, cmd, netfn, roffs, 0);
}

/* Get power level - PICMG command
 *
 *   RETURNS: status from sess->wrf
 *            0 on success
 *            non-zero for error
 */
int ipmiMsgGetPowerLevel(void* device, IpmiSess sess, uint8_t* data, int bridged, uint8_t rsAddr, uint8_t rqAddr,
                         size_t* responseSize, int roffs, uint8_t fruId, uint8_t parm) {
    uint8_t message[MSG_MAX_LENGTH] = {0};
    uint8_t imsg2Size               = sizeof(GET_POWER_LEVEL_MSG);
    uint8_t imsg2[imsg2Size];
    size_t  messageSize;
    uint8_t cmd   = IPMI_MSG_CMD_GET_POWER_LEVEL;
    uint8_t netfn = IPMI_MSG_NETFN_PICMG;

    memcpy(imsg2, GET_POWER_LEVEL_MSG, imsg2Size);
    imsg2[IPMI_MSG2_RQADDR_OFFSET]                      = IPMI_MSG_ADDR_SW;
    imsg2[IPMI_MSG2_CMD_OFFSET]                         = cmd;
    imsg2[IPMI_MSG2_SET_FRU_ACT_FRU_OFFSET]             = fruId;
    imsg2[PICMG_RPLY_IMSG2_GET_POWER_LEVEL_TYPE_OFFSET] = parm;

    if (bridged) // may need to distinguish between once and twice-bridged messages
        ipmiBuildSendMsg(sess, message, &messageSize, cmd, netfn, rsAddr, rqAddr, imsg2, imsg2Size, 0);
    else
        messageSize = ipmiMsgBuild(sess, message, cmd, netfn, imsg2, imsg2Size, 0, 0, 0, 0, 0, 0);

    return sess->wrf(device, sess, message, messageSize, data, responseSize, cmd, netfn, roffs, 0);
}

/* Get fan level - PICMG command
 *
 *   RETURNS: status from sess->wrf
 *            0 on success
 *            non-zero for error
 */

int ipmiMsgGetFanLevel(void* device, IpmiSess sess, uint8_t* data, int bridged, uint8_t rsAddr, uint8_t rqAddr,
                       size_t* responseSize, int roffs, uint8_t fruId) {
    uint8_t message[MSG_MAX_LENGTH] = {0};
    uint8_t imsg2Size               = sizeof(GET_FAN_LEVEL_MSG);
    uint8_t imsg2[imsg2Size];
    size_t  messageSize;
    uint8_t cmd   = IPMI_MSG_CMD_GET_FAN_LEVEL;
    uint8_t netfn = IPMI_MSG_NETFN_PICMG;

    memcpy(imsg2, GET_FAN_LEVEL_MSG, imsg2Size);
    imsg2[IPMI_MSG2_RQADDR_OFFSET]          = IPMI_MSG_ADDR_SW;
    imsg2[IPMI_MSG2_CMD_OFFSET]             = cmd;
    imsg2[IPMI_MSG2_SET_FRU_ACT_FRU_OFFSET] = fruId;

    if (bridged) // may need to distinguish between once and twice-bridged messages
        ipmiBuildSendMsg(sess, message, &messageSize, cmd, netfn, rsAddr, rqAddr, imsg2, imsg2Size, 0);
    else
        messageSize = ipmiMsgBuild(sess, message, cmd, netfn, imsg2, imsg2Size, 0, 0, 0, 0, 0, 0);

    return sess->wrf(device, sess, message, messageSize, data, responseSize, cmd, netfn, roffs, 0);
}

/* Set fan level - PICMG command
 *
 *   RETURNS: status from sess->wrf
 *            0 on success
 *            non-zero for error
 */

int ipmiMsgSetFanLevel(void* device, IpmiSess sess, uint8_t* data, int bridged, uint8_t rsAddr, uint8_t rqAddr,
                       size_t* responseSize, int roffs, uint8_t fruId, uint8_t level) {
    uint8_t message[MSG_MAX_LENGTH] = {0};
    uint8_t imsg2Size               = sizeof(GET_FAN_LEVEL_MSG);
    uint8_t imsg2[imsg2Size];
    size_t  messageSize;
    uint8_t cmd   = IPMI_MSG_CMD_GET_FAN_LEVEL;
    uint8_t netfn = IPMI_MSG_NETFN_PICMG;

    memcpy(imsg2, GET_FAN_LEVEL_MSG, imsg2Size);
    imsg2[IPMI_MSG2_RQADDR_OFFSET]          = IPMI_MSG_ADDR_SW;
    imsg2[IPMI_MSG2_CMD_OFFSET]             = cmd;
    imsg2[IPMI_MSG2_SET_FRU_ACT_FRU_OFFSET] = fruId;
    imsg2[IPMI_MSG2_SET_FAN_LEVEL_OFFSET]   = level;

    if (bridged) // may need to distinguish between once and twice-bridged messages
        ipmiBuildSendMsg(sess, message, &messageSize, cmd, netfn, rsAddr, rqAddr, imsg2, imsg2Size, 0);
    else
        messageSize = ipmiMsgBuild(sess, message, cmd, netfn, imsg2, imsg2Size, 0, 0, 0, 0, 0, 0);

    return sess->wrf(device, sess, message, messageSize, data, responseSize, cmd, netfn, roffs, 0);
}

/* Set FRU Activation - PICMG command
 *
 *   RETURNS: status from sess->wrf
 *            0 on success
 *            non-zero for error
 */

int ipmiMsgSetFruAct(void* device, IpmiSess sess, uint8_t* data, int bridged, uint8_t rsAddr, uint8_t rqAddr,
                     size_t* responseSize, int roffs, uint8_t fruId, int parm) {
    uint8_t message[MSG_MAX_LENGTH] = {0};
    uint8_t imsg2Size               = sizeof(SET_FRU_ACT_MSG);
    uint8_t imsg2[imsg2Size];
    size_t  messageSize;
    uint8_t cmd   = IPMI_MSG_CMD_SET_FRU_ACT;
    uint8_t netfn = IPMI_MSG_NETFN_PICMG;

    memcpy(imsg2, GET_FAN_LEVEL_MSG, imsg2Size);
    imsg2[IPMI_MSG2_RQADDR_OFFSET]          = IPMI_MSG_ADDR_SW;
    imsg2[IPMI_MSG2_CMD_OFFSET]             = cmd;
    imsg2[IPMI_MSG2_SET_FRU_ACT_FRU_OFFSET] = fruId;
    imsg2[IPMI_MSG2_SET_FRU_ACT_CMD_OFFSET] = parm;

    printf("*************ipmi set fru act parm %i\n", parm);
    if (bridged) // may need to distinguish between once and twice-bridged messages
        ipmiBuildSendMsg(sess, message, &messageSize, cmd, netfn, rsAddr, rqAddr, imsg2, imsg2Size, 0);
    else
        messageSize = ipmiMsgBuild(sess, message, cmd, netfn, imsg2, imsg2Size, 0, 0, 0, 0, 0, 0);

    return sess->wrf(device, sess, message, messageSize, data, responseSize, cmd, netfn, roffs, 0);
}

/* Get PICMG properties
 *
 *   RETURNS: status from sess->wrf
 *            0 on success
 *            non-zero for error
 */
int ipmiMsgGetFanProp(void* device, IpmiSess sess, uint8_t* data, int bridged, uint8_t rsAddr, uint8_t rqAddr,
                      size_t* responseSize, int roffs, uint8_t fruId) {
    uint8_t message[MSG_MAX_LENGTH] = {0};
    uint8_t imsg2Size               = sizeof(GET_FAN_PROP_MSG);
    uint8_t imsg2[imsg2Size];
    size_t  messageSize;
    uint8_t cmd   = IPMI_MSG_CMD_GET_FAN_PROP;
    uint8_t netfn = IPMI_MSG_NETFN_PICMG;

    memcpy(imsg2, GET_FAN_PROP_MSG, imsg2Size);
    imsg2[IPMI_MSG2_RQADDR_OFFSET]          = IPMI_MSG_ADDR_SW;
    imsg2[IPMI_MSG2_CMD_OFFSET]             = cmd;
    imsg2[IPMI_MSG2_SET_FRU_ACT_FRU_OFFSET] = fruId;

    if (bridged) // may need to distinguish between once and twice-bridged messages
        ipmiBuildSendMsg(sess, message, &messageSize, cmd, netfn, rsAddr, rqAddr, imsg2, imsg2Size, 0);
    else
        messageSize = ipmiMsgBuild(sess, message, cmd, netfn, imsg2, imsg2Size, 0, 0, 0, 0, 0, 0);

    return sess->wrf(device, sess, message, messageSize, data, responseSize, cmd, netfn, roffs, 0);
}

void ipmiBuildSendMsg(IpmiSess sess, uint8_t* message, size_t* messageSize, uint8_t cmd, uint8_t netfn, uint8_t rsAddr,
                      uint8_t rqAddr, uint8_t* msg2, size_t msg2Size, uint8_t lun) {
    size_t  imsg2Size  = sizeof(SEND_MSG_MSG);
    size_t  b1msg1Size = sizeof(IPMI_MSG1);
    uint8_t imsg2[imsg2Size];
    uint8_t b1msg1[b1msg1Size];

    memcpy(imsg2, SEND_MSG_MSG, imsg2Size);
    memcpy(b1msg1, IPMI_MSG1, b1msg1Size);

    imsg2[IPMI_MSG2_CHAN_OFFSET] = IPMI_MSG_CHAN_IPMB0 + IPMI_MSG_TRACKING;

    b1msg1[IPMI_MSG1_RSADDR_OFFSET]   = rsAddr;
    b1msg1[IPMI_MSG1_NETFNLUN_OFFSET] = lun | (netfn << 2);

    msg2[IPMI_MSG2_RQADDR_OFFSET] = rqAddr;

    *messageSize =
        ipmiMsgBuild(sess, message, cmd, IPMI_MSG_NETFN_APP_REQUEST, imsg2, imsg2Size, b1msg1, msg2, msg2Size, 0, 0, 0);
}
