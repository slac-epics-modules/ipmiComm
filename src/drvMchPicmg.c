//////////////////////////////////////////////////////////////////////////////
// This file is part of 'ipmiComm'.
// It is subject to the license terms in the LICENSE.txt file found in the
// top-level directory of this distribution and at:
//    https://confluence.slac.stanford.edu/display/ppareg/LICENSE.html.
// No part of 'ipmiComm', including this file,
// may be copied, modified, propagated, or distributed except according to
// the terms contained in the LICENSE.txt file.
//////////////////////////////////////////////////////////////////////////////
#include <stdint.h>
#include <stdio.h> /* fopen, etc. */
#include <string.h>
#include <math.h>  /* floor, pow */
#include <ctype.h> /* toupper */

#include <errlog.h>
#include <epicsExport.h>
#include <iocsh.h>
#include <registry.h>

#include <drvMch.h>
#include <drvMchMsg.h>
#include <ipmiMsg.h>
#include <picmgDef.h>

/* Arbitrary limit on numbers of FRUs
 * and management controllers. Implemented to
 * limit memory usage. Can be increased if
 * necessary
 */
static int MAX_MGMT_ATCA = 15;
static int MAX_FRU_ATCA  = 25;

/* Get Address Info - this is version that attempts to get physical location
 *                    information for a specific device using IPMB-0 address
 *
 *
 *   RETURNS: status from mchMsgWriteReadHelper
 *            0 on success
 *            non-zero for error
 */
int mchMsgGetAddressInfoIpmb0(MchData mchData, uint8_t* data, uint8_t fruId, uint8_t key) {
    uint8_t response[MSG_MAX_LENGTH] = {0};
    size_t  roffs, responseSize = 0, payloadSize = PICMG_RPLY_GET_ADDR_INFO_LENGTH;
    int     rval, bridged = 0;
    uint8_t rsAddr = IPMI_MSG_ADDR_BMC, rqAddr;

    mchSetSizeOffs(mchData->ipmiSess, payloadSize, &roffs, &responseSize, &bridged, &rsAddr, &rqAddr);

    if ((rval = ipmiMsgGetAddressInfoIpmb0(mchData->mchSess, mchData->ipmiSess, response, bridged, rsAddr, rqAddr,
                                           &responseSize, roffs, fruId, key)))
        goto bail;

    if ((rval = mchMsgCheckSizes(sizeof(response), roffs, payloadSize))) {
        printf("mchMsgGetAddressInfoIpmb0 size error\n");
        goto bail;
    }

    memcpy(data, response + roffs, payloadSize);

bail:
    return rval;
}

/* Get Address Info - this is version that attempts to get physical location
 *                    information for a specific device
 * needs updating
 *
 *   RETURNS: status from mchMsgWriteReadHelper
 *            0 on success
 *            non-zero for error
 */
int mchMsgGetAddressInfoHwAddr(MchData mchData, uint8_t* data, uint8_t fru, uint8_t keytype, uint8_t key,
                               uint8_t sitetype) {
    uint8_t response[MSG_MAX_LENGTH] = {0};
    size_t  roffs, responseSize = 0, payloadSize = PICMG_RPLY_GET_ADDR_INFO_LENGTH;
    int     rval, bridged = 0;
    uint8_t rsAddr = IPMI_MSG_ADDR_BMC, rqAddr;

    mchSetSizeOffs(mchData->ipmiSess, payloadSize, &roffs, &responseSize, &bridged, &rsAddr, &rqAddr);

    if ((rval = ipmiMsgGetAddressInfoHwAddr(mchData->mchSess, mchData->ipmiSess, response, bridged, rsAddr, rqAddr,
                                            &responseSize, roffs, fru, keytype, key, sitetype)))
        goto bail;

    if ((rval = mchMsgCheckSizes(sizeof(response), roffs, payloadSize))) {
        printf("mchMsgGetAddressInfoHwAddr size error\n");
        goto bail;
    }

    memcpy(data, response + roffs, payloadSize);

bail:
    return rval;
}

/* Get Address Info - this is version that does not seek information
 *                    about a specific FRU/hardware, and will receive info
 *                    about the IPM Controller that implements the command
 * may need updating
 *
 *   RETURNS: status from mchMsgWriteReadHelper
 *            0 on success
 *            non-zero for error
 */
int mchMsgGetAddressInfoIpmc(MchData mchData, uint8_t* data) {
    uint8_t response[MSG_MAX_LENGTH] = {0};
    size_t  roffs, responseSize = 0, payloadSize = IPMI_RPLY_IMSG2_GET_PICMG_PROP_LENGTH;
    int     rval, bridged = 0;
    uint8_t rsAddr = IPMI_MSG_ADDR_BMC, rqAddr;

    mchSetSizeOffs(mchData->ipmiSess, payloadSize, &roffs, &responseSize, &bridged, &rsAddr, &rqAddr);

    if ((rval = ipmiMsgGetAddressInfoIpmc(mchData->mchSess, mchData->ipmiSess, response, bridged, rsAddr, rqAddr,
                                          &responseSize, roffs)))
        goto bail;

    if ((rval = mchMsgCheckSizes(sizeof(response), roffs, payloadSize))) {
        printf("mchMsgGetAddressInfoIpmc size error\n");
        goto bail;
    }

    memcpy(data, response + roffs, payloadSize);

bail:
    return rval;
}

static void mchPicmgFruGetFbAddressInfo(MchData mchData) {
    MchSys  mchSys                   = mchData->mchSys;
    uint8_t response[MSG_MAX_LENGTH] = {0};
    uint8_t i;
    Fru     fru;
    int     dbg = MCH_DBG(mchStat[mchData->mchSess->instance]);

    for (i = 0; i < mchSys->fruCount; i++) {

        fru = &mchSys->fru[i];

        /* move this to data reset routine for pre-config reset: */
        fru->siteNumber = -1;
        fru->siteType   = -1;

        if (fru->sdr.entityId == ENTITY_ID_PICMG_FRONT_BOARD || fru->sdr.entityId == ENTITY_ID_COOLING_UNIT) {

            if (dbg >= MCH_DBG_MED)
                printf("mchPicmgFruGetFbAddressInfo: Found FB or CU FRU index %i addr 0x%02x\n", i, fru->sdr.addr);

            if (mchMsgGetAddressInfoIpmb0(mchData, response, 0, fru->sdr.addr))
                continue;

            fru->siteNumber = response[PICMG_RPLY_IMSG2_GET_ADDR_INFO_SITE_NUMBER_OFFSET];
            fru->siteType   = response[PICMG_RPLY_IMSG2_GET_ADDR_INFO_SITE_TYPE_OFFSET];

            if (dbg >= MCH_DBG_MED) {
                printf("mchPicmgFruGetFbAddressInfo: Got FRU %i entityId 0x%02x entityInst 0x%02x "
                       "addr 0x%02x phys loc info sitenumber %i, sitetype 0x%02x\n",
                       i, fru->sdr.entityId, fru->sdr.entityInst, fru->sdr.addr, fru->siteNumber, fru->siteType);
            }
        }
    }
}

static void mchPicmgFruGetRtmAddressInfo(MchData mchData) {
    MchSys mchSys = mchData->mchSys;
    Fru    fru, fruFb;
    int    i, j;
    int    dbg = MCH_DBG(mchStat[mchData->mchSess->instance]);

    for (i = 0; i < mchSys->fruCount; i++) {

        fru = &mchSys->fru[i];

        if (fru->sdr.entityId == ENTITY_ID_PICMG_RTM) {

            for (j = 0; j < mchSys->fruCount; j++) {

                fruFb = &mchSys->fru[j];
                if ((fru->sdr.addr == fruFb->sdr.addr) && (fru->sdr.chan == fruFb->sdr.chan) &&
                    (fruFb->sdr.entityId == ENTITY_ID_PICMG_FRONT_BOARD)) {
                    fru->siteNumber = fruFb->siteNumber;
                    fru->siteType   = PICMG_SITE_TYPE_RTM;

                    if (dbg >= MCH_DBG_MED) {
                        printf("mchPicmgFruGetRtmAddressInfo: Matched RTM FRU %i to "
                               "FB FRU %i siteNumber %i siteType %i\n",
                               i, j, fru->siteNumber, fru->siteType);
                    }
                    break;
                }
            }
        }
    }
}

static void assign_sys_sizes_atca(MchData mchData) {
    mchData->mchSys->fruCountMax  = MAX_FRU_ATCA;
    mchData->mchSys->mgmtCountMax = MAX_MGMT_ATCA;
}

/*
 * For ATCA systems:
 *   -For Front Board FRUs (use entityId to determine),
 *    use HWADDR and FRU 'ID' of 0 to query site number/type
 *    This 'ID' is actually just the order of FRUs associated with
 *    this device. So the 'first' FRU is always
 *    addressed by 0 in this message
 *
 *   -For RTMs, can infer the slot number from matching it
 *    to its Front Board, which will have the same hardware address
 *    and channel number.
 *
 * This does not support for the full functionality--Front Boards
 * may have multiple FRUs. But since we are grouping all sensors and
 * information for a single Front Board together (at least for now),
 * then we don't have to catalog the different sub-FRU device info
 *
 */
static void assign_site_info_atca(MchData mchData) {

    mchPicmgFruGetFbAddressInfo(mchData);
    mchPicmgFruGetRtmAddressInfo(mchData);
}

static void assign_fru_lkup_atca(MchData mchData) {
    MchSys  mchSys = mchData->mchSys;
    uint8_t i;
    Fru     fru;
    int     cuCnt = 0, shfCnt = 0, pmCnt = 0, shmCnt = 0;
    int     dbg = MCH_DBG(mchStat[mchData->mchSess->instance]);

    for (i = 0; i < mchSys->fruCount; i++) {

        fru = &mchSys->fru[i];

        if (fru->siteType == PICMG_SITE_TYPE_FRONT_BOARD) {
            fru->id                  = UTCA_FRU_TYPE_AMC_MIN + fru->siteNumber - 1;
            mchSys->fruLkup[fru->id] = i;
            if (dbg >= MCH_DBG_MED) {
                printf("assign_fru_lkup_atca: Found front board, "
                       "index in FRU array %i, index in lkup array (aka id) %i\n",
                       i, fru->id);
            }
        } else if (fru->siteType == PICMG_SITE_TYPE_RTM) {
            fru->id                  = UTCA_FRU_TYPE_RTM_MIN + fru->siteNumber - 1;
            mchSys->fruLkup[fru->id] = i;
            if (dbg >= MCH_DBG_MED) {
                printf("assign_fru_lkup_atca: Found RTM, index in FRU array %i, "
                       "index in lkup array (aka id) %i\n",
                       i, fru->id);
            }
        } else if (fru->sdr.entityId == ENTITY_ID_PICMG_SHMC) {
            if ((PICMG_FRU_TYPE_SHMC_MIN + shmCnt) > PICMG_FRU_TYPE_SHMC_MAX)
                continue;
            fru->id                  = PICMG_FRU_TYPE_SHMC_MAX - shmCnt;
            mchSys->fruLkup[fru->id] = i;
            if (dbg >= MCH_DBG_MED) {
                printf("assign_fru_lkup_atca: Found SHMC, index in FRU array %i, "
                       "index in lkup array (aka id) %i\n",
                       i, fru->id);
            }
            shmCnt++;
        } else if (fru->sdr.entityId == ENTITY_ID_PICMG_SHELF_FRU_INFO) {
            if ((UTCA_FRU_TYPE_SHELF_MIN + shfCnt) > UTCA_FRU_TYPE_SHELF_MAX)
                continue;
            fru->id                  = UTCA_FRU_TYPE_SHELF_MIN + shfCnt;
            mchSys->fruLkup[fru->id] = i;
            if (dbg >= MCH_DBG_MED) {
                printf("assign_fru_lkup_atca: Found shelf fru info, "
                       "fru index %i, index in lkup array (aka id) %i\n",
                       i, fru->id);
            }
            shfCnt++;
        } else if (fru->sdr.entityId == ENTITY_ID_COOLING_UNIT) {
            fru->id                  = UTCA_FRU_TYPE_CU_MIN + cuCnt;
            mchSys->fruLkup[fru->id] = i;
            cuCnt++;
            if (dbg >= MCH_DBG_MED) {
                printf("assign_fru_lkup_atca: Found CU, index in FRU array %i, "
                       "index in lkup array (aka id) %i\n",
                       i, fru->id);
            }
        } else if (fru->sdr.entityId == ENTITY_ID_POWER_MODULE) {
            if ((UTCA_FRU_TYPE_PM_MIN + pmCnt) > UTCA_FRU_TYPE_PM_MAX)
                continue;
            fru->id                  = UTCA_FRU_TYPE_PM_MIN + pmCnt;
            mchSys->fruLkup[fru->id] = i;
            pmCnt++;
            if (dbg >= MCH_DBG_MED) {
                printf("assign_fru_lkup_atca: Found PM, index in FRU array %i, "
                       "index in lkup array (aka id) %i\n",
                       i, fru->id);
            }
        }
    }
}

/* MicroTCA uses pre-defined unique FRU IDs
 * Assign FRU lookup ID to these same values
 */
static void assign_fru_lkup_microtca(MchData mchData) {
    int    i;
    MchSys mchSys = mchData->mchSys;
    Fru    fru;

    for (i = 0; i < mchSys->fruCount; i++) {

        fru = &mchSys->fru[i];

        if (fru->sdr.recType) { /* If real entity */

            fru->id                  = fru->sdr.fruId;
            mchSys->fruLkup[fru->id] = i;
        }
    }
}

int fru_data_suppl_picmg(MchData mchData, int index) {
    int     id, rval = 0;
    uint8_t response[MSG_MAX_LENGTH] = {0};
    Fru     fru;
    int     dbg = MCH_DBG(mchStat[mchData->mchSess->instance]);

    fru = &mchData->mchSys->fru[index];
    id  = fru->id;

    if (-1 == (index = mchData->mchSys->fruLkup[id])) /* Was not assigned FRU-lookup ID */
        return rval;

    if ((id >= UTCA_FRU_TYPE_CU_MIN) && (id <= UTCA_FRU_TYPE_CU_MAX)) {

        if (mchData->mchSys->mchcb->get_fan_prop) {
            if (mchData->mchSys->mchcb->get_fan_prop(mchData, response, index)) {
                /* For now, set fan properties to all zeros to indicate not read */
                fru->fanMin = fru->fanMax = fru->fanNom = fru->fanProp = 0;
                rval                                                   = -1;
            }
        } else
            return rval;

        fru->fanMin  = response[IPMI_RPLY_IMSG2_GET_FAN_PROP_MIN_OFFSET];
        fru->fanMax  = response[IPMI_RPLY_IMSG2_GET_FAN_PROP_MAX_OFFSET];
        fru->fanNom  = response[IPMI_RPLY_IMSG2_GET_FAN_PROP_NOM_OFFSET];
        fru->fanProp = response[IPMI_RPLY_IMSG2_GET_FAN_PROP_PROP_OFFSET];

        if (dbg >= MCH_DBG_MED)
            printf("fru_data_suppl_picmg: FRU index %i id %i fan properties "
                   "min %i max %i nom %i prop 0x%02x\n",
                   index, id, fru->fanMin, fru->fanMax, fru->fanNom, fru->fanProp);
    }
    return rval;
}

static void sensor_get_fru_atca(MchData mchData, Sensor sens) {
    MchSys mchSys = mchData->mchSys;
    Fru    fru;
    Mgmt   mgmt;
    int    i, j;

    for (i = 0; i < mchSys->fruCount; i++) {

        fru = &mchSys->fru[i];
        if ((sens->sdr.owner == fru->sdr.addr) && (sens->sdr.entityId == fru->sdr.entityId) &&
            (sens->sdr.entityInst == fru->sdr.entityInst)) {
            sens->fruIndex = i; /* FRU index for associated FRU */
            return;
        }
        for (j = 0; j < fru->entityCount; j++) {
            if ((sens->sdr.owner == fru->entity[j].addr) && (sens->sdr.entityId == fru->entity[j].entityId) &&
                (sens->sdr.entityInst == fru->entity[j].entityInst)) {
                sens->fruIndex = i; /* FRU index for associated FRU */
                return;
            }
        }
        /* If AMC entity & owner is same as existing front-board FRU, can associate sensor with that front board
         * In future, this may be sufficient information to create indepdendent AMC entities */
        if ((sens->sdr.entityId == ENTITY_ID_PICMG_AMC) && (fru->sdr.entityId == ENTITY_ID_PICMG_FRONT_BOARD) &&
            (sens->sdr.owner == fru->sdr.addr)) {
            sens->fruIndex = i;
            return;
        }
    }

    for (i = 0; i < mchSys->mgmtCount; i++) {

        mgmt = &mchSys->mgmt[i];
        if ((sens->sdr.owner == mchSys->mgmt[i].sdr.addr)) {
            sens->mgmtIndex = i;
            return;
        }
        for (j = 0; j < mgmt->entityCount; j++) {
            if ((sens->sdr.owner == mgmt->entity[j].addr) && (sens->sdr.entityId == mgmt->entity[j].entityId) &&
                (sens->sdr.entityInst == mgmt->entity[j].entityInst)) {
                sens->mgmtIndex = i; /* MGMT ID for associated MGMT */
                return;
            }
        }
    }

    return; /* Did not find match */
}

static void sensor_get_fru_microtca(MchData mchData, Sensor sens) {
    MchSys  mchSys = mchData->mchSys;
    SdrFull sdr    = &sens->sdr;
    char    buff[9];
    int     i, natid, id;

    /* NAT associates hotswap aka M-state sensors with carrier manager but
     * Vadatech associates them with the actual FRU described by the sensor.
     * For NAT, parse sensor description to get associated FRU number
     */
    if (MCH_IS_NAT(mchData->mchSess->type) && (sdr->sensType == SENSOR_TYPE_HOTSWAP)) {
        if (2 != sscanf((const char*)(sdr->str), "HS %d %s", &natid, buff))
            return;
        sens->fruIndex = mchSys->fruLkup[natid];
    }

    /* Loop through FRUs and Management Controllers
     * Start at 1 for MicroTCA because FRU 0 is reserved logical
     * entity with same entityId and entityInst as MCH 1
     */
    for (i = 0; i < mchSys->fruCount; i++) {

        if (mchSys->fru[i].sdr.fruId == 0) { /* Does fruId need to be reset to 0 on each config ? */
            continue;
        }
        if ((sens->sdr.entityId == mchSys->fru[i].sdr.entityId) &&
            (sens->sdr.entityInst == mchSys->fru[i].sdr.entityInst)) {
            sens->fruIndex = i; /* FRU index for associated FRU */
            return;
        }
    }
    for (i = 0; i < mchSys->mgmtCount; i++) {
        if ((sens->sdr.entityId == mchSys->mgmt[i].sdr.entityId) &&
            (sens->sdr.entityInst == mchSys->mgmt[i].sdr.entityInst)) {
            sens->mgmtIndex = i;
            return;
        }
    }

    if (MCH_IS_NAT(mchData->mchSess->type)) {
        /* Find entities that do not have SDRs */
        if (sens->sdr.entityId == VT_ENTITY_ID_AMC) {
            id             = UTCA_FRU_TYPE_AMC_MIN + sens->sdr.entityInst - 0x60 - 1;
            sens->fruIndex = mchSys->fruLkup[id];
            printf("sensor_get_fru_microtca: Found AMC that does not have SDR, fru id %i entity id 0x%02x entity inst "
                   "0x%02x\n",
                   id, sens->sdr.entityId, sens->sdr.entityInst);
            return;
        } else if (sens->sdr.entityId == VT_ENTITY_ID_RTM) {
            id             = UTCA_FRU_TYPE_RTM_MIN + sens->sdr.entityInst - 0x60 - 1;
            sens->fruIndex = mchSys->fruLkup[id];
            printf("sensor_get_fru_microtca: Found RTM that does not have SDR, FRU ID %i\n", id);
            return;
        }
    }

    return; /* Did not find match */
}

/* Get PICMG Properties - For ATCA (and MicroTCA?)
 *
 *
 *   RETURNS: status from mchMsgWriteReadHelper
 *            0 on success
 *            non-zero for error
 */
int mchMsgGetPicmgProp(MchData mchData, uint8_t* data) {
    uint8_t response[MSG_MAX_LENGTH] = {0};
    size_t  roffs, responseSize = 0, payloadSize = IPMI_RPLY_IMSG2_GET_PICMG_PROP_LENGTH;
    int     rval, bridged = 0;
    uint8_t rsAddr = IPMI_MSG_ADDR_BMC, rqAddr;

    mchSetSizeOffs(mchData->ipmiSess, payloadSize, &roffs, &responseSize, &bridged, &rsAddr, &rqAddr);

    if ((rval = ipmiMsgGetPicmgProp(mchData->mchSess, mchData->ipmiSess, response, bridged, rsAddr, rqAddr,
                                    &responseSize, roffs)))
        goto bail;

    if ((rval = mchMsgCheckSizes(sizeof(response), roffs, payloadSize))) {
        printf("mchMsgGetPicmgProp size error\n");
        goto bail;
    }

    memcpy(data, response + roffs, payloadSize);

bail:
    return rval;
}

/* Set FRU Activation - For NAT MCH, used to deactivate/activate FRU
 *
 *
 *   RETURNS: status from mchMsgWriteReadHelper
 *            0 on success
 *            non-zero for error
 */
int mchMsgSetFruActNat(MchData mchData, uint8_t* data, uint8_t fruIndex, uint8_t parm) {
    uint8_t message[MSG_MAX_LENGTH] = {0};
    size_t  imsg2Size               = sizeof(SET_FRU_ACT_MSG);
    uint8_t imsg2[imsg2Size];
    size_t  messageSize;
    size_t  responseSize = 0; /*IPMI_RPLY_SET_FRU_ACT_LENGTH_NAT; - determine length */
    uint8_t cmd          = IPMI_MSG_CMD_SET_FRU_ACT;
    uint8_t netfn        = IPMI_MSG_NETFN_PICMG;
    int     offs         = 0; /* determine needed offset */
    uint8_t fruId;

    memcpy(imsg2, SET_FRU_ACT_MSG, imsg2Size);

    fruId                                   = mchData->mchSys->fru[fruIndex].sdr.fruId;
    imsg2[IPMI_MSG2_RQADDR_OFFSET]          = IPMI_MSG_ADDR_SW;
    imsg2[IPMI_MSG2_SET_FRU_ACT_FRU_OFFSET] = fruId;
    imsg2[IPMI_MSG2_SET_FRU_ACT_CMD_OFFSET] = parm;

    messageSize = ipmiMsgBuild(mchData->ipmiSess, message, cmd, netfn, imsg2, imsg2Size, 0, 0, 0, 0, 0, 0);

    return mchMsgWriteReadHelper(mchData->mchSess, mchData->ipmiSess, message, messageSize, data, &responseSize, cmd,
                                 netfn, offs /*need to set codeoffs*/, 0);
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
int mchMsgSetFruActPolicyVt(MchData mchData, uint8_t* data, uint8_t fruIndex, uint8_t parm) {
    uint8_t message[MSG_MAX_LENGTH] = {0};
    size_t  imsg2Size               = sizeof(SEND_MSG_MSG);
    size_t  b1msg1Size              = sizeof(IPMI_MSG1);
    size_t  b1msg2Size              = sizeof(SET_FRU_POLICY_MSG);
    uint8_t imsg2[imsg2Size];
    uint8_t b1msg1[b1msg1Size];
    uint8_t b1msg2[b1msg2Size];
    size_t  messageSize;
    size_t  responseSize = IPMI_RPLY_SET_FRU_POLICY_LENGTH_VT;
    uint8_t cmd          = IPMI_MSG_CMD_SET_FRU_POLICY;
    uint8_t netfn        = IPMI_MSG_NETFN_PICMG;
    uint8_t fruId, mask, bits;

    if (parm == 0) {
        mask = 1 << 1; /* make this #define */
        bits = 0;
    } else if (parm == 1) {
        mask = 1;
        bits = 0;
    } else
        return -1;

    memcpy(imsg2, SEND_MSG_MSG, imsg2Size);
    memcpy(b1msg1, IPMI_MSG1, b1msg1Size);
    memcpy(b1msg2, SET_FRU_POLICY_MSG, b1msg2Size);

    imsg2[IPMI_MSG2_CHAN_OFFSET] = IPMI_MSG_CHAN_IPMB0 + IPMI_MSG_TRACKING;

    b1msg1[IPMI_MSG1_RSADDR_OFFSET]   = IPMI_MSG_ADDR_CM;
    b1msg1[IPMI_MSG1_NETFNLUN_OFFSET] = netfn << 2;

    fruId                                        = mchData->mchSys->fru[fruIndex].sdr.fruId;
    b1msg2[IPMI_MSG2_SET_FRU_ACT_FRU_OFFSET]     = fruId;
    b1msg2[IPMI_MSG2_SET_FRU_POLICY_MASK_OFFSET] = mask;
    b1msg2[IPMI_MSG2_SET_FRU_POLICY_BITS_OFFSET] = bits;

    messageSize = ipmiMsgBuild(mchData->ipmiSess, message, cmd, IPMI_MSG_NETFN_APP_REQUEST, imsg2, imsg2Size, b1msg1,
                               b1msg2, b1msg2Size, 0, 0, 0);

    return mchMsgWriteReadHelper(mchData->mchSess, mchData->ipmiSess, message, messageSize, data, &responseSize, cmd,
                                 netfn, 0 /*need to set codeoffs*/, 0);
}

/* Set FRU Activation - For ATCA
 *
 *
 *   RETURNS: status from mchMsgWriteReadHelper
 *            0 on success
 *            non-zero for error
 */
int mchMsgSetFruActAtca(MchData mchData, uint8_t* data, uint8_t fruIndex, uint8_t parm) {
    uint8_t response[MSG_MAX_LENGTH] = {0};
    size_t  roffs, responseSize = 0, payloadSize = PICMG_RPLY_SET_FRU_ACT_LENGTH;
    int     rval, bridged = 0;
    Fru     fru    = &mchData->mchSys->fru[fruIndex];
    uint8_t rsAddr = fru->sdr.addr, rqAddr;

    if (rsAddr != IPMI_MSG_ADDR_BMC)
        bridged = 1;

    mchSetSizeOffs(mchData->ipmiSess, payloadSize, &roffs, &responseSize, &bridged, &rsAddr, &rqAddr);

    if ((rval = ipmiMsgSetFruAct(mchData->mchSess, mchData->ipmiSess, response, bridged, rsAddr, rqAddr, &responseSize,
                                 roffs, fru->sdr.fruId, parm)))
        goto bail;

    if ((rval = mchMsgCheckSizes(sizeof(response), roffs, payloadSize))) {
        printf("mchMsgSetFruActAtca size error\n");
        goto bail;
    }

    memcpy(data, response + roffs, payloadSize);

bail:
    return rval;
}

/* Get FRU Activation Policy using Vadatech MCH; message contains 1 bridged message -> needs updating 3/23/16
 *
 *   RETURNS: status from mchMsgWriteReadHelper
 *            0 on success
 *            non-zero for error
 */
int mchMsgGetFruActPolicyVt(MchData mchData, uint8_t* data, uint8_t fru) {
    uint8_t message[MSG_MAX_LENGTH] = {0};
    size_t  imsg2Size               = sizeof(SEND_MSG_MSG);
    size_t  b1msg1Size              = sizeof(IPMI_MSG1);
    size_t  b1msg2Size              = sizeof(GET_FAN_PROP_MSG);
    uint8_t imsg2[imsg2Size];
    uint8_t b1msg1[b1msg1Size];
    uint8_t b1msg2[b1msg2Size];
    size_t  messageSize;
    size_t  responseSize = IPMI_RPLY_GET_FRU_POLICY_LENGTH_VT;
    uint8_t cmd          = IPMI_MSG_CMD_GET_FRU_POLICY;
    uint8_t netfn        = IPMI_MSG_NETFN_PICMG;

    memcpy(imsg2, SEND_MSG_MSG, imsg2Size);
    memcpy(b1msg1, IPMI_MSG1, b1msg1Size);
    memcpy(b1msg2, GET_FRU_POLICY_MSG, b1msg2Size);

    imsg2[IPMI_MSG2_CHAN_OFFSET] = IPMI_MSG_CHAN_IPMB0 + IPMI_MSG_TRACKING;

    b1msg1[IPMI_MSG1_RSADDR_OFFSET]   = IPMI_MSG_ADDR_CM;
    b1msg1[IPMI_MSG1_NETFNLUN_OFFSET] = netfn << 2;

    b1msg2[IPMI_MSG2_SET_FRU_ACT_FRU_OFFSET] = fru;

    messageSize = ipmiMsgBuild(mchData->ipmiSess, message, cmd, IPMI_MSG_NETFN_APP_REQUEST, imsg2, imsg2Size, b1msg1,
                               b1msg2, b1msg2Size, 0, 0, 0);

    return mchMsgWriteReadHelper(mchData->mchSess, mchData->ipmiSess, message, messageSize, data, &responseSize, cmd,
                                 netfn, 0 /*need to set codeoffs*/, 0);
}

/* Get FRU Activation Policy using NAT MCH - not yet tested; may be single message like SetFruActNat
 *
 *   RETURNS: status from mchMsgWriteReadHelper
 *            0 on success
 *            non-zero for error
 */
int mchMsgGetFruActPolicyNat(MchData mchData, uint8_t* data, uint8_t fru) {
    uint8_t message[MSG_MAX_LENGTH] = {0};
    size_t  imsg2Size               = sizeof(SEND_MSG_MSG);
    size_t  b1msg1Size              = sizeof(IPMI_MSG1);
    size_t  b1msg2Size              = sizeof(SEND_MSG_MSG);
    size_t  b2msg1Size              = sizeof(IPMI_MSG1);
    size_t  b2msg2Size              = sizeof(GET_FRU_POLICY_MSG);
    uint8_t imsg2[imsg2Size];
    uint8_t b1msg1[b1msg1Size];
    uint8_t b1msg2[b1msg2Size];
    uint8_t b2msg1[b2msg1Size];
    uint8_t b2msg2[b2msg2Size];
    size_t  messageSize;
    size_t  roffs;
    size_t  payloadSize  = IPMI_RPLY_GET_FRU_POLICY_LENGTH;
    size_t  responseSize = 2 * IPMI_MSG1_LENGTH + 2 * IPMI_RPLY_IMSG2_SEND_MSG_LENGTH;
    uint8_t cmd          = IPMI_MSG_CMD_GET_FRU_POLICY;
    uint8_t netfn        = IPMI_MSG_NETFN_PICMG;
    int     bridged      = 0;
    uint8_t rsAddr       = IPMI_MSG_ADDR_BMC, rqAddr;

    mchSetSizeOffs(mchData->ipmiSess, payloadSize, &roffs, &responseSize, &bridged, &rsAddr, &rqAddr);

    memcpy(imsg2, SEND_MSG_MSG, imsg2Size);
    memcpy(b1msg1, IPMI_MSG1, b1msg1Size);

    imsg2[IPMI_MSG2_CHAN_OFFSET]    = IPMI_MSG_CHAN_IPMB0 + IPMI_MSG_TRACKING;
    b1msg1[IPMI_MSG1_RSADDR_OFFSET] = IPMI_MSG_ADDR_CM;

    memcpy(b1msg2, SEND_MSG_MSG, b1msg2Size);
    memcpy(b2msg1, IPMI_MSG1, b2msg1Size);
    memcpy(b2msg2, GET_FRU_POLICY_MSG, b2msg2Size);

    b1msg2[IPMI_MSG2_RQADDR_OFFSET]          = IPMI_MSG_ADDR_BMC;
    b2msg1[IPMI_MSG1_NETFNLUN_OFFSET]        = netfn << 2;
    b1msg2[IPMI_MSG2_CHAN_OFFSET]            = IPMI_MSG_CHAN_IPMBL + IPMI_MSG_TRACKING;
    b2msg1[IPMI_MSG1_RSADDR_OFFSET]          = FRU_I2C_ADDR[fru];
    b2msg2[IPMI_MSG2_SET_FRU_ACT_FRU_OFFSET] = 0;

    messageSize = ipmiMsgBuild(mchData->ipmiSess, message, cmd, IPMI_MSG_NETFN_APP_REQUEST, imsg2, imsg2Size, b1msg1,
                               b1msg2, b1msg2Size, b2msg1, b2msg2, b2msg2Size);

    return mchMsgWriteReadHelper(mchData->mchSess, mchData->ipmiSess, message, messageSize, data, &responseSize, cmd,
                                 netfn, roffs, 0);
}

int mchMsgGetFruActPolicyHelper(MchData mchData, uint8_t* data, uint8_t fru) {
    if (!(MCH_IS_VT(mchData->mchSess->type)))
        return mchMsgGetFruActPolicyVt(mchData, data, fru);
    else
        return mchMsgGetFruActPolicyNat(mchData, data, fru);
}

/* Get Fan Speed Properties using Vadatech MCH; message contains 1 bridged message -> needs updating 3/23/16
 *
 *   RETURNS: status from mchMsgWriteReadHelper
 *            0 on success
 *            non-zero for error
 */
int mchMsgGetFanPropVt(MchData mchData, uint8_t* data, uint8_t fruIndex) {
    uint8_t message[MSG_MAX_LENGTH] = {0};
    uint8_t imsg2Size               = sizeof(GET_FAN_PROP_MSG);
    uint8_t imsg2[imsg2Size];
    size_t  messageSize;
    Fru     fru                      = &mchData->mchSys->fru[fruIndex];
    uint8_t response[MSG_MAX_LENGTH] = {0};
    size_t  roffs, responseSize, payloadSize = IPMI_RPLY_GET_FAN_PROP_LENGTH;
    int     rval, offs = 0, bridged = 0;
    uint8_t rsAddr = IPMI_MSG_ADDR_BMC, rqAddr, fruId = fru->sdr.fruId;
    uint8_t cmd   = IPMI_MSG_CMD_GET_FAN_PROP;
    uint8_t netfn = IPMI_MSG_NETFN_PICMG;

    mchSetSizeOffs(mchData->ipmiSess, payloadSize, &roffs, &responseSize, &bridged, &rsAddr, &rqAddr);

    memcpy(imsg2, GET_FAN_PROP_MSG, imsg2Size);

    imsg2[IPMI_MSG2_SET_FRU_ACT_FRU_OFFSET] = fruId;

    if (bridged) // may need to distinguish between once and twice-bridged messages
        ipmiBuildSendMsg(mchData->ipmiSess, message, &messageSize, cmd, netfn, rsAddr, rqAddr, imsg2, imsg2Size, 0);
    else
        messageSize = ipmiMsgBuild(mchData->ipmiSess, message, cmd, netfn, imsg2, imsg2Size, 0, 0, 0, 0, 0, 0);

    if ((rval = mchMsgWriteReadHelper(mchData->mchSess, mchData->ipmiSess, message, messageSize, response,
                                      &responseSize, cmd, netfn, offs /*need to set codeoffs*/, 0)))
        goto bail;

    if ((rval = mchMsgCheckSizes(sizeof(response), roffs, payloadSize))) {
        printf("mchMsgGetFanPropVt size error\n");
        goto bail;
    }

    memcpy(data, response + roffs, payloadSize);

bail:
    return rval;
}

/* Get Fan Speed Properties using NAT MCH; message contains 2 bridged messages
 *
 *   RETURNS: status from mchMsgWriteReadHelper
 *            0 on success
 *            non-zero for error
 */
int mchMsgGetFanPropNat(MchData mchData, uint8_t* data, uint8_t fruIndex) {
    uint8_t message[MSG_MAX_LENGTH]  = {0};
    uint8_t response[MSG_MAX_LENGTH] = {0};
    size_t  imsg2Size                = sizeof(SEND_MSG_MSG);
    size_t  b1msg1Size               = sizeof(IPMI_MSG1);
    size_t  b1msg2Size               = sizeof(SEND_MSG_MSG);
    size_t  b2msg1Size               = sizeof(IPMI_MSG1);
    size_t  b2msg2Size               = sizeof(GET_FAN_PROP_MSG);
    uint8_t imsg2[imsg2Size];
    uint8_t b1msg1[b1msg1Size];
    uint8_t b1msg2[b1msg2Size];
    uint8_t b2msg1[b2msg1Size];
    uint8_t b2msg2[b2msg2Size];
    size_t  messageSize;
    size_t  roffs;
    size_t  payloadSize  = IPMI_RPLY_GET_FAN_PROP_LENGTH;
    size_t  responseSize = 2 * IPMI_MSG1_LENGTH + 2 * IPMI_RPLY_IMSG2_SEND_MSG_LENGTH;
    uint8_t cmd          = IPMI_MSG_CMD_GET_FAN_PROP;
    uint8_t netfn        = IPMI_MSG_NETFN_PICMG;
    int     rval, bridged = 0;
    Fru     fru    = &mchData->mchSys->fru[fruIndex];
    uint8_t rsAddr = IPMI_MSG_ADDR_BMC, rqAddr, fruId = fru->sdr.fruId;

    mchSetSizeOffs(mchData->ipmiSess, payloadSize, &roffs, &responseSize, &bridged, &rsAddr, &rqAddr);

    memcpy(imsg2, SEND_MSG_MSG, imsg2Size);
    memcpy(b1msg1, IPMI_MSG1, b1msg1Size);

    imsg2[IPMI_MSG2_CHAN_OFFSET]    = IPMI_MSG_CHAN_IPMB0 + IPMI_MSG_TRACKING;
    b1msg1[IPMI_MSG1_RSADDR_OFFSET] = IPMI_MSG_ADDR_CM;

    memcpy(b1msg2, SEND_MSG_MSG, b1msg2Size);
    memcpy(b2msg1, IPMI_MSG1, b2msg1Size);
    memcpy(b2msg2, GET_FAN_PROP_MSG, b2msg2Size);

    b1msg2[IPMI_MSG2_RQADDR_OFFSET]          = IPMI_MSG_ADDR_BMC;
    b2msg1[IPMI_MSG1_NETFNLUN_OFFSET]        = netfn << 2;
    b1msg2[IPMI_MSG2_CHAN_OFFSET]            = IPMI_MSG_CHAN_IPMBL + IPMI_MSG_TRACKING;
    b2msg1[IPMI_MSG1_RSADDR_OFFSET]          = FRU_I2C_ADDR[fruId];
    b2msg2[IPMI_MSG2_SET_FRU_ACT_FRU_OFFSET] = 0;

    messageSize = ipmiMsgBuild(mchData->ipmiSess, message, cmd, IPMI_MSG_NETFN_APP_REQUEST, imsg2, imsg2Size, b1msg1,
                               b1msg2, b1msg2Size, b2msg1, b2msg2, b2msg2Size);

    if ((rval = mchMsgWriteReadHelper(mchData->mchSess, mchData->ipmiSess, message, messageSize, response,
                                      &responseSize, cmd, netfn, roffs, 0)))
        goto bail;

    if ((rval = mchMsgCheckSizes(sizeof(response), roffs, payloadSize))) {
        printf("mchMsgGetFanPropNat size error\n");
        goto bail;
    }

    memcpy(data, response + roffs, payloadSize);

bail:
    return rval;
}

/* Get Fan Properties - For ATCA
 *
 *
 *   RETURNS: status from mchMsgWriteReadHelper
 *            0 on success
 *            non-zero for error
 */
int mchMsgGetFanPropAtca(MchData mchData, uint8_t* data, uint8_t fruIndex) {
    uint8_t response[MSG_MAX_LENGTH] = {0};
    size_t  roffs, responseSize = 0, payloadSize = IPMI_RPLY_GET_FAN_PROP_LENGTH;
    int     rval, bridged = 0;
    Fru     fru    = &mchData->mchSys->fru[fruIndex];
    uint8_t rsAddr = fru->sdr.addr, rqAddr;

    if (rsAddr != IPMI_MSG_ADDR_BMC)
        bridged = 1;

    mchSetSizeOffs(mchData->ipmiSess, payloadSize, &roffs, &responseSize, &bridged, &rsAddr, &rqAddr);

    if ((rval = ipmiMsgGetFanProp(mchData->mchSess, mchData->ipmiSess, response, bridged, rsAddr, rqAddr, &responseSize,
                                  roffs, fru->sdr.fruId)))
        goto bail;

    if ((rval = mchMsgCheckSizes(sizeof(response), roffs, payloadSize))) {
        printf("mchMsgGetFanPropAtca size error\n");
        goto bail;
    }

    memcpy(data, response + roffs, payloadSize);

bail:
    return rval;
}

/* Details of this algorithm specified in PICMG 3.0 Rev 3.0 ATCA Base Spec, Table 3-87 */
void mchGetFanLevel(uint8_t* data, uint8_t* level, uint8_t fanProp) {
    uint8_t llevel, olevel, lenabled;

    olevel = data[PICMG_RPLY_IMSG2_GET_FAN_OVERRIDE_LEVEL_OFFSET];

    if (PICMG_FAN_LOCAL_CONTROL_SUPPORTED(fanProp)) {

        llevel   = data[PICMG_RPLY_IMSG2_GET_FAN_LOCAL_LEVEL_OFFSET];
        lenabled = data[PICMG_RPLY_IMSG2_GET_FAN_LOCAL_ENABLED_OFFSET];

        if (olevel == 0xFF)
            *level = llevel;

        else if (lenabled)
            *level = (llevel > olevel) ? llevel : olevel;

        else if ((0xFE == llevel) || (0xFE == olevel))
            *level = -1; /* Shut down state; arbitrarily choose -1 for this state for now */

        else
            *level = olevel;
    } else
        *level = olevel;
}

/* -> needs updating 3/23/16 */
int mchMsgGetFanLevelVt(MchData mchData, uint8_t* data, uint8_t fruIndex, uint8_t* level) {
    uint8_t message[MSG_MAX_LENGTH] = {0};
    uint8_t imsg2Size               = sizeof(GET_FAN_LEVEL_MSG);
    uint8_t imsg2[imsg2Size];
    size_t  messageSize;
    Fru     fru = &mchData->mchSys->fru[fruIndex];

    uint8_t response[MSG_MAX_LENGTH] = {0};
    size_t  roffs, responseSize, payloadSize = IPMI_RPLY_GET_FAN_LEVEL_LENGTH;
    int     rval, offs = 0, bridged = 0;
    uint8_t rsAddr = IPMI_MSG_ADDR_BMC, rqAddr, fruId = fru->sdr.fruId;
    uint8_t cmd   = IPMI_MSG_CMD_GET_FAN_LEVEL;
    uint8_t netfn = IPMI_MSG_NETFN_PICMG;

    mchSetSizeOffs(mchData->ipmiSess, payloadSize, &roffs, &responseSize, &bridged, &rsAddr, &rqAddr);

    memcpy(imsg2, GET_FAN_LEVEL_MSG, imsg2Size);

    imsg2[IPMI_MSG2_SET_FRU_ACT_FRU_OFFSET] = fruId;

    if (bridged) // may need to distinguish between once and twice-bridged messages
        ipmiBuildSendMsg(mchData->ipmiSess, message, &messageSize, cmd, netfn, rsAddr, rqAddr, imsg2, imsg2Size, 0);
    else
        messageSize = ipmiMsgBuild(mchData->ipmiSess, message, cmd, netfn, imsg2, imsg2Size, 0, 0, 0, 0, 0, 0);

    if ((rval = mchMsgWriteReadHelper(mchData->mchSess, mchData->ipmiSess, message, messageSize, response,
                                      &responseSize, cmd, netfn, offs /*need to set codeoffs*/, 0)))
        goto bail;

    if ((rval = mchMsgCheckSizes(sizeof(response), roffs, payloadSize))) {
        printf("mchMsgGetFanLevelVt size error\n");
        goto bail;
    }

    memcpy(data, response + roffs, payloadSize);

    mchGetFanLevel(data, level, fru->fanProp);

bail:
    return rval;
}

/* Get Fan Level using NAT MCH; message contains 2 bridged messages
 *
 *   RETURNS: status from mchMsgWriteReadHelper
 *            0 on success
 *            non-zero for error
 */
int mchMsgGetFanLevelNat(MchData mchData, uint8_t* data, uint8_t fruIndex, uint8_t* level) {
    uint8_t message[MSG_MAX_LENGTH]  = {0};
    uint8_t response[MSG_MAX_LENGTH] = {0};
    size_t  imsg2Size                = sizeof(SEND_MSG_MSG);
    size_t  b1msg1Size               = sizeof(IPMI_MSG1);
    size_t  b1msg2Size               = sizeof(SEND_MSG_MSG);
    size_t  b2msg1Size               = sizeof(IPMI_MSG1);
    size_t  b2msg2Size               = sizeof(GET_FAN_LEVEL_MSG);
    uint8_t imsg2[imsg2Size];
    uint8_t b1msg1[b1msg1Size];
    uint8_t b1msg2[b1msg2Size];
    uint8_t b2msg1[b2msg1Size];
    uint8_t b2msg2[b2msg2Size];
    size_t  messageSize;
    size_t  roffs;
    size_t  payloadSize  = IPMI_RPLY_GET_FAN_LEVEL_LENGTH;
    size_t  responseSize = 2 * IPMI_MSG1_LENGTH + 2 * IPMI_RPLY_IMSG2_SEND_MSG_LENGTH;
    uint8_t cmd          = IPMI_MSG_CMD_GET_FAN_LEVEL;
    uint8_t netfn        = IPMI_MSG_NETFN_PICMG;
    int     rval, bridged = 0;
    Fru     fru    = &mchData->mchSys->fru[fruIndex];
    uint8_t rsAddr = IPMI_MSG_ADDR_BMC, rqAddr, fruId = fru->sdr.fruId;

    mchSetSizeOffs(mchData->ipmiSess, payloadSize, &roffs, &responseSize, &bridged, &rsAddr, &rqAddr);

    memcpy(imsg2, SEND_MSG_MSG, imsg2Size);
    memcpy(b1msg1, IPMI_MSG1, b1msg1Size);

    imsg2[IPMI_MSG2_CHAN_OFFSET]    = IPMI_MSG_CHAN_IPMB0 + IPMI_MSG_TRACKING;
    b1msg1[IPMI_MSG1_RSADDR_OFFSET] = IPMI_MSG_ADDR_CM;

    memcpy(b1msg2, SEND_MSG_MSG, b1msg2Size);
    memcpy(b2msg1, IPMI_MSG1, b2msg1Size);
    memcpy(b2msg2, GET_FAN_LEVEL_MSG, b2msg2Size);

    b1msg2[IPMI_MSG2_RQADDR_OFFSET]          = IPMI_MSG_ADDR_BMC;
    b2msg1[IPMI_MSG1_NETFNLUN_OFFSET]        = netfn << 2;
    b1msg2[IPMI_MSG2_CHAN_OFFSET]            = IPMI_MSG_CHAN_IPMBL + IPMI_MSG_TRACKING;
    b2msg1[IPMI_MSG1_RSADDR_OFFSET]          = FRU_I2C_ADDR[fruId];
    b2msg2[IPMI_MSG2_SET_FRU_ACT_FRU_OFFSET] = 0;

    messageSize = ipmiMsgBuild(mchData->ipmiSess, message, cmd, IPMI_MSG_NETFN_APP_REQUEST, imsg2, imsg2Size, b1msg1,
                               b1msg2, b1msg2Size, b2msg1, b2msg2, b2msg2Size);

    if ((rval = mchMsgWriteReadHelper(mchData->mchSess, mchData->ipmiSess, message, messageSize, response,
                                      &responseSize, cmd, netfn, roffs, 0)))
        goto bail;

    if ((rval = mchMsgCheckSizes(sizeof(response), roffs, payloadSize))) {
        printf("mchMsgGetFanLevelNat size error\n");
        goto bail;
    }

    memcpy(data, response + roffs, payloadSize);

    mchGetFanLevel(data, level, fru->fanProp);

bail:
    return rval;
}

/* Get Fan Level - For ATCA
 *
 *
 *   RETURNS: status from mchMsgWriteReadHelper
 *            0 on success
 *            non-zero for error
 */
int mchMsgGetFanLevelAtca(MchData mchData, uint8_t* data, uint8_t fruIndex, uint8_t* level) {
    uint8_t response[MSG_MAX_LENGTH] = {0};
    size_t  roffs, responseSize = 0, payloadSize = IPMI_RPLY_GET_FAN_LEVEL_LENGTH;
    int     rval, bridged = 0;
    Fru     fru    = &mchData->mchSys->fru[fruIndex];
    uint8_t rsAddr = fru->sdr.addr, rqAddr;

    if (rsAddr != IPMI_MSG_ADDR_BMC)
        bridged = 1;

    mchSetSizeOffs(mchData->ipmiSess, payloadSize, &roffs, &responseSize, &bridged, &rsAddr, &rqAddr);

    if ((rval = ipmiMsgGetFanLevel(mchData->mchSess, mchData->ipmiSess, response, bridged, rsAddr, rqAddr,
                                   &responseSize, roffs, fru->sdr.fruId)))
        goto bail;

    if ((rval = mchMsgCheckSizes(sizeof(response), roffs, payloadSize))) {
        printf("mchMsgGetFanLevelAtca size error\n");
        goto bail;
    }

    memcpy(data, response + roffs, payloadSize);

    mchGetFanLevel(data, level, fru->fanProp);

bail:
    return rval;
}

/* Set Fan Level using Vadatech MCH; message contains 1 bridged message  -> needs updating 3/23/16
 *
 *   RETURNS: status from mchMsgWriteReadHelper
 *            0 on success
 *            non-zero for error
 */
int mchMsgSetFanLevelVt(MchData mchData, uint8_t* data, uint8_t fruIndex, uint8_t level) {
    uint8_t message[MSG_MAX_LENGTH] = {0};
    uint8_t imsg2Size               = sizeof(SET_FAN_LEVEL_MSG);
    uint8_t imsg2[imsg2Size];
    size_t  messageSize;

    uint8_t response[MSG_MAX_LENGTH] = {0};
    size_t  roffs, responseSize, payloadSize = IPMI_RPLY_SET_FAN_LEVEL_LENGTH;
    int     rval, offs = 0, bridged = 0;
    uint8_t rsAddr = IPMI_MSG_ADDR_BMC, rqAddr, fruId = mchData->mchSys->fru[fruIndex].sdr.fruId;
    uint8_t cmd   = IPMI_MSG_CMD_SET_FAN_LEVEL;
    uint8_t netfn = IPMI_MSG_NETFN_PICMG;

    mchSetSizeOffs(mchData->ipmiSess, payloadSize, &roffs, &responseSize, &bridged, &rsAddr, &rqAddr);

    memcpy(imsg2, SET_FAN_LEVEL_MSG, imsg2Size);

    imsg2[IPMI_MSG2_SET_FRU_ACT_FRU_OFFSET] = fruId;
    imsg2[IPMI_MSG2_SET_FAN_LEVEL_OFFSET]   = level;

    if (bridged) // may need to distinguish between once and twice-bridged messages
        ipmiBuildSendMsg(mchData->ipmiSess, message, &messageSize, cmd, netfn, rsAddr, rqAddr, imsg2, imsg2Size, 0);
    else
        messageSize = ipmiMsgBuild(mchData->ipmiSess, message, cmd, netfn, imsg2, imsg2Size, 0, 0, 0, 0, 0, 0);

    if ((rval = mchMsgWriteReadHelper(mchData->mchSess, mchData->ipmiSess, message, messageSize, response,
                                      &responseSize, cmd, netfn, offs /*need to set codeoffs*/, 0)))
        goto bail;

    if ((rval = mchMsgCheckSizes(sizeof(response), roffs, payloadSize))) {
        printf("mchMsgSetFanLevelVt size error\n");
        goto bail;
    }
    memcpy(data, response + roffs, payloadSize);

bail:
    return rval;
}

/* Set Fan Level using NAT MCH; message contains 2 bridged messages
 *
 *   RETURNS: status from mchMsgWriteReadHelper
 *            0 on success
 *            non-zero for error
 */
int mchMsgSetFanLevelNat(MchData mchData, uint8_t* data, uint8_t fruIndex, uint8_t level) {
    uint8_t message[MSG_MAX_LENGTH]  = {0};
    uint8_t response[MSG_MAX_LENGTH] = {0};
    size_t  imsg2Size                = sizeof(SEND_MSG_MSG);
    size_t  b1msg1Size               = sizeof(IPMI_MSG1);
    size_t  b1msg2Size               = sizeof(SEND_MSG_MSG);
    size_t  b2msg1Size               = sizeof(IPMI_MSG1);
    size_t  b2msg2Size               = sizeof(GET_FAN_LEVEL_MSG);
    uint8_t imsg2[imsg2Size];
    uint8_t b1msg1[b1msg1Size];
    uint8_t b1msg2[b1msg2Size];
    uint8_t b2msg1[b2msg1Size];
    uint8_t b2msg2[b2msg2Size];
    size_t  messageSize;
    size_t  offs         = IPMI_RPLY_2ND_BRIDGED_OFFSET_NAT;
    size_t  roffs        = IPMI_RPLY_HEADER_LENGTH + 3 + offs;
    size_t  payloadSize  = IPMI_RPLY_SET_FAN_LEVEL_LENGTH;
    size_t  responseSize = roffs + payloadSize + FOOTER_LENGTH;
    uint8_t cmd          = IPMI_MSG_CMD_SET_FAN_LEVEL;
    uint8_t netfn        = IPMI_MSG_NETFN_PICMG;
    int     rval;
    uint8_t fruId = mchData->mchSys->fru[fruIndex].sdr.fruId;

    memcpy(imsg2, SEND_MSG_MSG, imsg2Size);
    memcpy(b1msg1, IPMI_MSG1, b1msg1Size);

    imsg2[IPMI_MSG2_CHAN_OFFSET]    = IPMI_MSG_CHAN_IPMB0 + IPMI_MSG_TRACKING;
    b1msg1[IPMI_MSG1_RSADDR_OFFSET] = IPMI_MSG_ADDR_CM;

    memcpy(b1msg2, SEND_MSG_MSG, b1msg2Size);
    memcpy(b2msg1, IPMI_MSG1, b2msg1Size);
    memcpy(b2msg2, SET_FAN_LEVEL_MSG, b2msg2Size);

    b1msg2[IPMI_MSG2_RQADDR_OFFSET]          = IPMI_MSG_ADDR_BMC;
    b2msg1[IPMI_MSG1_NETFNLUN_OFFSET]        = netfn << 2;
    b1msg2[IPMI_MSG2_CHAN_OFFSET]            = IPMI_MSG_CHAN_IPMBL + IPMI_MSG_TRACKING;
    b2msg1[IPMI_MSG1_RSADDR_OFFSET]          = FRU_I2C_ADDR[fruId];
    b2msg2[IPMI_MSG2_SET_FRU_ACT_FRU_OFFSET] = 0;
    b2msg2[IPMI_MSG2_SET_FAN_LEVEL_OFFSET]   = level;

    messageSize = ipmiMsgBuild(mchData->ipmiSess, message, cmd, IPMI_MSG_NETFN_APP_REQUEST, imsg2, imsg2Size, b1msg1,
                               b1msg2, b1msg2Size, b2msg1, b2msg2, b2msg2Size);

    if ((rval = mchMsgWriteReadHelper(mchData->mchSess, mchData->ipmiSess, message, messageSize, response,
                                      &responseSize, cmd, netfn, offs /*need to set codeoffs*/, 0)))
        goto bail;

    if ((rval = mchMsgCheckSizes(sizeof(response), roffs, payloadSize))) {
        printf("mchMsgSetFanLevelNat size error\n");
        goto bail;
    }
    memcpy(data, response + roffs, payloadSize);

bail:
    return rval;
}

/* Set Fan Level - For ATCA
 *
 *
 *   RETURNS: status from mchMsgWriteReadHelper
 *            0 on success
 *            non-zero for error
 */
int mchMsgSetFanLevelAtca(MchData mchData, uint8_t* data, uint8_t fruIndex, uint8_t level) {
    uint8_t response[MSG_MAX_LENGTH] = {0};
    size_t  roffs, responseSize = 0, payloadSize = IPMI_RPLY_SET_FAN_LEVEL_LENGTH;
    int     rval, bridged = 0;
    Fru     fru    = &mchData->mchSys->fru[fruIndex];
    uint8_t rsAddr = fru->sdr.addr, rqAddr;

    if (rsAddr != IPMI_MSG_ADDR_BMC)
        bridged = 1;

    mchSetSizeOffs(mchData->ipmiSess, payloadSize, &roffs, &responseSize, &bridged, &rsAddr, &rqAddr);

    if ((rval = ipmiMsgSetFanLevel(mchData->mchSess, mchData->ipmiSess, response, bridged, rsAddr, rqAddr,
                                   &responseSize, roffs, fru->sdr.fruId, level)))
        goto bail;

    if ((rval = mchMsgCheckSizes(sizeof(response), roffs, payloadSize))) {
        printf("mchMsgSetFanLevelAtca size error\n");
        goto bail;
    }

    memcpy(data, response + roffs, payloadSize);

bail:
    return rval;
}

/*  -> needs updating 3/23/16 */
int mchMsgGetPowerLevelVt(MchData mchData, uint8_t* data, uint8_t fruIndex, uint8_t parm) {
    uint8_t message[MSG_MAX_LENGTH] = {0};
    uint8_t imsg2Size               = sizeof(GET_POWER_LEVEL_MSG);
    uint8_t imsg2[imsg2Size];
    size_t  messageSize;
    Fru     fru = &mchData->mchSys->fru[fruIndex];

    uint8_t response[MSG_MAX_LENGTH] = {0};
    size_t  roffs, responseSize, payloadSize = IPMI_RPLY_GET_POWER_LEVEL_LENGTH;
    int     rval, offs = 0, bridged = 0;
    uint8_t rsAddr = IPMI_MSG_ADDR_BMC, rqAddr;
    uint8_t cmd    = IPMI_MSG_CMD_GET_POWER_LEVEL;
    uint8_t netfn  = IPMI_MSG_NETFN_PICMG;

    mchSetSizeOffs(mchData->ipmiSess, payloadSize, &roffs, &responseSize, &bridged, &rsAddr, &rqAddr);

    memcpy(imsg2, GET_POWER_LEVEL_MSG, imsg2Size);

    imsg2[IPMI_MSG2_SET_FRU_ACT_FRU_OFFSET]             = fru->sdr.fruId;
    imsg2[PICMG_RPLY_IMSG2_GET_POWER_LEVEL_TYPE_OFFSET] = parm;

    if (bridged) // may need to distinguish between once and twice-bridged messages
        ipmiBuildSendMsg(mchData->ipmiSess, message, &messageSize, cmd, netfn, rsAddr, rqAddr, imsg2, imsg2Size, 0);
    else
        messageSize = ipmiMsgBuild(mchData->ipmiSess, message, cmd, netfn, imsg2, imsg2Size, 0, 0, 0, 0, 0, 0);

    if ((rval = mchMsgWriteReadHelper(mchData->mchSess, mchData->ipmiSess, message, messageSize, response,
                                      &responseSize, cmd, netfn, offs /*need to set codeoffs*/, 0)))
        goto bail;

    if ((rval = mchMsgCheckSizes(sizeof(response), roffs, payloadSize))) {
        printf("mchMsgGetPowerLevelVt size error\n");
        goto bail;
    }
    memcpy(data, response + roffs, payloadSize);

bail:
    return rval;
}

/* Set Fan Level - For ATCA
 *
 *
 *   RETURNS: status from mchMsgWriteReadHelper
 *            0 on success
 *            non-zero for error
 */
int mchMsgGetPowerLevelAtca(MchData mchData, uint8_t* data, uint8_t fruIndex, uint8_t parm) {
    uint8_t response[MSG_MAX_LENGTH] = {0};
    size_t  roffs, responseSize = 0, payloadSize = IPMI_RPLY_GET_POWER_LEVEL_LENGTH;
    int     rval, bridged = 0;
    Fru     fru    = &mchData->mchSys->fru[fruIndex];
    uint8_t rsAddr = fru->sdr.addr, rqAddr;

    if (rsAddr != IPMI_MSG_ADDR_BMC)
        bridged = 1;

    mchSetSizeOffs(mchData->ipmiSess, payloadSize, &roffs, &responseSize, &bridged, &rsAddr, &rqAddr);

    if ((rval = ipmiMsgGetPowerLevel(mchData->mchSess, mchData->ipmiSess, response, bridged, rsAddr, rqAddr,
                                     &responseSize, roffs, fru->sdr.fruId, parm)))
        goto bail;

    if ((rval = mchMsgCheckSizes(sizeof(response), roffs, payloadSize))) {
        printf("mchMsgSetFanLevelAtca size error\n");
        goto bail;
    }

    memcpy(data, response + roffs, payloadSize);

bail:
    return rval;
}

MchCbRec drvMchMicrotcaVtCb = {
    assign_sys_sizes : 0,
    assign_site_info : 0,
    assign_fru_lkup : assign_fru_lkup_microtca,
    fru_data_suppl : fru_data_suppl_picmg,
    sensor_get_fru : sensor_get_fru_microtca,
    get_chassis_status : 0,
    set_fru_act : mchMsgSetFruActPolicyVt,
    get_fan_prop : mchMsgGetFanPropVt,
    get_fan_level : mchMsgGetFanLevelVt,
    set_fan_level : mchMsgSetFanLevelVt,
    get_power_level : mchMsgGetPowerLevelVt
};

MchCbRec drvMchMicrotcaNatCb = {
    assign_sys_sizes : 0,
    assign_site_info : 0,
    assign_fru_lkup : assign_fru_lkup_microtca,
    fru_data_suppl : fru_data_suppl_picmg,
    sensor_get_fru : sensor_get_fru_microtca,
    get_chassis_status : 0,
    set_fru_act : mchMsgSetFruActNat,
    get_fan_prop : mchMsgGetFanPropNat,
    get_fan_level : mchMsgGetFanLevelNat,
    set_fan_level : mchMsgSetFanLevelNat,
    get_power_level : 0
};

MchCbRec drvMchAtcaCb = {
    assign_sys_sizes : assign_sys_sizes_atca,
    assign_site_info : assign_site_info_atca,
    assign_fru_lkup : assign_fru_lkup_atca,
    fru_data_suppl : fru_data_suppl_picmg,
    sensor_get_fru : sensor_get_fru_atca,
    get_chassis_status : mchMsgGetChassisStatus,
    set_fru_act : mchMsgSetFruActAtca,
    get_fan_prop : mchMsgGetFanPropAtca,
    get_fan_level : mchMsgGetFanLevelAtca,
    set_fan_level : mchMsgSetFanLevelAtca,
    get_power_level : mchMsgGetPowerLevelAtca
};

static void drvMchPicmgRegistrar(void) {
    registryAdd((void*)mchCbRegistryId, "drvMchMicrotcaVtCb", &drvMchMicrotcaVtCb);
    registryAdd((void*)mchCbRegistryId, "drvMchMicrotcaNatCb", &drvMchMicrotcaNatCb);
    registryAdd((void*)mchCbRegistryId, "drvMchAtcaCb", &drvMchAtcaCb);
}

epicsExportRegistrar(drvMchPicmgRegistrar);
