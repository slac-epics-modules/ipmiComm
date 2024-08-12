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
#include <stdio.h>

#include <errlog.h>
#include <epicsExport.h>
#include <iocsh.h>
#include <registry.h>

#include <drvMch.h>
#include <drvMchMsg.h>
#include <ipmiMsg.h>

static void assign_sys_sizes_supermicro(MchData mchData) {
    mchData->mchSys->fruCountMax  = 1;
    mchData->mchSys->mgmtCountMax = 1;
}

static void sensor_get_fru_supermicro(MchData mchData, Sensor sens) {
    /* Supermicro does not use FRU device locator records, so no means to read a FRU ID
     * Associate all Supermicro and Advantech sensors with single FRU, arbitrarily choose 0
     */
    sens->fruId = sens->fruIndex = 0;
}

static void assign_fru_lkup_supermicro(MchData mchData) {
    uint8_t id, index;

    /* Supermicro does not use FRU device locator records, so no means to read a FRU ID
     * Associate all Supermicro and Advantech sensors with single FRU, arbitrarily choose 0
     * Set FRU address to BMC
     * Force FRU count to be 1
     */
    id = index                           = 0;
    mchData->mchSys->fru[index].id       = id;
    mchData->mchSys->fru[index].sdr.addr = IPMI_MSG_ADDR_BMC;
    mchData->mchSys->fruLkup[id]         = index;
    mchData->mchSys->fruCount            = 1;
}

static void sensor_get_fru_advantech(MchData mchData, Sensor sens) {
    int i;
    Fru fru;
    /* Associate all Supermicro and Advantech sensors with single FRU, ID 0 */
    sens->fruId = 0;
    for (i = 0; i < mchData->mchSys->fruCount; i++) {
        fru = &mchData->mchSys->fru[i];
        if ((fru->id == 0)) {
            sens->fruIndex = i;
            break;
        }
    }
}

static void assign_fru_lkup_advantech(MchData mchData) {
    int i, found = 0;
    Fru fru;
    for (i = 0; i < mchData->mchSys->fruCount; i++) {

        fru = &mchData->mchSys->fru[i];

        if (fru->sdr.recType) { /* If real entity */

            fru->id                           = fru->sdr.fruId;
            mchData->mchSys->fruLkup[fru->id] = i;
            if (fru->id == 0) {
                found = 1;
            }
        }
    }
    if (found == 0) { /* If FRU 0 not already in the data structure, add it */
        fru                               = &mchData->mchSys->fru[mchData->mchSys->fruCount];
        fru->id                           = 0;
        mchData->mchSys->fruLkup[fru->id] = i;
        fru->sdr.addr                     = IPMI_MSG_ADDR_BMC;
        mchData->mchSys->fruCount++;
    }
}

MchCbRec drvMchSupermicroCb = {.assign_sys_sizes   = assign_sys_sizes_supermicro,
                               .assign_site_info   = 0,
                               .assign_fru_lkup    = assign_fru_lkup_supermicro,
                               .fru_data_suppl     = 0,
                               .sensor_get_fru     = sensor_get_fru_supermicro,
                               .get_chassis_status = mchMsgGetChassisStatus};

MchCbRec drvMchAdvantechCb = {.assign_sys_sizes   = 0,
                              .assign_site_info   = 0,
                              .assign_fru_lkup    = assign_fru_lkup_advantech,
                              .fru_data_suppl     = 0,
                              .sensor_get_fru     = sensor_get_fru_advantech,
                              .get_chassis_status = mchMsgGetChassisStatus};

static void drvMchServerPcRegistrar(void) {
    registryAdd((void*)mchCbRegistryId, "drvMchSupermicroCb", &drvMchSupermicroCb);
    registryAdd((void*)mchCbRegistryId, "drvMchAdvantechCb", &drvMchAdvantechCb);
}

epicsExportRegistrar(drvMchServerPcRegistrar);
