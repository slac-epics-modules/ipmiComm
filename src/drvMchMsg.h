//////////////////////////////////////////////////////////////////////////////
// This file is part of 'ipmiComm'.
// It is subject to the license terms in the LICENSE.txt file found in the
// top-level directory of this distribution and at:
//    https://confluence.slac.stanford.edu/display/ppareg/LICENSE.html.
// No part of 'ipmiComm', including this file,
// may be copied, modified, propagated, or distributed except according to
// the terms contained in the LICENSE.txt file.
//////////////////////////////////////////////////////////////////////////////
#ifndef DRV_MCH_MSG_H
#define DRV_MCH_MSG_H

#include <ipmiDef.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /* IMPORTANT: For all routines below, caller must perform locking */

    void mchSetSizeOffs(IpmiSess ipmiSess, size_t payloadSize, size_t* roffs, size_t* responseSize, int* bridged,
                        uint8_t* rsAddr, uint8_t* rqAddr);

    int mchMsgCheckSizes(size_t destSize, int offset, size_t srcSize);

    int mchMsgGetChanAuth(MchSess mchSess, IpmiSess ipmiSess, uint8_t* response);

    int mchMsgGetSess(MchSess mchSess, IpmiSess ipmiSess, uint8_t* response);

    int mchMsgActSess(MchSess mchSess, IpmiSess ipmiSess, uint8_t* response);

    int mchMsgSetPriv(MchSess mchSess, IpmiSess ipmiSess, uint8_t* response, uint8_t level);

    int mchMsgWriteReadHelper(MchSess mchSess, IpmiSess ipmiSess, uint8_t* message, size_t messageSize,
                              uint8_t* response, size_t* responseSize, uint8_t cmd, uint8_t netfn, int codeOffs,
                              int outSess);

    int mchMsgCloseSess(MchSess mchSess, IpmiSess ipmiSess, uint8_t* data);

    int mchMsgChassisControl(MchData mchData, uint8_t* data, uint8_t parm);

    int mchMsgGetChassisStatus(MchData mchData, uint8_t* data);

    int mchMsgGetFruInvInfoWrapper(MchData mchData, uint8_t* data, Fru fru);

    int mchMsgReadFruWrapper(MchData mchData, uint8_t* data, Fru fru, uint8_t* readOffset, uint8_t readSize);

    int mchMsgGetSdrRepInfoWrapper(MchData mchData, uint8_t* data, uint8_t parm, uint8_t addr);

    int mchMsgReserveSdrRepWrapper(MchData mchData, uint8_t* data, uint8_t parm, uint8_t addr);

    int mchMsgGetSdrWrapper(MchData mchData, uint8_t* data, uint8_t* id, uint8_t* res, uint8_t offset, uint8_t readSize,
                            uint8_t parm, uint8_t rsAddr);

    // int mchMsgGetDevSdrInfo(MchData mchData, uint8_t *data, int bridged, uint8_t rsAddr, uint8_t parm);

    int mchMsgReadSensorWrapper(MchData mchData, uint8_t* data, Sensor sens, size_t* sensReadMsgSize);

    int mchMsgGetSensorThresholdsWrapper(MchData mchData, uint8_t* data, Sensor sens);

    int mchMsgGetDeviceIdWrapper(MchData mchData, uint8_t* data, uint8_t rsAddr);

#ifdef __cplusplus
};
#endif

#endif
