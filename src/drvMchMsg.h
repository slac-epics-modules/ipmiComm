#ifndef DRV_MCH_MSG_H
#define DRV_MCH_MSG_H

#include <ipmiDef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* IMPORTANT: For all routines below, caller must perform locking */

int mchMsgGetChanAuth(MchSess mchSess, IpmiSess ipmiSess, uint8_t *response);

int mchMsgGetSess(MchSess mchSess, IpmiSess ipmiSess, uint8_t *response);

int mchMsgActSess(MchSess mchSess, IpmiSess ipmiSess, uint8_t *response);

int mchMsgSetPriv(MchSess mchSess, IpmiSess ipmiSess, uint8_t *response, uint8_t level);

int mchMsgWriteReadHelper(MchSess mchSess, IpmiSess ipmiSess, uint8_t *message, size_t messageSize, uint8_t *response, size_t *responseSize, uint8_t cmd, uint8_t netfn, int codeOffs, int outSess);

int mchMsgCloseSess(MchSess mchSess, IpmiSess ipmiSess, uint8_t *data);

int mchMsgChassisControl(MchData mchData, uint8_t *data, uint8_t parm);

int mchMsgGetChassisStatus(MchData mchData, uint8_t *data);

int mchMsgGetFruInvInfo(MchData mchData, uint8_t *data, uint8_t id);

int mchMsgReadFru(MchData mchData, uint8_t *data, uint8_t id, uint8_t *readOffset, uint8_t readSize);

int mchMsgGetSdrRepInfo(MchData mchData, uint8_t *data);

int mchMsgReserveSdrRep(MchData mchData, uint8_t *data);

int mchMsgGetSdr(MchData mchData, uint8_t *data, uint8_t *id, uint8_t *res, uint8_t offset, uint8_t readSize, uint8_t parm, uint8_t recordSize);

int mchMsgGetDevSdrInfo(MchData mchData, uint8_t *data, uint8_t parm);
				
int mchMsgReadSensor(MchData mchData, uint8_t *data, uint8_t sens, uint8_t lun, size_t *responseSize);

int mchMsgGetSensorThresholds(MchData mchData, uint8_t *data, uint8_t sens, uint8_t lun);

int mchMsgSetFruActHelper(MchData mchData, uint8_t *data, uint8_t fruId, int parm);

int mchMsgGetFruActPolicyHelper(MchData mchData, uint8_t *data, uint8_t fruId);

int mchMsgGetDeviceId(MchData mchData, uint8_t *data, int bridged, uint8_t rsAddr);

int mchMsgGetFanPropHelper(MchData mchData, uint8_t *data, uint8_t fruId);

int mchMsgGetFanLevelHelper(MchData mchData, uint8_t *data, uint8_t fruId, uint8_t fanProp, uint8_t *level);

int mchMsgSetFanLevelHelper(MchData mchData, uint8_t *data, uint8_t fruId, uint8_t level);

int mchMsgGetPowerLevelVt(MchData mchData, uint8_t *data, uint8_t fruId, uint8_t parm);

#ifdef __cplusplus
};
#endif

#endif
