#include <stdint.h>
#include <stdio.h>

#include <errlog.h>
#include <epicsExport.h>
#include <iocsh.h>
#include <registry.h>

#include <drvMch.h>
#include <drvMchMsg.h>
#include <ipmiMsg.h>

static void
assign_sys_sizes_serverpc(MchData mchData)
{
	mchData->mchSys->fruCountMax  = 1;
	mchData->mchSys->mgmtCountMax = 1;
}

static void
sensor_get_fru_serverpc(MchData mchData, Sensor sens)
{
	/* Supermicro does not use FRU device locator records, so no means to read a FRU ID
	 * Associate all Supermicro and Advantech sensors with single FRU, arbitrarily choose...0?
	 */
	sens->fruId = sens->fruIndex = 0;
}

static void
assign_fru_lkup_serverpc(MchData mchData)
{
uint8_t id, index;

	/* Supermicro does not use FRU device locator records, so no means to read a FRU ID
	 * Associate all Supermicro and Advantech sensors with single FRU, arbitrarily choose...0?
	 * Set FRU address to BMC 
	 * Force FRU count to be 1
	 */
	id = index = 0;
	mchData->mchSys->fru[index].id = id;
	mchData->mchSys->fru[index].sdr.addr = IPMI_MSG_ADDR_BMC;
	mchData->mchSys->fruLkup[id] = index;
	mchData->mchSys->fruCount = 1;

}

MchCbRec drvMchServerPcCb = {
    assign_sys_sizes: assign_sys_sizes_serverpc,   
    assign_site_info: 0,
    assign_fru_lkup:  assign_fru_lkup_serverpc,
    fru_data_suppl:   0,
    sensor_get_fru:   sensor_get_fru_serverpc,
    get_chassis_status: mchMsgGetChassisStatus

};

static void
drvMchServerPcRegistrar(void)
{
	registryAdd( (void*)mchCbRegistryId, "drvMchServerPcCb", &drvMchServerPcCb );
}

epicsExportRegistrar(drvMchServerPcRegistrar);
