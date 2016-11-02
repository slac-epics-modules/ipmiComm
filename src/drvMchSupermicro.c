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
sensor_get_fru_supermicro(MchData mchData, Sensor sens)
{
	/* Supermicro does not use FRU device locator records, so no means to read a FRU ID
	 * Associate all Supermicro sensors with single FRU, arbitrarily choose...0?
	 */
	sens->fruId = sens->fruIndex = 0;
	return;
}

MchCbRec drvMchSupermicroCb = {
    assign_site_info: 0,
    assign_fru_lkup:  0,
    fru_data_suppl:   0,
    sensor_get_fru:   sensor_get_fru_supermicro,
    get_chassis_status: mchMsgGetChassisStatus

};

static void
drvMchSupermicroRegistrar(void)
{
	registryAdd( (void*)mchCbRegistryId, "drvMchSupermicroCb", &drvMchSupermicroCb );
}

epicsExportRegistrar(drvMchSupermicroRegistrar);
