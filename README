############################################################
#
#	IPMI module README 
#
#	Auth: Sonya Hoobler
#
#	Last updated: 29 Aug 2017
#
############################################################

To add IPMI support to your application:

1. Add to configure/RELEASE:

IPMICOMM_MODULE_VERSION=ipmiComm-R*-*-*
IPMICOMM=$(EPICS_MODULES)/ipmiComm/$(IPMICOMM_MODULE_VERSION)

2. Add to src/Makefile

IOCManager_DBD  += ipmiComm.dbd
IOCManager_LIBS += ipmiComm

3. Add to Db/Makefile

At LCLS:

DB_INSTALLS += $(IPMICOMM)/db/shelf_microtca_12slot_lcls.db
DB_INSTALLS += $(IPMICOMM)/db/server_pc_lcls.db
DB_INSTALLS += $(IPMICOMM)/db/shelf_atca_7slot_lcls.db

At another facility that does not need the LCLS-specific functionality:

DB_INSTALLS += $(IPMICOMM)/db/shelf_microtca_12slot.db
DB_INSTALLS += $(IPMICOMM)/db/server_pc.db
DB_INSTALLS += $(IPMICOMM)/db/shelf_atca_7slot.db

4. Add to Db/<yourdbname>.substitutions, something like this, one substitutions
line for each device:

file server_pc_lcls.db
{
   pattern { dev            	, link     		, location		}  
           { CRAT:B084:PM01 	, cpu-b084-pm01-mgt 	, B084-Rm136		}
}

file shelf_microtca_12slot_lcls.db
{
   pattern { dev          	, link          	, location            	}  
           { CRAT:B34:CD43 	, mch-b34-cd43 		, B34...           	}
}

file shelf_atca_7slot_lcls.db
{
   pattern { dev                , link          	, location      }       
           { CRAT:LI28:SP02     , crat-li28-sp02	, KF28-2CXX     }    
}


5. Add these lines to st.cmd for each device before iocInit. 

Example for mch-b34-cd38, *Note that order is important for first two commands*:

drvAsynIPPortConfigure ("mch-b34-cd38","mch-b34-cd38:623 udp",0,0,0)
mchInit("mch-b34-cd43")

dbLoadRecords("../../db/<yourdbname>.db")

5. Archiving

For each 7-slot ATCA shelf, add the PVs in the IOCManager srcArchive/SHELF_ATCA_7SLOT_ARCHIVE.cwConfig
to the archiver, substituting the device name for $(dev)

For each Advantech or Supermicro PC, add the PVs in the IOCManager srcArchive/SERVER_PC.cwConfig
to the archiver, substituting the device name for $(dev)

(We do not currently have a MicroTCA archiving template.)

The templates contain all possible PVs and will (nearly always) be a superset of the 
PVs that are actually in use by the system. If you prefer, you can choose to only archive 
those that you know are in use.


NOTES ON THIS RELEASE
---------------------------

(below, 'ioc' refers to the managment ioc using ipmiComm to communicate 
 with a remote crate. at LCLS, this ioc is a soft ioc called IOCManager.)

-This release supports:

  MicroTCA NAT and Vadatech MCHs (not well tested, especially Vadatech)
  ATCA Pentair and Artesyn shelf managers
  Supermicro and Advantech PCs

-During ioc boot, the ioc will discover which modules/sensors exist
 for each MCH defined in st.cmd. If the system is offline, the ioc will do basic
 initialization and perform the module/sensor discovery later, when it comes online.

-During runtime, if the ioc detects that a system's configuration has changed, it will
 re-discover the modules/sensors and modify the PVs as appropriate.

-There is a EPICS PV $(crat):INIT for each system. It is set to "Initialized"
 after all of the modules/sensors have been discovered. It is set to "Not
 initialized" if not. During the discovery process, it is set to "Initializing...".

-There is a EPICS PV $(crat):CONNECT for each system. It must be set to "Connect". 