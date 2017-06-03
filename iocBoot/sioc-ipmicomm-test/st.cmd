#!../../bin/linux-x86/ipmiCommIoc
#==============================================================
#
#  Abs:  Startup script for Network IOC
#
#  Name: st.cmd
#
#  Facility: LCLS IOC Management
#
#  Auth: dd-mmm-yyyy, Sonya Hoobler   (SONYA):
#  Rev:  dd-mmm-yyyy, Reviewer's Name (USERNAME)
#--------------------------------------------------------------
#  Mod:
#
#       dd-mmm-yyyy, First Lastname    (USERNAME):
#         comment
#
#==============================================================
#
# Set environmment variables
< envPaths

# Change to TOP of ioc application
#cd ${TOP}
cd ../../

# Load EPICS Database
dbLoadDatabase("dbd/ipmiCommIoc.dbd")
ipmiCommIoc_registerRecordDeviceDriver(pdbbase)

dbLoadRecords("db/iocAdminSoft.db","IOC=MCHTEST")

# Order of commands is important
#drvAsynIPPortConfigure ("mch-li27-bp03","mch-li27-bp03:623 udp",0,0,0)
#drvAsynIPPortConfigure ("b34-mp03","ioc-b34-mp03-mgt:623 udp",0,0,0)
drvAsynIPPortConfigure ("cpu-test","cpu-b033-sp01-mgt:623 udp",0,0,0)
#drvAsynIPPortConfigure ("cpu-test","cpu-b084-pm01-mgt:623 udp",0,0,0)
#drvAsynIPPortConfigure ("shm-test1","shm-b084-hp05:623 udp",0,0,0)
##drvAsynIPPortConfigure ("shm-test1","shm-b084-hp02:623 udp",0,0,0)
drvAsynIPPortConfigure ("shm-test","shm-b033-sp01:623 udp",0,0,0)
#mchInit("mch-li27-bp03")
mchInit("cpu-test")
#mchInit("b34-mp03")
#mchInit("shm-test1")
mchInit("shm-test")
#dbLoadRecords("db/shelf.db","dev=MCH:LI27:BP03,link=mch-li27-bp03,location=")
dbLoadRecords("db/shelf_atca_7slot_lcls.db","dev=CRAT:TEST,link=shm-test,location=")
#dbLoadRecords("db/shelf_microtca_12slot.db","dev=MCH:TEST,link=shm-test2,location=")
##dbLoadRecords("db/shelf.db","dev=MCH:TEST2,link=mch-test2,location=")
dbLoadRecords("db/server_pc_lcls.db","dev=CPU:TEST,link=cpu-test,location=")
#dbLoadRecords("db/server_supermicro.db","dev=B34:MP03,link=b34-mp03,location=")
#dbLoadRecords("db/server_dell.db","dev=MCH:TEST,link=mch-test,location=")

#asynSetTraceMask("mch-test",-1,0x9)
#asynSetTraceIOMask("mch-test",-1,0x5)

cd ${TOP}/iocBoot/${IOC}
iocInit()

# End of file

