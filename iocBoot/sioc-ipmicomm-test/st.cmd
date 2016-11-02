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

#drvAsynIPPortConfigure ("crat-b34-hp01","shm-b34-hp01:623 udp",0,0,0)
#drvAsynIPPortConfigure ("crat-b34-hp02","shm-b34-hp02:623 udp",0,0,0)
#drvAsynIPPortConfigure ("crat-b34-hp03","shm-b34-hp03:623 udp",0,0,0)
#drvAsynIPPortConfigure ("crat-b34-hp04","shma-b34-hp04:623 udp",0,0,0)
#drvAsynIPPortConfigure ("crat-b34-hp05","shm-b34-hp05:623 udp",0,0,0)
#drvAsynIPPortConfigure ("crat-b84-hp01","shm-b084-hp01:623 udp",0,0,0)
#drvAsynIPPortConfigure ("crat-b84-hp02","shm-b084-hp02:623 udp",0,0,0)
#drvAsynIPPortConfigure ("crat-b84-hp03","shm-b084-hp03:623 udp",0,0,0)
#drvAsynIPPortConfigure ("crat-b84-hp04","shm-b084-hp04:623 udp",0,0,0)
#drvAsynIPPortConfigure ("crat-b84-hp05","shm-b084-hp05:623 udp",0,0,0)

#mchInit("crat-b34-hp01")
#mchInit("crat-b34-hp02")
#mchInit("crat-b34-hp03")
#mchInit("crat-b34-hp04")
#mchInit("crat-b34-hp05")
#mchInit("crat-b84-hp01")
#mchInit("crat-b84-hp02")
#mchInit("crat-b84-hp03")
#mchInit("crat-b84-hp04")
#mchInit("crat-b84-hp05")

#dbLoadRecords("db/shelf_atca_12slot.db","dev=CRAT:B34:HP01,link=crat-b34-hp01,location=")
#dbLoadRecords("db/shelf_atca_12slot.db","dev=CRAT:B34:HP02,link=crat-b34-hp02,location=")
#dbLoadRecords("db/shelf_atca_12slot.db","dev=CRAT:B34:HP03,link=crat-b34-hp03,location=")
#dbLoadRecords("db/shelf_atca_12slot.db","dev=CRAT:B34:HP04,link=crat-b34-hp04,location=")
#dbLoadRecords("db/shelf_atca_12slot.db","dev=CRAT:B34:HP05,link=crat-b34-hp05,location=")
#dbLoadRecords("db/shelf_atca_12slot.db","dev=CRAT:B84:HP01,link=crat-b84-hp01,location=")
#dbLoadRecords("db/shelf_atca_12slot.db","dev=CRAT:B84:HP02,link=crat-b84-hp02,location=")
#dbLoadRecords("db/shelf_atca_12slot.db","dev=CRAT:B84:HP03,link=crat-b84-hp03,location=")
#dbLoadRecords("db/shelf_atca_12slot.db","dev=CRAT:B84:HP04,link=crat-b84-hp04,location=")
#dbLoadRecords("db/shelf_atca_12slot.db","dev=CRAT:B84:HP05,link=crat-b84-hp05,location=")

# Order of commands is important
#drvAsynIPPortConfigure ("mch-li27-bp03","mch-li27-bp03:623 udp",0,0,0)
#drvAsynIPPortConfigure ("b34-mp03","ioc-b34-mp03-mgt:623 udp",0,0,0)
#drvAsynIPPortConfigure ("cpu-pm01","cpu-b084-pm01-mgt:623 udp",0,0,0)
drvAsynIPPortConfigure ("shm-test1","shm-b084-hp05:623 udp",0,0,0)
##drvAsynIPPortConfigure ("shm-test1","shm-b084-hp02:623 udp",0,0,0)
#drvAsynIPPortConfigure ("shm-test2","mch-b34-cd42:623 udp",0,0,0)
#mchInit("mch-li27-bp03")
#mchInit("cpu-pm01")
#mchInit("b34-mp03")
mchInit("shm-test1")
#mchInit("shm-test2")
#dbLoadRecords("db/shelf.db","dev=MCH:LI27:BP03,link=mch-li27-bp03,location=")
dbLoadRecords("db/shelf_atca_7slot.db","dev=CRAT:TEST1,link=shm-test1,location=")
#dbLoadRecords("db/shelf_microtca_12slot.db","dev=MCH:TEST,link=shm-test2,location=")
##dbLoadRecords("db/shelf.db","dev=MCH:TEST2,link=mch-test2,location=")
#dbLoadRecords("db/server_supermicro.db","dev=CPU:PM01,link=cpu-pm01,location=")
#dbLoadRecords("db/server_supermicro.db","dev=B34:MP03,link=b34-mp03,location=")
#dbLoadRecords("db/server_dell.db","dev=MCH:TEST,link=mch-test,location=")

#asynSetTraceMask("mch-test",-1,0x9)
#asynSetTraceIOMask("mch-test",-1,0x5)

cd ${TOP}/iocBoot/${IOC}
iocInit()

# End of file

