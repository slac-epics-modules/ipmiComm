#!../../bin/rhel7-x86_64/ipmicomm-test-IOC
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
cd ${TOP}

# Load EPICS Database
dbLoadDatabase("dbd/ipmicomm-test-IOC.dbd")
ipmicomm_test_IOC_registerRecordDeviceDriver(pdbbase)

# Order of commands is important
drvAsynIPPortConfigure ("shm-test","shm-b084-sp17-2:623 udp",0,0,0)
mchInit("shm-test")
dbLoadRecords("db/shelf_atca_7slot_lcls.db","dev=CRAT:TEST,link=shm-test,location=")

#asynSetTraceMask("mch-test",-1,0x9)
#asynSetTraceIOMask("mch-test",-1,0x5)

cd ${TOP}/iocBoot/${IOC}
iocInit()

# End of file

