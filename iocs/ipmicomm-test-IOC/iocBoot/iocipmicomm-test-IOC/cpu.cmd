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
drvAsynIPPortConfigure ("cpu-test","cpu-b084-sp18-mgt:623 udp",0,0,0)
mchInit("cpu-test")
dbLoadRecords("db/server_pc_lcls.db","dev=CPU:TEST,link=cpu-test,location=")

#asynSetTraceMask("mch-test",-1,0x9)
#asynSetTraceIOMask("mch-test",-1,0x5)

cd ${TOP}/iocBoot/${IOC}
iocInit()

# End of file

