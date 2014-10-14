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
cd ${TOP}

# Load EPICS Database
dbLoadDatabase("dbd/ipmiCommIoc.dbd")
ipmiCommIoc_registerRecordDeviceDriver(pdbbase)

# Order of commands is important
drvAsynIPPortConfigure ("mch-test","mch-b34-cd42:623 udp",0,0,0)
mchInit("mch-test")
dbLoadRecords("db/shelf.db","dev=MCH:TEST,link=mch-test")

#asynSetTraceMask("mch-test",-1,0x9)
#asynSetTraceIOMask("mch-test",-1,0x5)

cd ${TOP}/iocBoot/${IOC}
iocInit()

# End of file

