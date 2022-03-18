TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#=============================
# Build the IOC application

PROD_IOC = ipmicomm-test-IOC
# ipmicomm-test-IOC.dbd will be created and installed
DBD += ipmicomm-test-IOC.dbd

# ipmicomm-test-IOC.dbd will be made up from these files:
ipmicomm-test-IOC_DBD += base.dbd
ipmicomm-test-IOC_DBD += asyn.dbd
ipmicomm-test-IOC_DBD += drvAsynIPPort.dbd

# Include dbd files from all support applications:
ipmicomm-test-IOC_DBD += ipmiComm.dbd 

# Add all the support libraries needed by this IOC.
# Search path is generated from definitions in ipmicomm-test-IOC/configure/RELEASE.
ipmicomm-test-IOC_LIBS += ipmiComm
ipmicomm-test-IOC_LIBS += asyn

# ipmicomm-test-IOC_registerRecordDeviceDriver.cpp derives from ipmicomm-test-IOC.dbd
ipmicomm-test-IOC_SRCS += ipmicomm-test-IOC_registerRecordDeviceDriver.cpp

# Build the main IOC entry point on workstation OSs.
ipmicomm-test-IOC_SRCS_DEFAULT += ipmicomm-test-IOCMain.cpp
ipmicomm-test-IOC_SRCS_vxWorks += -nil-

# Add support from base/src/vxWorks if needed
#ipmicomm-test-IOC_OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary

# Finally link to the EPICS Base libraries
ipmicomm-test-IOC_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

