TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#========================================

# Build IOC application for testing

# This will attempt to build an IOC application 
# for every target including the host computer.
PROD_IOC = ipmiCommIoc

# ipmiCommIoc.dbd will be created and installed
DBD += ipmiCommIoc.dbd

# ipmiCommIoc.dbd will be made up from these files:
ipmiCommIoc_DBD += base.dbd
ipmiCommIoc_DBD += asyn.dbd
ipmiCommIoc_DBD += drvAsynIPPort.dbd
ipmiCommIoc_DBD += iocAdmin.dbd

ipmiCommIoc_DBD += ipmiComm.dbd

ipmiCommIoc_SRCS += ipmiCommIoc_registerRecordDeviceDriver.cpp

ipmiCommIoc_SRCS_vxWorks += -nil-
ipmiCommIoc_SRCS_RTEMS   += -nil-

ipmiCommIoc_SRCS_DEFAULT += ipmiCommIocMain.cpp

ipmiCommIoc_LIBS += asyn
ipmiCommIoc_LIBS += devIocStats

ipmiCommIoc_LIBS += ipmiComm

ipmiCommIoc_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================
include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
