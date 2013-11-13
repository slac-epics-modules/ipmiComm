# Makefile
TOP = ..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

LIBRARY_IOC += ipmiComm

DBD += ipmiComm.dbd

ipmiComm_SRCS += drvMch.c devMch.c drvMchMsg.c ipmiMsg.c ipmiDef.c

ipmiComm_LIBS += $(EPICS_BASE_IOC_LIBS) asyn

include $(TOP)/configure/RULES

#----------------------------------------
#  ADD RULES AFTER THIS LINE


