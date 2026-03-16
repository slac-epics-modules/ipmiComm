# Makefile
TOP = ..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#

#ifndef IPMI_USERNAME
#$(error Need to set IPMI_USERNAME)
#endif

#ifndef IPMI_PASSWORD
#$(error Need to set IPMI_PASSWORD)
#endif

ifdef IPMI_USERNAME
USR_CFLAGS += -DIPMI_USERNAME=\"$(IPMI_USERNAME)\"
endif
ifdef IPMI_PASSWORD
USR_CFLAGS += -DIPMI_PASSWORD=\"$(IPMI_PASSWORD)\"
endif

LIBRARY_IOC += ipmiComm

DBD += ipmiComm.dbd

ipmiComm_SRCS += drvMch.c devMch.c drvMchMsg.c ipmiMsg.c ipmiDef.c picmgDef.c
ipmiComm_SRCS += drvMchPicmg.c drvMchServerPc.c
ipmiComm_SRCS += subIpmiComm.c

ipmiComm_DBD += drvMchServerPc.dbd
ipmiComm_DBD += drvMchPicmg.dbd

ipmiComm_LIBS += $(EPICS_BASE_IOC_LIBS) asyn

include $(TOP)/configure/RULES

#----------------------------------------
#  ADD RULES AFTER THIS LINE


