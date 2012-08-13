# Makefile
TOP = ..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

JSON_INC=/afs/slac/g/lcls/package/json-c/current
JSON_LIB=/afs/slac/g/lcls/package/json-c/current/lib

LIBRARY_IOC += ipmiComm

DBD += ipmiComm.dbd

ipmiComm_SRCS += drvMch.c drvMchUtil.c devMch.c ipmiMsg.c ipmiDef.c

json_DIR = $(JSON_LIB)
USR_LIBS_Linux       += json


ipmiComm_LIBS += $(EPICS_BASE_IOC_LIBS) asyn

include $(TOP)/configure/RULES

#----------------------------------------
#  ADD RULES AFTER THIS LINE


