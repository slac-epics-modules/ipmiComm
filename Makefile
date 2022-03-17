#Makefile at top of application tree
TOP = .
include $(TOP)/configure/CONFIG
# add directories, making sure they aren't in the list more than once
DIRS := $(DIRS) $(filter-out $(DIRS), configure)
DIRS := $(DIRS) $(filter-out $(DIRS), src)
DIRS := $(DIRS) $(filter-out $(DIRS), Db)
ifeq ($(BUILD_IOCS), YES)
# BUILDS_IOCS is defined in configure/CONFIG_APP
DIRS := $(DIRS) $(filter-out $(DIRS), $(wildcard iocs))
endif

include $(TOP)/configure/RULES_TOP
include $(TOP)/configure/RULES_TOP

ifeq ($(BUILD_IOCS), YES)
uninstall: uninstall_iocs
uninstall_iocs:
	$(MAKE) -C iocs uninstall
.PHONY: uninstall uninstall_iocs

realuninstall: realuninstall_iocs
realuninstall_iocs:
	$(MAKE) -C iocs realuninstall
.PHONY: realuninstall realuninstall_iocs
endif
