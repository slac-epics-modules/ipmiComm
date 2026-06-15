#==============================================================
#
#  Abs:  
#
#  Name: Makefile
#
#  Facility:
#
#  Auth: 04-Jan-2012, Sonya Hoobler  (USERNAME)
#  Rev:  dd-mmm-yyyy, First Lastname  (USERNAME)
#--------------------------------------------------------------
#  Mod:
#
#==============================================================
#
TOP=..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

#----------------------------------------------------
#  Optimization of db files using dbst (DEFAULT: NO)
#DB_OPT = YES.

# General modules
DB += fru_common.db
DB += fru_cu_common.db
DB += sensor_ai.db
DB += system_chassis_status.db
DB += system_common.db

# Files that make up our high-level templates
DB += fru_basic.db
DB += fru_extended.db
DB += fru_pm.db
DB += fru_cu.db
DB += fru_atca_fb.db
DB += fru_atca_rtm.db

# Templates to be loaded per device 
DB += shelf_microtca_12slot.db
DB += shelf_atca_7slot.db
DB += server_pc.db

# Templates specific to LCLS
DB += system_common_lcls.db
DB += shelf_microtca_12slot_lcls.db
DB += server_pc_lcls.db
DB += fru_atca_fb_lcls.db
DB += fru_atca_rtm_lcls.db
DB += shelf_atca_7slot_lcls.db

#----------------------------------------------------
# Create and install (or just install) into <top>/db
# databases, templates, substitutions like this
#
include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
#

fru_cu.db$(DEP): $(COMMON_DIR)/fru_basic.db
shelf_microtca_12slot.db$(DEP): $(COMMON_DIR)/fru_basic.db $(COMMON_DIR)/fru_cu.db $(COMMON_DIR)/fru_pm.db
shelf_atca_7slot.db$(DEP): $(COMMON_DIR)/fru_basic.db $(COMMON_DIR)/fru_atca_fb.db $(COMMON_DIR)/fru_atca_rtm.db

shelf_microtca_12slot_lcls.db$(DEP): $(COMMON_DIR)/shelf_microtca_12slot.db
server_pc_lcls.db$(DEP): $(COMMON_DIR)/server_pc.db $(COMMON_DIR)/system_common_lcls.db
shelf_atca_7slot_lcls.db$(DEP): $(COMMON_DIR)/system_common_lcls.db $(COMMON_DIR)/fru_atca_fb_lcls.db $(COMMON_DIR)/fru_atca_rtm_lcls.db

USR_DBFLAGS = -I$(COMMON_DIR)

# End of file
