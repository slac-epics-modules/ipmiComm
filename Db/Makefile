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

DB += fru_basic.db
DB += fru_extended.db
DB += fru_pm.db
DB += fru_cu.db

DB += fru_atca_amc.db
DB += fru_atca_rtm.db

DB += shelf_microtca_12slot.db

DB += shelf_atca_7slot.db

DB += server_supermicro.db

#----------------------------------------------------
# Create and install (or just install) into <top>/db
# databases, templates, substitutions like this
#
include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
#
USR_DBFLAGS = -I$(COMMON_DIR)

# End of file






