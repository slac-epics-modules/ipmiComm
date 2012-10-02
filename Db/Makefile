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

# Default PVs that may be overloaded during ioc boot
DB += module_basic_default.db
DB += module_mch_default.db
DB += module_pm_default.db
DB += module_cu_default.db
DB += module_amc_rtm_default.db

DB += module.db
#DB += module_cu.db
DB += sensor_temp.db
DB += sensor_voltage.db
DB += sensor_current.db
DB += sensor_fan.db
DB += sensor_hotswap.db
DB += shelf.db

#----------------------------------------------------
# Create and install (or just install) into <top>/db
# databases, templates, substitutions like this
#
include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
#
# End of file






