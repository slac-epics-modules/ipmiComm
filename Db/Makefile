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

DB += module_basic.db
DB += module_pm.db
DB += module_cu.db
DB += shelf.db

#----------------------------------------------------
# Create and install (or just install) into <top>/db
# databases, templates, substitutions like this
#
include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
#
DBFLAGS = $($*_DBFLAGS) $(USR_DBFLAGS) -I. -I.. -I$(COMMON_DIR) $(INSTALL_DBFLAGS) $(RELEASE_DBFLAGS)

# End of file






