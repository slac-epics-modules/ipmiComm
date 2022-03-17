TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

#----------------------------------------------------
# Create and install (or just install) into <top>/db
# databases, templates, substitutions like this
#DB += 
DB_INSTALLS += $(IPMICOMM)/db/shelf_atca_7slot_lcls.db
DB_INSTALLS += $(IPMICOMM)/db/server_pc_lcls.db
DB_INSTALLS += $(ASYN)/db/asynRecord.db
# DB += for locally provided database files belonging to this app.
# DB_INSTALLS += for database files not owned by this app.

#----------------------------------------------------
# If <anyname>.db template is not named <anyname>*.template add
# <anyname>_template = <templatename>

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

