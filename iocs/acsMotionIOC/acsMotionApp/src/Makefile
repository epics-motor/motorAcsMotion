TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#=============================
# Build the IOC application

PROD_IOC = acsMotion
# acsMotion.dbd will be created and installed
DBD += acsMotion.dbd

# acsMotion.dbd will be made up from these files:
acsMotion_DBD += base.dbd

# Include dbd files from all support applications:
#acsMotion_DBD += xxx.dbd

# Add all the support libraries needed by this IOC
#acsMotion_LIBS += xxx

# acsMotion_registerRecordDeviceDriver.cpp derives from acsMotion.dbd
acsMotion_SRCS += acsMotion_registerRecordDeviceDriver.cpp

# Build the main IOC entry point on workstation OSs.
acsMotion_SRCS_DEFAULT += acsMotionMain.cpp
acsMotion_SRCS_vxWorks += -nil-

# Add support from base/src/vxWorks if needed
#acsMotion_OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary

# Finally link to the EPICS Base libraries
acsMotion_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

