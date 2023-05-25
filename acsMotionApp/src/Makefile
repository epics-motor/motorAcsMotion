# Makefile
TOP = ../..
include $(TOP)/configure/CONFIG

# The following are used for debugging messages.
#USR_CXXFLAGS += -DDEBUG
#USR_CXXFLAGS += -std=c++11

DBD += AcsMotionSupport.dbd

LIBRARY_IOC_DEFAULT = AcsMotion
LIBRARY_IOC_vxWorks = -nil-

SRCS += SPiiPlusBinComm.cpp
SRCS += SPiiPlusCommDriver.cpp
SRCS += SPiiPlusDriver.cpp
SRCS += SPiiPlusAuxDriver.cpp

AcsMotion_LIBS += motor asyn
AcsMotion_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES

