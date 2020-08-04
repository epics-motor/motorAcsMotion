# Makefile
TOP = ../..
include $(TOP)/configure/CONFIG

# The following are used for debugging messages.
USR_CXXFLAGS += -DDEBUG

DBD += AcsMotion.dbd

LIBRARY_IOC = AcsMotion

SRCS += SPiiPlusDriver.cpp

AcsMotion_LIBS += motor asyn
AcsMotion_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES

