TOP = ../..
include $(TOP)/configure/CONFIG

# This doesn't work yet
ifdef MOTOR_ACSMOTION
IOCSH_INSTALLS += $(MOTOR_ACSMOTION)/iocsh/ACS_Motion.iocsh
IOCSH_INSTALLS += $(MOTOR_ACSMOTION)/iocsh/ACS_Motion_tcp.iocsh
else
IOCSH_INSTALLS += $(MOTOR)/iocsh/ACS_Motion.iocsh
IOCSH_INSTALLS += $(MOTOR)/iocsh/ACS_Motion_tcp.iocsh
endif

include $(TOP)/configure/RULES

