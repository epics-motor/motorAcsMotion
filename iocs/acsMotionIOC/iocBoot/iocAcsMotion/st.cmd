#!../../bin/linux-x86_64/acsMotion

< envPaths
< envVars

cd "${TOP}"

## Register all support components
dbLoadDatabase "dbd/acsMotion.dbd"
acsMotion_registerRecordDeviceDriver pdbbase

cd "${TOP}/iocBoot/${IOC}"

## motorUtil (allstop & alldone)
dbLoadRecords("$(MOTOR)/db/motorUtil.db", "P=$(PREFIX)")

## 
< AcsMotion.cmd
##
< AcsMotionAuxIO.cmd

iocInit

## motorUtil (allstop & alldone)
motorUtilInit("$(PREFIX)")

# Boot complete
