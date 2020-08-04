#!../../bin/linux-x86_64/acsMotion

## You may have to change acsMotion to something else
## everywhere it appears in this file

#< envPaths

## Register all support components
dbLoadDatabase("../../dbd/acsMotion.dbd",0,0)
acsMotion_registerRecordDeviceDriver(pdbbase) 

## Load record instances
dbLoadRecords("../../db/acsMotion.db","user=kpetersn")

iocInit()

## Start any sequence programs
#seq sncacsMotion,"user=kpetersn"
