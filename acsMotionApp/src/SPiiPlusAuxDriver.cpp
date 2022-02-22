
#include <iocsh.h>
#include <epicsThread.h>
#include <epicsExport.h>

#include <asynPortDriver.h>

#include "SPiiPlusAuxDriver.h"

static const char *driverName = "SPiiPlusAuxIO";


/* C Function which runs the profile thread */ 
static void SPiiPlusAuxIOThreadC(void *pPvt)
{
  SPiiPlusAuxIO *pAux = (SPiiPlusAuxIO*)pPvt;
  pAux->pollerThread();
}


SPiiPlusAuxIO::SPiiPlusAuxIO(const char *ACSAuxPortName, const char* asynPortName, int numChannels, int pollPeriod)
  : asynPortDriver(ACSAuxPortName, numChannels, 
      asynInt32Mask | asynUInt32DigitalMask | asynDrvUserMask,  // Interfaces that we implement
      asynUInt32DigitalMask,                                    // Interfaces that do callbacks
      ASYN_MULTIDEVICE | ASYN_CANBLOCK, 1, /* ASYN_CANBLOCK=1, ASYN_MULTIDEVICE=1, autoConnect=1 */
      0, 0),  /* Default priority and stack size */
    pollPeriod_(pollPeriod / 1000.0),
    forceCallback_(1)
{
  const char* ACSCommPortSuffix = "Comm";
  char* ACSCommPortName;
  
  ACSCommPortName = (char *) malloc(strlen(ACSAuxPortName) + strlen(ACSCommPortSuffix));
  strcpy(ACSCommPortName, ACSAuxPortName);
  strcat(ACSCommPortName, ACSCommPortSuffix);
  
  pComm_ = new SPiiPlusComm(ACSCommPortName, asynPortName, numChannels);
  
  /*
   * MP4U controllers have significantly larger arrays for the AIN, AOUT, IN and OUT commands than the controller
   * has I/O channels.  Assume the user will specify a numChannels that is appropriate for their controller.
   */
  
  // Digital I/O parameters
  createParam(digitalInputString,      asynParamUInt32Digital, &digitalInput_);
  createParam(digitalOutputString,     asynParamUInt32Digital, &digitalOutput_);
  createParam(analogInputString,       asynParamFloat64,       &analogInput_);
  createParam(analogOutputString,      asynParamFloat64,       &analogOutput_);
  
  /* Start the thread to poll digital inputs and do callbacks to 
   * device support */
  epicsThreadCreate("SPiiPlusAuxIOPoller",
                    epicsThreadPriorityLow,
                    epicsThreadGetStackSize(epicsThreadStackMedium),
                    (EPICSTHREADFUNC)SPiiPlusAuxIOThreadC,
                    this);
}

void SPiiPlusAuxIO::pollerThread()
{
  /* This function runs in a separate thread.  It waits for the poll time */
  static const char *functionName = "pollerThread";
  epicsUInt32 changedInBits, changedOutBits;
  int i;
  int status;

  while(1) { 
    lock();
    
    // assume each IN() or OUT() channel always returns 32 bits
    // max IN/OUT/AIN/AOUT index before MP4U returns an error is 255 -> 256
    
    status = pComm_->getDoubleArray(buffer_, "AIN", 0, maxAddr, 0, 0);
    memcpy(ain_, buffer_, maxAddr * sizeof(double));
    
    status |= pComm_->getDoubleArray(buffer_, "AOUT", 0, maxAddr, 0, 0);
    memcpy(aout_, buffer_, maxAddr * sizeof(double));
    
    status |= pComm_->getIntegerArray(buffer_, "IN", 0, maxAddr, 0, 0);
    memcpy(in_, buffer_, maxAddr * sizeof(int));
    
    status |= pComm_->getIntegerArray(buffer_, "OUT", 0, maxAddr, 0, 0);
    memcpy(out_, buffer_, maxAddr * sizeof(int));
    
    if (status) 
      asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, 
                "%s:%s: ERROR reading I/O, status=%d\n", 
                driverName, functionName, status);
    
    for (i=0; i<maxAddr; i++) {
      setDoubleParam(i, analogInput_, ain_[i]);
      setDoubleParam(i, analogOutput_, aout_[i]);
      
      changedInBits = in_[i] ^ prev_in_[i];
      
      if (forceCallback_ || (changedInBits != 0)) {
        prev_in_[i] = in_[i];
        setUIntDigitalParam(i, digitalInput_, in_[i], 0xFFFFFFFF);
      }
    
      changedOutBits = out_[i] ^ prev_out_[i];
      
      if (forceCallback_ || (changedOutBits != 0)) {
        prev_out_[i] = out_[i];
        setUIntDigitalParam(i, digitalOutput_, out_[i], 0xFFFFFFFF);
      }
      
      if (forceCallback_) {
        forceCallback_ = 0;
      }
      
      for (i=0; i<maxAddr; i++) {
        callParamCallbacks(i);
      }
    }
    
    unlock();
    epicsThreadSleep(pollPeriod_);
  }
}


/** Configuration command, called directly or from iocsh */
extern "C" int SPiiPlusAuxIOConfig(const char *auxIOPortName, const char* asynPortName, int numChannels, int pollPeriod)
{
  SPiiPlusAuxIO *pSPiiPlusAuxIO = new SPiiPlusAuxIO(auxIOPortName, asynPortName, numChannels, pollPeriod);
  pSPiiPlusAuxIO = NULL;  /* This is just to avoid compiler warnings */
  return(asynSuccess);
}


static const iocshArg configArg0 = { "Aux IO port name", iocshArgString};
static const iocshArg configArg1 = { "Asyn port name",   iocshArgString};
static const iocshArg configArg2 = { "Num channels",     iocshArgInt};
static const iocshArg configArg3 = { "Poll period (ms)", iocshArgInt};
static const iocshArg * const configArgs[] = {&configArg0,
                                              &configArg1,
                                              &configArg2,
                                              &configArg3};
static const iocshFuncDef configFuncDef = {"SPiiPlusAuxIO", 4, configArgs};
static void configCallFunc(const iocshArgBuf *args)
{
  SPiiPlusAuxIOConfig(args[0].sval, args[1].sval, args[2].ival, args[3].ival);
}

void SPiiPlusAuxIORegister(void)
{
  iocshRegister(&configFuncDef,configCallFunc);
}

extern "C" {
epicsExportRegistrar(SPiiPlusAuxIORegister);
}
