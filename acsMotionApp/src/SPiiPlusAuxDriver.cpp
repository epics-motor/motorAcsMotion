
#include <stdlib.h>
#include <sstream>

#include <iocsh.h>
#include <epicsThread.h>

// asynMotorController.h includes asynPortDriver.h
#include <asynMotorController.h>
#include <asynMotorAxis.h>

#include <epicsExport.h>

#include "SPiiPlusAuxDriver.h"

static const char *driverName = "SPiiPlusAuxIO";


/* C Function which runs the profile thread */ 
static void SPiiPlusAuxIOThreadC(void *pPvt)
{
  SPiiPlusAuxIO *pAux = (SPiiPlusAuxIO*)pPvt;
  pAux->pollerThread();
}


SPiiPlusAuxIO::SPiiPlusAuxIO(const char *ACSAuxPortName, const char* asynPortName, int numChannels, double pollPeriod)
  : asynPortDriver(ACSAuxPortName, numChannels, 
      asynInt32Mask | asynFloat64Mask | asynUInt32DigitalMask | asynDrvUserMask,  // Interfaces that we implement
      asynInt32Mask | asynFloat64Mask | asynUInt32DigitalMask,                                    // Interfaces that do callbacks
      ASYN_MULTIDEVICE | ASYN_CANBLOCK, 1, /* ASYN_CANBLOCK=1, ASYN_MULTIDEVICE=1, autoConnect=1 */
      0, 0),  /* Default priority and stack size */
    pollPeriod_(pollPeriod),
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


// This is needed to resolve a build error: undefined reference to `vtable for SPiiPlusAuxIO'
SPiiPlusAuxIO::~SPiiPlusAuxIO()
{ 
  // Does the SPiiPlusComm destructor need to be explicitly called here?
}


asynStatus SPiiPlusAuxIO::writeBits(epicsUInt32 chan, epicsUInt32 mask, epicsUInt32 value)
{
  asynStatus status=asynSuccess;
  std::stringstream cmd;
  int i;
  epicsUInt32 outValue=0, outMask;
  static const char *functionName = "writeBits";
  
  if (mask == 0xffffffff)
  {
    // Use OUT(#) to set all 32-bits simultaneously
    cmd << "OUT(" << chan << ") = " << value;
    status = pComm_->writeReadAck(cmd);
  }
  else
  {
    // Use OUT(#).# to set the desired bits individually
    for (i=0, outMask=1; i<MAX_BITS; i++, outMask = (outMask<<1)) {
      //      
      outValue = ((value & outMask) == 0) ? 0 : 1; 
      if ((mask & outMask) != 0) {
        // Only write the value if the mask has this bit set
        cmd << "OUT(" << chan << ")." << i << " = " << outValue;
        status = pComm_->writeReadAck(cmd);
      }
    }
  }
  
  if (status == asynSuccess) {
    asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER, 
             "%s:%s, port %s, wrote outValue=0x%x, value=0x%x, mask=0x%x, chan=%d\n",
             driverName, functionName, this->portName, outValue, value, mask, chan);
  } else {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
             "%s:%s, port %s, ERROR writing outValue=0x%x, value=0x%x, mask=0x%x, chan=%d, status=%d\n",
             driverName, functionName, this->portName, outValue, value, mask, chan, status);
  }
  
  return status;
}


asynStatus SPiiPlusAuxIO::writeUInt32Digital(asynUser *pasynUser, epicsUInt32 value, epicsUInt32 mask)
{
  int function = pasynUser->reason;
  int status=asynSuccess;
  int chan;
  //static const char *functionName = "writeUInt32Digital";

  // Get the channel num
  status = getAddress(pasynUser, &chan);

  setUIntDigitalParam(chan, function, value, mask);
   
  if (function == digitalOutput_) {
      status |= writeBits(chan, mask, value);
  }
  
  callParamCallbacks();
  
  return (status==0) ? asynSuccess : asynError;
}


asynStatus SPiiPlusAuxIO::writeAnalog(epicsUInt32 chan, epicsFloat64 value)
{
  asynStatus status=asynSuccess;
  std::stringstream cmd;
  static const char *functionName = "writeAnalog";
  
  // Enforce the limits to avoid controller errors
  if (value > 100.0)
    value = 100.0;
  if (value < -100.0)
    value = -100.0;
  
  // The analog output value is in percent [-100, 100]
  cmd << "AOUT(" << chan << ") = " << value;
  status = pComm_->writeReadAck(cmd);
   
  if (status == asynSuccess) {
    asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER, 
             "%s:%s, port %s, wrote value=%lf, chan=%d\n",
             driverName, functionName, this->portName, value, chan);
  } else {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
             "%s:%s, port %s, ERROR writing value=%lf, chan=%d, status=%d\n",
             driverName, functionName, this->portName, value, chan, status);
  }
  
  return status;
}


asynStatus SPiiPlusAuxIO::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
  int function = pasynUser->reason;
  int status=asynSuccess;
  int chan;
  //static const char *functionName = "writeFloat64";

  this->getAddress(pasynUser, &chan);
  setDoubleParam(chan, function, value);

  if (function == analogOutput_) {
      status = writeAnalog(chan, value);
  }
  
  callParamCallbacks(chan);
  
  return (status==0) ? asynSuccess : asynError;
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
    
    // assume status is good
    status = asynSuccess;
    
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
      
      callParamCallbacks(i);
    }
    
    if (forceCallback_) {
      forceCallback_ = 0;
    }
    
    unlock();
    epicsThreadSleep(pollPeriod_);
  }
}


extern "C"
{

/** Configuration command, called directly or from iocsh */
int AcsMotionAuxIOConfig(const char *auxIOPortName, const char* asynPortName, int numChannels, double pollPeriod)
{
  SPiiPlusAuxIO *pSPiiPlusAuxIO = new SPiiPlusAuxIO(auxIOPortName, asynPortName, numChannels, pollPeriod);
  pSPiiPlusAuxIO = NULL;  /* This is just to avoid compiler warnings */
  return(asynSuccess);
}

static const iocshArg configArg0 = { "Aux IO port name", iocshArgString};
static const iocshArg configArg1 = { "Asyn port name",   iocshArgString};
static const iocshArg configArg2 = { "Max I/O channels", iocshArgInt};
static const iocshArg configArg3 = { "Poll period (s)",  iocshArgDouble};
static const iocshArg * const AcsMotionConfigArgs[] = {&configArg0,
                                              &configArg1,
                                              &configArg2,
                                              &configArg3};
static const iocshFuncDef configAcsMotionAuxIO = {"AcsMotionAuxIOConfig", 4, AcsMotionConfigArgs};
static void AcsMotionAuxIOCallFunc(const iocshArgBuf *args)
{
  AcsMotionAuxIOConfig(args[0].sval, args[1].sval, args[2].ival, args[3].dval);
}

void AcsMotionAuxIORegister(void)
{
  iocshRegister(&configAcsMotionAuxIO,AcsMotionAuxIOCallFunc);
}

epicsExportRegistrar(AcsMotionAuxIORegister);

}
