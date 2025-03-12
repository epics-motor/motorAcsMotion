#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <algorithm>
#include <string>
#include <sstream>
#include <cstdarg>

#include <iocsh.h>
#include <epicsThread.h>
#include <epicsStdlib.h>
#include <epicsString.h>
#include <cantProceed.h>

#include <asynOctetSyncIO.h>
#include "asynDriver.h"
#include "asynMotorController.h"
#include "asynMotorAxis.h"

#include <epicsExport.h>

// SPiiPlusDriver.h includes SPiiPlusCommDriver.h
#include "SPiiPlusDriver.h"

static const char *driverName = "SPiiPlusController";

static void SPiiPlusProfileThreadC(void *pPvt);

#ifndef MAX
#define MAX(a,b) ((a)>(b)? (a): (b))
#endif
#ifndef MIN
#define MIN(a,b) ((a)<(b)? (a): (b))
#endif

SPiiPlusController::SPiiPlusController(const char* ACSPortName, const char* asynPortName, int numAxes,
                                             double movingPollPeriod, double idlePollPeriod,
                                             const char* virtualAxisList)
 : asynMotorController(ACSPortName, numAxes, NUM_SPIIPLUS_PARAMS, 0, 0, ASYN_CANBLOCK | ASYN_MULTIDEVICE, 1, 0, 0),
 drvUser_(NULL),
 initialized_(false)
{
	const char* ACSCommPortSuffix = "Comm";
	char* ACSCommPortName;
	// Don't connect to the asyn port associated with the controller's ip address, the comm class will do that
	//asynStatus status = pasynOctetSyncIO->connect(asynPortName, 0, &pasynUserController_, NULL);
	
	ACSCommPortName = (char *) malloc(strlen(ACSPortName) + strlen(ACSCommPortSuffix));
	strcpy(ACSCommPortName, ACSPortName);
	strcat(ACSCommPortName, ACSCommPortSuffix);
	// should numAxes be hard-coded to zero for the comm class?
	pComm_ = new SPiiPlusComm(ACSCommPortName, asynPortName, numAxes);
	
	pAxes_ = (SPiiPlusAxis **)(asynMotorController::pAxes_);
	std::stringstream cmd;
	static const char *functionName="SPiiPlusController";
	
	// Create parameters
	createParam(SPiiPlusHomingMethodString,               asynParamInt32, &SPiiPlusHomingMethod_);
	createParam(SPiiPlusMaxVelocityString,                asynParamFloat64, &SPiiPlusMaxVelocity_);
	createParam(SPiiPlusMaxAccelerationString,            asynParamFloat64, &SPiiPlusMaxAcceleration_);
	createParam(SPiiPlusReadIntVarString,                 asynParamInt32,   &SPiiPlusReadIntVar_);
	createParam(SPiiPlusWriteIntVarString,                asynParamInt32,   &SPiiPlusWriteIntVar_);
	createParam(SPiiPlusReadRealVarString,                asynParamFloat64, &SPiiPlusReadRealVar_);
	createParam(SPiiPlusWriteRealVarString,               asynParamFloat64, &SPiiPlusWriteRealVar_);
	createParam(SPiiPlusStartProgramString,               asynParamInt32,   &SPiiPlusStartProgram_);
	createParam(SPiiPlusStopProgramString,                asynParamInt32,   &SPiiPlusStopProgram_);
	createParam(SPiiPlusSafeTorqueOffString,              asynParamInt32,   &SPiiPlusSafeTorqueOff_);
	createParam(SPiiPlusHomingProcedureDoneString,        asynParamInt32,   &SPiiPlusHomingProcedureDone_);
	//
	createParam(SPiiPlusStepFactorString,                 asynParamFloat64,   &SPiiPlusStepFactor_);
	createParam(SPiiPlusEncTypeString,                    asynParamInt32,     &SPiiPlusEncType_);
	createParam(SPiiPlusEnc2TypeString,                   asynParamInt32,     &SPiiPlusEnc2Type_);
	createParam(SPiiPlusEncFactorString,                  asynParamFloat64,   &SPiiPlusEncFactor_);
	createParam(SPiiPlusEnc2FactorString,                 asynParamFloat64,   &SPiiPlusEnc2Factor_);
	//
	createParam(SPiiPlusAxisPosString,                    asynParamFloat64,   &SPiiPlusAxisPos_);
	createParam(SPiiPlusRefPosString,                     asynParamFloat64,   &SPiiPlusRefPos_);
	createParam(SPiiPlusEncPosString,                     asynParamFloat64,   &SPiiPlusEncPos_);
	createParam(SPiiPlusFdbkPosString,                    asynParamFloat64,   &SPiiPlusFdbkPos_);
	createParam(SPiiPlusFdbk2PosString,                   asynParamFloat64,   &SPiiPlusFdbk2Pos_);
	createParam(SPiiPlusVFdbkPosString,                   asynParamFloat64,   &SPiiPlusVFdbkPos_);
	//
	createParam(SPiiPlusRefOffsetString,                  asynParamFloat64,   &SPiiPlusRefOffset_);
	createParam(SPiiPlusEncOffsetString,                  asynParamFloat64,   &SPiiPlusEncOffset_);
	createParam(SPiiPlusEnc2OffsetString,                 asynParamFloat64,   &SPiiPlusEnc2Offset_);
	createParam(SPiiPlusAbsEncOffsetString,               asynParamFloat64,   &SPiiPlusAbsEncOffset_);
	createParam(SPiiPlusAbsEnc2OffsetString,              asynParamFloat64,   &SPiiPlusAbsEnc2Offset_);
	//
	createParam(SPiiPlusEncFaultString,                   asynParamInt32,     &SPiiPlusEncFault_);
	createParam(SPiiPlusEnc2FaultString,                  asynParamInt32,     &SPiiPlusEnc2Fault_);
	//
	createParam(SPiiPlusSetEncOffsetString,               asynParamFloat64,   &SPiiPlusSetEncOffset_);
	createParam(SPiiPlusSetEnc2OffsetString,              asynParamFloat64,   &SPiiPlusSetEnc2Offset_);
	//
	createParam(SPiiPlusFWVersionString,                  asynParamOctet,     &SPiiPlusFWVersion_);
	createParam(SPiiPlusVFdbkPosSupportString,            asynParamInt32,     &SPiiPlusVFdbkPosSupport_);
	//
	createParam(SPiiPlusHomingMaxDistString,              asynParamFloat64,   &SPiiPlusHomingMaxDist_);
	createParam(SPiiPlusHomingOffsetPosString,            asynParamFloat64,   &SPiiPlusHomingOffsetPos_);
	createParam(SPiiPlusHomingOffsetNegString,            asynParamFloat64,   &SPiiPlusHomingOffsetNeg_);
	createParam(SPiiPlusHomingCurrLimitString,            asynParamFloat64,   &SPiiPlusHomingCurrLimit_);
	//
	createParam(SPiiPlusDisableSetPosString,              asynParamInt32, &SPiiPlusDisableSetPos_);
	//
	createParam(SPiiPlusPulseModeString,                  asynParamInt32,     &SPiiPlusPulseMode_);
	createParam(SPiiPlusPulsePosString,                   asynParamFloat64Array, &SPiiPlusPulsePos_);
	createParam(SPiiPlusNumPulsesString,                  asynParamInt32,     &SPiiPlusNumPulses_);
	//
	createParam(SPiiPlusPulseAxisString,                  asynParamInt32,   &SPiiPlusPulseAxis_);
	createParam(SPiiPlusPEGEngEncCodeString,              asynParamOctet,   &SPiiPlusPEGEngEncCode_);
	createParam(SPiiPlusPEGOutAssignCodeString,           asynParamOctet,   &SPiiPlusPEGOutAssignCode_);
	createParam(SPiiPlusPOUTSOutputIndexString,           asynParamInt32,   &SPiiPlusPOUTSOutputIndex_);
	createParam(SPiiPlusPOUTSBitCodeString,               asynParamOctet,   &SPiiPlusPOUTSBitCode_);
	createParam(SPiiPlusPulseWidthString,                 asynParamFloat64, &SPiiPlusPulseWidth_);
	//
	createParam(SPiiPlusTestString,                       asynParamInt32, &SPiiPlusTest_);
	
	// Initialize variables to avoid freeing random memory
	fullProfileTimes_ = 0;
	profilePulses_ = NULL;
	profilePulsesUser_ = NULL;
	profilePulsePositions_ = NULL;
	maxProfilePoints_ = 0;
	
	// Query system info
	cmd << "?VR";
	pComm_->writeReadStr(cmd, firmwareVersion_);
	setStringParam(SPiiPlusFWVersion_, firmwareVersion_);
	
	// Determine whether virtual axis feedback positions are supported
	virtualFeedbackPositionSupported_ = false;
	pComm_->isVariableDefined(&virtualFeedbackPositionSupported_, "VPOS");
	setIntegerParam(SPiiPlusVFdbkPosSupport_, virtualFeedbackPositionSupported_);
	
	// Create the axes and set them as not virtual
	for (int index = 0; index < numAxes; index += 1)
	{
		new SPiiPlusAxis(this, index);
		pAxes_[index]->virtual_ = false;
	}
	
	// Parse the virtual axis list and set the listed axes as virtual
	char *tmpVirtualAxisList = epicsStrDup(virtualAxisList);
	char *strtok_rLast;
	anyAxisVirtual_ = false;
	for (char *p = epicsStrtok_r(tmpVirtualAxisList, ",", &strtok_rLast); p != NULL; p = epicsStrtok_r(NULL, ",", &strtok_rLast))
	{
		long index;
		int status = epicsParseLong(p, &index, 10, NULL);
		if (status != 0)
		{
			asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Invalid virtual axis list axis \"%s\"; ignoring\n", driverName, functionName, p);
			continue;
		}
		if (index < 0 || index >= numAxes)
		{
			asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Virtual axis list axis %ld out of range; ignoring\n", driverName, functionName, index);
			continue;
		}
		pAxes_[index]->virtual_ = true;
		anyAxisVirtual_ = true;
	}
	free(tmpVirtualAxisList);
	
	// Query setup parameters
	pComm_->getIntegerArray((char *)motorFlags_, "MFLAGS", 0, numAxes_-1, 0, 0);
	pComm_->getDoubleArray((char *)stepperFactor_, "STEPF", 0, numAxes_-1, 0, 0);
	pComm_->getDoubleArray((char *)encoderFactor_, "EFAC", 0, numAxes_-1, 0, 0);
	pComm_->getDoubleArray((char *)encoder2Factor_, "E2FAC", 0, numAxes_-1, 0, 0);
	pComm_->getIntegerArray((char *)encoderType_, "E_TYPE", 0, numAxes_-1, 0, 0);
	pComm_->getIntegerArray((char *)encoder2Type_, "E2_TYPE", 0, numAxes_-1, 0, 0);
	
	for (int index = 0; index < numAxes; index += 1)
	{
		// Parse the setup parameters
		// Bit 0 is #DUMMY (dummy axis)
		pAxes_[index]->dummy_ = motorFlags_[index] & SPIIPLUS_MFLAGS_DUMMY;
		// Bit 1 is #OPEN (open-loop control)
		pAxes_[index]->open_ = motorFlags_[index] & SPIIPLUS_MFLAGS_OPEN;
		// Bit 2 is #MICRO (microstepper mode)
		pAxes_[index]->micro_ = motorFlags_[index] & SPIIPLUS_MFLAGS_MICRO;
		// Bit 3 is #HOME (homing procedure done)
		pAxes_[index]->home_ = motorFlags_[index] & SPIIPLUS_MFLAGS_HOME;
		// Bit 4 is #STEPPER (pulse/direction stepper)
		pAxes_[index]->stepper_ = motorFlags_[index] & SPIIPLUS_MFLAGS_STEPPER;
		// Bit 5 is #ENCLOOP (stepper with steps as feedback)
		pAxes_[index]->encloop_ = motorFlags_[index] & SPIIPLUS_MFLAGS_ENCLOOP;
		// Bit 6 is #STEPENC (stepper with encoder as feedback)
		pAxes_[index]->stepenc_ = motorFlags_[index] & SPIIPLUS_MFLAGS_STEPENC;
		// Bit 8 is #BRUSHL (brushless motor)
		pAxes_[index]->brushl_ = motorFlags_[index] & SPIIPLUS_MFLAGS_BRUSHL;
		// Bit 9 is #BRUSHOK (brushless commutation OK)
		pAxes_[index]->brushok_ = motorFlags_[index] & SPIIPLUS_MFLAGS_BRUSHOK;
		// Bit 10 is #PHASE2 (2-phase motor)
		pAxes_[index]->phase2_ = motorFlags_[index] & SPIIPLUS_MFLAGS_PHASE2;
		// Bit 17 is #DEFCON (default connection)
		pAxes_[index]->defcon_ = motorFlags_[index] & SPIIPLUS_MFLAGS_DEFCON;
		// Bit 21 is #LINEAR (linear motor)
		pAxes_[index]->linear_ = motorFlags_[index] & SPIIPLUS_MFLAGS_LINEAR;
		// Bit 22 is #ABSCOMM (absolute encoder commutation)
		pAxes_[index]->abscomm_ = motorFlags_[index] & SPIIPLUS_MFLAGS_ABSCOMM;
		// Bit 27 is #HALL (hall commutation)
		pAxes_[index]->hall_ = motorFlags_[index] & SPIIPLUS_MFLAGS_HALL;
		
		// axis resolution (used to convert motor record steps into controller EGU)
		// TODO: how should nanomotion piezo ceramic motors (bit 7 of mflags) be handled?
		// TODO: does the following logic fail to identify non-PD steppers that aren't microstepping as stepper motors?
		if (!pAxes_[index]->virtual_ && ((pAxes_[index]->micro_ > 0) || ((pAxes_[index]->stepper_ > 0)  && (pAxes_[index]->stepenc_ == 0))))
		{
			// Use the stepper factor as the resolution for stepper motors
			pAxes_[index]->resolution_ = stepperFactor_[index];
		}
		else
		{
			// Use the encoder factor as the resolution for brushless and linear motors and virtual axes
			// TODO: how to handle mutliple encoders?
			pAxes_[index]->resolution_ = encoderFactor_[index];
		}
		
		// Update parameters that shouldn't change while the IOC is running
		setDoubleParam(index, SPiiPlusStepFactor_, stepperFactor_[index]);
		setIntegerParam(index, SPiiPlusEncType_, encoderType_[index]);
		setIntegerParam(index, SPiiPlusEnc2Type_, encoder2Type_[index]);
		setDoubleParam(index, SPiiPlusEncFactor_, encoderFactor_[index]);
		setDoubleParam(index, SPiiPlusEnc2Factor_, encoder2Factor_[index]);
		
		// Initialize absolute encoders
		if (encoderType_[index] > 4)
		{
			// Clear encoder error so absolute encoder position will be valid
			cmd << "FCLEAR " << index;
			pComm_->writeReadAck(cmd);
			
		}
		
		// Initialize this variable to avoid freeing random memory
		pAxes_[index]->fullProfilePositions_ = 0;
	}
	
	drvUser_ = (SPiiPlusDrvUser_t *) callocMustSucceed(1, sizeof(SPiiPlusDrvUser_t), functionName);
	drvUser_->programName = "undefined";
	drvUser_->len = -1;
	
	this->startPoller(movingPollPeriod, idlePollPeriod, 2);
	
	// Create the event that wakes up the thread for profile moves
	profileExecuteEvent_ = epicsEventMustCreate(epicsEventEmpty);
	
	// Create the thread that will execute profile moves
	epicsThreadCreate("SPiiPlusProfile", 
		epicsThreadPriorityLow,
		epicsThreadGetStackSize(epicsThreadStackMedium),
		(EPICSTHREADFUNC)SPiiPlusProfileThreadC, (void *)this);
	
	initialized_ = true;
}


asynStatus SPiiPlusController::drvUserCreate(asynUser *pasynUser,
                                       const char *drvInfo,
                                       const char **pptypeName, size_t *psize)
{
    static const char *functionName = "drvUserCreate";
    int index;
    const char *drvInfoNew;
    
    pasynUser->drvUser = drvUser_;
    if (initialized_ == false) {
       pasynManager->enable(pasynUser, 0);
       return asynDisabled;
    }
    
    // drvUserCreate(pasynUser=0x23e29e8, drvInfo=SPIIPLUS_READ_REAL_VAR, pptypeName=(nil), psize=(nil))
    //printf("drvUserCreate(pasynUser=%p, drvInfo=%s, pptypeName=%p, psize=%p)\n", pasynUser, drvInfo, pptypeName, psize);
    
    // findParam returns 0 if drvInfo matches a parameter string
    if (findParam(drvInfo, &index))
    {
        //printf("drvUserCreate(pasynUser=%p, drvInfo=%s, pptypeName=%p, psize=%p, drvUser=%p)\n", pasynUser, drvInfo, pptypeName, psize, (const char*) pasynUser->drvUser);
        
        if (strlen(drvInfo) > 15 && !epicsStrnCaseCmp(drvInfo, SPiiPlusStartProgramString, 15))
        {
            SPiiPlusDrvUser_t *drvUser = (SPiiPlusDrvUser_t *) callocMustSucceed(1, sizeof(SPiiPlusDrvUser_t), functionName);
            drvUser->programName = epicsStrDup(drvInfo+15);
            drvUser->len = strlen(drvUser->programName);
            pasynUser->drvUser = drvUser;
            drvInfo = SPiiPlusStartProgramString;
            //printf("drvUserCreate(pasynUser=%p, drvInfo=%s, pptypeName=%p, psize=%p, drvUser=%p)\n", pasynUser, drvInfo, pptypeName, psize, pasynUser->drvUser);
        }
        else if (strlen(drvInfo) > 14 && !epicsStrnCaseCmp(drvInfo, SPiiPlusStopProgramString, 14))
        {
            SPiiPlusDrvUser_t *drvUser = (SPiiPlusDrvUser_t *) callocMustSucceed(1, sizeof(SPiiPlusDrvUser_t), functionName);
            drvUser->programName = epicsStrDup(drvInfo+14);
            drvUser->len = strlen(drvUser->programName);
            pasynUser->drvUser = drvUser;
            drvInfo = SPiiPlusStopProgramString;
            //printf("drvUserCreate(pasynUser=%p, drvInfo=%s, pptypeName=%p, psize=%p, drvUser=%p)\n", pasynUser, drvInfo, pptypeName, psize, pasynUser->drvUser);
        }
    }
    
    return asynPortDriver::drvUserCreate(pasynUser, drvInfo, pptypeName, psize);
}


asynStatus SPiiPlusController::drvUserDestroy(asynUser *pasynUser)
{
    if (pasynUser->drvUser != drvUser_) {
        free(pasynUser->drvUser);
    }

    pasynUser->drvUser = NULL;

    return asynSuccess;
}


// It is necessary to implement getAddress to avoid the maxAddr checks from the base class
asynStatus SPiiPlusController::getAddress(asynUser *pasynUser, int *address)
{
    pasynManager->getAddr(pasynUser, address);
    
    if (*address > this->maxAddr-1)
    {
        // A larger address corresponds to a global variable tag
        *address = 0;
    }
    else
    {
        // asynMotorController doesn't implement get address, so asynPortDriver::getAddress is called
        asynMotorController::getAddress(pasynUser, address);
    } 
    
    return asynSuccess;
}

asynStatus SPiiPlusController::readInt32(asynUser *pasynUser, epicsInt32 *value)
{
  int function = pasynUser->reason;
  int status = asynSuccess;
  //static const char *functionName = "readInt32";

  *value = 0;
  
  if (function == SPiiPlusReadIntVar_)
  {
    status = readGlobalIntVar(pasynUser, value);
  }
  /*else if (function == SPiiPlusSafeTorqueOff_) 
  {
    // Is this necessary? Would the default method do the same thing?
    int axisNo_;
    // Get the addr, which is the axis number
    pasynManager->getAddr(pasynUser, &axisNo_);
    // The poller queries the STO value; return the latest value
    getIntegerParam(axisNo_, SPiiPlusSafeTorqueOff_, value);
  }*/
  else
  {
    /* Call base class method */
    status = asynMotorController::readInt32(pasynUser, value);
  }

  return (asynStatus)status;
}

asynStatus SPiiPlusController::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
  int function = pasynUser->reason;
  int status = asynSuccess;
  SPiiPlusAxis *pAxis;
  static const char *functionName = "writeInt32";
  
  pAxis = this->getAxis(pasynUser);
  if (!pAxis) return asynError;
  
  /* Set the parameter and readback in the parameter library.  This may be overwritten when we read back the
   * status at the end, but that's OK */
  status = pAxis->setIntegerParam(function, value);
  
  if (function == SPiiPlusHomingMethod_)
  {
    //
    asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s: homingMethod = %i\n", driverName, functionName, value);
  }
  
  if (function == SPiiPlusTest_)
  {
    /* Do something helpful during development */
    status = test();
  }
  else if (function == SPiiPlusWriteIntVar_)
  {
    status = writeGlobalIntVar(pasynUser, value);
  }
  else if (function == SPiiPlusStartProgram_)
  {
    status = startProgram(pasynUser, value);
  }
  else if (function == SPiiPlusStopProgram_)
  {
    status = stopProgram(pasynUser, value);
  }
  else
  {
    /* Call base class method */
    status = asynMotorController::writeInt32(pasynUser, value);
  }

  /* Do callbacks so higher layers see any changes */
  pAxis->callParamCallbacks();

  return (asynStatus)status;
}

asynStatus SPiiPlusController::readFloat64(asynUser *pasynUser, epicsFloat64 *value)
{
  int function = pasynUser->reason;
  int status = asynSuccess;
  //static const char *functionName = "readFloat64";

  *value = 0;
  
  if (function == SPiiPlusReadRealVar_)
  {
    status = readGlobalRealVar(pasynUser, value);
  }
  else
  {
    /* Call base class method */
    status = asynMotorController::readFloat64(pasynUser, value);
  }

  return (asynStatus)status;
}

/** Called when asyn clients call pasynFloat64->write().
  * \param[in] pasynUser asynUser structure that encodes the reason and address.
  * \param[in] value Value to write. */
asynStatus SPiiPlusController::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
  int function = pasynUser->reason;
  SPiiPlusAxis *pAxis;
  asynStatus status = asynError;
  //static const char *functionName = "writeFloat64";

  pAxis = getAxis(pasynUser);
  if (!pAxis) return asynError;

  /* Set the parameter and readback in the parameter library. */
  status = pAxis->setDoubleParam(function, value);

  if (function == SPiiPlusMaxVelocity_)
  {
    status = pAxis->setMaxVelocity(value);
  }
  else if (function == SPiiPlusMaxAcceleration_)
  {
    status = pAxis->setMaxAcceleration(value);
  } 
  else if (function == SPiiPlusWriteRealVar_) 
  {
    status = writeGlobalRealVar(pasynUser, value);
  }
  else if (function == SPiiPlusSetEncOffset_)
  {
    status = pAxis->setEncoderOffset(value);
  }
  else if (function == SPiiPlusSetEnc2Offset_)
  {
    status = pAxis->setEncoder2Offset(value);
  }
  else
  {
    /* Call base class method */
    status = asynMotorController::writeFloat64(pasynUser, value);
  }
  
  /* Do callbacks so higher layers see any changes */
  pAxis->callParamCallbacks();
  
  return status;
}

/** Called when asyn clients call pasynFloat64Array->write().
  * \param[in] pasynUser pasynUser structure that encodes the reason and address.
  * \param[in] value Pointer to the array to write.
  * \param[in] nElements Number of elements to write. */
asynStatus SPiiPlusController::writeFloat64Array(asynUser *pasynUser, epicsFloat64 *value,
                                                  size_t nElements)
{
    int function = pasynUser->reason;
    asynMotorAxis *pAxis;
    asynStatus status = asynSuccess;
    static const char *functionName = "writeFloat64Array";
    
    pAxis = getAxis(pasynUser);
    if (!pAxis) return asynError;
    
    if (function == SPiiPlusPulsePos_) {
        // Just copy the positions for now; calculations on the positions will occur when the profile is built
        memcpy(profilePulsesUser_, value, nElements*sizeof(double));
        // Do we want to store nElements?  The user might change (shorten) the number of pulses after writing the array.
        numPulses_ = nElements;
    } 
    else {
        status = asynMotorController::writeFloat64Array(pasynUser, value, nElements);
    }
    return status;
}

SPiiPlusAxis* SPiiPlusController::getAxis(asynUser *pasynUser)
{
	return static_cast<SPiiPlusAxis*>(asynMotorController::getAxis(pasynUser));
}

SPiiPlusAxis* SPiiPlusController::getAxis(int axisNo)
{
	return static_cast<SPiiPlusAxis*>(asynMotorController::getAxis(axisNo));
}


asynStatus SPiiPlusController::poll()
{
	asynStatus status;
	static const char *functionName = "poll";
	
	/*
	 * Read position and status using binary queries here and parse the replies in the axis poll method
	 * 
	 * FPOS = FP*EFAC + EOFFS
	 * F2POS = FP2*E2FAC + E2OFFS
	 * 
	 */
	
	asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s: POLL_START\n", driverName, functionName);
	
	/* positions */
	status = pComm_->getDoubleArray((char *)axisPosition_, "APOS", 0, numAxes_-1, 0, 0);
	if (status != asynSuccess) return status;
	
	// RPOS = APOS if MFLAGS(index).#DEFCON=1
	status = pComm_->getDoubleArray((char *)referencePosition_, "RPOS", 0, numAxes_-1, 0, 0);
	if (status != asynSuccess) return status;
	
	status = pComm_->getDoubleArray((char *)encoderPosition_, "EPOS", 0, numAxes_-1, 0, 0);
	if (status != asynSuccess) return status;
	
	status = pComm_->getDoubleArray((char *)feedbackPosition_, "FPOS", 0, numAxes_-1, 0, 0);
	if (status != asynSuccess) return status;
	
	status = pComm_->getDoubleArray((char *)feedback2Position_, "F2POS", 0, numAxes_-1, 0, 0);
	if (status != asynSuccess) return status;
	
	if (virtualFeedbackPositionSupported_ && anyAxisVirtual_)
	{
		status = pComm_->getDoubleArray((char *)virtualFeedbackPosition_, "VPOS", 0, numAxes_-1, 0, 0);
		if (status != asynSuccess) return status;
	}
	
	/* offsets */
	// RPOS = 0 if MFLAGS(index).#DEFCON=1
	status = pComm_->getDoubleArray((char *)referenceOffset_, "ROFFS", 0, numAxes_-1, 0, 0);
	if (status != asynSuccess) return status;
	
	status = pComm_->getDoubleArray((char *)encoderOffset_, "EOFFS", 0, numAxes_-1, 0, 0);
	if (status != asynSuccess) return status;
	
	status = pComm_->getDoubleArray((char *)encoder2Offset_, "E2OFFS", 0, numAxes_-1, 0, 0);
	if (status != asynSuccess) return status;
	
	status = pComm_->getDoubleArray((char *)absoluteEncoderOffset_, "E_AOFFS", 0, numAxes_-1, 0, 0);
	if (status != asynSuccess) return status;
	
	// TODO: re-add E2_AOFFS query after querying the firmware version
	// E2_AOFFS doesn't exist in firmware v2.70
	//status = pComm_->getDoubleArray((char *)absoluteEncoder2Offset_, "E2_AOFFS", 0, numAxes_-1, 0, 0);
	//if (status != asynSuccess) return status;
	
	/* statuses */
	status = pComm_->getIntegerArray((char *)axisStatus_, "AST", 0, numAxes_-1, 0, 0);
	if (status != asynSuccess) return status;
	
	status = pComm_->getIntegerArray((char *)motorStatus_, "MST", 0, numAxes_-1, 0, 0);
	if (status != asynSuccess) return status;
	
	status = pComm_->getIntegerArray((char *)faultStatus_, "FAULT", 0, numAxes_-1, 0, 0);
	if (status != asynSuccess) return status;
	
	// MFLAGS need to be polled here for the homed status
	status = pComm_->getIntegerArray((char *)motorFlags_, "MFLAGS", 0, numAxes_-1, 0, 0);
	if (status != asynSuccess) return status;
	
	// TODO: only get max values when idle polling
	/* max values */
	status = pComm_->getDoubleArray((char *)maxVelocity_, "XVEL", 0, numAxes_-1, 0, 0);
	if (status != asynSuccess) return status;
	status = pComm_->getDoubleArray((char *)maxAcceleration_, "XACC", 0, numAxes_-1, 0, 0);
	if (status != asynSuccess) return status;
	
	asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s: POLL_END\n", driverName, functionName);
	
	return status;
}

asynStatus SPiiPlusController::readGlobalIntVar(asynUser *pasynUser, epicsInt32 *value)
{
	asynStatus status;
	std::stringstream cmd;
	int tag;
	
	// Get the address, which is the tag of the global integer, rather than an axis index
	pasynManager->getAddr(pasynUser, &tag);
	
	// ?GETVAR(tag)
	cmd << "?GETVAR(" << tag << ")";
	status = pComm_->writeReadInt(cmd, value);
	
	return status;
}

asynStatus SPiiPlusController::writeGlobalIntVar(asynUser *pasynUser, epicsInt32 value)
{
	asynStatus status;
	std::stringstream cmd;
	int tag;
	
	// Get the address, which is the tag of the global integer, rather than an axis index
	pasynManager->getAddr(pasynUser, &tag);
	
	// SETVAR(value, tag)
	cmd << "SETVAR(" << value << "," << tag << ")";
	status = pComm_->writeReadAck(cmd);
	
	return status;
}

asynStatus SPiiPlusController::readGlobalRealVar(asynUser *pasynUser, epicsFloat64 *value)
{
	asynStatus status;
	std::stringstream cmd;
	int tag;
	
	// Get the address, which is the tag of the global integer, rather than an axis index
	pasynManager->getAddr(pasynUser, &tag);
	
	// ?GETVAR(tag)
	cmd << "?GETVAR(" << tag << ")";
	status = pComm_->writeReadDouble(cmd, value);
	
	return status;
}

asynStatus SPiiPlusController::writeGlobalRealVar(asynUser *pasynUser, epicsFloat64 value)
{
	asynStatus status;
	std::stringstream cmd;
	int tag;
	
	// Get the address, which is the tag of the global integer, rather than an axis index
	pasynManager->getAddr(pasynUser, &tag);
	
	// SETVAR(value, tag)
	cmd << "SETVAR(" << value << "," << tag << ")";
	status = pComm_->writeReadAck(cmd);
	
	return status;
}

asynStatus SPiiPlusController::startProgram(asynUser *pasynUser, epicsFloat64 value)
{
	asynStatus status;
	std::stringstream cmd;
	int buffer;
	static const char *functionName = "startProgram";
	
	// Get the address, which is the buffer # containing the program, rather than an axis index
	pasynManager->getAddr(pasynUser, &buffer);

	asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s: drvUser->programName = %s, drvUser->len = %i\n", driverName, functionName, ((SPiiPlusDrvUser_t *)pasynUser->drvUser)->programName, ((SPiiPlusDrvUser_t *)pasynUser->drvUser)->len);
	
	// START buffer,label -or- START buffer,line_no
	cmd << "START " << buffer << "," << ((SPiiPlusDrvUser_t *)pasynUser->drvUser)->programName;
	status = pComm_->writeReadAck(cmd);
	
	return status;
}

asynStatus SPiiPlusController::stopProgram(asynUser *pasynUser, epicsFloat64 value)
{
	asynStatus status;
	std::stringstream cmd;
	int buffer;
	static const char *functionName = "stopProgram";
	
	// Get the address, which is the buffer # containing the program, rather than an axis index
	pasynManager->getAddr(pasynUser, &buffer);
	
	asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s: drvUser->programName = %s, drvUser->len = %i\n", driverName, functionName, ((SPiiPlusDrvUser_t *)pasynUser->drvUser)->programName, ((SPiiPlusDrvUser_t *)pasynUser->drvUser)->len);
	// STOP buffer
	cmd << "STOP " << buffer;
	status = pComm_->writeReadAck(cmd);
	
	return status;
}


SPiiPlusAxis::SPiiPlusAxis(SPiiPlusController *pC, int axisNo)
: asynMotorAxis(pC, axisNo),
  pC_(pC)
{
	// Initialize variables to avoid freeing random memory
	profilePositionsUser_ = NULL;
	
	setIntegerParam(pC->motorStatusHasEncoder_, 1);
	// Gain Support is required for setClosedLoop to be called
	setIntegerParam(pC->motorStatusGainSupport_, 1);
}

asynStatus SPiiPlusAxis::poll(bool* moving)
{
	asynStatus status = asynSuccess;
	SPiiPlusController* controller = (SPiiPlusController*) pC_;
	//static const char *functionName = "poll";
	std::stringstream cmd;
	
	// APOS (queried in controller poll method)
	setDoubleParam(controller->motorPosition_, (controller->axisPosition_[axisNo_] / resolution_));
	
	if (dummy_)
	{
		// APOS (queried in controller poll method)
		setDoubleParam(controller->motorEncoderPosition_, (controller->axisPosition_[axisNo_] / resolution_));
		// Faults are disabled for dummy axes
		setIntegerParam(controller->motorStatusLowLimit_, 0);
		setIntegerParam(controller->motorStatusHighLimit_, 0);
	}
	else if (virtual_)
	{
		if (controller->virtualFeedbackPositionSupported_)
		{
			// VPOS (queried in controller poll method)
			setDoubleParam(controller->motorEncoderPosition_, controller->virtualFeedbackPosition_[axisNo_] / controller->encoderFactor_[axisNo_]);
		}
		else
		{
			// APOS (queried in controller poll method)
			setDoubleParam(controller->motorEncoderPosition_, (controller->axisPosition_[axisNo_] / resolution_));
		}
		// Faults are disabled for virtual axes
		setIntegerParam(controller->motorStatusLowLimit_, 0);
		setIntegerParam(controller->motorStatusHighLimit_, 0);
	}
	else
	{
		// FPOS (queried in controller poll method) = FP * EFAC + EOFFS
		// TODO: detect when there is no encoder and fix the encoder position at zero
		setDoubleParam(controller->motorEncoderPosition_, controller->feedbackPosition_[axisNo_] / controller->encoderFactor_[axisNo_]);
		
		// FAULT (queried in controller poll method)
		int left_limit, right_limit, sto, home;
		left_limit = controller->faultStatus_[axisNo_] & SPIIPLUS_FAULT_HARD_LEFT_LIMIT;
		setIntegerParam(controller->motorStatusLowLimit_, left_limit);
		
		right_limit = controller->faultStatus_[axisNo_] & SPIIPLUS_FAULT_HARD_RIGHT_LIMIT;
		setIntegerParam(controller->motorStatusHighLimit_, right_limit);
		
		sto = controller->faultStatus_[axisNo_] & SPIIPLUS_FAULT_SAFE_TORQUE_OFF;
		setIntegerParam(controller->SPiiPlusSafeTorqueOff_, sto);

		// MFLAGS HOME (queried in controller poll method)
		home = controller->motorFlags_[axisNo_] & SPIIPLUS_MFLAGS_HOME;
		setIntegerParam(controller->SPiiPlusHomingProcedureDone_, home);
	}
	
	getMaxParams();
	
	updateFeedbackParams();
	
	// AST (queried in controller poll method)
	
	pegReady_ = controller->axisStatus_[axisNo_] & SPIIPLUS_AXIS_STATUS_PEGREADY;
	
	int enabled;
	int motion;
	//int open_loop;
	//int in_pos;
	
	if (dummy_ || virtual_)
	{
		enabled = 0;
		motion = controller->axisStatus_[axisNo_] & (1<<5);
	}
	else
	{
		// MST (queried in controller poll method)
		
		enabled = controller->motorStatus_[axisNo_] & (1<<0);
		motion = controller->motorStatus_[axisNo_] & (1<<5);
		//open_loop = controller->motorStatus_[axisNo_] & (1<<1);
		//in_pos = controller->motorStatus_[axisNo_] & (1<<4);
	}
	
	setIntegerParam(controller->motorStatusDone_, !motion);
	setIntegerParam(controller->motorStatusMoving_, motion);
	setIntegerParam(controller->motorStatusPowerOn_, enabled);
	
	callParamCallbacks();
	
	moving_ = *moving;
	
	if (motion)    { *moving = true; }
	else           { *moving = false; }
	
	return status;
}

asynStatus SPiiPlusAxis::getMaxParams()
{
	SPiiPlusController* controller = (SPiiPlusController*) pC_;
	asynStatus status = asynSuccess;
	double motorRecResolution;
	double maxVelocity, maxAcceleration;
	std::stringstream cmd;
	
	// motorRecResolution (EGU/step)
	controller->getDoubleParam(axisNo_, controller->motorRecResolution_,   &motorRecResolution);
	
	// XVEL [SPiiPlus-units/sec] (queried in controller poll method)
	maxVelocity = controller->maxVelocity_[axisNo_] / resolution_ * motorRecResolution;
	
	// XACC [SPiiPlus-units/sec^2] (queried in controller poll method)
	maxAcceleration = controller->maxAcceleration_[axisNo_] / resolution_ * motorRecResolution;
	
	// (SPiiPlus-units/time-unit) / (SPiiPlus-units/step) * (EGU/step) = (EGU/time-unit)
	controller->setDoubleParam(axisNo_, controller->SPiiPlusMaxVelocity_, maxVelocity);
	controller->setDoubleParam(axisNo_, controller->SPiiPlusMaxAcceleration_, maxAcceleration);
	
	// Assume the calling method will call callParamCallbacks()
	
	return status;
}


asynStatus SPiiPlusAxis::updateFeedbackParams()
{
	SPiiPlusController* controller = (SPiiPlusController*) pC_;
	asynStatus status = asynSuccess;
	std::stringstream cmd;
	
	// Update the axis parameters with the values that were queried in the controller poll method
	controller->setDoubleParam(axisNo_, controller->SPiiPlusAxisPos_, controller->axisPosition_[axisNo_]);
	controller->setDoubleParam(axisNo_, controller->SPiiPlusRefPos_, controller->referencePosition_[axisNo_]);
	controller->setDoubleParam(axisNo_, controller->SPiiPlusEncPos_, controller->encoderPosition_[axisNo_]);
	controller->setDoubleParam(axisNo_, controller->SPiiPlusFdbkPos_, controller->feedbackPosition_[axisNo_]);
	controller->setDoubleParam(axisNo_, controller->SPiiPlusFdbk2Pos_, controller->feedback2Position_[axisNo_]);
	controller->setDoubleParam(axisNo_, controller->SPiiPlusVFdbkPos_, controller->virtualFeedbackPosition_[axisNo_]);
	//
	controller->setDoubleParam(axisNo_, controller->SPiiPlusRefOffset_, controller->referenceOffset_[axisNo_]);
	controller->setDoubleParam(axisNo_, controller->SPiiPlusEncOffset_, controller->encoderOffset_[axisNo_]);
	controller->setDoubleParam(axisNo_, controller->SPiiPlusEnc2Offset_, controller->encoder2Offset_[axisNo_]);
	controller->setDoubleParam(axisNo_, controller->SPiiPlusAbsEncOffset_, controller->absoluteEncoderOffset_[axisNo_]);
	controller->setDoubleParam(axisNo_, controller->SPiiPlusAbsEnc2Offset_, controller->absoluteEncoder2Offset_[axisNo_]);
	//
	controller->encoderFault_[axisNo_] = controller->faultStatus_[axisNo_] & SPIIPLUS_FAULT_ENCODER_NC;
	controller->setIntegerParam(axisNo_, controller->SPiiPlusEncFault_, controller->encoderFault_[axisNo_]);
	controller->encoder2Fault_[axisNo_] = controller->faultStatus_[axisNo_] & SPIIPLUS_FAULT_ENCODER_2_NC;
	controller->setIntegerParam(axisNo_, controller->SPiiPlusEnc2Fault_, controller->encoder2Fault_[axisNo_]);
	// Assume the calling method will call callParamCallbacks()
	
	return status;
}


asynStatus SPiiPlusAxis::setMaxVelocity(double maxVelocity)
{
	SPiiPlusController* controller = (SPiiPlusController*) pC_;
	asynStatus status;
	double motorRecResolution;
	std::stringstream cmd;
	
	// motorRecResolution is in EGU / step
	controller->getDoubleParam(axisNo_, controller->motorRecResolution_,   &motorRecResolution);
	
	// (EGU/s) / (EGU/step) * (SPiiPlus-units/step) = (SPiiPlus-units/s)
	cmd << "XVEL(" << axisNo_ << ")=" << (maxVelocity / motorRecResolution * resolution_);
	status = controller->pComm_->writeReadAck(cmd);
	
	return status;
}

asynStatus SPiiPlusAxis::setMaxAcceleration(double maxAcceleration)
{
	SPiiPlusController* controller = (SPiiPlusController*) pC_;
	asynStatus status;
	double motorRecResolution;
	std::stringstream cmd;
	
	// motorRecResolution is in EGU / step
	controller->getDoubleParam(axisNo_, controller->motorRecResolution_,   &motorRecResolution);
	
	// (EGU/s^2) / (EGU/step) * (SPiiPlus-units/step) = (SPiiPlus-units/s^2)
	cmd << "XACC(" << axisNo_ << ")=" << (maxAcceleration / motorRecResolution * resolution_);
	status = controller->pComm_->writeReadAck(cmd);
	
	return status;
}


asynStatus SPiiPlusAxis::move(double position, int relative, double minVelocity, double maxVelocity, double acceleration)
{
	SPiiPlusController* controller = (SPiiPlusController*) pC_;
	asynStatus status;
	double deviceUnits;
	std::stringstream cmd;
	
	//cmd << "XACC(" << axisNo_ << ")=" << ((acceleration + 10) * resolution_);
	//status = writeReadAck(controller, cmd);
	cmd << "ACC(" << axisNo_ << ")=" << (acceleration * resolution_);
	status = controller->pComm_->writeReadAck(cmd);
	cmd << "DEC(" << axisNo_ << ")=" << (acceleration * resolution_);
	status = controller->pComm_->writeReadAck(cmd);
	
	//cmd << "XVEL(" << axisNo_ << ")=" << ((maxVelocity + 10) * resolution_);
	//status = writeReadAck(controller, cmd);
	cmd << "VEL(" << axisNo_ << ")=" << (maxVelocity * resolution_);
	status = controller->pComm_->writeReadAck(cmd);
	
	if (relative)
	{
		cmd << "PTP/r " << axisNo_ << ", " << (position * resolution_);
		status = controller->pComm_->writeReadAck(cmd);
	}
	else
	{
		cmd << "PTP " << axisNo_ << ", " << (position * resolution_);
		status = controller->pComm_->writeReadAck(cmd);
	}
	
	return status;
}

asynStatus SPiiPlusAxis::moveVelocity(double minVelocity, double maxVelocity, double acceleration)
{
  //Push hold jogging
  SPiiPlusController* controller = (SPiiPlusController*) pC_;
	asynStatus status;
	double deviceUnits;
	std::stringstream cmd;

	cmd << "ACC(" << axisNo_ << ")=" << (acceleration * resolution_);
	status = controller->pComm_->writeReadAck(cmd);

	cmd << "DEC(" << axisNo_ << ")=" << (acceleration * resolution_);
	status = controller->pComm_->writeReadAck(cmd);

	char motionDirection = maxVelocity > 0 ? '+' : '-';

	// Pass the jog velocity to the jog command, rather than change the normal velocity
	cmd << "JOG/v " << axisNo_ << ", " << (abs(maxVelocity) * resolution_) << ", " << motionDirection;
	status = controller->pComm_->writeReadAck(cmd);

	return status;
}

asynStatus SPiiPlusAxis::setPosition(double position)
{
	SPiiPlusController* controller = (SPiiPlusController*) pC_;
	asynStatus status;
	int disableSetPos;
	std::stringstream cmd;
	static const char *functionName = "setPosition";
	
	if (virtual_)
	{
		asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: axis %i position change not supported because axis is virtual\n", driverName, functionName, axisNo_);
		return asynError;
	}
	
	controller->getIntegerParam(axisNo_, controller->SPiiPlusDisableSetPos_, &disableSetPos);
	
	if (!disableSetPos)
	{
	  // The controller automatically updates APOS and FPOS when RPOS is updated 
	  cmd << "SET RPOS(" << axisNo_ << ")=" << (position * resolution_);
	  status = controller->pComm_->writeReadAck(cmd);
	}
	else
	{
	  asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Changing the position of axis %i is disabled; Ignoring requested position: %lf\n", driverName, functionName, axisNo_, (position * resolution_));
	  status = asynError;
	}
	
	return status;
}

asynStatus SPiiPlusAxis::setEncoderOffset(double newEncoderOffset)
{
	SPiiPlusController* controller = (SPiiPlusController*) pC_;
	asynStatus status;
	std::stringstream cmd;
	double oldEncoderOffset;
	const char* sign;
	static const char *functionName = "setEncoderOffset";
	
	if (virtual_)
	{
		asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: axis %i encoder offset change not supported because axis is virtual\n", driverName, functionName, axisNo_);
		return asynError;
	}
	
	// We can't use getDoubleParam to get the old encoder 2 offset because it has already been updated
	oldEncoderOffset = pC_->encoderOffset_[axisNo_];
	
	// If the value is positive or zero, add a '+'.  If the value is negative, no sign needs to be added
	sign = (newEncoderOffset < 0.0) ? " " : "+";
	
	// The encoder offset is in SPiiPlus units
	// SET FPOS(n) = FPOS(n) - EOFFS(n) + newEncoderOffset
	cmd << "SET FPOS(" << axisNo_ << ")=FPOS(" << axisNo_ << ")-EOFFS(" << axisNo_ << ")" << sign << newEncoderOffset;
	status = controller->pComm_->writeReadAck(cmd);
	
	if (status != asynSuccess)
	{
		asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: axis %i encoder offset change failed\n", driverName, functionName, axisNo_);
	} else {
		// Always print this line so there is a record in an IOC's log for offset changes
		asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: axis %i encoder offset changed from %lf to %lf\n", driverName, functionName, axisNo_, oldEncoderOffset, newEncoderOffset);
	}
	
	return status;
}

asynStatus SPiiPlusAxis::setEncoder2Offset(double newEncoder2Offset)
{
	SPiiPlusController* controller = (SPiiPlusController*) pC_;
	asynStatus status;
	std::stringstream cmd;
	double oldEncoder2Offset;
	const char* sign;
	static const char *functionName = "setEncoder2Offset";
	
	if (virtual_)
	{
		asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: axis %i encoder 2 offset change not supported because axis is virtual\n", driverName, functionName, axisNo_);
		return asynError;
	}
	
	// We can't use getDoubleParam to get the old encoder 2 offset because it has already been updated
	oldEncoder2Offset = pC_->encoder2Offset_[axisNo_];
	
	// If the value is positive or zero, add a '+'.  If the value is negative, no sign needs to be added
	sign = (newEncoder2Offset < 0.0) ? " " : "+";
	
	// The encoder 2 offset is in SPiiPlus units
	// SET F2POS(n) = F2POS(n) - E2OFFS(n) + newEncoder2Offset
	cmd << "SET F2POS(" << axisNo_ << ")=F2POS(" << axisNo_ << ")-E2OFFS(" << axisNo_ << ")" << sign << newEncoder2Offset;
	status = controller->pComm_->writeReadAck(cmd);
	
	if (status != asynSuccess)
	{
		asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: axis %i encoder 2 offset change failed\n", driverName, functionName, axisNo_);
	} else {
		// Always print this line so there is a record in an IOC's log for offset changes
		asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: axis %i encoder 2 offset changed from %lf to %lf\n", driverName, functionName, axisNo_, oldEncoder2Offset, newEncoder2Offset);
	}
	
	return status;
}

asynStatus SPiiPlusAxis::stop(double acceleration)
{
	SPiiPlusController* controller = (SPiiPlusController*) pC_;
	asynStatus status;
	std::stringstream cmd;
	
	cmd << "HALT " << axisNo_;
	status = controller->pComm_->writeReadAck(cmd);
	
	return status;
}

/** Set the motor closed loop status. 
  * \param[in] closedLoop true = close loop, false = open looop. */
asynStatus SPiiPlusAxis::setClosedLoop(bool closedLoop)
{
	SPiiPlusController* controller = (SPiiPlusController*) pC_;
	asynStatus status=asynSuccess;
	std::stringstream cmd;
	
	if (!dummy_ && !virtual_)
	{
		/*
		Enable/disable the axis instead of changing the closed-loop state.
		*/
		if (closedLoop)
		{
			cmd << "ENABLE " << axisNo_;
		}
		else
		{
			cmd << "DISABLE " << axisNo_;
		}
		status = controller->pComm_->writeReadAck(cmd);
	}
	
	return status;
}

/** Move the motor to the home position.
  * \param[in] minVelocity The initial velocity, often called the base velocity. Units=steps/sec.
  * \param[in] maxVelocity The maximum velocity, often called the slew velocity. Units=steps/sec.
  * \param[in] acceleration The acceleration value. Units=steps/sec/sec.
  * \param[in] forwards  Flag indicating to move the motor in the forward direction(1) or reverse direction(0).
  *                      Some controllers need to be told the direction, others know which way to go to home. */
asynStatus SPiiPlusAxis::home(double minVelocity, double maxVelocity, double acceleration, int forwards)
{
	SPiiPlusController* controller = (SPiiPlusController*) pC_;
	asynStatus status=asynSuccess;
	std::stringstream cmd;
	epicsInt32 mbboHomingMethod;
	epicsInt32 homingMethod;
	epicsFloat64 homingMaxDistance;
	epicsFloat64 homingOffset;
	epicsFloat64 homingCurrLimit;
	static const char *functionName = "home";
	
	if (virtual_)
	{
		asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: axis %i home not supported because axis is virtual\n", driverName, functionName, axisNo_);
		return asynError;
	}
	
	controller->getIntegerParam(axisNo_, controller->SPiiPlusHomingMethod_, &mbboHomingMethod);
	controller->getDoubleParam(axisNo_, controller->SPiiPlusHomingMaxDist_, &homingMaxDistance);
	controller->getDoubleParam(axisNo_, controller->SPiiPlusHomingCurrLimit_, &homingCurrLimit);
	if (forwards == 0)
	{
		controller->getDoubleParam(axisNo_, controller->SPiiPlusHomingOffsetNeg_, &homingOffset);
	}
	else
	{
		controller->getDoubleParam(axisNo_, controller->SPiiPlusHomingOffsetPos_, &homingOffset);
	}
	
	switch(mbboHomingMethod)
	{
		case MBBO_HOME_NONE:
			homingMethod = SPIIPLUS_HOME_NONE;
			break;
		
		case MBBO_HOME_LIMIT_INDEX:
			if (forwards == 0)
				homingMethod = SPIIPLUS_HOME_NEG_LIMIT_INDEX;
			else
				homingMethod = SPIIPLUS_HOME_POS_LIMIT_INDEX;
			break;
		
		case MBBO_HOME_LIMIT:
			if (forwards == 0)
				homingMethod = SPIIPLUS_HOME_NEG_LIMIT;
			else
				homingMethod = SPIIPLUS_HOME_POS_LIMIT;
			break;
		
		case MBBO_HOME_INDEX:
			if (forwards == 0)
				homingMethod = SPIIPLUS_HOME_NEG_INDEX;
			else
				homingMethod = SPIIPLUS_HOME_POS_INDEX;
			break;
		
		case MBBO_HOME_CURRENT_POS:
			homingMethod = SPIIPLUS_HOME_CURRENT_POS;
			break;
		
		case MBBO_HOME_HARDSTOP_INDEX:
			if (forwards == 0)
				homingMethod = SPIIPLUS_HOME_NEG_HARDSTOP_INDEX;
			else
				homingMethod = SPIIPLUS_HOME_POS_HARDSTOP_INDEX;
			break;
		
		case MBBO_HOME_HARDSTOP:
			if (forwards == 0)
				homingMethod = SPIIPLUS_HOME_NEG_HARDSTOP;
			else
				homingMethod = SPIIPLUS_HOME_POS_HARDSTOP;
			break;
		
		case MBBO_HOME_CUSTOM:
			//if (forwards == 0)
			//	???
			//else
			//	???
			// Do nothing for now
			homingMethod = SPIIPLUS_HOME_NONE;
			break;
		
		default:
			homingMethod = SPIIPLUS_HOME_NONE;
			break;
	}
	
	if (homingMethod == SPIIPLUS_HOME_NONE)
	{
		asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: No homing method selected for axis %i\n", driverName, functionName, axisNo_);
		// Should status be set to asynError here?
	}
	else
	{
		// HOME Axis, [opt]HomingMethod,[opt]HomingVel,[opt]MaxDistance,[opt]HomingOffset,[opt]HomingCurrLimit,[opt]HardStopThreshold
		cmd << "HOME " << axisNo_ << "," << homingMethod << "," << (maxVelocity * resolution_);
		
		if (homingMaxDistance == 0.0)
		{
			// The controller takes the homingMaxDistance literally. Leave it empty instead of setting it to zero so homing will still occur.
			cmd << ",";
		}
		else
		{
			cmd << "," << homingMaxDistance;
		}
		
		//
		cmd << "," << homingOffset;
		
		if (homingCurrLimit == 0.0)
		{
			// The controller takes the homingCurrLimit literally. Omit the argument instead of setting it to zero so homing will still occur.
			// Uncomment the following line in the future if the HardStopThreshold or gantry arguments are added.
			//cmd << ",";
		}
		else
		{
			cmd << "," << homingCurrLimit;
		}
		
		//asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: home command = %s\n", driverName, functionName, cmd.str().c_str());
		status = controller->pComm_->writeReadAck(cmd);
	}
	return status;
}

/** Function to define the motor positions for a profile move. 
  * Called by asynMotorController::writeFloat64Array
  * This calls the base class defineProfile method to convert to steps, but since the
  * SPiiPlus works in user-units we need to do an additional conversion by resolution_. 
  * \param[in] positions Array of profile positions for this axis in user units.
  * \param[in] numPoints The number of positions in the array.
  */
asynStatus SPiiPlusAxis::defineProfile(double *positions, size_t numPoints)
{
  size_t i;
  asynStatus status;
  //static const char *functionName = "defineProfile";
  
  /* 
   * If the user positions are relative positions, the pulse positions
   * should also be relative positions. Doesn't the conversion from EGU
   * to steps break relative positions?
   */
  // Make a backup of the user-specified array, so that displacements can recalculated
  memcpy(profilePositionsUser_, positions, numPoints*sizeof(double));
  
  // Call the base class function (converts from EGU to steps)
  status = asynMotorAxis::defineProfile(positions, numPoints);
  if (status) return status;
  
  // Convert from steps to SPiiPlus user units
  for (i=0; i<numPoints; i++) {
    profilePositions_[i] = profilePositions_[i]*resolution_;
  }
  return asynSuccess;
}


/** Function to correct the motor positions for a profile move. 
  * Called by SPiiPlusController::buildProfile
  * This reapplies the correct conversion from EPICS units
  * to SPiiPlus units for relative displacements.
  * \param[in] numPoints The number of positions in the array.
  */
asynStatus SPiiPlusAxis::correctProfile(size_t numPoints)
{
  size_t i;
  double resolution;
  int direction;
  double scale;  
  int status = asynSuccess;
  static const char *functionName = "correctProfile";
  
  asynPrint(pasynUser_, ASYN_TRACE_FLOW,
            "%s:%s: axis=%d, numPoints=%d\n",
            driverName, functionName, axisNo_, (int)numPoints);

  if (numPoints > pC_->maxProfilePoints_) return asynError;

  status |= pC_->getDoubleParam(axisNo_, pC_->motorRecResolution_, &resolution);
  status |= pC_->getIntegerParam(axisNo_, pC_->motorRecDirection_, &direction);
  asynPrint(pasynUser_, ASYN_TRACE_FLOW,
            "%s:%s: axis=%d, status=%d, direction=%d, resolution=%f\n",
            driverName, functionName, axisNo_, status, direction, resolution);
  if (status) return asynError;
  if (resolution == 0.0) return asynError;
  
  scale = 1.0/resolution;
  if (direction != 0) scale = -scale;
  for (i=0; i<numPoints; i++)
  {
    // Reconvert user-specified displacements from EPICS units to SPiiPLus units, ignoring the EPICS offset
    profilePositions_[i] = profilePositionsUser_[i] * scale * resolution_;
  }
  return (asynStatus)status;
}

/** Reports on status of the axis
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * If details > 0 then information is printed about each axis.
  * After printing controller-specific information it calls asynMotorController::report()
  */
void SPiiPlusAxis::report(FILE *fp, int level)
{
  SPiiPlusController* controller = (SPiiPlusController*) pC_;
  epicsInt32 homingMethod;
  
  controller->getIntegerParam(axisNo_, controller->SPiiPlusHomingMethod_, &homingMethod);
  
  fprintf(fp, "Configuration for axis %i:\n", axisNo_);
  fprintf(fp, "  mflags: %i\n", controller->motorFlags_[axisNo_]);
  fprintf(fp, "    dummy:  %i\n", dummy_);
  fprintf(fp, "    open:  %i\n", open_);
  fprintf(fp, "    micro:  %i\n", micro_);
  fprintf(fp, "    homed:  %i\n", home_);
  fprintf(fp, "    stepper:  %i\n", stepper_);
  fprintf(fp, "    encloop:  %i\n", encloop_);
  fprintf(fp, "    stepenc:  %i\n", stepenc_);
  fprintf(fp, "    brushl:  %i\n", brushl_);
  fprintf(fp, "    brushok  %i\n", brushok_);
  fprintf(fp, "    phase2:  %i\n", phase2_);
  fprintf(fp, "    defcon:  %i\n", defcon_);
  fprintf(fp, "    linear:  %i\n", linear_);
  fprintf(fp, "    abscomm:  %i\n", abscomm_);
  fprintf(fp, "    hall:  %i\n", hall_);
  fprintf(fp, "  resolution: %.6e\n", resolution_);
  fprintf(fp, "  reference offset: %lf\n", controller->referenceOffset_[axisNo_]);
  fprintf(fp, "  homing method: %i\n", homingMethod);
  fprintf(fp, "  max velocity: %lf\n", controller->maxVelocity_[axisNo_]);
  fprintf(fp, "  max acceleration: %lf\n", controller->maxAcceleration_[axisNo_]);
  fprintf(fp, "  virtual: %s\n", virtual_ ? "Yes" : "No");
  fprintf(fp, "Encoder info for axis %i:\n", axisNo_);
  fprintf(fp, "  encoder type: %i\n", controller->encoderType_[axisNo_]);
  fprintf(fp, "  encoder resolution: %.6e\n", controller->encoderFactor_[axisNo_]);
  fprintf(fp, "  encoder offset: %lf\n", controller->encoderOffset_[axisNo_]);
  fprintf(fp, "  encoder 2 type: %i\n", controller->encoder2Type_[axisNo_]);
  fprintf(fp, "  encoder 2 resolution: %.6e\n", controller->encoder2Factor_[axisNo_]);
  fprintf(fp, "  encoder 2 offset: %lf\n", controller->encoder2Offset_[axisNo_]);
  fprintf(fp, "Position info for axis %i:\n", axisNo_);
  fprintf(fp, "  axis position: %lf\n", controller->axisPosition_[axisNo_]);
  fprintf(fp, "  reference position: %lf\n", controller->referencePosition_[axisNo_]);
  fprintf(fp, "  encoder position: %lf\n", controller->encoderPosition_[axisNo_]);
  fprintf(fp, "  feedback position: %lf\n", controller->feedbackPosition_[axisNo_]);
  fprintf(fp, "  feedback 2 position: %lf\n", controller->feedback2Position_[axisNo_]);
  fprintf(fp, "  virtual feedback position: %lf\n", controller->virtualFeedbackPosition_[axisNo_]);
  fprintf(fp, "Status for axis %i:\n", axisNo_);
  fprintf(fp, "  moving: %i\n", moving_);
  fprintf(fp, "  axis status: %i\n", controller->axisStatus_[axisNo_]);
  fprintf(fp, "  motor status: %i\n", controller->motorStatus_[axisNo_]);
  fprintf(fp, "  fault status: %i\n", controller->faultStatus_[axisNo_]);
  fprintf(fp, "\n");
  
  // Call the base class method
  asynMotorAxis::report(fp, level);
}

std::string SPiiPlusController::axesToString(std::vector <int> axes)
{
  //static const char *functionName = "axesToString";
  unsigned int i;
  std::stringstream outputStr;
  
  for (i=0; i<axes.size(); i++)
  {
    if (axes[i] == axes.front())
    {
      if (axes.size() > 1)
      {
          // Parentheses are only required when multiple axes are used
          outputStr << '(';
      }
      outputStr << axes[i];
    }
    else if (axes[i] == axes.back())
    {
      outputStr << ',' << axes[i] << ')';
    }
    else
    {
      outputStr << ',' << axes[i];
    }
  }
  
  return outputStr.str();
}

// Create a motor list string to be displayed in a message to the user
std::string SPiiPlusController::motorsToString(std::vector <int> axes)
{
  //static const char *functionName = "motorsToString";
  unsigned int i;
  std::stringstream outputStr;
  
  for (i=0; i<axes.size(); i++)
  {
    if (axes[i] == axes.front())
    {
      outputStr << (axes[i]+1);
    }
    else
    {
      outputStr << ", " << (axes[i]+1);
    }
  }
  
  return outputStr.str();
}

std::string SPiiPlusController::positionsToString(int positionIndex)
{
  //static const char *functionName = "positionsToString";
  unsigned int i;
  SPiiPlusAxis *pAxis;
  std::stringstream outputStr;
  
  for (i=0; i<profileAxes_.size(); i++)
  {
    pAxis = getAxis(profileAxes_[i]);
    
    if (profileAxes_[i] == profileAxes_.front())
    {
      outputStr << pAxis->fullProfilePositions_[positionIndex];
    }
    else 
    {
      outputStr << ',' << pAxis->fullProfilePositions_[positionIndex];
    }
  }
  
  return outputStr.str();
}

asynStatus SPiiPlusController::initializeProfile(size_t maxProfilePoints, size_t maxProfilePulses)
{
  int axis;
  SPiiPlusAxis *pAxis;
  asynStatus status;
  int i;
  std::stringstream cmd;
  // static const char *functionName = "initializeProfile";
  
  /*
   * Create point and time arrays that have extra elements for the 
   * acceleration and deceleration, not including the starting position
   */
  if (fullProfileTimes_) free(fullProfileTimes_);
  fullProfileTimes_ = (double *)calloc(maxProfilePoints+(2*MAX_ACCEL_SEGMENTS)-1, sizeof(double));
  
  for (axis=0; axis<numAxes_; axis++) {
    pAxis = getAxis(axis);
    if (!pAxis) continue;
    
    if (pAxis->fullProfilePositions_) free(pAxis->fullProfilePositions_);
    pAxis->fullProfilePositions_ = (double *)calloc(maxProfilePoints+(2*MAX_ACCEL_SEGMENTS)-1, sizeof(double));
    
    if (pAxis->profilePositionsUser_) free(pAxis->profilePositionsUser_);
    pAxis->profilePositionsUser_ = (double *)calloc(maxProfilePoints, sizeof(double));
  }
  
  // This sets maxProfilePoints_
  status = asynMotorController::initializeProfile(maxProfilePoints);
  //
  maxProfilePulses_ = maxProfilePulses;
  
  /*
   *  Create pulse arrays
   */
  if (profilePulses_) free(profilePulses_);
  profilePulses_ = (double *)calloc(maxProfilePulses, sizeof(double));
  
  if (profilePulsesUser_) free(profilePulsesUser_);
  profilePulsesUser_ = (double *)calloc(maxProfilePulses, sizeof(double));
  
  if (profilePulsePositions_) free(profilePulsePositions_);
  profilePulsePositions_ = (double *)calloc(maxProfilePulses, sizeof(double));
  
  // Create the arrays in the controller to hold the data that is recorded during profile moves
  for (i=0; i<SPIIPLUS_MAX_DC_AXES; i++)
  {
    // Delete the array, if it exists, in case maxProfilePoints changed
    cmd << "#VGV DC_DATA_" << (i+1);
    pComm_->writeReadAck(cmd);
    
    // Data recorded with the DC command will reside in DC_DATA_{1,2,3,4,5,6,7,8} 2D arrays
    cmd << "GLOBAL REAL DC_DATA_" << (i+1) << " (3)(" << maxProfilePoints << ")";
    pComm_->writeReadAck(cmd);
  }
  
  return status;
}

/** Function to define the pulse positions for a profile move. 
  * Converts the positions from user units to steps, using the profileMotorOffset_, 
  * profileMotorDirection_, and profileMotorResolution_ parameters. 
  * \param[in] positions Array of pulse positions for the pulse axis in EPICS user units.
  * \param[in] numPoints The number of pulse positions in the array.
  */
asynStatus SPiiPlusController::definePulses(int pulseAxis, int moveMode, size_t numPulses)
{
  size_t i;
  double resolution;
  double offset;
  int direction;
  double scale;
  int status=0;
  SPiiPlusAxis *axis;
  static const char *functionName = "definePulses";
  
  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
            "%s:%s: pulseAxis=%d, numPulses=%d, profilePulsesUser_[0]=%f\n",
            driverName, functionName, pulseAxis, (int)numPulses, profilePulsesUser_[0]);

  if (numPulses > maxProfilePulses_) return asynError;
  
  status |= getDoubleParam(pulseAxis, motorRecResolution_, &resolution);
  status |= getDoubleParam(pulseAxis, motorRecOffset_, &offset);
  status |= getIntegerParam(pulseAxis, motorRecDirection_, &direction);
  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
            "%s:%s: status=%d, offset=%f direction=%d, resolution=%f\n",
            driverName, functionName, status, offset, direction, resolution);
  if (status) return asynError;
  if (resolution == 0.0) return asynError;
  
  axis = getAxis(pulseAxis);
  
  // Convert from EPICS user coordinates to pulseAxis Automation1 units.
  scale = 1.0/resolution;
  if (direction != 0) scale = -scale;
  if (moveMode == PROFILE_MOVE_MODE_ABSOLUTE)
  {
    // The pulse positions are absolute positions
    for (i=0; i<numPulses; i++)
    {
      profilePulses_[i] = (profilePulsesUser_[i] - offset) * scale * axis->resolution_;
    }
  }
  else
  {
    // The pulse positions are relative displacements -- don't apply the offset
    for (i=0; i<numPulses; i++)
    {
      profilePulses_[i] = profilePulsesUser_[i] * scale * axis->resolution_;
    }
  }
  
  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
            "%s:%s: scale=%f, offset=%f, resolution_=%f, profilePulsesUser_[0]=%f, profilePulses_[0]=%f\n",
            driverName, functionName, scale, offset, axis->resolution_, profilePulsesUser_[0], profilePulses_[0]);
  
  return asynSuccess;
}

/** Function to build a coordinated move of multiple axes. */
asynStatus SPiiPlusController::buildProfile()
{
  int i; 
  unsigned int j;
  unsigned int idx;
  int status;
  bool buildOK=true;
  //bool verifyOK=true;
  int numPoints;
  int startPulses;
  int endPulses;
  int numPulses;
  int pulseMode;
  int pulseAxis;
  double pulseAxisCurrentPos;
  double pulseAxisCurrentRawPos;
  //int numElements;
  //double trajVel;
  //double D0, D1, T0, T1;
  int moveMode;
  char message[MAX_MESSAGE_LEN];
  int buildStatus;
  double maxVelocity;
  double maxAcceleration;
  //double maxVelocityActual=0.0;
  //double maxAccelerationActual=0.0;
  //double minPositionActual=0.0, maxPositionActual=0.0;
  //double minProfile, maxProfile;
  //double lowLimit, highLimit;
  //double minJerkTime, maxJerkTime;
  double preTimeMax, postTimeMax;
  double preVelocity[SPIIPLUS_MAX_AXES], postVelocity[SPIIPLUS_MAX_AXES];
  double preTime, postTime;
  double preDistance, postDistance;
  double accelTime;
  std::string axisList;
  int useAxis;
  std::stringstream cmd;
  SPiiPlusAxis *pPulseAxis;
  
    
  static const char *functionName = "buildProfile";
  
  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
            "%s:%s: entry\n",
            driverName, functionName);
            
  // Call the base class method which will build the time array if needed
  asynMotorController::buildProfile();
  
  strcpy(message, "");
  setStringParam(profileBuildMessage_, message);
  setIntegerParam(profileBuildState_, PROFILE_BUILD_BUSY);
  setIntegerParam(profileBuildStatus_, PROFILE_STATUS_UNDEFINED);
  callParamCallbacks();
  
  // Get asyn parameters
  getIntegerParam(profileMoveMode_,    &moveMode);
  getIntegerParam(profileNumPoints_,   &numPoints);
  getDoubleParam(profileAcceleration_, &accelTime);
  getIntegerParam(SPiiPlusPulseMode_,  &pulseMode);
  getIntegerParam(SPiiPlusPulseAxis_,  &pulseAxis);
  getIntegerParam(profileNumPulses_,   &numPulses);
  getIntegerParam(profileStartPulses_, &startPulses);
  getIntegerParam(profileEndPulses_,   &endPulses);
  getDoubleParam(pulseAxis,  motorPosition_,      &pulseAxisCurrentRawPos);
  
  // 
  profileAxes_.clear();
  memset(profileAccelTimes_, 0, MAX_ACCEL_SEGMENTS*sizeof(double));
  memset(profileDecelTimes_, 0, MAX_ACCEL_SEGMENTS*sizeof(double));
  
  for (i=0; i<numAxes_; i++) {
    // Zero the velocity arrays
    preVelocity[i] = 0.;
    postVelocity[i] = 0.;
    // Zero the accel/decel arrays
    memset(pAxes_[i]->profileAccelPositions_, 0, MAX_ACCEL_SEGMENTS*sizeof(double));
    memset(pAxes_[i]->profileDecelPositions_, 0, MAX_ACCEL_SEGMENTS*sizeof(double));
    // Check which axes should be used
    getIntegerParam(i, profileUseAxis_, &useAxis);
    asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s: %i axis will be used: %i\n", driverName, functionName, i, useAxis);
    if (useAxis)
    {
      profileAxes_.push_back(i);
      
      if (moveMode == PROFILE_MOVE_MODE_RELATIVE)
      {
        // NOTE: the profile positions were converted from EPICS units to SPiiPlus units in 
        // SPiiPlusAxis::defineProfile, however, the calculation was incorrect for relative 
        // displacements.  Correct the calculation here.
        pAxes_[i]->correctProfile(numPoints);
      }
    }
  }
  
  if (profileAxes_.size() == 0)
  {
    strcpy(message, "No axes selected");
    buildOK = false;
    goto done;
  }
  
  asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s: axisList = %s\n", driverName, functionName, axesToString(profileAxes_).c_str());
  sprintf(message, "Selected axes: %s", motorsToString(profileAxes_).c_str()); 
  setStringParam(profileBuildMessage_, message);
  callParamCallbacks();
  
  /* We create trajectories with extra elements at the beginning and at the end.
   * The distance and time of the prepended elements are defined so that the motors will
   * accelerate from 0 to the velocity of the first "real" element in the user-specified
   * acceleration time, as long as it doesn't exceed the maximum allowed acceleration.
   * Similarly, the distance and time of appended elements are defined so that the 
   * motors will decelerate from the velocity of the last "real" element to 0 
   * in the user-specified acceleration time. */
  
  preTimeMax = 0.;
  postTimeMax = 0.;
  
  for (j=0; j<profileAxes_.size(); j++)
  {
    // j != axis index
    idx = profileAxes_[j];
     
    // Query the max velocity and acceleration
    cmd << "?XVEL(" << idx << ")";
    status = pComm_->writeReadDouble(cmd, &maxVelocity);
    if (status) {
      buildOK = false;
      sprintf(message, "Error getting XVEL, status=%d\n", status);
      goto done;
    }
    cmd << "?XACC(" << idx << ")";
    status = pComm_->writeReadDouble(cmd, &maxAcceleration);
    if (status) {
      buildOK = false;
      sprintf(message, "Error getting XACC, status=%d\n", status);
      goto done;
    }
    
    /* The calculation using maxAcceleration read from controller below
     * is "correct" but subject to roundoff errors when sending ASCII commands.
     * Reduce acceleration 10% to account for this. */
    maxAcceleration *= 0.9;
    
    if (moveMode == PROFILE_MOVE_MODE_ABSOLUTE)
    {
      preDistance = pAxes_[idx]->profilePositions_[1] - pAxes_[idx]->profilePositions_[0];
    }
    else
    {
      preDistance = pAxes_[idx]->profilePositions_[0];
    }
    // Use the 2nd element of the times array instead of the 1st; the 1st will be used for the preDistance move.
    preVelocity[idx] = preDistance/profileTimes_[1];
    preTime = fabs(preVelocity[idx]) / maxAcceleration;
    preTimeMax = MAX(preTimeMax, preTime);
    // Use the acceleration specified by the user, if it is less than the max acceleration
    preTimeMax = MAX(preTimeMax, accelTime);
    
    if (moveMode == PROFILE_MOVE_MODE_ABSOLUTE)
    {
      postDistance = pAxes_[idx]->profilePositions_[numPoints-1] - pAxes_[idx]->profilePositions_[numPoints-2];
    }
    else
    {
      postDistance = pAxes_[idx]->profilePositions_[numPoints-1];
    }
    postVelocity[idx] = postDistance/profileTimes_[numPoints-1];
    postTime = fabs(postVelocity[idx]) / maxAcceleration;
    postTimeMax = MAX(postTimeMax, postTime);
    // Use the acceleration specified by the user, if it is less than the max acceleration
    postTimeMax = MAX(postTimeMax, accelTime);
    
    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
              "%s:%s: axis %d profilePositions[0]=%f, profilePositions[%d]=%f, maxAcceleration=%f, preTimeMax=%f, postTimeMax=%f\n",
              driverName, functionName, idx, pAxes_[idx]->profilePositions_[0], numPoints-1, pAxes_[idx]->profilePositions_[numPoints-1],
              maxAcceleration, preTimeMax, postTimeMax);
  }
  
  // calculate the number of acceleration segments
  numAccelSegments_ = getNumAccelSegments(preTimeMax);
  numDecelSegments_ = getNumAccelSegments(postTimeMax);
  
  // populate the profileAccelTimes_ and profileDecelTimes_ arrays
  createAccDecTimes(preTimeMax, postTimeMax);
  
  /*
   * Every segment of PATH/POINT/ENDS motion is at a constant velocity.
   * The driver is resposnible for creating the acceleration and 
   * deceleration phases of the full profile move.
   */
  for (j=0; j<profileAxes_.size(); j++)
  {
    // j != axis index
    idx = profileAxes_[j];     

    pAxes_[idx]->profilePreDistance_  =  0.5 * preVelocity[idx]  * preTimeMax;
    pAxes_[idx]->profilePostDistance_ =  0.5 * postVelocity[idx] * postTimeMax;
    
    if (moveMode == PROFILE_MOVE_MODE_ABSOLUTE)
    {
      pAxes_[idx]->profileStartPos_ = pAxes_[idx]->profilePositions_[0] - pAxes_[idx]->profilePreDistance_;
      pAxes_[idx]->profileFlybackPos_ = pAxes_[idx]->profilePositions_[numPoints-1];
    }
    else
    {
      pAxes_[idx]->profileStartPos_ = -pAxes_[idx]->profilePreDistance_;
      pAxes_[idx]->profileFlybackPos_ = -pAxes_[idx]->profilePostDistance_;
    }
    
    // populate the profileAccelPositions_ and profileDecelPositions_ arrays
    createAccDecPositions(pAxes_[idx], moveMode, numPoints, preTimeMax, postTimeMax, preVelocity[idx], postVelocity[idx]);
  }
  
  // populate the fullProfileTimes_ and fullProfilePositions_ arrays
  assembleFullProfile(numPoints);
  
  /*
   * Turn the profilePulses_ array into an array of displacements in the correct format
   * 
   * Notes: 
   *   * user specified positions will be in EPICS user units
   *   * numPulses is used instead of numPulses_ (the number of elements written to the pulse position array)
   *     because the user might want to reduce the number of pulses without changing the array.
   */
  
  pPulseAxis = getAxis(pulseAxis);
  
  // Convert pulse positions in EPICS user units to SPiiPlus units
  definePulses(pulseAxis, moveMode, numPulses);
  // Convert the raw position of the pulse axis into SPiiPlus units (needed for relative profiles) 
  pulseAxisCurrentPos = pulseAxisCurrentRawPos * pPulseAxis->resolution_;
  
  if (pulseMode == 0)
  {
    /* Fixed Mode (divide trajectory by numPulses) will use PEG_I */
    
    double totalDistance = 0.0;
    
    if (moveMode == PROFILE_MOVE_MODE_ABSOLUTE)
    {
      // The total distance is the difference between the final and initial user-specified positions
      // Note: this probably doesn't work if the axis changes direction
      totalDistance = pPulseAxis->profilePositions_[numPoints-1] - pPulseAxis->profilePositions_[0];
      
      // The start position is the 1st user-specified position
      pulseStartPos_ = pPulseAxis->profilePositions_[0];
      
      // The end position is the last user-specified position
      pulseEndPos_ =  pPulseAxis->profilePositions_[numPoints-1];
    }
    else
    {
      // The total distance is the sum of the displacements for each segment
      // Note: this probably doesn't work if the axis changes direction
      for (i=0; i<(numPoints-1); i++)
      {
        totalDistance += pPulseAxis->profilePositions_[i];
      }
      
      // The start position is the motor's current position
      pulseStartPos_ = pulseAxisCurrentPos;
      
      // The end position is the starting position + the user-specified displacement
      pulseEndPos_ = pulseStartPos_ + totalDistance;
    }
    
    // The pulse position tracking will start at the starting position, so the nth pulse will occur at the end of the nth segment
    pulseSpacing_ = totalDistance / numPulses;
  }
  else if (pulseMode == 1)
  {
    /* Array Mode will use PEG_R */
    // Should the number of pulses in the array be stored in a private variable
    // so that problems aren't introduced by users who increase the number of 
    // pulses between building an executing the profileMove?
    
    if (moveMode == PROFILE_MOVE_MODE_ABSOLUTE)
    {
      // The user-specified pulse array is already in the correct form. Send it
      // to the controller now, to minimize delays when executing the profile.
      // NOTE: maxProfilePulses is used instead of numPulses because the global
      // variable might not exist in the controller yet and the user can modify
      // The number of pulses at any time.
      status = pComm_->putDoubleArray(profilePulses_, "pulsePos", 0, maxProfilePulses_-1, 0, 0);
      if (status) {
        buildOK = false;
        sprintf(message, "Error writing pulse positions, status=%d\n", status);
        goto done;
      }
    }
    else
    {
      double current_pos = pulseAxisCurrentPos;
      
      // Convert the pulse displacements to absolute positions
      for (i=0; i<numPulses; i++)
      {
        profilePulses_[i] = current_pos + profilePulses_[i];
        current_pos = profilePulses_[i];
      }
    }
  }
  else if (pulseMode == 2)
  {
    /* TrajPts Mode (get traj point positions) will use PEG_I */
    
    double totalDistance = 0.0;
    
    if (moveMode == PROFILE_MOVE_MODE_ABSOLUTE)
    {
      // The total distance is the difference between the user-specified end and start waypoints
      // Note: this probably doesn't work if the axis changes direction
      totalDistance = pPulseAxis->profilePositions_[endPulses] - pPulseAxis->profilePositions_[startPulses];
      
      // The start position is the 1st user-specified position
      pulseStartPos_ = pPulseAxis->profilePositions_[startPulses];
      
      // The end position is the last user-specified position
      pulseEndPos_ =  pPulseAxis->profilePositions_[endPulses];
    }
    else
    {
      // The start position is the pulse axis's current start position + displacements to the user-specified pulse start
      pulseStartPos_ = pulseAxisCurrentPos;
      for (i=0; i<startPulses; i++)
      {
        pulseStartPos_ += pPulseAxis->profilePositions_[i];
      }
      
      // The end position is pulse start position + displacements to the user-specified pulse end
      pulseEndPos_ = pulseStartPos_;
      // There might be an off-by-one problem in the following loop
      for (i=startPulses; i<endPulses; i++)
      {
        pulseEndPos_ += pPulseAxis->profilePositions_[i];
      }
      
      totalDistance = pulseEndPos_ - pulseStartPos_;
    }
    
    // The pulse position tracking will start at the user-specified starting waypoint; the last pulse will occur at the end of the user-specified ending waypoint
    pulseSpacing_ = totalDistance / numPulses;
  } 
  else if (pulseMode == 3)
  {
    // None Mode - what should be done in this case?  Set numPulses to zero?  Zero the pulse array?
  }
  
  // Sanity check
  //sanityCheckProfile();
  
  // calculate the time interval for data collection
  calculateDataCollectionInterval();
  // TODO: clear the data arrays heare instead of in runProfile?
  
  // POINT commands have this syntax: POINT (0,1,5), 1000,2000,3000, 500
  
  // Verfiy the profile (check speed, accel, limit violations)
  
  done:
  buildStatus = (buildOK) ? PROFILE_STATUS_SUCCESS : PROFILE_STATUS_FAILURE;
  setIntegerParam(profileBuildStatus_, buildStatus);
  setStringParam(profileBuildMessage_, message);
  if (buildStatus != PROFILE_STATUS_SUCCESS) {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
              "%s:%s: %s\n",
              driverName, functionName, message);
  }
  else
  {
    // Set the execute and readback status to undefined so that users know those haven't occurred since the build was done
    setIntegerParam(profileExecuteStatus_, PROFILE_STATUS_UNDEFINED);
    setIntegerParam(profileReadbackStatus_, PROFILE_STATUS_UNDEFINED);
  }
  /* Clear build command.  This is a "busy" record, don't want to do this until build is complete. */
  setIntegerParam(profileBuild_, 0);
  setIntegerParam(profileBuildState_, PROFILE_BUILD_DONE);
  callParamCallbacks();
  //return status ? asynError : asynSuccess;
  return asynSuccess;
}

int SPiiPlusController::getNumAccelSegments(double time)
{
  long numSegments;
  // The following value was chosen somewhat arbitrarily -- it gives MAX_ACCEL_SEGMENTS at an acceleration time of 0.2s
  double minPeriod = 0.01;
  
  numSegments = lround(time / minPeriod);
  
  if (numSegments > MAX_ACCEL_SEGMENTS)
  {
    numSegments = MAX_ACCEL_SEGMENTS;
  }
  
  return numSegments;
}

void SPiiPlusController::createAccDecTimes(double preTimeMax, double postTimeMax)
{
  int i;
  //static const char *functionName = "createAccDecTimes";
  
  // Use a constant time for accel/decel segments
  for (i=0; i<numAccelSegments_; i++)
  {
    profileAccelTimes_[i] = preTimeMax / numAccelSegments_;
  }
  for (i=0; i<numDecelSegments_; i++)
  {
    profileDecelTimes_[i] = postTimeMax / numDecelSegments_;
  }
}

void SPiiPlusController::createAccDecPositions(SPiiPlusAxis* axis, int moveMode, int numPoints, double preTimeMax, double postTimeMax, double preVelocity, double postVelocity)
{
  int i;
  double time;
  double time2;
  double time1;
  //static const char *functionName = "createAccDecPositions";
  
  if (moveMode == PROFILE_MOVE_MODE_ABSOLUTE)
  {
    // Acceleration (absolute)
    for (i=0; i<numAccelSegments_; i++)
    {
      time = preTimeMax * (i+1) / numAccelSegments_;
      // position during accel period = starting position of user profile - acceleration distance + distance traveled in i acceleration segments
      axis->profileAccelPositions_[i] = axis->profilePositions_[0] - axis->profilePreDistance_ + 0.5 * (preVelocity / preTimeMax) * pow(time, 2);
    }
    
    // Deceleration (absolute)
    for (i=0; i<numDecelSegments_; i++)
    {
      time = postTimeMax * (i+1) / numDecelSegments_;
      // position during decel period = ending position of user profile + distance traveled in i deceleration segments
      axis->profileDecelPositions_[i] = axis->profilePositions_[numPoints-1] + postVelocity * time - 0.5 * (postVelocity / postTimeMax) * pow(time, 2);
    }
  }
  else
  {
    // Acceleration (relative)
    for (i=0; i<numAccelSegments_; i++)
    {
      // position during accel period = 0.5 * a * (t2^2 - t1^2)
      time2 = preTimeMax*(i+1)/numAccelSegments_;
      time1 = preTimeMax*i/numAccelSegments_;
      axis->profileAccelPositions_[i] = 0.5 * (preVelocity / preTimeMax) * (pow(time2, 2) - pow(time1, 2));
    }
    
    // Deceleration (relative)
    for (i=0; i<numDecelSegments_; i++)
    {
      // position during decel period = vf * (t2 - t1) - 0.5 * a * (t2^2 - t1^2)
      time2 = postTimeMax*(i+1)/numDecelSegments_;
      time1 = postTimeMax*i/numDecelSegments_;
      axis->profileDecelPositions_[i] = postVelocity * (time2 - time1) - 0.5 * (postVelocity / postTimeMax) * (pow(time2, 2) - pow(time1, 2));
    }
  }
}

void SPiiPlusController::assembleFullProfile(int numPoints)
{
  int i;
  unsigned int j;
  int profileIdx;
  //static const char *functionName = "assembleFullProfile";

  /*
   * Assemble the full profile array from the component arrays.
   * The starting point, pAxes_[j]->profilePreDistance_, is not included.
   * The first point of the user-specified profile is ignored because the
   * first time in the user-specified array is not meaningful and the 
   * first position in the user-specified array is the same as the last 
   * position of the acceleration position array.
   */
  profileIdx = 0;
  for (i=0; i<numAccelSegments_; i++)
  {
    fullProfileTimes_[profileIdx] = profileAccelTimes_[i];
    for (j=0; j<profileAxes_.size(); j++)
    {
      pAxes_[profileAxes_[j]]->fullProfilePositions_[profileIdx] = pAxes_[profileAxes_[j]]->profileAccelPositions_[i];
    }
    profileIdx++;
  }
  for (i=1; i<numPoints; i++)
  {
    fullProfileTimes_[profileIdx] = profileTimes_[i];
    for (j=0; j<profileAxes_.size(); j++)
    {
      pAxes_[profileAxes_[j]]->fullProfilePositions_[profileIdx] = pAxes_[profileAxes_[j]]->profilePositions_[i];
    }
    profileIdx++;
  }
  for (i=0; i<numDecelSegments_; i++)
  {
    fullProfileTimes_[profileIdx] = profileDecelTimes_[i];
    for (j=0; j<profileAxes_.size(); j++)
    {
      pAxes_[profileAxes_[j]]->fullProfilePositions_[profileIdx] = pAxes_[profileAxes_[j]]->profileDecelPositions_[i];
    }
    profileIdx++;
  }
  // fullProfileSize_ == profileIdx at this point
  fullProfileSize_ = numAccelSegments_ + (numPoints-1) + numDecelSegments_;
}

void SPiiPlusController::sanityCheckProfile()
{
  int i;
  unsigned int idx, j, numProfileAxes;
  
  numProfileAxes = profileAxes_.size();
  
  for (i=0; i<fullProfileSize_; i++)
  {
    printf("%i\t", i);
    for (j=0; j<numProfileAxes; j++)
    {
      idx = profileAxes_[j];
      printf("%f\t", pAxes_[idx]->fullProfilePositions_[i]);
    }
    printf("\n");
  }
}

void SPiiPlusController::calculateDataCollectionInterval()
{
  int i;
  double time;
  // static const char *functionName = "calculateDataCollectionInterval";
  
  for (i=0; i<fullProfileSize_; i++)
  {
    time += fullProfileTimes_[i];
  }
  
  dataCollectionInterval_ = time / maxProfilePoints_;
}

long int SPiiPlusController::calculateCurrentPulse(int currentPoint, int startPulse, int endPulse, int numPulses)
{
  long int currentPulse;
  double rawPulse;
  //static const char *functionName = "calculateCurrentPulse";

  if (currentPoint < startPulse)
  {
    currentPulse = 0;
  }
  else if (currentPoint > endPulse)
  {
    currentPulse = numPulses;
  }
  else
  {
    /*
     * currentPulse = (currentPoint - startPulse) * ( (numPulses - 1) / (endPulse - startPulse) ) + 1
     */
    rawPulse = (currentPoint - startPulse) * ( (numPulses - 1) / (double)(endPulse - startPulse) ) + 1;
    //asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: rawPulse = %lf, currentPoint = %i, startPulse = %i, endPulse = %i, numPulses = %i\n", driverName, functionName, rawPulse, currentPoint, startPulse, endPulse, numPulses);

    currentPulse = lrint(rawPulse);
  }
  
  return currentPulse;
}

/** Function to execute a coordinated move of multiple axes. */
asynStatus SPiiPlusController::executeProfile()
{
  // static const char *functionName = "executeProfile";
  epicsEventSignal(profileExecuteEvent_);
  return asynSuccess;
}

/* C Function which runs the profile thread */ 
static void SPiiPlusProfileThreadC(void *pPvt)
{
  SPiiPlusController *pC = (SPiiPlusController*)pPvt;
  pC->profileThread();
}

/* Function which runs in its own thread to execute profiles */ 
void SPiiPlusController::profileThread()
{
  while (true) {
    epicsEventWait(profileExecuteEvent_);
    runProfile();
  }
}

/* Function to run trajectory.  It runs in a dedicated thread, so it's OK to block.
 * It needs to lock and unlock when it accesses class data. */ 
asynStatus SPiiPlusController::runProfile()
{
  int status;
  bool executeOK=true;
  bool aborted=false;
  int startPulses, endPulses;
  //int lastTime;
  int numPoints, numPulses;
  int numElements;
  int executeStatus;
  int pulseAxis, outputIndex;
  int pulseMode;
  std::string pegEngEncCode, pegOutAssignCode, poutsBitCode;
  double pulseWidth;
  double position;
  //double time;
  int i;
  unsigned int j;
  int moveMode;
  char message[MAX_MESSAGE_LEN];
  std::string positions;
  std::stringstream positionStr;
  std::stringstream commandStr;
  std::stringstream cmd;
  SPiiPlusAxis *pAxis;
  int ptExecIdx;
  int ptLoadedIdx;
  int ptFree;
  int ptIdx;
  std::string posData;
  static const char *functionName = "runProfile";
  
  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s: start\n", driverName, functionName);

  if (profileAxes_.size() == 0)
  {
    strcpy(message, "No axes selected");
    executeOK = false;
    goto done;
  }
  
  asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s: axisList = %s\n", driverName, functionName, axesToString(profileAxes_).c_str());
  
  lock();
  // These are also used by buildProfile
  getIntegerParam(profileStartPulses_, &startPulses);
  getIntegerParam(profileEndPulses_,   &endPulses);
  getIntegerParam(profileNumPoints_,   &numPoints);
  getIntegerParam(profileNumPulses_,   &numPulses);
  getIntegerParam(SPiiPlusPulseMode_,  &pulseMode);
  getIntegerParam(SPiiPlusPulseAxis_,  &pulseAxis);
  // These aren't used by buildProfile
  getStringParam(SPiiPlusPEGEngEncCode_,     pegEngEncCode);
  getStringParam(SPiiPlusPEGOutAssignCode_,  pegOutAssignCode);
  getIntegerParam(SPiiPlusPOUTSOutputIndex_, &outputIndex);
  getStringParam(SPiiPlusPOUTSBitCode_,      poutsBitCode);
  getDoubleParam(SPiiPlusPulseWidth_,        &pulseWidth);
  
  sprintf(message, "Selected axes: %s", motorsToString(profileAxes_).c_str()); 
  setStringParam(profileExecuteMessage_, message);
  setIntegerParam(profileExecuteState_, PROFILE_EXECUTE_MOVE_START);
  setIntegerParam(profileExecuteStatus_, PROFILE_STATUS_UNDEFINED);
  
  // Set the readback status to undefined so the user can see that a new read should be done when this move is done executing
  setIntegerParam(profileReadbackStatus_, PROFILE_STATUS_UNDEFINED);
  
  callParamCallbacks();
  unlock();
  
  getIntegerParam(profileMoveMode_, &moveMode);
  
  // move motors to the starting position
  if (moveMode == PROFILE_MOVE_MODE_ABSOLUTE)
  {
    cmd << "PTP/m ";
  }
  else
  {
    cmd << "PTP/mr ";
  }
  for (j=0; j<profileAxes_.size(); j++)
  {
    pAxis = getAxis(profileAxes_[j]);
    
    position = pAxis->profileStartPos_;
    
    if (profileAxes_[j] == profileAxes_.front())
    {
      positionStr << position;
    }
    else 
    {
      positionStr << ',' << position;
    }
  }
  cmd << axesToString(profileAxes_) << ", " << positionStr.str();
  status = pComm_->writeReadAck(cmd);
  // Should this be done after every command in this method?
  if (status)
  {
    executeOK = false;
    goto done;
  }
  
  // Wait for the motors to get there
  wakeupPoller();
  waitMotors();
  
  if (halted_)
  {
    aborted = true;
    executeOK = false;
    strcpy(message, "Aborted during move to start");
    goto done;
  }
  
  lock();
  setIntegerParam(profileExecuteState_, PROFILE_EXECUTE_EXECUTING);
  callParamCallbacks();
  unlock();
  
  /* configure data recording, which will start when the GO command is issued */
  int axesToRecord;
  if (profileAxes_.size() > 8)
    axesToRecord = 8;
  else
    axesToRecord = profileAxes_.size();
  for (i=0; i<axesToRecord; i++)
  {
    // Zero the data array
    cmd << "FILL(0,DC_DATA_" << (i+1) << ")";
    status = pComm_->writeReadAck(cmd);
    
    // DC/sw DC_DATA_#,maxProfilePoints_,3,FPOS(a),PE(a),TIME
    if (pAxes_[profileAxes_[i]]->dummy_)
    {
      // use the desired position for dummy axes, since FPOS and PE are always zero
      posData = "APOS";
    }
    else if (pAxes_[profileAxes_[i]]->virtual_)
    {
      if (virtualFeedbackPositionSupported_)
      {
        // use the virtual feedback position for virtual axes
        posData = "VPOS";
      }
      else
      {
        // use the desired position for virtual axes when virtual feedback positions are not supported
        posData = "APOS";
      }
    }
    else
    {
      // use the feedback position for real motors
      posData = "FPOS";
    }
    cmd << "DC/sw " << profileAxes_[i] << ",DC_DATA_" << (i+1) << "," << maxProfilePoints_ << ",";
    cmd << lround(dataCollectionInterval_ * 1000.0) << "," << posData << "(" << profileAxes_[i] << "),PE(" << profileAxes_[i] << "),TIME";
    status = pComm_->writeReadAck(cmd);
  }
  
  /*
   *  There is a bug in the controller firmware that prevents synchronized data 
   * collection from starting when a the GO is issued for the PTP/tw move.  If 
   * more than two axes are scanned, data collection must be started *before* 
   * the PTP/tw is sent to the controller, which results in somewhat synchronized 
   * data.  If only one axis is scanned, two GO commands could be sent after 
   * the PTP/tw command, but that isn't implemented.
   */
  // Ugly hack: A GO is needed here to start data collection.
  cmd << "GO " << axesToString(profileAxes_);
  status = pComm_->writeReadAck(cmd);
  
  // Configure pulse output (does this need to happen before data recording is setup/started?)

  /* The number of user-specified trajectory elements is numPoints-1
     The actual number of is greater due to the accel and decel elements added in assembleFullProfile */
  numElements = numPoints - 1;
  
  // TODO: move this to buildProfile?
  // TODO: only enforce this for the correct pulse modes
  // Check valid range of start and end pulses;  these start at 1, not 0.
  // numElements+1 is the first deceleration element
  if ((startPulses < 1)           || (startPulses > numElements+1) ||
      (endPulses   < startPulses) || (endPulses   > numElements+1)) {
    executeOK = false;
    sprintf(message, "Error: start or end pulses outside valid range");
    goto done;
  }
  
  //asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: pulseAxis=%i, pegEngEncCode=%s, pegOutAssignCode=%s, outputIndex=%i, poutsBitCode=%s\n", driverName, functionName, pulseAxis, pegEngEncCode.c_str(), pegOutAssignCode.c_str(), outputIndex, poutsBitCode.c_str());
  
  // TODO: store the pulseAxis to pulseAxis_ at build time to avoid problems with a profile built using one pulseAxis and executed using another
  // Assign PEG engine to an encoder
  // ASSIGNPEG axis, engines_to_encoders_code, gp_out_assign_code
  if (pulseMode == 1)
  {
    // ASSIGNPEG/f is required if PEG_R/d will be used
    cmd << "ASSIGNPEG/f " << pulseAxis << ", " << pegEngEncCode << ", " << pegOutAssignCode;
  }
  else
  {
    cmd << "ASSIGNPEG " << pulseAxis << ", " << pegEngEncCode << ", " << pegOutAssignCode;
  }
  status = pComm_->writeReadAck(cmd);
  
  // Assign PEG output to output pins
  // ASSIGNPOUTS axis, output_index, bit_code
  cmd << "ASSIGNPOUTS " << pulseAxis << ", " << outputIndex << ", " << poutsBitCode;
  status = pComm_->writeReadAck(cmd);
  
  if ((pulseMode == 0) || (pulseMode == 2))
  {
    // PEG_I axis, width, first_point, interval, last_point
    asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s: startPulsePos=%f, endPulsePos=%f, numElements=%i, pulseInterval=%f\n", driverName, functionName, pulseStartPos_, pulseEndPos_, numPulses, pulseSpacing_);
    cmd << "PEG_I " << pulseAxis << ", " << pulseWidth << ", " << pulseStartPos_ << ", " << pulseSpacing_ << ", " << pulseEndPos_;
    status = pComm_->writeReadAck(cmd);
    // Should a failed PEG_I command cause the scan to fail?  Yes, for now.
    if (status)
    {
      executeOK = false;
      strcpy(message, "Aborting due to PEG_I error");
      goto done;
    }
  }
  else if (pulseMode == 1)
  {
    // PEG_R peg_engine width mode first_index last_index POS_ARRAY
    
    // TODO: is 0x4444 the correct mode? Is pulseAxis the correct peg_engine argument?
    // NOTE: ASSIGNPEG's /f switch is also required
    cmd << "PEG_R/d" << pulseAxis << ", " << pulseWidth << ", " << "0x4444" << ", " << "0" << ", " << (numPulses-1) << ", " << "pulsePos";
    status = pComm_->writeReadAck(cmd);
    // Should a failed PEG_R command cause the scan to fail?  Yes, for now.
    if (status)
    {
      executeOK = false;
      strcpy(message, "Aborting due to PEG_R not implemented yet");
      goto done;
    }
  }
  else if (pulseMode == 3)
  {
    // What should happen here?
  }
  
  // Wait for PEGREADY 
  while (pAxes_[pulseAxis]->pegReady_ == 0)
  {
    asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s: axisStatus(%i).#PEGREAD = %i\n", driverName, functionName, pulseAxis, pAxes_[pulseAxis]->pegReady_);
    // Sleep to give the poller time to reread the the axis status
    epicsThreadSleep(0.1);
  }
  
  // wake up poller
  //wakeupPoller();
  
  /* run the trajectory */
  
  ptLoadedIdx = 0;
  ptExecIdx = 0;
  
  lock();
  setIntegerParam(profileCurrentPoint_, ptExecIdx);
  setIntegerParam(profileActualPulses_, calculateCurrentPulse(ptExecIdx, startPulses, endPulses, numPulses));
  callParamCallbacks();
  unlock();
  
  // Send the command to start the coordinated motion, but wait for the GO command to move motors
  if (moveMode == PROFILE_MOVE_MODE_ABSOLUTE)
  {
    cmd << "PATH/tw ";
  }
  else
  {
    cmd << "PATH/twr ";
  }
  cmd << axesToString(profileAxes_);
  status = pComm_->writeReadAck(cmd);
  
  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s: point buffer fill start\n", driverName, functionName);
  
  // Fill the point buffer, which can only hold 50 points
  for (ptIdx = 0; ptIdx < MIN(50, fullProfileSize_); ptIdx++)
  {
    // Create and send the point command (should this be ptIdx+1?)
    cmd << "POINT " << axesToString(profileAxes_) << ", " << positionsToString(ptIdx) << ", " << lround(fullProfileTimes_[ptIdx] * 1000.0);
    status = pComm_->writeReadAck(cmd);
    
    // Increment the counter of points that have been loaded
    ptLoadedIdx++;
  }
  
  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s: point buffer fill end\n", driverName, functionName);
  
  if (fullProfileSize_ > 50)
  {
    // Send the GO command
    cmd << "GO " << axesToString(profileAxes_);
    status = pComm_->writeReadAck(cmd);
    
    while (ptLoadedIdx < fullProfileSize_)
    {
      if (halted_)
      {
        aborted = true;
        executeOK = false;
        status = stopDataCollection();
        status = stopPEG(pulseAxis);
        strcpy(message, "Aborted during profile move");
        goto done;
      }
      
      // Sleep for a short period of time
      epicsThreadSleep(0.1);
      
      // Query the number of free points in the buffer (the first axis in the vector is the lead axis)
      cmd << "?GSFREE(" << profileAxes_[0] << ")";
      status = pComm_->writeReadInt(cmd, &ptFree);
      
      // Increment the counter of points that have been executed
      ptExecIdx += ptFree;
      
      // load the rest of the points as needed
      for (ptIdx=ptLoadedIdx; ptIdx<(ptLoadedIdx+ptFree); ptIdx++)
      {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s: sending another point\n", driverName, functionName);
        
        // Create and send the point command (should this be ptIdx+1?)
        cmd << "POINT " << axesToString(profileAxes_) << ", " << positionsToString(ptIdx) << ", " << lround(fullProfileTimes_[ptIdx] * 1000.0);
        status = pComm_->writeReadAck(cmd);
      }
      
      // Increment the counter of points that have been loaded
      ptLoadedIdx += ptFree;
      
      lock();
      // Only report the current point of the user-specified array
      if (ptExecIdx > numAccelSegments_)
      {
        setIntegerParam(profileCurrentPoint_, ptExecIdx-numAccelSegments_);
        setIntegerParam(profileActualPulses_, calculateCurrentPulse(ptExecIdx-numAccelSegments_, startPulses, endPulses, numPulses));
      }
      else
      {
        setIntegerParam(profileCurrentPoint_, 0);
        setIntegerParam(profileActualPulses_, 0);
      }
      callParamCallbacks();
      unlock();
    }
    
    // End the point sequence
    cmd << "ENDS " << axesToString(profileAxes_);
    status = pComm_->writeReadAck(cmd);
  }
  else
  {
    // End the point sequence
    cmd << "ENDS " << axesToString(profileAxes_);
    status = pComm_->writeReadAck(cmd);
    
    // Send the GO command
    cmd << "GO " << axesToString(profileAxes_);
    status = pComm_->writeReadAck(cmd);
  }
  
  // Wait for the remaining points to be executed
  while (ptExecIdx < fullProfileSize_)
  {
    if (halted_)
    {
      aborted = true;
      executeOK = false;
      status = stopDataCollection();
      status = stopPEG(pulseAxis);
      strcpy(message, "Aborted during profile move");
      goto done;
    }
    
    // Sleep for a short period of time
    epicsThreadSleep(0.1);
    
    // Query the number of free points in the buffer
    cmd << "?GSFREE(" << profileAxes_[0] << ")";
    status = pComm_->writeReadInt(cmd, &ptFree);
    
    // Update the number of points that have been executed
    ptExecIdx = fullProfileSize_ - 50 + ptFree;
    
    lock();
    // Stop updating current point when numPoints is reached
    if (ptExecIdx < numAccelSegments_)
    {
      // This only gets executed if the user-specified profile has very few points in it
      setIntegerParam(profileCurrentPoint_, 0);
      setIntegerParam(profileActualPulses_, 0);
    }
    else if ((ptExecIdx >= numAccelSegments_) && (ptExecIdx < numAccelSegments_+numPoints))
    {
      setIntegerParam(profileCurrentPoint_, ptExecIdx-numAccelSegments_);
      setIntegerParam(profileActualPulses_, calculateCurrentPulse(ptExecIdx-numAccelSegments_, startPulses, endPulses, numPulses));
    }
    else
    {
      setIntegerParam(profileCurrentPoint_, numPoints);
      setIntegerParam(profileActualPulses_, calculateCurrentPulse(numPoints, startPulses, endPulses, numPulses));
    }
    callParamCallbacks();
    unlock();
  }
  
  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s: profile move is done\n", driverName, functionName);
  
  // Confirm that the motors are done moving?
  
  lock();
  setIntegerParam(profileExecuteState_, PROFILE_EXECUTE_FLYBACK);
  callParamCallbacks();
  unlock();
  
  /* move motors to the end position */
  positionStr.str("");
  positionStr.clear();
  if (moveMode == PROFILE_MOVE_MODE_ABSOLUTE)
  {
    cmd << "PTP/m ";
  }
  else
  {
    cmd << "PTP/mr ";
  }
  // Create the comma-separated list of final positions
  for (j=0; j<profileAxes_.size(); j++)
  {
    pAxis = getAxis(profileAxes_[j]);
    
    position = pAxis->profileFlybackPos_;
    
    if (profileAxes_[j] == profileAxes_.front())
    {
      positionStr << position;
    }
    else 
    {
      positionStr << ',' << position;
    }
  }
  // Send the group move command
  cmd << axesToString(profileAxes_) << ", " << positionStr.str();
  status = pComm_->writeReadAck(cmd);

  // Wait for the motors to get there
  wakeupPoller();
  waitMotors();
  
  if (halted_)
  {
    aborted = true;
    executeOK = false;
    strcpy(message, "Aborted during flyback");
    goto done;
  }
  
  done:
  lock();
  if (executeOK)    executeStatus = PROFILE_STATUS_SUCCESS;
  else if (aborted) executeStatus = PROFILE_STATUS_ABORT;
  else              executeStatus = PROFILE_STATUS_FAILURE;
  setIntegerParam(profileExecuteStatus_, executeStatus);
  setStringParam(profileExecuteMessage_, message);
  if (!executeOK) {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
              "%s:%s: %s\n",
              driverName, functionName, message);
  }
  /* Clear execute command.  This is a "busy" record, don't want to do this until build is complete. */
  setIntegerParam(profileExecute_, 0);
  setIntegerParam(profileExecuteState_, PROFILE_EXECUTE_DONE);
  callParamCallbacks();
  halted_ = false;
  unlock();
  return executeOK ? asynSuccess : asynError; 
}

asynStatus SPiiPlusController::waitMotors()
{
  unsigned int j;
  SPiiPlusAxis* pAxis;
  int moving;
  static const char *functionName = "waitMotors";
  
  while (1) {
    epicsThreadSleep(0.1);
    
    // assume no motors are moving
    moving = 0;
    for (j=0; j<profileAxes_.size(); j++)
    {
      pAxis = getAxis(profileAxes_[j]);
      moving |= pAxis->moving_;
    }
    if (moving == 0) break;
  }
  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s: motors are done moving\n", driverName, functionName);
  return asynSuccess;
}

asynStatus SPiiPlusController::stopDataCollection()
{
  asynStatus status;
  std::stringstream cmd;
  int i;
  int axesToRecord;
  // static const char *functionName = "stopDataCollection";  
  
  if (profileAxes_.size() > 8)
    axesToRecord = 8;
  else
    axesToRecord = profileAxes_.size();
  for (i=0; i<axesToRecord; i++)
  {
    cmd << "STOPDC/s " << profileAxes_[i];
    status = pComm_->writeReadAck(cmd);
  }
  
  return status;
}

asynStatus SPiiPlusController::stopPEG(int pulseAxis)
{
  asynStatus status;
  std::stringstream cmd;
  // static const char *functionName = "stopPEG";  
  
  cmd << "STOPPEG " << pulseAxis;
  status = pComm_->writeReadAck(cmd);
  
  return status;
}

/** Function to abort a profile. */
asynStatus SPiiPlusController::abortProfile()
{
  asynStatus status;
  std::stringstream cmd;
  int executeState;
  // static const char *functionName = "abortProfile";
  
  getIntegerParam(profileExecuteState_,   &executeState);
    
  if (executeState != PROFILE_EXECUTE_DONE)
  {
    cmd << "HALT " << axesToString(profileAxes_);
    status = pComm_->writeReadAck(cmd);
    
    halted_ = true;
  }
  
  return status;
}

asynStatus SPiiPlusController::readbackProfile()
{
  char message[MAX_MESSAGE_LEN];
  bool readbackOK=true;
  char* buffer=NULL;
  int readbackStatus;
  int status;
  int i; 
  unsigned int j;
  std::stringstream cmd;
  char var[MAX_MESSAGE_LEN];
  int bytesPerDim;
  SPiiPlusAxis* pAxis;
  static const char *functionName = "readbackProfile";

  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
            "%s:%s: entry\n",
            driverName, functionName);

  strcpy(message, "");
  setStringParam(profileReadbackMessage_, message);
  setIntegerParam(profileReadbackState_, PROFILE_READBACK_BUSY);
  setIntegerParam(profileReadbackStatus_, PROFILE_STATUS_UNDEFINED);
  callParamCallbacks();
  
  if (profileAxes_.size() == 0)
  {
    strcpy(message, "No axes selected");
    readbackOK = false;
    goto done;
  }
  
  asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s: axisList = %s\n", driverName, functionName, axesToString(profileAxes_).c_str());
  sprintf(message, "Selected axes: %s", motorsToString(profileAxes_).c_str()); 
  setStringParam(profileReadbackMessage_, message);
  callParamCallbacks();
  
  /* Erase the readback and error arrays */
  for (i=0; i<numAxes_; i++) {
    memset(pAxes_[i]->profileReadbacks_,       0, maxProfilePoints_*sizeof(double));
    memset(pAxes_[i]->profileFollowingErrors_, 0, maxProfilePoints_*sizeof(double));
  }
  
  buffer = (char *)calloc(MAX_BINARY_READ_LEN, sizeof(char));
  
  for (j=0; j<profileAxes_.size(); j++)
  {
    pAxis = getAxis(j);
    
    sprintf(var, "DC_DATA_%i", j+1);
    status = pComm_->getDoubleArray(buffer, var, 0, 2, 0, (maxProfilePoints_-1));
    if (status != asynSuccess)
    {
      readbackOK = false;
      goto done;
    }
    
    bytesPerDim = maxProfilePoints_ * sizeof(double);
    // Position
    memcpy(pAxis->profileReadbacks_, buffer, bytesPerDim);
    // Postion error
    memcpy(pAxis->profileFollowingErrors_, buffer+bytesPerDim, bytesPerDim);
    // Time
    //memcpy(pAxis->profileTimes_, buffer+bytesPerDim*2, bytesPerDim);
  }
  
  done:
  if (buffer) free(buffer);
  setIntegerParam(profileNumReadbacks_, maxProfilePoints_);
  /* Convert from controller to user units and post the arrays */
  for (i=0; i<numAxes_; i++) {
    pAxes_[i]->readbackProfile();
  }
  readbackStatus = readbackOK ?  PROFILE_STATUS_SUCCESS : PROFILE_STATUS_FAILURE;
  setIntegerParam(profileReadbackStatus_, readbackStatus);
  setStringParam(profileReadbackMessage_, message);
  if (!readbackOK) {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
              "%s:%s: %s\n",
              driverName, functionName, message);
  }
  /* Clear readback command.  This is a "busy" record, don't want to do this until readback is complete. */
  setIntegerParam(profileReadback_, 0);
  setIntegerParam(profileReadbackState_, PROFILE_READBACK_DONE);
  callParamCallbacks();
  return status ? asynError : asynSuccess; 
  
  return asynSuccess;
}

asynStatus SPiiPlusController::test()
{
  asynStatus status;
  char* buffer=NULL;
  double* data=NULL;
  long maxDoubles;
  long dataSize;
  int i;
  static const char *functionName = "test";
  
  asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: calling test function\n", driverName, functionName);
  
  buffer = (char *)calloc(MAX_BINARY_READ_LEN, sizeof(char));
  
  // MAX_BINARY_READ_LEN is in bytes so we need to calculate how many doubles that will hold
  maxDoubles = floorl(MAX_BINARY_READ_LEN/sizeof(double));
  data = (double *)calloc(maxDoubles, sizeof(double));
  
  //status = pComm_->getDoubleArray(buffer, "DC_DATA_1", 0, 2, 0, (maxProfilePoints_-1));
  
  // Create test data
  //dataSize = 101;
  dataSize = 2000;
  for (i=0; i<dataSize; i++)
  {
      data[i] = i * 1.0;
  }
  
  /*
  // create larger test data
  dataSize = 100000;
  buffer = (char *)calloc(dataSize*sizeof(double), sizeof(char));
  maxDoubles = dataSize;
  data = (double *)calloc(maxDoubles, sizeof(double));
  
  for (i=0; i<dataSize; i++)
  {
      data[i] = i * 1.0;
  }
  */
  
  // Note: it is assumed that data has enough values to fill the specified array
  status = pComm_->putDoubleArray(data, "testVar", 0, dataSize-1, 0, 0);
  /*
  // 2D testing
  // This results in a real(500)(4) array and the data from the 11th packet doesn't get appended properly
  //status = pComm_->putDoubleArray(data, "testVar", 0, dataSize/4-1, 0, 4-1);
  // This results in a real(4)(500) array and the data from the 11th packet doesn't get appended properly
  //status = pComm_->putDoubleArray(data, "testVar", 0, 4-1, 0, dataSize/4-1);
  */
  
  free(buffer);
  
  return status;
}


/** Reports on status of the driver
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * If details > 0 then information is printed about each axis.
  * After printing controller-specific information it calls asynMotorController::report()
  */
void SPiiPlusController::report(FILE *fp, int level)
{
  fprintf(fp, "====================================\n");
  fprintf(fp, "SPiiPlus motor driver:\n");
  fprintf(fp, "    asyn port: %s\n", this->portName);
  fprintf(fp, "    num axes: %i\n", numAxes_);
  fprintf(fp, "    moving poll period: %lf\n", movingPollPeriod_);
  fprintf(fp, "    idle poll period: %lf\n", idlePollPeriod_);
  fprintf(fp, "    firmware version: %s\n", firmwareVersion_);
  fprintf(fp, "    virtual feedback position support: %s\n", virtualFeedbackPositionSupported_ ? "Yes" : "No");
  fprintf(fp, "\n");
  
  // Print more axis detail if level = 1
  // Print all the asyn parameters if level > 1
  if (level > 0)
    level -= 1;
  
  // Call the base class method
  asynMotorController::report(fp, level-1);
  fprintf(fp, "====================================\n");
}

static void AcsMotionConfig(const char* acs_port, const char* asyn_port, int num_axes, double moving_rate, double idle_rate, const char* virtual_axis_list)
{
	new SPiiPlusController(acs_port, asyn_port, num_axes, moving_rate, idle_rate, virtual_axis_list);
}


extern "C"
{

asynStatus SPiiPlusCreateProfile(const char *SPiiPlusName,         /* specify which controller by port name */
                            int maxPoints,               /* maximum number of profile points */
                            int maxPulses)               /* maximum number of profile pulses */
{
  SPiiPlusController *pC;
  static const char *functionName = "SPiiPlusCreateProfile";

  pC = (SPiiPlusController*) findAsynPortDriver(SPiiPlusName);
  if (!pC) {
    printf("%s:%s: Error port %s not found\n",
           driverName, functionName, SPiiPlusName);
    return asynError;
  }
  pC->lock();
  pC->initializeProfile(maxPoints, maxPulses);
  pC->unlock();
  return asynSuccess;
}

// Profile Setup arguments
static const iocshArg SPiiPlusCreateProfileArg0 = {"ACS port name", iocshArgString};
static const iocshArg SPiiPlusCreateProfileArg1 = {"Max points", iocshArgInt};
static const iocshArg SPiiPlusCreateProfileArg3 = {"Max pulses", iocshArgInt};

static const iocshArg * const SPiiPlusCreateProfileArgs[3] = {&SPiiPlusCreateProfileArg0, &SPiiPlusCreateProfileArg1, &SPiiPlusCreateProfileArg1};

static const iocshFuncDef configSPiiPlusProfile = {"SPiiPlusCreateProfile", 3, SPiiPlusCreateProfileArgs};

static void configSPiiPlusProfileCallFunc(const iocshArgBuf *args)
{
    SPiiPlusCreateProfile(args[0].sval, args[1].ival, args[2].ival);
}

// ACS Setup arguments
static const iocshArg configArg0 = {"ACS port name", iocshArgString};
static const iocshArg configArg1 = {"asyn port name", iocshArgString};
static const iocshArg configArg2 = {"num axes", iocshArgInt};
static const iocshArg configArg3 = {"Moving polling rate", iocshArgDouble};
static const iocshArg configArg4 = {"Idle polling rate", iocshArgDouble};
static const iocshArg configArg5 = {"Virtual axis list", iocshArgString};

static const iocshArg * const AcsMotionConfigArgs[6] = {&configArg0, &configArg1, &configArg2, &configArg3, &configArg4, &configArg5};

static const iocshFuncDef configAcsMotion = {"AcsMotionConfig", 6, AcsMotionConfigArgs};

static void AcsMotionCallFunc(const iocshArgBuf *args)
{
    AcsMotionConfig(args[0].sval, args[1].sval, args[2].ival, args[3].dval, args[4].dval, args[5].sval);
}

// Register all the commands users can call from the command line
static void AcsMotionRegister(void)
{
	iocshRegister(&configAcsMotion, AcsMotionCallFunc);
	iocshRegister(&configSPiiPlusProfile, configSPiiPlusProfileCallFunc);
}

epicsExportRegistrar(AcsMotionRegister);

}
