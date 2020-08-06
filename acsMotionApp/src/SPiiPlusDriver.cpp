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

#include <asynOctetSyncIO.h>

#include "asynDriver.h"
#include "asynMotorController.h"
#include "asynMotorAxis.h"

#include <epicsExport.h>

#include "SPiiPlusDriver.h"

SPiiPlusController::SPiiPlusController(const char* ACSPortName, const char* asynPortName, int numAxes,
                                             double movingPollPeriod, double idlePollPeriod)
 : asynMotorController(ACSPortName, numAxes, 0, 0, 0, ASYN_CANBLOCK | ASYN_MULTIDEVICE, 1, 0, 0)
{
	asynStatus status = pasynOctetSyncIO->connect(asynPortName, 0, &pasynUserController_, NULL);
	
	if (status)
	{
		asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, 
		"SPiiPlusController::SPiiPlusController: cannot connect to SPii+ controller\n");
		
		return;
	}
	
	for (int index = 0; index < numAxes; index += 1)
	{
		new SPiiPlusAxis(this, index);
	}
	
	this->startPoller(movingPollPeriod, idlePollPeriod, 2);
}

SPiiPlusAxis* SPiiPlusController::getAxis(asynUser *pasynUser)
{
	return static_cast<SPiiPlusAxis*>(asynMotorController::getAxis(pasynUser));
}

SPiiPlusAxis* SPiiPlusController::getAxis(int axisNo)
{
	return static_cast<SPiiPlusAxis*>(asynMotorController::getAxis(axisNo));
}

asynStatus SPiiPlusController::writeread(const char* format, ...)
{
	va_list args;
	va_start(args, format);
	
	std::fill(outString_, outString_ + 256, '\0');
	std::fill(outString_, inString_ + 256, '\0');
	
	vsprintf(outString_, format, args);
	
	size_t response;
	asynStatus out = this->writeReadController(outString_, inString_, 256, &response, -1);
	
	this->instring = std::string(inString_);
	
	va_end(args);
	
	return out;
}

SPiiPlusAxis::SPiiPlusAxis(SPiiPlusController *pC, int axisNo)
: asynMotorAxis(pC, axisNo)
{
	setIntegerParam(pC->motorStatusHasEncoder_, 1);
}

asynStatus SPiiPlusAxis::poll(bool* moving)
{
	asynStatus status;
	SPiiPlusController* controller = (SPiiPlusController*) pC_;	
	
	status = controller->writeread("?APOS(%d)", axisNo_);
	controller->instring.replace(0,1," ");
	
	double position;
	std::stringstream pos_convert;

	pos_convert << controller->instring;
	pos_convert >> position;
	
	setDoubleParam(controller->motorPosition_, position);
	
	
	status = controller->writeread("?FPOS(%d)", axisNo_);
	controller->instring.replace(0,1," ");
	
	double enc_position;
	std::stringstream enc_pos_convert;	

	enc_pos_convert << controller->instring;
	enc_pos_convert >> enc_position;
	
	setDoubleParam(controller->motorEncoderPosition_, enc_position);
	
	
	int left_limit, right_limit;
	std::stringstream fault_convert;
	
	status = controller->writeread("?FAULT(%d).#LL", axisNo_);
	controller->instring.replace(0,1," ");
	
	fault_convert << controller->instring;
	fault_convert >> left_limit;
	
	status = controller->writeread("?FAULT(%d).#RL", axisNo_);
	controller->instring.replace(0,1," ");
	
	fault_convert.str("");
	fault_convert.clear();
	
	fault_convert << controller->instring;
	fault_convert >> right_limit;
	
	setIntegerParam(controller->motorStatusHighLimit_, right_limit);
	setIntegerParam(controller->motorStatusLowLimit_, left_limit);
	
	status = controller->writeread("?MST(%d).#MOVE", axisNo_);
	controller->instring.replace(0,1," ");
	
	int motion;
	std::stringstream moving_convert;
	
	moving_convert << controller->instring;
	moving_convert >> motion;
	
	setIntegerParam(controller->motorStatusDone_, !motion);
	setIntegerParam(controller->motorStatusMoving_, motion);
	
	callParamCallbacks();
	
	if (motion)    { *moving = false; }
	else           { *moving = true; }
	
	return status;
}

asynStatus SPiiPlusAxis::move(double position, int relative, double minVelocity, double maxVelocity, double acceleration)
{
	asynStatus status;
	SPiiPlusController* controller = (SPiiPlusController*) pC_;
	
	status = controller->writeread("XACC(%d)=%f", axisNo_, acceleration + 10);
	status = controller->writeread("ACC(%d)=%f", axisNo_, acceleration);
	status = controller->writeread("DEC(%d)=%f", axisNo_, acceleration);
	
	status = controller->writeread("XVEL(%d)=%f", axisNo_, maxVelocity + 10);
	status = controller->writeread("VEL(%d)=%f", axisNo_, maxVelocity);
	
	if (relative)
	{
		status = controller->writeread("PTP/r %d, %f", axisNo_, position);
	}
	else
	{
		status = controller->writeread("PTP %d, %f", axisNo_, position);
	}
	
	return status;
}

asynStatus SPiiPlusAxis::setPosition(double position)
{
	SPiiPlusController* controller = (SPiiPlusController*) pC_;
	return controller->writeread("SET %d_APOS=%f", axisNo_, position);
}

asynStatus SPiiPlusAxis::stop(double acceleration)
{
	SPiiPlusController* controller = (SPiiPlusController*) pC_;
	return controller->writeread("HALT %d", axisNo_);
}

static void AcsMotionConfig(const char* acs_port, const char* asyn_port, int num_axes, double moving_rate, double idle_rate)
{
	new SPiiPlusController(acs_port, asyn_port, num_axes, moving_rate, idle_rate);
}


extern "C"
{

// ACS Setup arguments
static const iocshArg configArg0 = {"ACS port name", iocshArgString};
static const iocshArg configArg1 = {"asyn port name", iocshArgString};
static const iocshArg configArg2 = {"num axes", iocshArgInt};
static const iocshArg configArg3 = {"Moving polling rate", iocshArgDouble};
static const iocshArg configArg4 = {"Idle polling rate", iocshArgDouble};

static const iocshArg * const AcsMotionConfigArgs[5] = {&configArg0, &configArg1, &configArg2, &configArg3, &configArg4};

static const iocshFuncDef configAcsMotion = {"AcsMotionConfig", 5, AcsMotionConfigArgs};

static void AcsMotionCallFunc(const iocshArgBuf *args)
{
    AcsMotionConfig(args[0].sval, args[1].sval, args[2].ival, args[3].dval, args[4].dval);
}

static void AcsMotionRegister(void)
{
	iocshRegister(&configAcsMotion, AcsMotionCallFunc);
}

epicsExportRegistrar(AcsMotionRegister);

}
