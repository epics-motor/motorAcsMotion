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

static const char *driverName = "SPiiPlusController";

static void SPiiPlusProfileThreadC(void *pPvt);

#ifndef MAX
#define MAX(a,b) ((a)>(b)? (a): (b))
#endif
#ifndef MIN
#define MIN(a,b) ((a)<(b)? (a): (b))
#endif

SPiiPlusController::SPiiPlusController(const char* ACSPortName, const char* asynPortName, int numAxes,
                                             double movingPollPeriod, double idlePollPeriod)
 : asynMotorController(ACSPortName, numAxes, 0, 0, 0, ASYN_CANBLOCK | ASYN_MULTIDEVICE, 1, 0, 0)
{
	asynStatus status = pasynOctetSyncIO->connect(asynPortName, 0, &pasynUserController_, NULL);
	pAxes_ = (SPiiPlusAxis **)(asynMotorController::pAxes_);
	
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
	
	// Create the event that wakes up the thread for profile moves
	profileExecuteEvent_ = epicsEventMustCreate(epicsEventEmpty);
	
	// Create the thread that will execute profile moves
	epicsThreadCreate("SPiiPlusProfile", 
		epicsThreadPriorityLow,
		epicsThreadGetStackSize(epicsThreadStackMedium),
		(EPICSTHREADFUNC)SPiiPlusProfileThreadC, (void *)this);
	
	// TODO: should this be an arg to the controller creation call or a separate call like it is in the XPS driver
	// hard-code the max number of points for now
	initializeProfile(2000);
}

SPiiPlusAxis* SPiiPlusController::getAxis(asynUser *pasynUser)
{
	return static_cast<SPiiPlusAxis*>(asynMotorController::getAxis(pasynUser));
}

SPiiPlusAxis* SPiiPlusController::getAxis(int axisNo)
{
	return static_cast<SPiiPlusAxis*>(asynMotorController::getAxis(axisNo));
}

asynStatus SPiiPlusController::writeReadInt(std::stringstream& cmd, int* val)
{
	static const char *functionName = "writeReadInt";
	char inString[MAX_CONTROLLER_STRING_SIZE];
	std::stringstream val_convert;
	
	std::fill(inString, inString + 256, '\0');
	
	size_t response;
	asynStatus status = this->writeReadController(cmd.str().c_str(), inString, 256, &response, -1);
	
	asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s: output = %s\n", driverName, functionName, cmd.str().c_str());
	asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s:  input = %s\n", driverName, functionName, inString);
	asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s: status = %i\n", driverName, functionName, status);
	
	if (status != asynSuccess) return status;
	if (inString[0] == '?') return asynError;
	
	// inString ends with \r:\r, but that isn't a problem for the following conversion
	val_convert << std::string(inString);
	val_convert >> *val;
	
	asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s:    val = %i\n", driverName, functionName, *val);
	
	// clear the command stringstream
	cmd.str("");
	cmd.clear();
	
	return status;
}

asynStatus SPiiPlusController::writeReadDouble(std::stringstream& cmd, double* val)
{
	static const char *functionName = "writeReadDouble";
	char inString[MAX_CONTROLLER_STRING_SIZE];
	std::stringstream val_convert;
	
	std::fill(inString, inString + 256, '\0');
	
	size_t response;
	asynStatus status = this->writeReadController(cmd.str().c_str(), inString, 256, &response, -1);
	
	asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s: output = %s\n", driverName, functionName, cmd.str().c_str());
	asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s:  input = %s\n", driverName, functionName, inString);
	asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s: status = %i\n", driverName, functionName, status);
	
	if (status != asynSuccess) return status;
	if (inString[0] == '?') return asynError;
	
	// inString ends with \r:\r, but that isn't a problem for the following conversion
	val_convert << std::string(inString);
	val_convert >> *val;
	
	asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s:    val = %lf\n", driverName, functionName, *val);
	
	// clear the command stringstream -- this doesn't work
	cmd.str("");
	cmd.clear();
	
	return status;
}

asynStatus SPiiPlusController::writeReadAck(std::stringstream& cmd)
{
	static const char *functionName = "writeReadAck";
	char inString[MAX_CONTROLLER_STRING_SIZE];
	
	std::fill(inString, inString + 256, '\0');
	
	size_t response;
	asynStatus status = this->writeReadController(cmd.str().c_str(), inString, 256, &response, -1);
	
	asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s: output = %s\n", driverName, functionName, cmd.str().c_str());
	asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s:  input = %s\n", driverName, functionName, inString);
	asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s: status = %i\n", driverName, functionName, status);
	
	if (status != asynSuccess) return status;
	if (inString[0] == '?') return asynError;
	
	// clear the command stringstream
	cmd.str("");
	cmd.clear();
	
	return status;
}


// The following parse methods would be better as a template method, but that might break VxWorks compatibility
int SPiiPlusController::parseInt()
{
	int out_val;
	std::stringstream val_convert;
	
	instring.replace(0,1," ");
	
	val_convert << instring;
	val_convert >> out_val;
	
	return out_val;
}

double SPiiPlusController::parseDouble()
{
	double out_val;
	std::stringstream val_convert;
	
	instring.replace(0,1," ");
	
	val_convert << instring;
	val_convert >> out_val;
	
	return out_val;
}

SPiiPlusAxis::SPiiPlusAxis(SPiiPlusController *pC, int axisNo)
: asynMotorAxis(pC, axisNo),
  pC_(pC)
{
	setIntegerParam(pC->motorStatusHasEncoder_, 1);
	// Gain Support is required for setClosedLoop to be called
	setIntegerParam(pC->motorStatusGainSupport_, 1);
}

asynStatus SPiiPlusAxis::poll(bool* moving)
{
	asynStatus status;
	SPiiPlusController* controller = (SPiiPlusController*) pC_;	
	static const char *functionName = "poll";
	std::stringstream cmd;
	
	double position;
	cmd << "?APOS(" << axisNo_ << ")";
	status = controller->writeReadDouble(cmd, &position);
	if (status != asynSuccess) return status;
	setDoubleParam(controller->motorPosition_, position);
	
	double enc_position;
	cmd << "?FPOS(" << axisNo_ << ")";
	status = controller->writeReadDouble(cmd, &enc_position);
	if (status != asynSuccess) return status;
	setDoubleParam(controller->motorEncoderPosition_, enc_position);
	
	int left_limit, right_limit;
	cmd << "?FAULT(" << axisNo_ << ").#LL";
	status = controller->writeReadInt(cmd, &left_limit);
	if (status != asynSuccess) return status;
	setIntegerParam(controller->motorStatusLowLimit_, left_limit);
	cmd << "?FAULT(" << axisNo_ << ").#RL";
	status = controller->writeReadInt(cmd, &right_limit);
	if (status != asynSuccess) return status;
	setIntegerParam(controller->motorStatusHighLimit_, right_limit);
	
	// Read the entire motor status and parse the relevant bits
	int axis_status;
	cmd << "?D/MST(" << axisNo_ << ")";
	status = controller->writeReadInt(cmd, &axis_status);
	if (status != asynSuccess) return status;
	
	int enabled;
	//int open_loop;
	//int in_pos;
	int motion;
	
	enabled = axis_status & (1<<0);
	//open_loop = axis_status & (1<<1);
	//in_pos = axis_status & (1<<4);
	motion = axis_status & (1<<5);
	
	//asynPrint(pC_->pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s: axis %i status: %i\n", driverName, functionName, axisNo_, axis_status);
	//asynPrint(pC_->pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s: axis %i motion: %i\n", driverName, functionName, axisNo_, motion);
	
	setIntegerParam(controller->motorStatusDone_, !motion);
	setIntegerParam(controller->motorStatusMoving_, motion);
	setIntegerParam(controller->motorStatusPowerOn_, enabled);
	
	callParamCallbacks();
	
	moving_ = *moving;
	
	if (motion)    { *moving = true; }
	else           { *moving = false; }
	
	return status;
}

asynStatus SPiiPlusAxis::move(double position, int relative, double minVelocity, double maxVelocity, double acceleration)
{
	SPiiPlusController* controller = (SPiiPlusController*) pC_;
	asynStatus status;
	std::stringstream cmd;
	
	cmd << "XACC(" << axisNo_ << ")=" << (acceleration + 10);
	status = controller->writeReadAck(cmd);
	cmd << "ACC(" << axisNo_ << ")=" << acceleration;
	status = controller->writeReadAck(cmd);
	cmd << "DEC(" << axisNo_ << ")=" << acceleration;
	status = controller->writeReadAck(cmd);
	
	cmd << "XVEL(" << axisNo_ << ")=" << (maxVelocity + 10);
	status = controller->writeReadAck(cmd);
	cmd << "VEL(" << axisNo_ << ")=" << maxVelocity;
	status = controller->writeReadAck(cmd);
	
	if (relative)
	{
		cmd << "PTP/r " << axisNo_ << ", " << position;
		status = controller->writeReadAck(cmd);
	}
	else
	{
		cmd << "PTP " << axisNo_ << ", " << position;
		status = controller->writeReadAck(cmd);
	}
	
	return status;
}

asynStatus SPiiPlusAxis::setPosition(double position)
{
	SPiiPlusController* controller = (SPiiPlusController*) pC_;
	asynStatus status;
	std::stringstream cmd;
	
	cmd << "SET APOS(" << axisNo_ << ")=" << position;
	status = controller->writeReadAck(cmd);
	
	return status;
}

asynStatus SPiiPlusAxis::stop(double acceleration)
{
	SPiiPlusController* controller = (SPiiPlusController*) pC_;
	asynStatus status;
	std::stringstream cmd;
	
	cmd << "HALT " << axisNo_;
	status = controller->writeReadAck(cmd);
	
	return status;
}

/** Set the motor closed loop status. 
  * \param[in] closedLoop true = close loop, false = open looop. */
asynStatus SPiiPlusAxis::setClosedLoop(bool closedLoop)
{
	SPiiPlusController* controller = (SPiiPlusController*) pC_;
	asynStatus status;
	std::stringstream cmd;
	
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
	status = controller->writeReadAck(cmd);
	
	return status;
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
  if (level > 0) {
    fprintf(fp, "Configuration for axis %i:\n", axisNo_);
    fprintf(fp, "  Moving: %i\n", moving_);
    fprintf(fp, "\n");
  }
  
  // Call the base class method
  asynMotorAxis::report(fp, level);
}

std::string SPiiPlusController::axesToString(std::vector <int> axes)
{
  static const char *functionName = "axesToString";
  uint i;
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

std::string SPiiPlusController::positionsToString(int positionIndex)
{
  static const char *functionName = "positionsToString";
  uint i;
  SPiiPlusAxis *pAxis;
  std::stringstream outputStr;
  
  for (i=0; i<profileAxes_.size(); i++)
  {
    pAxis = getAxis(i);
    
    if (profileAxes_[i] == profileAxes_.front())
    {
      outputStr << round(pAxis->profilePositions_[positionIndex]);
    }
    else 
    {
      outputStr << ',' << round(pAxis->profilePositions_[positionIndex]);
    }
  }
  
  return outputStr.str();
}

/** Function to build a coordinated move of multiple axes. */
asynStatus SPiiPlusController::buildProfile()
{
  int i; 
  uint j; 
  int status;
  bool buildOK=true;
  //bool verifyOK=true;
  int numPoints;
  //int numElements;
  //double trajVel;
  //double D0, D1, T0, T1;
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
  //int axis
  std::string axisList;
  int useAxis;
  std::stringstream cmd;
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
  
  // 
  profileAxes_.clear();
  
  for (i=0; i<numAxes_; i++) {
    // Zero the velocity arrays
    preVelocity[i] = 0.;
    postVelocity[i] = 0.;
    // Check which axes should be used
    getIntegerParam(i, profileUseAxis_, &useAxis);
    asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s: %i axis will be used: %i\n", driverName, functionName, i, useAxis);
    if (useAxis)
    {
      profileAxes_.push_back(i);
    }
  }
  
  axisList = axesToString(profileAxes_);
  asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s: axisList = %s\n", driverName, functionName, axisList.c_str());
  
  // TODO: how should an empty axis list be handled?
  
  /* We create trajectories with an extra element at the beginning and at the end.
   * The distance and time of the first element is defined so that the motors will
   * accelerate from 0 to the velocity of the first "real" element at their 
   * maximum allowed acceleration.
   * Similarly, the distance and time of last element is defined so that the 
   * motors will decelerate from the velocity of the last "real" element to 0 
   * at the maximum allowed acceleration. */

  preTimeMax = 0.;
  postTimeMax = 0.;
  getIntegerParam(profileNumPoints_, &numPoints);
  
  for (j=0; j<profileAxes_.size(); j++)
  {
    // Query the max velocity and acceleration
    cmd << "?XVEL(" << j << ")";
    status = writeReadDouble(cmd, &maxVelocity);
    if (status) {
      buildOK = false;
      sprintf(message, "Error getting XVEL, status=%d\n", status);
      goto done;
    }
    cmd << "?XACC(" << j << ")";
    status = writeReadDouble(cmd, &maxAcceleration);
    if (status) {
      buildOK = false;
      sprintf(message, "Error getting XACC, status=%d\n", status);
      goto done;
    }
    
    /* The calculation using maxAcceleration read from controller below
     * is "correct" but subject to roundoff errors when sending ASCII commands.
     * Reduce acceleration 10% to account for this. */
    maxAcceleration *= 0.9;
    preDistance = pAxes_[j]->profilePositions_[1] - pAxes_[j]->profilePositions_[0];
    preVelocity[j] = preDistance/profileTimes_[0];
    preTime = fabs(preVelocity[j]) / maxAcceleration;
    preTimeMax = MAX(preTimeMax, preTime);
    postDistance = pAxes_[j]->profilePositions_[numPoints-1] - 
               pAxes_[j]->profilePositions_[numPoints-2];
    postVelocity[j] = postDistance/profileTimes_[numPoints-1];
    postTime = fabs(postVelocity[j]) / maxAcceleration;
    postTimeMax = MAX(postTimeMax, postTime);
    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
              "%s:%s: axis %d profilePositions[0]=%f, profilePositions[%d]=%f, maxAcceleration=%f, preTimeMax=%f, postTimeMax=%f\n",
              driverName, functionName, j, pAxes_[j]->profilePositions_[0], numPoints-1, pAxes_[j]->profilePositions_[numPoints-1],
              maxAcceleration, preTimeMax, postTimeMax);
  }
    
  for (j=0; j<profileAxes_.size(); j++)
  {
    pAxes_[j]->profilePreDistance_  =  0.5 * preVelocity[j]  * preTimeMax; 
    pAxes_[j]->profilePostDistance_ =  0.5 * postVelocity[j] * postTimeMax; 
  }  
  
  // POINT commands have this syntax: POINT (0,1,5), 1000,2000,3000, 500
  
  // Verfiy the profile (check speed, accel, limit violations)
  
  done:
  // Can't fail if nothing is verified
  buildStatus = PROFILE_STATUS_SUCCESS;
  setIntegerParam(profileBuildStatus_, buildStatus);
  setStringParam(profileBuildMessage_, message);
  if (buildStatus != PROFILE_STATUS_SUCCESS) {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
              "%s:%s: %s\n",
              driverName, functionName, message);
  }
  /* Clear build command.  This is a "busy" record, don't want to do this until build is complete. */
  setIntegerParam(profileBuild_, 0);
  setIntegerParam(profileBuildState_, PROFILE_BUILD_DONE);
  callParamCallbacks();
  //return status ? asynError : asynSuccess;
  return asynSuccess;
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
  //bool executeOK=true;
  //bool aborted=false;
  int startPulses, endPulses;
  //int lastTime;
  int numPoints, numElements, numPulses;
  //int executeStatus;
  //double pulsePeriod;
  double position;
  //double time;
  //int i;
  uint j;
  int moveMode;
  char message[MAX_MESSAGE_LEN];
  //char buffer[MAX_GATHERING_STRING];
  std::string positions;
  std::stringstream positionStr;
  std::stringstream commandStr;
  std::stringstream cmd;
  //int eventId;
  SPiiPlusAxis *pAxis;
  int ptExecIdx;
  int ptLoadedIdx;
  int ptFree;
  int ptIdx;
  static const char *functionName = "runProfile";

  lock();
  getIntegerParam(profileStartPulses_, &startPulses);
  getIntegerParam(profileEndPulses_,   &endPulses);
  getIntegerParam(profileNumPoints_,   &numPoints);
  getIntegerParam(profileNumPulses_,   &numPulses);
  
  strcpy(message, " ");
  setStringParam(profileExecuteMessage_, message);
  setIntegerParam(profileExecuteState_, PROFILE_EXECUTE_MOVE_START);
  setIntegerParam(profileExecuteStatus_, PROFILE_STATUS_UNDEFINED);
  callParamCallbacks();
  unlock();
  
  /* move motors to the starting position */
  getIntegerParam(profileMoveMode_, &moveMode);
  for (j=0; j<profileAxes_.size(); j++)
  {
    pAxis = getAxis(profileAxes_[j]);
    if (moveMode == PROFILE_MOVE_MODE_ABSOLUTE)
    {
      // calculate the absolute starting position
      position = pAxis->profilePositions_[0] - pAxis->profilePreDistance_;
    }
    else
    {
      // calculate the relative starting position
      position = -pAxis->profilePreDistance_;
    }
    
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
  if (moveMode == PROFILE_MOVE_MODE_ABSOLUTE)
  {
    cmd << "PTP/m ";
  }
  else
  {
    cmd << "PTP/mr ";
  }
  cmd << axesToString(profileAxes_) << ", " << positionStr.str();
  status = writeReadAck(cmd);

  // Wait for the motors to get there
  wakeupPoller();
  waitMotors();

  lock();
  setIntegerParam(profileExecuteState_, PROFILE_EXECUTE_EXECUTING);
  callParamCallbacks();
  unlock();

  // configure data recording

  // configure pulse output

  // start data recording?

  // wake up poller
  
  /* run the trajectory */
  
  ptLoadedIdx = 0;
  ptExecIdx = 0;
  
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
  status = writeReadAck(cmd);
  
  // Fill the point buffer, which can only hold 50 points
  for (ptIdx = 0; ptIdx < MIN(50, numPoints); ptIdx++)
  {
    // Create and send the point command (should this be ptIdx+1?)
    cmd << "POINT " << axesToString(profileAxes_) << ", " << positionsToString(ptIdx) << ", "<< round(profileTimes_[ptIdx] * 1000.0);
    status = writeReadAck(cmd);
    // Increment the counter of points that have been loaded
    ptLoadedIdx++;
  }
  
  if (numPoints > 50)
  {
    // Send the GO command
    cmd << "GO " << axesToString(profileAxes_);
    status = writeReadAck(cmd);
    
    while (ptLoadedIdx < numPoints)
    {
      // Sleep for a short period of time
      epicsThreadSleep(0.1);
      
      // Query the number of free points in the buffer (the first axis in the vector is the lead axis)
      cmd << "?GSFREE(" << profileAxes_[0] << ")";
      status = writeReadInt(cmd, &ptFree);
      
      // Increment the counter of points that have been executed
      ptExecIdx += ptFree;
      
      // load the rest of the points as needed
      for (ptIdx=ptLoadedIdx; ptIdx<(ptLoadedIdx+ptFree); ptIdx++)
      {
        // Create and send the point command (should this be ptIdx+1?)
        cmd << "POINT " << axesToString(profileAxes_) << ", " << positionsToString(ptIdx) << ", "<< round(profileTimes_[ptIdx] * 1000.0);
        status = writeReadAck(cmd);
      }
      
      // Increment the counter of points that have been loaded
      ptLoadedIdx += ptFree;
    }
    
    // End the point sequence
    cmd << "ENDS " << axesToString(profileAxes_);
    status = writeReadAck(cmd);
  }
  else
  {
    // End the point sequence
    cmd << "ENDS " << axesToString(profileAxes_);
    status = writeReadAck(cmd);
    
    // Send the GO command
    cmd << "GO " << axesToString(profileAxes_);
    status = writeReadAck(cmd);
  }
  
  // Wait for the remaining points to be executed
  while (ptExecIdx < numPoints)
  {
    // Sleep for a short period of time
    epicsThreadSleep(0.1);
    
    // Query the number of free points in the buffer
    cmd << "?GSFREE(" << profileAxes_[0] << ")";
    status = writeReadInt(cmd, &ptFree);
    
    // Update the number of points that have been executed
    ptExecIdx = numPoints - 50 + ptFree;
  }
  
  // Confirm that the motors are done moving?
  
  // Move motors to the real end position
  
  // cleanup

  return asynSuccess;
}

asynStatus SPiiPlusController::waitMotors()
{
  uint j;
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

/** Function to abort a profile. */
asynStatus SPiiPlusController::abortProfile()
{
  // static const char *functionName = "abortProfile";
  // TODO
  return asynSuccess;
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
  fprintf(fp, "\n");
  
  // Call the base class method
    asynMotorController::report(fp, level);
  fprintf(fp, "====================================\n");
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
