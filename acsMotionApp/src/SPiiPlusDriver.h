#include <string>

#include "asynMotorController.h"
#include "asynMotorAxis.h"

#include "SPiiPlusCommDriver.h"

#define SPIIPLUS_MAX_AXES 64
#define SPIIPLUS_MAX_DC_AXES 8
#define SPIIPLUS_CMD_TIMEOUT 0.05
#define SPIIPLUS_ACK_TIMEOUT 0.2
#define SPIIPLUS_ARRAY_TIMEOUT 10.0
#define MAX_MESSAGE_LEN   256
#define MAX_ACCEL_SEGMENTS 20

// Maximum number of bytes that can be returned by a binary read
#define MAX_BINARY_READ_LEN 65536
#define MAX_BINARY_WRITE_LEN 65536

// The following values need to match the homingMethod mbbo record
#define MBBO_HOME_NONE			0
#define MBBO_HOME_LIMIT_INDEX		1
#define MBBO_HOME_LIMIT			2
#define MBBO_HOME_INDEX			3
#define MBBO_HOME_CURRENT_POS		4
#define MBBO_HOME_HARDSTOP_INDEX	5
#define MBBO_HOME_HARDSTOP		6
#define MBBO_HOME_CUSTOM		7
//
#define SPIIPLUS_HOME_NONE			0
#define SPIIPLUS_HOME_NEG_LIMIT_INDEX		1
#define SPIIPLUS_HOME_POS_LIMIT_INDEX		2
#define SPIIPLUS_HOME_NEG_LIMIT			17
#define SPIIPLUS_HOME_POS_LIMIT			18
#define SPIIPLUS_HOME_NEG_INDEX			33
#define SPIIPLUS_HOME_POS_INDEX			34
#define SPIIPLUS_HOME_CURRENT_POS		37
#define SPIIPLUS_HOME_NEG_HARDSTOP_INDEX	50
#define SPIIPLUS_HOME_POS_HARDSTOP_INDEX	51
#define SPIIPLUS_HOME_NEG_HARDSTOP		52
#define SPIIPLUS_HOME_POS_HARDSTOP		53
//
#define SPIIPLUS_FAULT_HARD_RIGHT_LIMIT		1<<0
#define SPIIPLUS_FAULT_HARD_LEFT_LIMIT		1<<1
#define SPIIPLUS_FAULT_NETWORK_ERROR		1<<2
#define SPIIPLUS_FAULT_MOTOR_OVERHEAT		1<<4
#define SPIIPLUS_FAULT_SOFT_RIGHT_LIMIT		1<<5
#define SPIIPLUS_FAULT_SOFT_LEFT_LIMIT		1<<6
#define SPIIPLUS_FAULT_ENCODER_NC		1<<7
#define SPIIPLUS_FAULT_ENCODER_2_NC		1<<8
#define SPIIPLUS_FAULT_DRIVE_ALARM		1<<9
#define SPIIPLUS_FAULT_ENCODER_ERROR		1<<10
#define SPIIPLUS_FAULT_ENCODER_2_ERROR		1<<11
#define SPIIPLUS_FAULT_POSITION_ERROR		1<<12
#define SPIIPLUS_FAULT_CRITICAL_POS_ERROR	1<<13
#define SPIIPLUS_FAULT_VELOCITY_LIMIT		1<<14
#define SPIIPLUS_FAULT_ACCELERATION_LIMIT	1<<15
#define SPIIPLUS_FAULT_CURRENT_LIMIT		1<<16
#define SPIIPLUS_FAULT_SERVO_PROC_ALARM		1<<17
#define SPIIPLUS_FAULT_SAFE_TORQUE_OFF		1<<18
#define SPIIPLUS_FAULT_HSSI_NC			1<<20
//
#define SPIIPLUS_MFLAGS_DUMMY               (1<<0)
#define SPIIPLUS_MFLAGS_OPEN                (1<<1)
#define SPIIPLUS_MFLAGS_MICRO               (1<<2)
#define SPIIPLUS_MFLAGS_HOME                (1<<3)
#define SPIIPLUS_MFLAGS_STEPPER             (1<<4)
#define SPIIPLUS_MFLAGS_ENCLOOP             (1<<5)
#define SPIIPLUS_MFLAGS_STEPENC             (1<<6)
#define SPIIPLUS_MFLAGS_BRUSHL              (1<<8)
#define SPIIPLUS_MFLAGS_BRUSHOK             (1<<9)
#define SPIIPLUS_MFLAGS_PHASE2              (1<<10)
#define SPIIPLUS_MFLAGS_DEFCON              (1<<17)
#define SPIIPLUS_MFLAGS_LINEAR              (1<<21)
#define SPIIPLUS_MFLAGS_ABSCOMM             (1<<22)
#define SPIIPLUS_MFLAGS_HALL                (1<<27)
//
#define SPIIPLUS_AXIS_STATUS_LEAD       1<<0
#define SPIIPLUS_AXIS_STATUS_PEG        1<<2
#define SPIIPLUS_AXIS_STATUS_DC         1<<3
#define SPIIPLUS_AXIS_STATUS_PEGREADY   1<<4
#define SPIIPLUS_AXIS_STATUS_MOVE       1<<5
#define SPIIPLUS_AXIS_STATUS_ACC        1<<6
#define SPIIPLUS_AXIS_STATUS_BUILDUP    1<<7
#define SPIIPLUS_AXIS_STATUS_VELLOCK    1<<8
#define SPIIPLUS_AXIS_STATUS_POSLOCK    1<<9
#define SPIIPLUS_AXIS_STATUS_TRIGGER    1<<11
#define SPIIPLUS_AXIS_STATUS_NEWSEGM    1<<16
#define SPIIPLUS_AXIS_STATUS_STARV      1<<17
#define SPIIPLUS_AXIS_STATUS_ENCWARN    1<<18
#define SPIIPLUS_AXIS_STATUS_ENC2WARN   1<<19
#define SPIIPLUS_AXIS_STATUS_INRANGE    1<<20
#define SPIIPLUS_AXIS_STATUS_LCTICKLE   1<<21
#define SPIIPLUS_AXIS_STATUS_LCMODUL    1<<22
#define SPIIPLUS_AXIS_STATUS_FOLLOWED   1<<23
#define SPIIPLUS_AXIS_STATUS_HOLD       1<<24
#define SPIIPLUS_AXIS_STATUS_INHOMING   1<<25
#define SPIIPLUS_AXIS_STATUS_DECOMPON   1<<26
#define SPIIPLUS_AXIS_STATUS_INSHAPE    1<<27
#define SPIIPLUS_AXIS_STATUS_ENCPROC    1<<29


// drvInfo strings for extra parameters that the XPS controller supports
#define SPiiPlusHomingMethodString             "SPIIPLUS_HOMING_METHOD"
#define SPiiPlusMaxVelocityString              "SPIIPLUS_MAX_VELOCITY"
#define SPiiPlusMaxAccelerationString          "SPIIPLUS_MAX_ACCELERATION"
#define SPiiPlusReadIntVarString               "SPIIPLUS_READ_INT_VAR"
#define SPiiPlusWriteIntVarString              "SPIIPLUS_WRITE_INT_VAR"
#define SPiiPlusReadRealVarString              "SPIIPLUS_READ_REAL_VAR"
#define SPiiPlusWriteRealVarString             "SPIIPLUS_WRITE_REAL_VAR"
#define SPiiPlusStartProgramString             "SPIIPLUS_START_"
#define SPiiPlusStopProgramString              "SPIIPLUS_STOP_"
#define SPiiPlusSafeTorqueOffString            "SPIIPLUS_SAFE_TORQUE_OFF"
#define SPiiPlusHomingProcedureDoneString      "SPIIPLUS_HOMING_DONE"
//
#define SPiiPlusStepFactorString               "SPIIPLUS_STEP_FACTOR"
#define SPiiPlusEncTypeString                  "SPIIPLUS_ENC_TYPE"
#define SPiiPlusEnc2TypeString                 "SPIIPLUS_ENC2_TYPE"
#define SPiiPlusEncFactorString                "SPIIPLUS_ENC_FACTOR"
#define SPiiPlusEnc2FactorString               "SPIIPLUS_ENC2_FACTOR"
//
#define SPiiPlusAxisPosString                  "SPIIPLUS_AXIS_POS"
#define SPiiPlusRefPosString                   "SPIIPLUS_REF_POS"
#define SPiiPlusEncPosString                   "SPIIPLUS_ENC_POS"
#define SPiiPlusFdbkPosString                  "SPIIPLUS_FDBK_POS"
#define SPiiPlusFdbk2PosString                 "SPIIPLUS_FDBK2_POS"
#define SPiiPlusVFdbkPosString                 "SPIIPLUS_VFDBK_POS"
//
#define SPiiPlusRefOffsetString                "SPIIPLUS_REF_OFFSET"
#define SPiiPlusEncOffsetString                "SPIIPLUS_ENC_OFFSET"
#define SPiiPlusEnc2OffsetString               "SPIIPLUS_ENC2_OFFSET"
#define SPiiPlusAbsEncOffsetString             "SPIIPLUS_ABS_ENC_OFFSET"
#define SPiiPlusAbsEnc2OffsetString            "SPIIPLUS_ABS_ENC2_OFFSET"
//
#define SPiiPlusEncFaultString                 "SPIIPLUS_ENC_FAULT"
#define SPiiPlusEnc2FaultString                "SPIIPLUS_ENC2_FAULT"
//
#define SPiiPlusSetEncOffsetString             "SPIIPLUS_SET_ENC_OFFSET"
#define SPiiPlusSetEnc2OffsetString            "SPIIPLUS_SET_ENC2_OFFSET"
//
#define SPiiPlusFWVersionString                "SPIIPLUS_FW_VERSION"
#define SPiiPlusVFdbkPosSupportString          "SPIIPLUS_VFDBK_POS_SUPPORT"
//
#define SPiiPlusHomingMaxDistString            "SPIIPLUS_HOMING_MAX_DIST"
#define SPiiPlusHomingOffsetPosString          "SPIIPLUS_HOMING_OFFSET_POS"
#define SPiiPlusHomingOffsetNegString          "SPIIPLUS_HOMING_OFFSET_NEG"
#define SPiiPlusHomingCurrLimitString          "SPIIPLUS_HOMING_CURR_LIMIT"
//
#define SPiiPlusDisableSetPosString            "SPIIPLUS_DISABLE_SET_POS"
//
#define SPiiPlusPulseModeString                "SPIIPLUS_PULSE_MODE"
#define SPiiPlusPulsePosString                 "SPIIPLUS_PULSE_POS"
#define SPiiPlusNumPulsesString                "SPIIPLUS_NUM_PULSES"
//
#define SPiiPlusPulseAxisString                "SPIIPLUS_PULSE_AXIS"
#define SPiiPlusPEGEngEncCodeString            "SPIIPLUS_PEG_ENG_ENC_CODE"
#define SPiiPlusPEGOutAssignCodeString         "SPIIPLUS_PEG_OUT_ASSIGN_CODE"
#define SPiiPlusPOUTSOutputIndexString         "SPIIPLUS_POUTS_OUTPUT_INDEX"
#define SPiiPlusPOUTSBitCodeString             "SPIIPLUS_POUTS_BIT_CODE"
#define SPiiPlusPulseWidthString               "SPIIPLUS_PULSE_WIDTH"
//
#define SPiiPlusTestString                     "SPIIPLUS_TEST"

struct SPiiPlusDrvUser_t {
    const char *programName;
    int              len;
};

class epicsShareClass SPiiPlusAxis : public asynMotorAxis
{
public:
	SPiiPlusAxis(class SPiiPlusController *pC, int axisNo);
	void report(FILE *fp, int level);
	
	asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
	asynStatus moveVelocity(double minVelocity, double maxVelocity, double acceleration);
	asynStatus home(double minVelocity, double maxVelocity, double acceleration, int forwards);
	asynStatus stop(double acceleration);
	asynStatus poll(bool *moving);
	asynStatus setPosition(double position);
	asynStatus setClosedLoop(bool closedLoop);
	asynStatus defineProfile(double *positions, size_t numPoints);
	asynStatus readbackProfile(size_t numPoints);
	
	asynStatus getMaxParams();
	asynStatus updateFeedbackParams();
	asynStatus setMaxVelocity(double maxVelocity);
	asynStatus setMaxAcceleration(double maxAcceleration);
	asynStatus setEncoderOffset(double newEncoderOffset);
	asynStatus setEncoder2Offset(double newEncoder2Offset);
	
	asynStatus correctProfile(size_t numPoints);
	
private:
	SPiiPlusController *pC_;	/**< Pointer to the asynMotorController to which this axis belongs.
				*   Abbreviated because it is used very frequently */
	double profileAccelPositions_[MAX_ACCEL_SEGMENTS];  /**< Array of target positions for acceleration of profile moves */
	double profileDecelPositions_[MAX_ACCEL_SEGMENTS];  /**< Array of target positions for deceleration of profile moves */
	double *fullProfilePositions_;                      /**< Array of target positions for profile moves */
	double *profilePositionsUser_;
	double profilePreDistance_;
	double profilePostDistance_;
	double profileStartPos_;
	double profileFlybackPos_;
	int moving_;
	int pegReady_;      // AST, bit 4
	int dummy_;			// MFLAGS, bit 0
	int open_;			// MFLAGS, bit 1
	int micro_;			// MFLAGS, bit 2
	int home_;              // MFLAGS, bit 3
	int stepper_;			// MFLAGS, bit 4
	int encloop_;			// MFLAGS, bit 5
	int stepenc_;			// MFLAGS, bit 6
	int brushl_;			// MFLAGS, bit 8
	int brushok_;			// MFLAGS, bit 9
	int phase2_;			// MFLAGS, bit 10
	int defcon_;			// MFLAGS, bit 17
	int linear_;			// MFLAGS, bit 21
	int abscomm_;			// MFLAGS, bit 22
	int hall_;			// MFLAGS, bit 27
	double resolution_;		// STEPF
	bool virtual_;			// Is virtual axis
	
friend class SPiiPlusController;
};

class epicsShareClass SPiiPlusController : public asynMotorController
{
public:
	SPiiPlusController(const char* ACSPort, const char* asynPort, int numAxes, double moving_poll, double idle_poll, const char* virtualAxisList);
	asynStatus readInt32(asynUser *pasynUser, epicsInt32 *value);
	asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
	asynStatus readFloat64(asynUser *pasynUser, epicsFloat64 *value);
	asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
	asynStatus writeFloat64Array(asynUser *pasynUser, epicsFloat64 *value, size_t nElements);
	asynStatus getAddress(asynUser *pasynUser, int *address);
	asynStatus drvUserCreate(asynUser *pasynUser, const char *drvInfo, const char **pptypeName, size_t *psize);
	asynStatus drvUserDestroy(asynUser *pasynUser);
	SPiiPlusAxis* getAxis(asynUser* pasynUser);
	SPiiPlusAxis* getAxis(int axisNo);
	asynStatus poll();
	void report(FILE *fp, int level);
	
	/* These are functions for profile moves */
	asynStatus initializeProfile(size_t maxProfilePoints, size_t maxProfilePulses);
	asynStatus buildProfile();
	asynStatus executeProfile();
	asynStatus abortProfile();
	asynStatus readbackProfile();
	
	/* These are the methods that are new to this class */
	void profileThread();
	void assembleFullProfile(int numPoints);
	void sanityCheckProfile();
	void createAccDecTimes(double preTimeMax, double postTimeMax);
	void createAccDecPositions(SPiiPlusAxis* axis, int moveMode, int numPoints, double preTimeMax, double postTimeMax, double preVelocity, double postVelocity);
	asynStatus definePulses(int pulseAxis, int moveMode, size_t numPulses);
	asynStatus runProfile();
	int getNumAccelSegments(double time);
	long int calculateCurrentPulse(int currentPoint, int startPulse, int endPulse, int numPulses, int pulseMode);
	asynStatus readGlobalIntVar(asynUser *pasynUser, epicsInt32 *value);
	asynStatus writeGlobalIntVar(asynUser *pasynUser, epicsInt32 value);
	asynStatus readGlobalRealVar(asynUser *pasynUser, epicsFloat64 *value);
	asynStatus writeGlobalRealVar(asynUser *pasynUser, epicsFloat64 value);
	asynStatus startProgram(asynUser *pasynUser, epicsFloat64 value);
	asynStatus stopProgram(asynUser *pasynUser, epicsFloat64 value);
	
protected:
	SPiiPlusAxis **pAxes_;       /**< Array of pointers to axis objects */
	SPiiPlusComm *pComm_;
	std::string instring;
	bool virtualFeedbackPositionSupported_;
	bool anyAxisVirtual_;
	
	#define FIRST_SPIIPLUS_PARAM SPiiPlusHomingMethod_
	int SPiiPlusHomingMethod_;
	int SPiiPlusMaxVelocity_;
	int SPiiPlusMaxAcceleration_;
	int SPiiPlusReadIntVar_;
	int SPiiPlusWriteIntVar_;
	int SPiiPlusReadRealVar_;
	int SPiiPlusWriteRealVar_;
	int SPiiPlusStartProgram_;
	int SPiiPlusStopProgram_;
	int SPiiPlusSafeTorqueOff_;
	int SPiiPlusHomingProcedureDone_;
	//
	int SPiiPlusStepFactor_;
	int SPiiPlusEncType_;
	int SPiiPlusEnc2Type_;
	int SPiiPlusEncFactor_;
	int SPiiPlusEnc2Factor_;
	//
	int SPiiPlusAxisPos_;
	int SPiiPlusRefPos_;
	int SPiiPlusEncPos_;
	int SPiiPlusFdbkPos_;
	int SPiiPlusFdbk2Pos_;
	int SPiiPlusVFdbkPos_;
	//
	int SPiiPlusRefOffset_;
	int SPiiPlusEncOffset_;
	int SPiiPlusEnc2Offset_;
	int SPiiPlusAbsEncOffset_;
	int SPiiPlusAbsEnc2Offset_;
	//
	int SPiiPlusEncFault_;
	int SPiiPlusEnc2Fault_;
	//
	int SPiiPlusSetEncOffset_;
	int SPiiPlusSetEnc2Offset_;
	//
	int SPiiPlusFWVersion_;
	int SPiiPlusVFdbkPosSupport_;
	//
	int SPiiPlusHomingMaxDist_;
	int SPiiPlusHomingOffsetPos_;
	int SPiiPlusHomingOffsetNeg_;
	int SPiiPlusHomingCurrLimit_;
	//
	int SPiiPlusDisableSetPos_;
	//
	int SPiiPlusPulseMode_;
	int SPiiPlusPulsePos_;
	int SPiiPlusNumPulses_;
	//
	int SPiiPlusPulseAxis_;
	int SPiiPlusPEGEngEncCode_;
	int SPiiPlusPEGOutAssignCode_;
	int SPiiPlusPOUTSOutputIndex_;
	int SPiiPlusPOUTSBitCode_;
	int SPiiPlusPulseWidth_;
	//
	int SPiiPlusTest_;
	#define LAST_SPIIPLUS_PARAM SPiiPlusTest_
	
	
private:
	SPiiPlusDrvUser_t *drvUser_;                          /** Drv user structure */
	bool initialized_;                                    /** If initialized successfully */
	double profileAccelTimes_[MAX_ACCEL_SEGMENTS];        /**< Array of times per profile acceleration point */
	double profileDecelTimes_[MAX_ACCEL_SEGMENTS];        /**< Array of times per profile deceleration point */
	double *fullProfileTimes_;                            /**< Array of times per profile point */
	int fullProfileSize_;
	std::string axesToString(std::vector <int> axes);
	std::string motorsToString(std::vector <int> axes);
	std::string positionsToString(int positionIndex);
	std::string accelPositionsToString(int positionIndex);
	std::string decelPositionsToString(int positionIndex);
	asynStatus waitMotors();
	void calculateDataCollectionInterval();
	asynStatus stopDataCollection();
	asynStatus stopPEG(int pulseAxis);
	asynStatus test();
	char firmwareVersion_[MAX_MESSAGE_LEN];
	
	epicsEventId profileExecuteEvent_;
	std::vector <int> profileAxes_;
	int numAccelSegments_;
	int numDecelSegments_;
	double dataCollectionInterval_;
	bool halted_;
	epicsFloat64 stepperFactor_[SPIIPLUS_MAX_AXES];
	epicsFloat64 encoderFactor_[SPIIPLUS_MAX_AXES];
	epicsFloat64 encoder2Factor_[SPIIPLUS_MAX_AXES];
	epicsFloat64 axisPosition_[SPIIPLUS_MAX_AXES];
	epicsFloat64 encoderPosition_[SPIIPLUS_MAX_AXES];
	epicsFloat64 referencePosition_[SPIIPLUS_MAX_AXES];
	epicsFloat64 feedbackPosition_[SPIIPLUS_MAX_AXES];
	epicsFloat64 feedback2Position_[SPIIPLUS_MAX_AXES];
	epicsFloat64 virtualFeedbackPosition_[SPIIPLUS_MAX_AXES];
	epicsFloat64 referenceOffset_[SPIIPLUS_MAX_AXES];
	epicsFloat64 encoderOffset_[SPIIPLUS_MAX_AXES];
	epicsFloat64 encoder2Offset_[SPIIPLUS_MAX_AXES];
	epicsFloat64 absoluteEncoderOffset_[SPIIPLUS_MAX_AXES];
	epicsFloat64 absoluteEncoder2Offset_[SPIIPLUS_MAX_AXES];
	epicsFloat64 maxVelocity_[SPIIPLUS_MAX_AXES];
	epicsFloat64 maxAcceleration_[SPIIPLUS_MAX_AXES];
	epicsInt32 motorFlags_[SPIIPLUS_MAX_AXES];
	epicsInt32 faultStatus_[SPIIPLUS_MAX_AXES];
	epicsInt32 encoderFault_[SPIIPLUS_MAX_AXES];
	epicsInt32 encoder2Fault_[SPIIPLUS_MAX_AXES];
	epicsInt32 axisStatus_[SPIIPLUS_MAX_AXES];
	epicsInt32 motorStatus_[SPIIPLUS_MAX_AXES];
	epicsInt32 encoderType_[SPIIPLUS_MAX_AXES];
	epicsInt32 encoder2Type_[SPIIPLUS_MAX_AXES];
	
	size_t maxProfilePulses_;
	double *profilePulses_;
	double *profilePulsesUser_;
	double *profilePulsePositions_;
	double pulseStartPos_;
	double pulseSpacing_;
	double pulseEndPos_;
	int numPulses_;
	
friend class SPiiPlusAxis;
friend class SPiiPlusComm;
};
#define NUM_SPIIPLUS_PARAMS ((int)(&LAST_SPIIPLUS_PARAM - &FIRST_SPIIPLUS_PARAM + 1))
