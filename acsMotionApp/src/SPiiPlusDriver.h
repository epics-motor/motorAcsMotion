#include <string>

#include "asynMotorController.h"
#include "asynMotorAxis.h"

#define SPIIPLUS_MAX_AXES 64
#define SPIIPLUS_MAX_DC_AXES 8
#define SPIIPLUS_CMD_TIMEOUT 0.05
#define SPIIPLUS_ARRAY_TIMEOUT 5.0
#define MAX_MESSAGE_LEN   256
#define MAX_ACCEL_SEGMENTS 20

// Maximum number of bytes that can be returned by a binary read
#define MAX_BINARY_READ_LEN 65536
#define MAX_PACKET_SIZE 1405
#define MAX_PACKET_DATA 1400
//
#define FRAME_START 		0xd3
#define FRAME_END 		0xd6
#define INT_DATA_SIZE 		0x04
#define DOUBLE_DATA_SIZE	0x08
#define READ_D_ARRAY_CMD	0xf0
#define READ_I_ARRAY_CMD	0xf1
#define READ_LD_ARRAY_CMD	0x41
#define READ_LD_SLICE_CMD	0x42
#define READ_LI_ARRAY_CMD	0x44
#define READ_LI_SLICE_CMD	0x45
#define SLICE_AVAILABLE		1 << 7
/*
#define WRITE_I_ARRAY_CMD	0xf3
#define WRITE_D_ARRAY_CMD	0xf2
#define WRITE_LD_ARRAY_CMD	0x37
#define WRITE_LD_SLICE_CMD	0x38
#define WRITE_LD_END_CMD	0x39
#define WRITE_LI_ARRAY_CMD	0x3A
#define WRITE_LI_SLICE_CMD	0x3B
*/

// drvInfo strings for extra parameters that the XPS controller supports
#define SPiiPlusTestString                      "SPIIPLUS_TEST"

class epicsShareClass SPiiPlusAxis : public asynMotorAxis
{
public:
	SPiiPlusAxis(class SPiiPlusController *pC, int axisNo);
	void report(FILE *fp, int level);
	
	asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
	asynStatus stop(double acceleration);
	asynStatus poll(bool *moving);
	asynStatus setPosition(double position);
	asynStatus setClosedLoop(bool closedLoop);
	
	
private:
	SPiiPlusController *pC_;	/**< Pointer to the asynMotorController to which this axis belongs.
				*   Abbreviated because it is used very frequently */
	double profileAccelPositions_[MAX_ACCEL_SEGMENTS];  /**< Array of target positions for acceleration of profile moves */
	double profileDecelPositions_[MAX_ACCEL_SEGMENTS];  /**< Array of target positions for deceleration of profile moves */
	double *fullProfilePositions_;                      /**< Array of target positions for profile moves */
	double profilePreDistance_;
	double profilePostDistance_;
	double profileStartPos_;
	double profileFlybackPos_;
	int moving_;
	int mflags_;
	int dummy_;
	
friend class SPiiPlusController;
};

class epicsShareClass SPiiPlusController : public asynMotorController
{
public:
	SPiiPlusController(const char* ACSPort, const char* asynPort, int numAxes, double moving_poll, double idle_poll);
	asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
	SPiiPlusAxis* getAxis(asynUser* pasynUser);
	SPiiPlusAxis* getAxis(int axisNo);
	void report(FILE *fp, int level);
	asynStatus writeReadInt(std::stringstream& cmd, int* val);
	asynStatus writeReadDouble(std::stringstream& cmd, double* val);
	asynStatus writeReadAck(std::stringstream& cmd);
	asynStatus writeReadDoubleArray(std::stringstream& cmd, char* buffer, int numBytes);
	asynStatus writeReadBinary(char *output, int outBytes, char *input, int inBytes, size_t *readBytes);
	
	/* These are functions for profile moves */
	asynStatus initializeProfile(size_t maxProfilePoints);
	asynStatus buildProfile();
	asynStatus executeProfile();
	asynStatus abortProfile();
	asynStatus readbackProfile();
	
	/* These are the methods that are new to this class */
	void profileThread();
	void assembleFullProfile(int numPoints);
	void createAccDecTimes(double preTimeMax, double postTimeMax);
	void createAccDecPositions(SPiiPlusAxis* axis, int moveMode, int numPoints, double preTimeMax, double postTimeMax, double preVelocity, double postVelocity);
	asynStatus runProfile();
	int getNumAccelSegments(double time);
	
protected:
	SPiiPlusAxis **pAxes_;       /**< Array of pointers to axis objects */
	std::string instring;
	
	#define FIRST_SPIIPLUS_PARAM SPiiPlusTest_
	int SPiiPlusTest_;
	#define LAST_SPIIPLUS_PARAM SPiiPlusTest_
	
	
	
private:
	double profileAccelTimes_[MAX_ACCEL_SEGMENTS];        /**< Array of times per profile acceleration point */
	double profileDecelTimes_[MAX_ACCEL_SEGMENTS];        /**< Array of times per profile deceleration point */
	double *fullProfileTimes_;                            /**< Array of times per profile point */
	int fullProfileSize_;
	std::string axesToString(std::vector <int> axes);
	std::string motorsToString(std::vector <int> axes);
	std::string positionsToString(int positionIndex);
	std::string accelPositionsToString(int positionIndex);
	std::string decelPositionsToString(int positionIndex);
	int parseInt();
	double parseDouble();
	asynStatus waitMotors();
	void calculateDataCollectionInterval();
	asynStatus stopDataCollection();
	asynStatus test();
	
	epicsEventId profileExecuteEvent_;
	std::vector <int> profileAxes_;
	int numAccelSegments_;
	int numDecelSegments_;
	double dataCollectionInterval_;
	bool halted_;
	
friend class SPiiPlusAxis;
};
#define NUM_SPIIPLUS_PARAMS ((int)(&LAST_SPIIPLUS_PARAM - &FIRST_SPIIPLUS_PARAM + 1))
