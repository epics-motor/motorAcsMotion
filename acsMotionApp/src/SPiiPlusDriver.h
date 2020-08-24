#include <string>

#include "asynMotorController.h"
#include "asynMotorAxis.h"

#define SPIIPLUS_MAX_AXES 64
#define MAX_MESSAGE_LEN   256

class epicsShareClass SPiiPlusAxis : public asynMotorAxis
{
public:
	SPiiPlusAxis(class SPiiPlusController *pC, int axisNo);
	
	asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
	asynStatus stop(double acceleration);
	asynStatus poll(bool *moving);
	asynStatus setPosition(double position);
	asynStatus setClosedLoop(bool closedLoop);
	
friend class SPiiPlusController;
};

class epicsShareClass SPiiPlusController : public asynMotorController
{
public:
	SPiiPlusController(const char* ACSPort, const char* asynPort, int numAxes, double moving_poll, double idle_poll);
	
	SPiiPlusAxis* getAxis(asynUser* pasynUser);
	SPiiPlusAxis* getAxis(int axisNo);
	
	asynStatus writeread(const char* format, ...);
	
	/* These are functions for profile moves */
	asynStatus buildProfile();
	asynStatus executeProfile();
	asynStatus abortProfile();
	
	/* These are the methods that are new to this class */
	void profileThread();
	asynStatus runProfile();
	
protected:
	std::string instring;
	
private:
	epicsEventId profileExecuteEvent_;
	std::vector <int> profileAxes_;
	
friend class SPiiPlusAxis;
};
