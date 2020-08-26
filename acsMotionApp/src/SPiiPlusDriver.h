#include <string>

#include "asynMotorController.h"
#include "asynMotorAxis.h"

#define SPIIPLUS_MAX_AXES 64
#define MAX_MESSAGE_LEN   256

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
	double profilePreDistance_;
	double profilePostDistance_;
	int moving_;
	
friend class SPiiPlusController;
};

class epicsShareClass SPiiPlusController : public asynMotorController
{
public:
	SPiiPlusController(const char* ACSPort, const char* asynPort, int numAxes, double moving_poll, double idle_poll);
	
	SPiiPlusAxis* getAxis(asynUser* pasynUser);
	SPiiPlusAxis* getAxis(int axisNo);
	void report(FILE *fp, int level);
	asynStatus writeread(const char* format, ...);
	
	/* These are functions for profile moves */
	asynStatus buildProfile();
	asynStatus executeProfile();
	asynStatus abortProfile();
	
	/* These are the methods that are new to this class */
	void profileThread();
	asynStatus runProfile();
	
protected:
	SPiiPlusAxis **pAxes_;       /**< Array of pointers to axis objects */
	std::string instring;
	
private:
	std::string axesToString(std::vector <int> axes);
	std::string positionsToString(int positionIndex);
	int parseInt();
	double parseDouble();
	asynStatus waitMotors();
	
	epicsEventId profileExecuteEvent_;
	std::vector <int> profileAxes_;
	
friend class SPiiPlusAxis;
};
