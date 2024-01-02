
#include "asynDriver.h"

class SPiiPlusController;

class epicsShareClass SPiiPlusComm : public asynPortDriver {
public:
  SPiiPlusComm(const char *commPortName, const char* asynPortName, int numChannels);
  ~SPiiPlusComm();

  /* These are the methods that we override from asynPortDriver */
  //virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
  //virtual asynStatus readInt32(asynUser *pasynUser, epicsInt32 *value);
  //virtual void report(FILE *fp, int details);
  // These should be private but are called from C
  void pollerThread(void);

  asynStatus writeReadController(const char *output, char *input, size_t maxChars, size_t *nread, double timeout);
  asynStatus writeReadInt(std::stringstream& cmd, int* val);
  asynStatus writeReadDouble(std::stringstream& cmd, double* val);
  asynStatus writeReadStr(std::stringstream& cmd, char* val);
  asynStatus writeReadAck(std::stringstream& cmd);
  asynStatus writeReadErrorMessage(char* errNoReply);
  asynStatus getIntegerArray(char *output, const char *var, int idx1start, int idx1end, int idx2start, int idx2end);
  asynStatus getDoubleArray(char *output, const char *var, int idx1start, int idx1end, int idx2start, int idx2end);
  asynStatus writeReadBinary(char *output, int outBytes, char *input, int inBytes, size_t *dataBytes, bool* sliceAvailable);
  asynStatus binaryErrorCheck(char *buffer);

protected:
  //int something_;
  //#define FIRST_SPIIPLUS_COMM_PARAM  digitalOutput_

  //#define LAST_SPIIPLUS_COMM_PARAM digitalOutput_

  asynUser *pasynUserComm_;
  //char outString_[MAX_CONTROLLER_STRING_SIZE];
  //char inString_[MAX_CONTROLLER_STRING_SIZE];

private:
  SPiiPlusController *pC_;
  double pollPeriod_;
  int forceCallback_;

//friend class SPiiPlusController;
};

//#define NUM_SPIIPLUS_COMM_PARAM ((int)(&LAST_SPIIPLUS_COMM_PARAM - &FIRST_SPIIPLUS_COMM_PARAM + 1))
#define NUM_SPIIPLUS_COMM_PARAM 0
