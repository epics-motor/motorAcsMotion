#include <cstring>


//#include "SPiiPlusBinComm.h"
// SPiiPlusDriver.h includes SPiiPlusCommDriver.h
#include "SPiiPlusDriver.h"

// The index of AOUT and AIN command is a signal
#define MAX_SIGNALS 256
// The index of OUT and IN commands is a port
#define MAX_PORTS 256
// Each OUT/IN port has 32 bits
#define MAX_BITS 32

// Digital I/O parameters
#define digitalInputString        "DIGITAL_INPUT"
#define digitalOutputString       "DIGITAL_OUTPUT"
#define analogInputString         "ANALOG_INPUT"
#define analogOutputString        "ANALOG_OUTPUT"

class epicsShareClass SPiiPlusAuxIO : public asynPortDriver {
public:
  SPiiPlusAuxIO(const char *auxIOPortName, const char* asynPortName, int numChannels, double pollPeriod);
  ~SPiiPlusAuxIO();
  
  /* These are the methods that we override from asynPortDriver */
  //virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
  //virtual asynStatus readInt32(asynUser *pasynUser, epicsInt32 *value);
  virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
  virtual asynStatus writeUInt32Digital(asynUser *pasynUser, epicsUInt32 value, epicsUInt32 mask);
  //virtual void report(FILE *fp, int details);
  // These should be private but are called from C
  virtual void pollerThread(void);

  /* These are methods unique to SPiiPlusAuxIO */
  asynStatus writeBits(epicsUInt32 chan, epicsUInt32 mask, epicsUInt32 value);
  asynStatus writeAnalog(epicsUInt32 chan, epicsFloat64 value);

protected:
  SPiiPlusComm *pComm_;
  // Digital I/O parameters
  int digitalInput_;
  #define FIRST_SPIIPLUS_AUXIO_PARAM  digitalInput_
  int digitalOutput_;
  int analogInput_;
  int analogOutput_;
  #define LAST_SPIIPLUS_AUXIO_PARAM analogOutput_

private:
  SPiiPlusController *pC_;
  double pollPeriod_;
  int forceCallback_;
  double ain_[MAX_SIGNALS];
  double aout_[MAX_SIGNALS];
  int in_[MAX_PORTS];
  int prev_in_[MAX_PORTS];
  int out_[MAX_PORTS];
  int prev_out_[MAX_PORTS];
  char buffer_[MAX_BINARY_READ_LEN];
};

#define NUM_PARAMS ((int)(&LAST_SPIIPLUS_AUXIO_PARAM - &FIRST_SPIIPLUS_AUXIO_PARAM + 1))
