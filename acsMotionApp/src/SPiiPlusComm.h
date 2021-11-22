
#include "asynDriver.h"

asynStatus writeReadInt(asynPortDriver *apd, std::stringstream& cmd, int* val);
asynStatus writeReadDouble(asynPortDriver *apd, std::stringstream& cmd, double* val);
asynStatus writeReadAck(asynPortDriver *apd, std::stringstream& cmd);
asynStatus getIntegerArray(asynPortDriver *apd, char *output, const char *var, int idx1start, int idx1end, int idx2start, int idx2end);
asynStatus getDoubleArray(asynPortDriver *apd, char *output, const char *var, int idx1start, int idx1end, int idx2start, int idx2end);
asynStatus writeReadBinary(asynPortDriver *apd, char *output, int outBytes, char *input, int inBytes, size_t *dataBytes, bool* sliceAvailable);
asynStatus binaryErrorCheck(asynPortDriver *apd, char *buffer);
