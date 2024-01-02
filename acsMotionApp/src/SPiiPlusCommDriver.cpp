
#include <string.h>
#include <cstdlib>
#include <sstream>

#include <iocsh.h>
#include <epicsThread.h>
#include <epicsExport.h>

#include <asynPortDriver.h>
#include <asynOctetSyncIO.h>

#include "SPiiPlusBinComm.h"
// SPiiPlusDriver.h includes SPiiPlusCommDriver.h
#include "SPiiPlusDriver.h"

static const char *driverName = "SPiiPlusComm";

SPiiPlusComm::SPiiPlusComm(const char *commPortName, const char* asynPortName, int numChannels)
  : asynPortDriver(commPortName, numChannels, 
      asynInt32Mask | asynUInt32DigitalMask | asynDrvUserMask,  // Interfaces that we implement
      asynUInt32DigitalMask,                                    // Interfaces that do callbacks
      ASYN_MULTIDEVICE | ASYN_CANBLOCK, 1, /* ASYN_CANBLOCK=1, ASYN_MULTIDEVICE=1, autoConnect=1 */
      0, 0),  /* Default priority and stack size */
    forceCallback_(1)
{
	// Can numChannels be zero for this class?
	
	// 
	asynStatus status = pasynOctetSyncIO->connect(asynPortName, 0, &pasynUserComm_, NULL);
	
	if (status)
	{
		asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, 
		"SPiiPlusComm::SPiiPlusComm: cannot connect to SPii+ controller\n");
		
		return;
	}
}


// This is needed to resolve a build error: undefined reference to `vtable for SPiiPlusComm'
SPiiPlusComm::~SPiiPlusComm()
{
	pasynOctetSyncIO->disconnect(pasynUserComm_);
}

// Note: This method is copied from asynMotorController.cpp
/** Writes a string to the controller and reads a response.
  * \param[in] output Pointer to the output string.
  * \param[out] input Pointer to the input string location.
  * \param[in] maxChars Size of the input buffer.
  * \param[out] nread Number of characters read.
  * \param[out] timeout Timeout before returning an error.*/
asynStatus SPiiPlusComm::writeReadController(const char *output, char *input, 
                                                    size_t maxChars, size_t *nread, double timeout)
{
  size_t nwrite;
  asynStatus status;
  int eomReason;
  // const char *functionName="writeReadController";
  
  status = pasynOctetSyncIO->writeRead(pasynUserComm_, output,
                                       strlen(output), input, maxChars, timeout,
                                       &nwrite, nread, &eomReason);
                        
  return status;
}

asynStatus SPiiPlusComm::writeReadInt(std::stringstream& cmd, int* val)
{
	static const char *functionName = "writeReadInt";
	char inString[MAX_CONTROLLER_STRING_SIZE];
	std::stringstream val_convert;

	std::fill(inString, inString + 256, '\0');
	
	asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s: output = %s\n", driverName, functionName, cmd.str().c_str());
	
	size_t response;
	lock();
	asynStatus status = writeReadController(cmd.str().c_str(), inString, 256, &response, -1);
	unlock();
	
	asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s:  input = %s\n", driverName, functionName, inString);
	asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s: status = %i\n", driverName, functionName, status);
	
	if (status == asynSuccess)
	{
		if (inString[0] != '?')
		{
			// inString ends with \r:\r, but that isn't a problem for the following conversion
			val_convert << std::string(inString);
			val_convert >> *val;
			
			asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s:    val = %i\n", driverName, functionName, *val);
		}
		else
		{
			asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Command failed: %s\n", driverName, functionName, cmd.str().c_str());
			
			// Query the controller for more detail about the error
			writeReadErrorMessage(inString);
			
			status = asynError;
		}
	}
	
	// clear the command stringstream
	cmd.str("");
	cmd.clear();
	
	return status;
}

asynStatus SPiiPlusComm::writeReadDouble(std::stringstream& cmd, double* val)
{
	static const char *functionName = "writeReadDouble";
	char inString[MAX_CONTROLLER_STRING_SIZE];
	std::stringstream val_convert;
	
	std::fill(inString, inString + 256, '\0');
	
	asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s: output = %s\n", driverName, functionName, cmd.str().c_str());
	
	size_t response;
	lock();
	asynStatus status = writeReadController(cmd.str().c_str(), inString, 256, &response, -1);
	unlock();
	
	asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s:  input = %s\n", driverName, functionName, inString);
	asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s: status = %i\n", driverName, functionName, status);
	
	
	if (status == asynSuccess)
	{
		if (inString[0] != '?')
		{
			// inString ends with \r:\r, but that isn't a problem for the following conversion
			val_convert << std::string(inString);
			val_convert >> *val;
			
			asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s:    val = %lf\n", driverName, functionName, *val);
		}
		else
		{
			asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Command failed: %s\n", driverName, functionName, cmd.str().c_str());
			
			// Query the controller for more detail about the error
			writeReadErrorMessage(inString);
			
			status = asynError;
		}
	}
	
	// clear the command stringstream -- this doesn't work
	cmd.str("");
	cmd.clear();
	
	return status;
}

asynStatus SPiiPlusComm::writeReadStr(std::stringstream& cmd, char* val)
{
	static const char *functionName = "writeReadStr";
	char inString[MAX_CONTROLLER_STRING_SIZE];
	long unsigned int idx;

	std::fill(inString, inString + 256, '\0');
	
	asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s: output = %s\n", driverName, functionName, cmd.str().c_str());
	
	size_t response;
	lock();
	asynStatus status = writeReadController(cmd.str().c_str(), inString, 256, &response, -1);
	unlock();
	
	asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s:  input = %s\n", driverName, functionName, inString);
	asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s: status = %i\n", driverName, functionName, status);
	
	if (status == asynSuccess)
	{
		if (inString[0] != '?')
		{
			// inString sometimes ends with \r:\r, copy until the first \r or null is found
			for (idx = 0; idx < strlen(inString); idx++)
			{
				if ((inString[idx] == '\0') || (inString[idx] == '\r'))
				{
					val[idx] = '\0';
					break;
				}
				else
				{
					val[idx] = inString[idx];
				} 
			}
			
			asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s:    val = %s\n", driverName, functionName, val);
		}
		else
		{
			asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Command failed: %s\n", driverName, functionName, cmd.str().c_str());
			
			// Query the controller for more detail about the error
			writeReadErrorMessage(inString);
			
			status = asynError;
		}
	}
	
	// clear the command stringstream
	cmd.str("");
	cmd.clear();
	
	return status;
}

asynStatus SPiiPlusComm::writeReadAck(std::stringstream& cmd)
{
	static const char *functionName = "writeReadAck";
	char inString[MAX_CONTROLLER_STRING_SIZE];
	std::stringstream val_convert;

	std::fill(inString, inString + 256, '\0');
	
	asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s: output = %s\n", driverName, functionName, cmd.str().c_str());
	
	size_t response;
	lock();
	asynStatus status = writeReadController(cmd.str().c_str(), inString, 256, &response, -1);
	unlock();
	
	asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s:  input = %s\n", driverName, functionName, inString);
	asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s: status = %i\n", driverName, functionName, status);
	
	if (inString[0] == '?')
	{
		asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Command failed: %s\n", driverName, functionName, cmd.str().c_str());
		
		// Query the controller for more detail about the error
		writeReadErrorMessage(inString);
		
		status = asynError;
	}
	
	// clear the command stringstream
	cmd.str("");
	cmd.clear();
	
	return status;
}

asynStatus SPiiPlusComm::writeReadErrorMessage(char* errNoReply)
{
	static const char *functionName = "writeReadErrorMessage";
	std::stringstream val_convert;
	std::stringstream local_cmd;
	char inString[MAX_CONTROLLER_STRING_SIZE];
	int errNo;
	
	/* errNoReply is of the form ?#### */
	
	std::fill(inString, inString + 256, '\0');
	
	// The command to query the error message is ??####
	local_cmd << "?" << errNoReply;
	
	// Overwrite the '?' so the conversion can succeed
	errNoReply[0] = ' ';
	val_convert << std::string(errNoReply);
	val_convert >> errNo;
	
	asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s: output = %s\n", driverName, functionName, local_cmd.str().c_str());
	
	size_t response;
	lock();
	asynStatus status = writeReadController(local_cmd.str().c_str(), inString, 256, &response, -1);
	unlock();
	
	asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s:  input = %s\n", driverName, functionName, inString);
	asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s: status = %i\n", driverName, functionName, status);
	
	if (inString[0] != '?')
	{
		asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: ERROR #%i: %s\n", driverName, functionName, errNo, inString);
	}
	else {
		// We should never get here unless a controller returns an error for which it doesn't have an error message defined
		asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: ERROR #%i\n", driverName, functionName, errNo);
		
		status = asynError;
	}
	
	return status;
}

// NOTE: readBytes the number of data bytes that were read, excluding the command header and suffix
// NOTE: there is no error checking on inBytes and outBytes
asynStatus SPiiPlusComm::writeReadBinary(char *output, int outBytes, char *input, int inBytes, size_t *dataBytes, bool *sliceAvailable)
{
	char outString[MAX_CONTROLLER_STRING_SIZE];
	char* packetBuffer;
	size_t nwrite, nread;
	int eomReason;
	asynStatus status;
	static const char *functionName = "writeReadBinary";
	
	asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s: start\n", driverName, functionName);
	
	lock();
	
	std::fill(outString, outString + MAX_CONTROLLER_STRING_SIZE, '\0');
	packetBuffer = (char *)calloc(MAX_PACKET_DATA+5, sizeof(char));
	
	// Clear the EOS characters
	pasynOctetSyncIO->setInputEos(pasynUserComm_, "", 0);
	pasynOctetSyncIO->setOutputEos(pasynUserComm_, "", 0);
	
	// Flush the receive buffer
	status = pasynOctetSyncIO->flush(pasynUserComm_);
	
	asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s: output bytes = %i, output = %s\n", driverName, functionName, outBytes, output);
	
	// Send the query command
	memcpy(outString, output, outBytes);
	status = pasynOctetSyncIO->write(pasynUserComm_, outString, outBytes, SPIIPLUS_CMD_TIMEOUT, &nwrite);
	
	// The reply from the controller has a 4-byte header and a 1-byte suffix
	status = pasynOctetSyncIO->read(pasynUserComm_, packetBuffer, inBytes, SPIIPLUS_ARRAY_TIMEOUT, &nread, &eomReason);
	
	asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s:  input bytes = %i\n", driverName, functionName, inBytes);
	asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s: status = %i\n", driverName, functionName, status);
	
	if (status == asynSuccess)
	{
		// Check for an error reply
		status = binaryErrorCheck(packetBuffer);
		if (status == asynError)
		{
			*sliceAvailable = false;
			asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Binary read failed (controller)\n", driverName, functionName);
		}
		else
		{
			// Check if there is another slice
			/*
			 * Bit 7 of the 3rd byte, which is the most-significant byte of the BE message size, indicates if another slice is available
			 */
			if (packetBuffer[3] & SLICE_AVAILABLE)
			{
				*sliceAvailable = true;
			}
			else
			{
				*sliceAvailable = false;
			}
			
			// Subtract the 5 header bytes to get the number of bytes in the data
			*dataBytes = nread - 5;
			
			// The data is already in little-endian format, so just copy it
			memcpy(input, packetBuffer+4, *dataBytes);
		}
	}
	else
	{
		*sliceAvailable = false;
		asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Binary read failed (asyn): status=%i, nread=%li\n", driverName, functionName, status, nread);
	}
	
	// Restore the EOS characters
	pasynOctetSyncIO->setInputEos(pasynUserComm_, "\r", 1);
	pasynOctetSyncIO->setOutputEos(pasynUserComm_, "\r", 1);

	// Free up allocated memory
	free(packetBuffer);
	
	unlock();
	
	asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s: end\n", driverName, functionName);
	
	return status;
	}

asynStatus SPiiPlusComm::binaryErrorCheck(char *buffer)
{
	asynStatus status=asynSuccess;
	std::stringstream val_convert;
	int errNo;
	static const char *functionName = "binaryErrorCheck";
	
	// If the first character of the data is a question mark, the error number follows it
	if ((buffer[4] == 0x3f) && (buffer[9] == 0x0d))
	{
		/*
		 *  Error response: [E3][XX][06][00]?####[0D][E6]
		 */
		 
		// replace the carriage return with a null byte
		buffer[9] = 0;
		
		// convert the error number bytes into an int
		val_convert << buffer+5;
		val_convert >> errNo;
		
		asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Binary command error #%i\n", driverName, functionName, errNo);
		status = asynError;
	}
	
	return status;
}

asynStatus SPiiPlusComm::getDoubleArray(char *output, const char *var, int idx1start, int idx1end, int idx2start, int idx2end)
{
	char outString[MAX_CONTROLLER_STRING_SIZE];
	char command[MAX_MESSAGE_LEN];
	asynStatus status;
	int remainingBytes;
	int readBytes;
	int outBytes, inBytes, dataBytes;
	size_t nread;
	int slice=1;
	bool sliceAvailable;
	static const char *functionName = "getDoubleArray";
	
	asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s: start\n", driverName, functionName);
	
	std::fill(outString, outString + MAX_CONTROLLER_STRING_SIZE, '\0');
	
	// Create the command to query array data. This could be the only command
	// that needs to be sent or it could be the first of many.
	readFloat64ArrayCmd(command, var, idx1start, idx1end, idx2start, idx2end, &outBytes, &inBytes, &dataBytes);
	
	remainingBytes = dataBytes;
	readBytes = 0;
	
	asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s: var = %s, ((%i, %i), (%i, %i))\n", driverName, functionName, var, idx1start, idx1end, idx2start, idx2end);
	
	// Send the command
	status = writeReadBinary((char*)command, outBytes, output+readBytes, inBytes, &nread, &sliceAvailable);
	asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s: Initial array query: request = %i; read = %li\n", driverName, functionName, inBytes, nread);
	
	asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s: status = %i\n", driverName, functionName, status);
	
	remainingBytes -= nread;
	readBytes += nread;
	
	// Look at the response to see if there are more slices to read
	while (sliceAvailable)
	{
		// Create the command to query the next slice of the array data
		readFloat64SliceCmd(command, slice, var, idx1start, idx1end, idx2start, idx2end, &outBytes, &inBytes, &dataBytes);
		
		asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s: var = %s, ((%i, %i), (%i, %i)), slice %i\n", driverName, functionName, var, idx1start, idx1end, idx2start, idx2end, slice);
		
		// Send the command
		status = writeReadBinary((char*)command, outBytes, output+readBytes, inBytes, &nread, &sliceAvailable);
		asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s: Array slice #%i query: expected = %i; read = %li; sliceAvailable = %d\n", driverName, functionName, slice, inBytes, nread, sliceAvailable);
		
		asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s: status = %i\n", driverName, functionName, status);
		
		remainingBytes -= nread;
		readBytes += nread;
		slice++;
	}
	
	asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s: end\n", driverName, functionName);
	
	return status;
}

asynStatus SPiiPlusComm::getIntegerArray(char *output, const char *var, int idx1start, int idx1end, int idx2start, int idx2end)
{
	char outString[MAX_CONTROLLER_STRING_SIZE];
	char command[MAX_MESSAGE_LEN];
	asynStatus status;
	int remainingBytes;
	int readBytes;
	int outBytes, inBytes, dataBytes;
	size_t nread;
	int slice=1;
	bool sliceAvailable;
	static const char *functionName = "getIntegerArray";
	
	asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s: start\n", driverName, functionName);
	
	std::fill(outString, outString + MAX_CONTROLLER_STRING_SIZE, '\0');
	
	// Create the command to query array data. This could be the only command
	// that needs to be sent or it could be the first of many.
	readInt32ArrayCmd(command, var, idx1start, idx1end, idx2start, idx2end, &outBytes, &inBytes, &dataBytes);
	
	remainingBytes = dataBytes;
	readBytes = 0;
	
	asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s: var = %s, ((%i, %i), (%i, %i))\n", driverName, functionName, var, idx1start, idx1end, idx2start, idx2end);
	
	// Send the command
	status = writeReadBinary((char*)command, outBytes, output+readBytes, inBytes, &nread, &sliceAvailable);
	asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s: Initial array query: request = %i; read = %li\n", driverName, functionName, inBytes, nread);
	
	asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s: status = %i\n", driverName, functionName, status);
	
	remainingBytes -= nread;
	readBytes += nread;
	
	// Look at the response to see if there are more slices to read
	while (sliceAvailable)
	{
		// Create the command to query the next slice of the array data
		readInt32SliceCmd(command, slice, var, idx1start, idx1end, idx2start, idx2end, &outBytes, &inBytes, &dataBytes);
		
		asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s: var = %s, ((%i, %i), (%i, %i)), slice %i\n", driverName, functionName, var, idx1start, idx1end, idx2start, idx2end, slice);
		
		// Send the command
		status = writeReadBinary((char*)command, outBytes, output+readBytes, inBytes, &nread, &sliceAvailable);
		asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s: Array slice #%i query: expected = %i; read = %li; sliceAvailable = %d\n", driverName, functionName, slice, inBytes, nread, sliceAvailable);
		
		asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s: status = %i\n", driverName, functionName, status);
		
		remainingBytes -= nread;
		readBytes += nread;
		slice++;
	}
	
	asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s: end\n", driverName, functionName);
	
	return status;
}

static void AcsMotionCommConfig(const char *commPortName, const char* asynPortName, int numChannels)
{
	new SPiiPlusComm(commPortName, asynPortName, numChannels);
}

extern "C"
{

/** Configuration command, called directly or from iocsh */
int SPiiPlusCommConfig(const char *auxIOPortName, const char* asynPortName, int numChannels)
{
  SPiiPlusComm *pSPiiPlusComm = new SPiiPlusComm(auxIOPortName, asynPortName, numChannels);
  pSPiiPlusComm = NULL;  /* This is just to avoid compiler warnings */
  return(asynSuccess);
}


static const iocshArg configArg0 = { "Comm port name", iocshArgString};
static const iocshArg configArg1 = { "Asyn port name",   iocshArgString};
static const iocshArg configArg2 = { "Num channels",     iocshArgInt};
static const iocshArg * const AcsMotionCommArgs[] = {&configArg0,
                                              &configArg1,
                                              &configArg2};
static const iocshFuncDef AcsMotionCommFuncDef = {"SPiiPlusComm", 3, AcsMotionCommArgs};
static void AcsMotionCommCallFunc(const iocshArgBuf *args)
{
  AcsMotionCommConfig(args[0].sval, args[1].sval, args[2].ival);
}

void AcsMotionCommRegister(void)
{
  iocshRegister(&AcsMotionCommFuncDef,AcsMotionCommCallFunc);
}

epicsExportRegistrar(AcsMotionCommRegister);

}
