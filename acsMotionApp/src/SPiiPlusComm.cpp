#include <string.h>
#include <sstream>
#include <math.h>
#include <stdio.h>

#include "asynPortDriver.h"
#include "asynMotorController.h"

#include "SPiiPlusBinComm.h"
#include "SPiiPlusComm.h"

/*
 * The following functions communicate with the ACS controller and
 * are used by both the motor driver and the auxillary driver
 */

asynStatus writeReadInt(asynPortDriver *apd, std::stringstream& cmd, int* val)
{
	static const char *functionName = "writeReadInt";
	char inString[MAX_CONTROLLER_STRING_SIZE];
	std::stringstream val_convert;
	int errNo;

	std::fill(inString, inString + 256, '\0');
	
	asynPrint(apd->pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s: output = %s\n", driverName, functionName, cmd.str().c_str());
	
	size_t response;
	apd->lock();
	asynStatus status = apd->writeReadController(cmd.str().c_str(), inString, 256, &response, -1);
	apd->lock();
	
	asynPrint(apd->pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s:  input = %s\n", driverName, functionName, inString);
	asynPrint(apd->pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s: status = %i\n", driverName, functionName, status);
	
	if (status == asynSuccess)
	{
		if (inString[0] != '?')
		{
			// inString ends with \r:\r, but that isn't a problem for the following conversion
			val_convert << std::string(inString);
			val_convert >> *val;
			
			asynPrint(apd->pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s:    val = %i\n", driverName, functionName, *val);
		}
		else
		{
			// Overwrite the '?' so the conversion can succeed
			inString[0] = ' ';
			val_convert << std::string(inString);
			val_convert >> errNo;
			
			asynPrint(apd->pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: ERROR #%i - command: %s\n", driverName, functionName, errNo, cmd.str().c_str());
			
			status = asynError;
		}
	}
	
	// clear the command stringstream
	cmd.str("");
	cmd.clear();
	
	return status;
}

asynStatus writeReadDouble(asynPortDriver *apd, std::stringstream& cmd, double* val)
{
	static const char *functionName = "writeReadDouble";
	char inString[MAX_CONTROLLER_STRING_SIZE];
	std::stringstream val_convert;
	int errNo;
	
	std::fill(inString, inString + 256, '\0');
	
	asynPrint(apd->pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s: output = %s\n", driverName, functionName, cmd.str().c_str());
	
	size_t response;
	apd->lock();
	asynStatus status = apd->writeReadController(cmd.str().c_str(), inString, 256, &response, -1);
	apd->lock();
	
	asynPrint(apd->pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s:  input = %s\n", driverName, functionName, inString);
	asynPrint(apd->pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s: status = %i\n", driverName, functionName, status);
	
	
	if (status == asynSuccess)
	{
		if (inString[0] != '?')
		{
			// inString ends with \r:\r, but that isn't a problem for the following conversion
			val_convert << std::string(inString);
			val_convert >> *val;
			
			asynPrint(apd->pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s:    val = %lf\n", driverName, functionName, *val);
		}
		else
		{
			// Overwrite the '?' so the conversion can succeed
			inString[0] = ' ';
			val_convert << std::string(inString);
			val_convert >> errNo;
			
			asynPrint(apd->pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: ERROR #%i - command: %s\n", driverName, functionName, errNo, cmd.str().c_str());
			
			status = asynError;
		}
	}
	
	// clear the command stringstream -- this doesn't work
	cmd.str("");
	cmd.clear();
	
	return status;
}

asynStatus writeReadAck(asynPortDriver *apd, std::stringstream& cmd)
{
	static const char *functionName = "writeReadAck";
	char inString[MAX_CONTROLLER_STRING_SIZE];
	std::stringstream val_convert;
	int errNo;

	std::fill(inString, inString + 256, '\0');
	
	asynPrint(apd->pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s: output = %s\n", driverName, functionName, cmd.str().c_str());
	
	size_t response;
	apd->lock();
	asynStatus status = apd->writeReadController(cmd.str().c_str(), inString, 256, &response, -1);
	apd->lock();
	
	asynPrint(apd->pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s:  input = %s\n", driverName, functionName, inString);
	asynPrint(apd->pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s: status = %i\n", driverName, functionName, status);
	
	if (inString[0] == '?')
	{
		// Overwrite the '?' so the conversion can succeed
		inString[0] = ' ';
		val_convert << std::string(inString);
		val_convert >> errNo;
		
		asynPrint(apd->pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: ERROR #%i - command: %s\n", driverName, functionName, errNo, cmd.str().c_str());
		
		status = asynError;
	}
	
	// clear the command stringstream
	cmd.str("");
	cmd.clear();
	
	return status;
}

// NOTE: readBytes the number of data bytes that were read, excluding the command header and suffix
// NOTE: there is no error checking on inBytes and outBytes
asynStatus writeReadBinary(asynPortDriver *apd, char *output, int outBytes, char *input, int inBytes, size_t *dataBytes, bool *sliceAvailable)
{
	char* packetBuffer;
	size_t nwrite, nread;
	int eomReason;
	asynStatus status;
	static const char *functionName = "writeReadBinary";
	
	asynPrint(apd->pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s: start\n", driverName, functionName);
	
	apd->lock();
	
	std::fill(outString_, outString_ + MAX_CONTROLLER_STRING_SIZE, '\0');
	packetBuffer = (char *)calloc(MAX_PACKET_DATA+5, sizeof(char));
	
	// Clear the EOS characters
	pasynOctetSyncIO->setInputEos(pasynUserController_, "", 0);
	pasynOctetSyncIO->setOutputEos(pasynUserController_, "", 0);
	
	// Flush the receive buffer
	status = pasynOctetSyncIO->flush(pasynUserController_);
	
	asynPrint(apd->pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s: output bytes = %i, output = %s\n", driverName, functionName, outBytes, output);
	
	// Send the query command
	memcpy(outString_, output, outBytes);
	status = pasynOctetSyncIO->write(pasynUserController_, outString_, outBytes, SPIIPLUS_CMD_TIMEOUT, &nwrite);
	
	// The reply from the controller has a 4-byte header and a 1-byte suffix
	status = pasynOctetSyncIO->read(pasynUserController_, packetBuffer, inBytes, SPIIPLUS_ARRAY_TIMEOUT, &nread, &eomReason);
	
	asynPrint(apd->pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s:  input bytes = %i\n", driverName, functionName, inBytes);
	asynPrint(apd->pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s: status = %i\n", driverName, functionName, status);
	
	if (status == asynSuccess)
	{
		// Check for an error reply
		status = binaryErrorCheck(packetBuffer);
		if (status == asynError)
		{
			*sliceAvailable = false;
			asynPrint(apd->pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Binary read failed (controller)\n", driverName, functionName);
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
		asynPrint(apd->pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Binary read failed (asyn): status=%i, nread=%li\n", driverName, functionName, status, nread);
	}
	
	// Restore the EOS characters
	pasynOctetSyncIO->setInputEos(pasynUserController_, "\r", 1);
	pasynOctetSyncIO->setOutputEos(pasynUserController_, "\r", 1);
	
	apd->lock();
	
	asynPrint(apd->pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s: end\n", driverName, functionName);
	
	return status;
	}

asynStatus binaryErrorCheck(asynPortDriver *apd, char *buffer)
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
		
		asynPrint(apd->pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Binary command error #%i\n", driverName, functionName, errNo);
		status = asynError;
	}
	
	return status;
}

asynStatus getDoubleArray(asynPortDriver *apd, char *output, const char *var, int idx1start, int idx1end, int idx2start, int idx2end)
{
	char command[MAX_MESSAGE_LEN];
	asynStatus status;
	int remainingBytes;
	int readBytes;
	int outBytes, inBytes, dataBytes;
	size_t nread;
	int slice=1;
	bool sliceAvailable;
	static const char *functionName = "getDoubleArray";
	
	asynPrint(apd->pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s: start\n", driverName, functionName);
	
	std::fill(outString_, outString_ + MAX_CONTROLLER_STRING_SIZE, '\0');
	
	// Create the command to query array data. This could be the only command
	// that needs to be sent or it could be the first of many.
	readFloat64ArrayCmd(command, var, idx1start, idx1end, idx2start, idx2end, &outBytes, &inBytes, &dataBytes);
	
	remainingBytes = dataBytes;
	readBytes = 0;
	
	asynPrint(apd->pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s: var = %s, ((%i, %i), (%i, %i))\n", driverName, functionName, var, idx1start, idx1end, idx2start, idx2end);
	
	// Send the command
	status = writeReadBinary((char*)command, outBytes, output+readBytes, inBytes, &nread, &sliceAvailable);
	asynPrint(apd->pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s: Initial array query: request = %i; read = %li\n", driverName, functionName, inBytes, nread);
	
	asynPrint(apd->pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s: status = %i\n", driverName, functionName, status);
	
	remainingBytes -= nread;
	readBytes += nread;
	
	// Look at the response to see if there are more slices to read
	while (sliceAvailable)
	{
		// Create the command to query the next slice of the array data
		readFloat64SliceCmd(command, slice, var, idx1start, idx1end, idx2start, idx2end, &outBytes, &inBytes, &dataBytes);
		
		asynPrint(apd->pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s: var = %s, ((%i, %i), (%i, %i)), slice %i\n", driverName, functionName, var, idx1start, idx1end, idx2start, idx2end, slice);
		
		// Send the command
		status = writeReadBinary((char*)command, outBytes, output+readBytes, inBytes, &nread, &sliceAvailable);
		asynPrint(apd->pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s: Array slice #%i query: expected = %i; read = %li; sliceAvailable = %d\n", driverName, functionName, slice, inBytes, nread, sliceAvailable);
		
		asynPrint(apd->pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s: status = %i\n", driverName, functionName, status);
		
		remainingBytes -= nread;
		readBytes += nread;
		slice++;
	}
	
	asynPrint(apd->pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s: end\n", driverName, functionName);
	
	return status;
}

asynStatus getIntegerArray(asynPortDriver *apd, char *output, const char *var, int idx1start, int idx1end, int idx2start, int idx2end)
{
	char command[MAX_MESSAGE_LEN];
	asynStatus status;
	int remainingBytes;
	int readBytes;
	int outBytes, inBytes, dataBytes;
	size_t nread;
	int slice=1;
	bool sliceAvailable;
	static const char *functionName = "getIntegerArray";
	
	asynPrint(apd->pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s: start\n", driverName, functionName);
	
	std::fill(outString_, outString_ + MAX_CONTROLLER_STRING_SIZE, '\0');
	
	// Create the command to query array data. This could be the only command
	// that needs to be sent or it could be the first of many.
	readInt32ArrayCmd(command, var, idx1start, idx1end, idx2start, idx2end, &outBytes, &inBytes, &dataBytes);
	
	remainingBytes = dataBytes;
	readBytes = 0;
	
	asynPrint(apd->pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s: var = %s, ((%i, %i), (%i, %i))\n", driverName, functionName, var, idx1start, idx1end, idx2start, idx2end);
	
	// Send the command
	status = writeReadBinary((char*)command, outBytes, output+readBytes, inBytes, &nread, &sliceAvailable);
	asynPrint(apd->pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s: Initial array query: request = %i; read = %li\n", driverName, functionName, inBytes, nread);
	
	asynPrint(apd->pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s: status = %i\n", driverName, functionName, status);
	
	remainingBytes -= nread;
	readBytes += nread;
	
	// Look at the response to see if there are more slices to read
	while (sliceAvailable)
	{
		// Create the command to query the next slice of the array data
		readInt32SliceCmd(command, slice, var, idx1start, idx1end, idx2start, idx2end, &outBytes, &inBytes, &dataBytes);
		
		asynPrint(apd->pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s: var = %s, ((%i, %i), (%i, %i)), slice %i\n", driverName, functionName, var, idx1start, idx1end, idx2start, idx2end, slice);
		
		// Send the command
		status = writeReadBinary((char*)command, outBytes, output+readBytes, inBytes, &nread, &sliceAvailable);
		asynPrint(apd->pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s: Array slice #%i query: expected = %i; read = %li; sliceAvailable = %d\n", driverName, functionName, slice, inBytes, nread, sliceAvailable);
		
		asynPrint(apd->pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s: status = %i\n", driverName, functionName, status);
		
		remainingBytes -= nread;
		readBytes += nread;
		slice++;
	}
	
	asynPrint(apd->pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s: end\n", driverName, functionName);
	
	return status;
}
