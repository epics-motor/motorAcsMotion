
#include <string.h>
#include <cstdlib>
#include <sstream>

#include <iocsh.h>
#include <epicsThread.h>
#include <asynPortDriver.h>
#include <asynOctetSyncIO.h>

#include "asynMotorController.h"
#include "asynMotorAxis.h"

#include <epicsExport.h>

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
// FYI: motor/motorApp/MotorSrc/asynMotorController.h:#define MAX_CONTROLLER_STRING_SIZE 256
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
	
	asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s: input bytes = %i\n", driverName, functionName, inBytes);
	asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s: status = %i\n", driverName, functionName, status);
	
	if (status == asynSuccess)
	{
		// Check for an error reply
		status = binaryErrorCheck(packetBuffer, nread);
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

asynStatus SPiiPlusComm::binaryErrorCheck(char *buffer, int readBytes)
{
	asynStatus status=asynSuccess;
	std::stringstream val_convert;
	int errNo, idx;
	char replyStart, replyEnd, cmdId, bodyLength, bodyStart, bodyEnd;
	char errorStr[5] = {0, 0, 0, 0, 0};
	bool errNoIsValid = true;
	static const char *functionName = "binaryErrorCheck";
	
	/*
	 * This was the original expected error response (11 bytes), but I can't find it anywhere in the documentation now.
	 * Error response: [E3][XX][06][00]?####[0D][E6]
	 *
	 * This is documented error response (10 bytes) that is present in many verions of the low level host communication user guide.
	 * Error response: [E3][XX]6?####[0D][E6]
	 */
	
	if (readBytes == 10)
	{
		replyStart = buffer[0];
		cmdId = buffer[1];
		bodyLength = buffer[2];
		bodyStart = buffer[3];
		bodyEnd = buffer[8];
		replyEnd = buffer[9];
		
		if ((replyStart == 0xe3) && (bodyLength == '6') && (replyEnd == 0xe6))
		{
			// '?' is 0x3f
			if ((bodyStart == 0x3f) && (bodyEnd == 0x0d))
			{
				for (idx=0; idx<4; idx++)
				{
					/* 
					 * The error number starts at index = 4 in the error reply
					 * Confirm the error number has valid characters (digits 0-9)
					 * '0' is 48; '9' is 57
					 */
					if ((buffer[4+idx] < 48) && (buffer[4+idx] > 57))
					{
						errNoIsValid = false;
						break;
					}
					else
					{
						errorStr[idx] = buffer[4+idx];
					}
				}
				
				if (errNoIsValid)
				{
					// The error string is valid and can be converted into an int and reported on the IOC's shell
					val_convert << errorStr;
					val_convert >> errNo;
					
					asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Binary command error #%i for command id %x\n", driverName, functionName, errNo, cmdId);
					status = asynError;
				}
			}
			else
			{
				asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Incorrect error body start/end: bodyStart = %x, bodyEnd = %x\n", driverName, functionName, bodyStart, bodyEnd);
			}
		}
		else
		{
			asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Incorrect error reply prefix/suffix: replyStart = %x, bodyLength = %x, replyEnd = %x\n", driverName, functionName, replyStart, bodyLength, replyEnd);
		}
	}
	else if (readBytes == 11)
	{
		asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Possible old binary command error; read %d bytes\n", driverName, functionName, readBytes);
	}
	
	return status;
}

asynStatus SPiiPlusComm::isVariableDefined(bool *isDefined, const char *var)
{
	static const char *functionName = "isVariableDefined";
	std::stringstream cmd;
	char inString[MAX_CONTROLLER_STRING_SIZE];
	
	cmd << "?" << var;
	std::fill(inString, inString + 256, '\0');
	
	asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s: output = %s\n", driverName, functionName, cmd.str().c_str());
	
	size_t response;
	lock();
	asynStatus status = writeReadController(cmd.str().c_str(), inString, 256, &response, -1);
	unlock();
	
	asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s:  input = %s\n", driverName, functionName, inString);
	asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s: status = %i\n", driverName, functionName, status);
	
	*isDefined = (inString[0] != '?');
	
	// clear the command stringstream
	cmd.str("");
	cmd.clear();
	
	return status;
}

/*
 * This method is used for writing binary arrays, which can be quite large and is
 * called one time for each packet that needs to be sent.  The controller only 
 * response with an acknowledgement.
 */
asynStatus SPiiPlusComm::writeReadAckBinary(char *output, int outBytes, char *input, int inBytes)
{
	size_t nwrite, nread, extraRead;
	int eomReason;
	int commandID;
	asynStatus status;
	static const char *functionName = "writeReadAckBinary";
	
	asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s: start\n", driverName, functionName);
	
	lock();
	
	// Clear the EOS characters
	pasynOctetSyncIO->setInputEos(pasynUserComm_, "", 0);
	pasynOctetSyncIO->setOutputEos(pasynUserComm_, "", 0);
	
	// Flush the receive buffer
	status = pasynOctetSyncIO->flush(pasynUserComm_);
	
	// Save the command ID
	commandID = output[1];
	
	// Send the command
	status = pasynOctetSyncIO->write(pasynUserComm_, output, outBytes, SPIIPLUS_ARRAY_TIMEOUT, &nwrite);
	
	asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s: status = %i; output bytes = %i, nwrite = %li\n", driverName, functionName, status, outBytes, nwrite);

	// A successful reply from the controller is 2 bytes (ack & command ID) 
	// NOTE: the comand timeout is too short and the array timeout is overkill
	status = pasynOctetSyncIO->read(pasynUserComm_, input, inBytes, SPIIPLUS_ACK_TIMEOUT, &nread, &eomReason);
	
	asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s: status = %i; input bytes = %i, nread = %li\n", driverName, functionName, status, inBytes, nread);
	
	// A successful read doesn't necessarily mean the command succeeded
	if (status == asynSuccess)
	{
		// Confirm write was successful
		if ((unsigned char)input[0] != ACKNOWLEDGE)
		{
			// Error messages are more than 2 characters and we only read 2 so far. Read the rest now.
			status = pasynOctetSyncIO->read(pasynUserComm_, input+inBytes, MAX_MESSAGE_LEN, SPIIPLUS_ACK_TIMEOUT, &extraRead, &eomReason);
			
			asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER,    "%s:%s: status = %i; extraRead = %li, eomReason = %i\n", driverName, functionName, status, extraRead, eomReason);
			
			// Check for an error reply -- this overwrites the buffer if an error occurs
			status = binaryErrorCheck(input, nread+extraRead);
			if (status == asynError)
			{
				asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Binary read failed (controller)\n", driverName, functionName);
			}
		
		}
		else
		{
			if (input[1] == commandID)
			{
				asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s: Command ID matches: write ID = %i, read ID = %i\n", driverName, functionName, commandID, input[1]);
			}
			else
			{
				asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Command ID mismatch: write ID = %i, read ID = %i\n", driverName, functionName, commandID, input[1]);
			}
		}
	}
	else
	{
		asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Binary read failed (asyn): status=%i, nread=%li\n", driverName, functionName, status, nread);
	}
	
	// Restore the EOS characters
	pasynOctetSyncIO->setInputEos(pasynUserComm_, "\r", 1);
	pasynOctetSyncIO->setOutputEos(pasynUserComm_, "\r", 1);
	
	unlock();
	
	asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s: end\n", driverName, functionName);
	
	return status;
}

asynStatus SPiiPlusComm::getDoubleArray(char *output, const char *var, int idx1start, int idx1end, int idx2start, int idx2end)
{
	//char outString[MAX_CONTROLLER_STRING_SIZE];
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
	
	//std::fill(outString, outString + MAX_CONTROLLER_STRING_SIZE, '\0');
	
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

asynStatus SPiiPlusComm::globalVarCheck(const char *var, int idx1start, int idx1end, int idx2start, int idx2end, int *dimensions, int *numElements, int *errNo)
{
	std::stringstream cmd;
	std::stringstream val_convert;
	char inString[MAX_CONTROLLER_STRING_SIZE];
	asynStatus status;
	size_t response;
	std::fill(inString, inString + 256, '\0');
	static const char *functionName = "globalVarCheck";
	
	/*
	 * There is no command to check for the existence of a variable.
	 * A workaround for this is to try to access the last element of
	 * array.  There are three possible outcomes:
	 * 1. A value is returned (the global array exists and is at least as large as it needs to be)
	 * 2. Error #1064 is returned (the global array doesn't exist and can be easily created)
	 * 3. Error #1035 is returned (the global variable exists, but the array isn't large enough)
	 */
	
	// 
	if ((idx2end - idx2start) > 0)
	{
		// The var is a 2D array
		cmd << "?" << var << "(" << idx1end << ")(" << idx2end << ")";
		*dimensions = 2;
		*numElements = (idx1end - idx1start + 1) * (idx2end - idx1start + 1);
	}
	else if ((idx1end - idx1start) > 0)
	{
		// The var is a 1D array
		cmd << "?" << var << "(" << idx1end << ")";
		*dimensions = 1;
		*numElements = idx1end - idx1start + 1;
	}
	else
	{
		// The var is a scaler value
		cmd << "?" << var << "(0)";
		*dimensions = 0;
		*numElements = 1;
	}
	
	asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s: output = %s\n", driverName, functionName, cmd.str().c_str());
	
	lock();
	status = writeReadController(cmd.str().c_str(), inString, 256, &response, -1);
	unlock();
	
	asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s:  input = %s\n", driverName, functionName, inString);
	asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s: status = %i\n", driverName, functionName, status);
	
	if (status == asynSuccess)
	{
		if (inString[0] != '?')
		{
			/* Command succeeded */
			
			// The variable exists and is large enough to hold the data 
			*errNo = 0;
		}
		else
		{
			/* Command returned an error */
			
			// Overwrite the '?' so the conversion can succeed
			inString[0] = ' ';
			// Convert the response string into an integer
			val_convert << std::string(inString);
			val_convert >> *errNo;
			
			// This isn't an ASYN_TRACE_ERROR message, but the user might want to see it for troubleshooting
			asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s: Command failed: %s\n", driverName, functionName, cmd.str().c_str());
			
			if (*errNo == 1064)
			{
				/* The variable doesn't exist, but it can be created. The calling method should create the variable, since it knows the data type. */
				// status is already asynSuccess here
				status = asynSuccess;
			}
			else if (*errNo == 1035)
			{
				/* The variable exists, but it isn't large enough */
				status = asynError;
			}
			else
			{
				// An unexpected error occurred.  Let the user know.
				asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Unexpected error! errNo = %i\n", driverName, functionName, *errNo);
				status = asynError;
			}
		}
	}
	
	return status;
}

// TODO: make a more sophistcated version of this method that accepts an integer tag argument
asynStatus SPiiPlusComm::createGlobalRealVar(const char *var, int idx1start, int idx1end, int idx2start, int idx2end)
{
	std::stringstream cmd;
	asynStatus status;
	//static const char *functionName = "createGlobalRealVar";
	 
	if ((idx2end - idx2start) > 0)
	{
		// There is a 2nd array dimension
		cmd << "global REAL " << var << "(" << (idx1end+1) << ")(" << (idx2end+1) << ")";
	}
	else if ((idx1end - idx1start) > 0)
	{
		// There is a 1st array dimension
		cmd << "global REAL " << var << "(" << (idx1end+1) << ")";
	}
	else
	{
		// The var is a scaler value
		cmd << "global REAL " << var;
	}
	
	status = writeReadAck(cmd);
	
	return status;
}

asynStatus SPiiPlusComm::putDoubleArray(double *data, const char *var, int idx1start, int idx1end, int idx2start, int idx2end)
{
	char *inBuff;
	char *command;
	asynStatus status;
	int dimensions, numElements, errNo;
	int outBytes, inBytes, packetDoubles;
	int slice=0;
	int remainingSlices;
	int numSlices;
	int sentDoubles=0;
	int dataOffset=0;
	static const char *functionName = "putDoubleArray";
	
	asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s: start\n", driverName, functionName);
	
	// TODO: how to handle local variables?
	
	// Confirm the global variable exists and is large enough to hold the
	// array to be written to it.
	status = globalVarCheck(var, idx1start, idx1end, idx2start, idx2end, &dimensions, &numElements, &errNo);
	
	if (status == asynSuccess)
	{
		// Check the error number
		if (errNo == 1064)
		{
			// The variable doesn't exist and can be created
			status = createGlobalRealVar(var, idx1start, idx1end, idx2start, idx2end);
			
			if (status != asynSuccess)
			{
				// TODO: add asyn error message
				asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,    "%s:%s: Failed to create gobal variable: %s\n", driverName, functionName, var);
				return asynError;
			}
		}
	}
	else
	{
		// Variable isn't large enough to hold the data; can't proceed.
		asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,    "%s:%s: Global variable exists but isn't large enough: %s\n", driverName, functionName, var);
		return asynError;
	}
	
	command = (char *)calloc(MAX_PACKET_SIZE, sizeof(char));
	// Is MAX_PACKET_SIZE unnecessarily large for the Ack / error reply?  Ack is 2 bytes. Error is longer, but not that long.
	inBuff = (char *)calloc(MAX_PACKET_DATA, sizeof(char));
	
	/* 
	 * Unlike getDoubleArray, which parses the reply from the controller to determine if there is another
	 * slice to be read, putDoubleArray is responsible for keeping track of how many slices need to be sent.
	 * This is complicated by the fact that the command + data to be written (not including the prefix or 
	 * suffix) is limited (by GETCONF(99,8)--but is usually 1400).  The command varies because it includes
	 * the variable name and array indices.  It is also complicated by the fact that only 10 slices can be
	 * sent (the slice index is a single-character representation of an integer).
	 */
	
	// Create the command to send the first packet of array data. This could be the only 
	// command that needs to be sent or it could be the first of many.  Slice is always 0 here
	// but it gets omitted from the command if only one packet needs to be sent.
	writeFloat64ArrayCmd(command, var, idx1start, idx1end, idx2start, idx2end, data, slice, &remainingSlices, &outBytes, &inBytes, &packetDoubles);
	numSlices = remainingSlices + 1;
	
	asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s: var = %s, ((%i, %i), (%i, %i)), slices = %i\n", driverName, functionName, var, idx1start, idx1end, idx2start, idx2end, numSlices);
	//asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: var = %s, ((%i, %i), (%i, %i)), slices = %i\n", driverName, functionName, var, idx1start, idx1end, idx2start, idx2end, numSlices);
	
	// Send the command
	status = writeReadAckBinary((char*)command, outBytes, inBuff, inBytes);
	sentDoubles += packetDoubles;
	
	asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s: Slice %i write: packetDoubles = %i; outBytes = %i; status = %i\n", driverName, functionName, slice, packetDoubles, outBytes, status);
	//asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,    "%s:%s: Slice %i write: packetDoubles = %i; outBytes = %i; status = %i\n", driverName, functionName, slice, packetDoubles, outBytes, status);
	
	// Send the remaining packets, if necessary
	while (remainingSlices)
	{
		slice += 1;
		
		// Update the start indices when the 10th packet is reached, because the slice index needs to be reset to zero
		if (slice == 10)
		{
			// Reset the slice index
			slice = 0;
			
			// Use sentDoubles to determine new indices
			if (dimensions == 1)
			{
				idx1start = sentDoubles;
				dataOffset = sentDoubles;
			}
			else if (dimensions == 2)
			{
				/* NOTE: writing 2D data is currently broken after the 10th packet/slice */
				
				// Integer math will truncate this value
				idx2start = sentDoubles / (idx1end+1);
				idx1start = sentDoubles % (idx1end+1);
				dataOffset = sentDoubles;
			}
			
			// Share the new indices
			asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s: var = %s, ((%i, %i), (%i, %i)), slices = %i\n", driverName, functionName, var, idx1start, idx1end, idx2start, idx2end, remainingSlices);
			//asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: var = %s, ((%i, %i), (%i, %i)), slices = %i\n", driverName, functionName, var, idx1start, idx1end, idx2start, idx2end, remainingSlices);
		}
		
		
		// Created the command to send the next slice (packet)
		writeFloat64ArrayCmd(command, var, idx1start, idx1end, idx2start, idx2end, data+dataOffset, slice, &remainingSlices, &outBytes, &inBytes, &packetDoubles);
		
		// Send the command
		status = writeReadAckBinary((char*)command, outBytes, inBuff, inBytes);
		sentDoubles += packetDoubles;
		
		asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s: Slice %i write: packetDoubles = %i; outBytes = %i; status = %i\n", driverName, functionName, slice, packetDoubles, outBytes, status);
		//asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,    "%s:%s: Slice %i write: packetDoubles = %i; outBytes = %i; status = %i\n", driverName, functionName, slice, packetDoubles, outBytes, status);
	}
	
	asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s: end\n", driverName, functionName);
	
	free(command);
	free(inBuff);
	
	// The returnd status is the status of the last packet set to the controller
	return status;
}

asynStatus SPiiPlusComm::getIntegerArray(char *output, const char *var, int idx1start, int idx1end, int idx2start, int idx2end)
{
	//char outString[MAX_CONTROLLER_STRING_SIZE];
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
	
	//std::fill(outString, outString + MAX_CONTROLLER_STRING_SIZE, '\0');
	
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
