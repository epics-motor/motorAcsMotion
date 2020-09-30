#include <string.h>
#include <sstream>
#include <math.h>
#include <stdio.h>

#include "SPiiPlusBinComm.h"

// readFloat64ArrayCmd(buffer, "MFLAGS", 0, 7)
int readFloat64ArrayCmd(char *output, const char *var, int idx1start, int idx1end, int *outBytes, int *inBytes, int *dataBytes)
{
	int status;
	status = readFloat64ArrayCmd(output, var, idx1start, idx1end, 0, 0, false, outBytes, inBytes, dataBytes);
	return status;
}

// readFloat64ArrayCmd(buffer, "MFLAGS", 0, 7, false)
int readFloat64ArrayCmd(char *output, const char *var, int idx1start, int idx1end, bool checksum, int *outBytes, int *inBytes, int *dataBytes)
{
	int status;
	status = readFloat64ArrayCmd(output, var, idx1start, idx1end, 0, 0, checksum, outBytes, inBytes, dataBytes);
	return status;
}

// readFloat64ArrayCmd(buffer, "MFLAGS", 0, 7, 0, 0)
int readFloat64ArrayCmd(char *output, const char *var, int idx1start, int idx1end, int idx2start, int idx2end, int *outBytes, int *inBytes, int *dataBytes)
{
	int status;
	status = readFloat64ArrayCmd(output, var, idx1start, idx1end, idx2start, idx2end, false, outBytes, inBytes, dataBytes);
	return status;
}

// readFloat64ArrayCmd(buffer, "MFLAGS", 0, 7, 0, 0, false)
/*
 * Create a binary read command to read a 64-bit real array from the controller
 * Note: outBytes and inBytes are the number of bytes including the command header and suffix
 */
int readFloat64ArrayCmd(char *output, const char *var, int idx1start, int idx1end, int idx2start, int idx2end, bool checksum, int *outBytes, int *inBytes, int *dataBytes)
{
	std::stringstream varst;
	int asciiVarSize;
	int cmdSize;
	
	// The number of bytes to be read from the specified variable, plus the header and suffix
	*dataBytes = (idx1end - idx1start + 1) * (idx2end - idx2start + 1) * 8;
	
	if (*dataBytes > MAX_PACKET_DATA)
		*inBytes = MAX_PACKET_SIZE;
	else
		*inBytes = *dataBytes + 5;
	
	// The variable string part of the command
	varst << var << "(" << idx1start << "," << idx1end << ")(" << idx2start << "," << idx2end << ")";
	asciiVarSize = varst.str().size();
	// Command = %?? + 0x8 + varst (doesn't include prefix or suffix)
	cmdSize = asciiVarSize+4;
	
	/*
	 * Binary query double array format: 
	 *   if array < 1400:
	 *     [D3][F0][XX][XX]%??[08]cmd[D6]
	 *   else:
	 *     [D3][41][XX][XX]%??[08]cmd[D6]
	 *   where XX XX is the command length (little endian)
	 * 
	 */
	
	// header
	output[0] = FRAME_START;
	if (*dataBytes > MAX_PACKET_DATA)
		output[1] = READ_LD_ARRAY_CMD;
	else
		output[1] = READ_D_ARRAY_CMD;
	output[2] = (cmdSize >> 0) & 0xFF;
	output[3] = (cmdSize >> 8) & 0xFF;
	// command
	strncpy(output+4, "%??", 3);
	output[7] = DOUBLE_DATA_SIZE;
	strncpy(output+8, varst.str().c_str(), asciiVarSize);
	// end
	output[8+asciiVarSize] = FRAME_END;
	
	// The number of bytes in the binary query command
	*outBytes = 9+asciiVarSize;
	
	return *dataBytes;
}

int readFloat64SliceCmd(char *output, int slice, const char *var, int idx1start, int idx1end, int idx2start, int idx2end, int *outBytes, int *inBytes, int *dataBytes)
{
	int status;
	status = readFloat64SliceCmd(output, slice, var, idx1start, idx1end, idx2start, idx2end, false, outBytes, inBytes, dataBytes);
	return status;
}

/*
 *
 */
int readFloat64SliceCmd(char *output, int slice, const char *var, int idx1start, int idx1end, int idx2start, int idx2end, bool checksum, int *outBytes, int *inBytes, int *dataBytes)
{
	std::stringstream varst;
	int asciiVarSize;
	int cmdSize;
	int numSlices;
	int bytesRemaining;
	char sliceStr[3] = {0, 0, 0};
	int sliceSize;
	
	// The number of bytes to be read from the specified variable, plus the header and suffix
	*dataBytes = (idx1end - idx1start + 1) * (idx2end - idx2start + 1) * 8;
	
	numSlices = (int)ceil(*dataBytes / MAX_PACKET_DATA);
	
	if (slice < numSlices)
	{
		*inBytes = MAX_PACKET_SIZE;
	}
	else
	{
		// Calculate the remaining number of bytes to be read
		bytesRemaining = *dataBytes - numSlices * MAX_PACKET_DATA;
		*inBytes = bytesRemaining + 5;
	}
	
	// The variable string part of the command
	varst << var << "(" << idx1start << "," << idx1end << ")(" << idx2start << "," << idx2end << ")";
	asciiVarSize = varst.str().size();
	// The slice string part of the command (one or two characters)
	sprintf(sliceStr, "%i", slice);
	sliceSize = strlen(sliceStr);
	// Command = % + slice + %?? + 0x8 + varst (doesn't include prefix or suffix)
	cmdSize = sliceSize + asciiVarSize + 5;
	
	/*
	 * Binary query double array format: 
	 *     [D3][42][XX][XX]%#%??[08]cmd[D6]
	 *   where XX XX is the command length (little endian)
	 */
	
	// header
	output[0] = FRAME_START;
	output[1] = READ_LD_SLICE_CMD;
	output[2] = (cmdSize >> 0) & 0xFF;
	output[3] = (cmdSize >> 8) & 0xFF;
	// command
	strncpy(output+4, "%", 1);
	strncpy(output+5, sliceStr, sliceSize);
	strncpy(output+(sliceSize+5), "%??", 3);
	output[sliceSize+8] = DOUBLE_DATA_SIZE;
	strncpy(output+(sliceSize+9), varst.str().c_str(), asciiVarSize);
	// end
	// NOTE: asciiVarSize + 9 + sliceSize = cmdSize + 4
	output[cmdSize+4] = FRAME_END;
	
	// The number of bytes in the binary query command
	*outBytes = cmdSize+5;
	
	return *dataBytes;
}
