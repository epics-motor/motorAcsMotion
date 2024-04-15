#include <string.h>
#include <sstream>
#include <math.h>
#include <stdio.h>

#include "SPiiPlusBinComm.h"

/*
 * The following functions create the commands necessary to read array data from ACS controllers.
 * The SPiiPlusDriver uses its writeReadBinary method to send the commands.
 */

//  readFloat64ArrayCmd(      buffer,          "APOS",             0,           7,          &out,          &in,          &data)
int readFloat64ArrayCmd(char *output, const char *var, int idx1start, int idx1end, int *outBytes, int *inBytes, int *dataBytes)
{
	int status;
	status = readFloat64ArrayCmd(output, var, idx1start, idx1end, 0, 0, false, outBytes, inBytes, dataBytes);
	return status;
}

//  readFloat64ArrayCmd(      buffer,          "APOS",             0,           7,         false,          &out,          &in,          &data)
int readFloat64ArrayCmd(char *output, const char *var, int idx1start, int idx1end, bool checksum, int *outBytes, int *inBytes, int *dataBytes)
{
	int status;
	status = readFloat64ArrayCmd(output, var, idx1start, idx1end, 0, 0, checksum, outBytes, inBytes, dataBytes);
	return status;
}

//  readFloat64ArrayCmd(      buffer,          "APOS",             0,           7,             0,           0,          &out,          &in,          &data)
int readFloat64ArrayCmd(char *output, const char *var, int idx1start, int idx1end, int idx2start, int idx2end, int *outBytes, int *inBytes, int *dataBytes)
{
	int status;
	status = readFloat64ArrayCmd(output, var, idx1start, idx1end, idx2start, idx2end, false, outBytes, inBytes, dataBytes);
	return status;
}

/*
 * Create a binary read command to read a 64-bit real array from the controller
 * Note: outBytes and inBytes are the number of bytes including the command header and suffix
 */
//  readFloat64ArrayCmd(      buffer,          "APOS",             0,           7,             0,           0,         false,          &out,          &in,          &data)
int readFloat64ArrayCmd(char *output, const char *var, int idx1start, int idx1end, int idx2start, int idx2end, bool checksum, int *outBytes, int *inBytes, int *dataBytes)
{
	std::stringstream varst;
	int asciiVarSize;
	int cmdSize;
	
	// The number of bytes to be read from the specified variable, plus the header and suffix
	*dataBytes = (idx1end - idx1start + 1) * (idx2end - idx2start + 1) * DOUBLE_DATA_SIZE;
	
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
	*dataBytes = (idx1end - idx1start + 1) * (idx2end - idx2start + 1) * DOUBLE_DATA_SIZE;
	
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

//  readInt32ArrayCmd(      buffer,         "FAULT",             0,           7,          &out,          &in,          &data)
int readInt32ArrayCmd(char *output, const char *var, int idx1start, int idx1end, int *outBytes, int *inBytes, int *dataBytes)
{
	int status;
	status = readInt32ArrayCmd(output, var, idx1start, idx1end, 0, 0, false, outBytes, inBytes, dataBytes);
	return status;
}

//  readInt32ArrayCmd(      buffer,         "FAULT",             0,           7,         false,          &out,          &in,          &data)
int readInt32ArrayCmd(char *output, const char *var, int idx1start, int idx1end, bool checksum, int *outBytes, int *inBytes, int *dataBytes)
{
	int status;
	status = readInt32ArrayCmd(output, var, idx1start, idx1end, 0, 0, checksum, outBytes, inBytes, dataBytes);
	return status;
}

//  readInt32ArrayCmd(      buffer,         "FAULT",             0,           7,             0,           0,          &out,          &in,          &data)
int readInt32ArrayCmd(char *output, const char *var, int idx1start, int idx1end, int idx2start, int idx2end, int *outBytes, int *inBytes, int *dataBytes)
{
	int status;
	status = readInt32ArrayCmd(output, var, idx1start, idx1end, idx2start, idx2end, false, outBytes, inBytes, dataBytes);
	return status;
}

/*
 * Create a binary read command to read a 64-bit real array from the controller
 * Note: outBytes and inBytes are the number of bytes including the command header and suffix
 */
//  readInt32ArrayCmd(      buffer,         "FAULT",             0,           7,             0,           0,         false,          &out,          &in,          &data)
int readInt32ArrayCmd(char *output, const char *var, int idx1start, int idx1end, int idx2start, int idx2end, bool checksum, int *outBytes, int *inBytes, int *dataBytes)
{
	std::stringstream varst;
	int asciiVarSize;
	int cmdSize;
	
	// The number of bytes to be read from the specified variable, plus the header and suffix
	*dataBytes = (idx1end - idx1start + 1) * (idx2end - idx2start + 1) * INT_DATA_SIZE;
	
	if (*dataBytes > MAX_PACKET_DATA)
		*inBytes = MAX_PACKET_SIZE;
	else
		*inBytes = *dataBytes + 5;
	
	// The variable string part of the command
	varst << var << "(" << idx1start << "," << idx1end << ")(" << idx2start << "," << idx2end << ")";
	asciiVarSize = varst.str().size();
	// Command = %?? + 0x4 + varst (doesn't include prefix or suffix)
	cmdSize = asciiVarSize+4;
	
	/*
	 * Binary query integer array format: 
	 *   if array < 1400:
	 *     [D3][F1][XX][XX]%??[04]cmd[D6]
	 *   else:
	 *     [D3][44][XX][XX]%??[04]cmd[D6]
	 *   where XX XX is the command length (little endian)
	 * 
	 */
	
	// header
	output[0] = FRAME_START;
	if (*dataBytes > MAX_PACKET_DATA)
		output[1] = READ_LI_ARRAY_CMD;
	else
		output[1] = READ_I_ARRAY_CMD;
	output[2] = (cmdSize >> 0) & 0xFF;
	output[3] = (cmdSize >> 8) & 0xFF;
	// command
	strncpy(output+4, "%??", 3);
	output[7] = INT_DATA_SIZE;
	strncpy(output+8, varst.str().c_str(), asciiVarSize);
	// end
	output[8+asciiVarSize] = FRAME_END;
	
	// The number of bytes in the binary query command
	*outBytes = 9+asciiVarSize;
	
	return *dataBytes;
}

int readInt32SliceCmd(char *output, int slice, const char *var, int idx1start, int idx1end, int idx2start, int idx2end, int *outBytes, int *inBytes, int *dataBytes)
{
	int status;
	status = readInt32SliceCmd(output, slice, var, idx1start, idx1end, idx2start, idx2end, false, outBytes, inBytes, dataBytes);
	return status;
}

/*
 *
 */
int readInt32SliceCmd(char *output, int slice, const char *var, int idx1start, int idx1end, int idx2start, int idx2end, bool checksum, int *outBytes, int *inBytes, int *dataBytes)
{
	std::stringstream varst;
	int asciiVarSize;
	int cmdSize;
	int numSlices;
	int bytesRemaining;
	char sliceStr[3] = {0, 0, 0};
	int sliceSize;
	
	// The number of bytes to be read from the specified variable, plus the header and suffix
	*dataBytes = (idx1end - idx1start + 1) * (idx2end - idx2start + 1) * INT_DATA_SIZE;
	
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
	// Command = % + slice + %?? + 0x4 + varst (doesn't include prefix or suffix)
	cmdSize = sliceSize + asciiVarSize + 5;
	
	/*
	 * Binary query integer array format: 
	 *     [D3][45][XX][XX]%#%??[04]cmd[D6]
	 *   where XX XX is the command length (little endian)
	 */
	
	// header
	output[0] = FRAME_START;
	output[1] = READ_LI_SLICE_CMD;
	output[2] = (cmdSize >> 0) & 0xFF;
	output[3] = (cmdSize >> 8) & 0xFF;
	// command
	strncpy(output+4, "%", 1);
	strncpy(output+5, sliceStr, sliceSize);
	strncpy(output+(sliceSize+5), "%??", 3);
	output[sliceSize+8] = INT_DATA_SIZE;
	strncpy(output+(sliceSize+9), varst.str().c_str(), asciiVarSize);
	// end
	// NOTE: asciiVarSize + 9 + sliceSize = cmdSize + 4
	output[cmdSize+4] = FRAME_END;
	
	// The number of bytes in the binary query command
	*outBytes = cmdSize+5;
	
	return *dataBytes;
}

/* WIP: Implement array writing */

//  writeFloat64ArrayCmd(      buffer,          "APOS",             0,           7,        &data,         0,     &remainingSlices,          &out,          &in)
int writeFloat64ArrayCmd(char *output, const char *var, int idx1start, int idx1end, double *data, int slice, int *remainingSlices, int *outBytes, int *inBytes)
{
	int status;
	status = writeFloat64ArrayCmd(output, var, idx1start, idx1end, 0, 0, data, false, slice, remainingSlices, outBytes, inBytes);
	return status;
}

//  writeFloat64ArrayCmd(      buffer,          "APOS",             0,           7,        &data,         false,         0,     &remainingSlices,          &out,          &in)
int writeFloat64ArrayCmd(char *output, const char *var, int idx1start, int idx1end, double *data, bool checksum, int slice, int *remainingSlices, int *outBytes, int *inBytes)
{
	int status;
	status = writeFloat64ArrayCmd(output, var, idx1start, idx1end, 0, 0, data, checksum, slice, remainingSlices, outBytes, inBytes);
	return status;
}

//  writeFloat64ArrayCmd(      buffer,          "APOS",             0,           7,             0,           0,        &data,         0,     &remainingSlices,          &out,          &in)
int writeFloat64ArrayCmd(char *output, const char *var, int idx1start, int idx1end, int idx2start, int idx2end, double *data, int slice, int *remainingSlices, int *outBytes, int *inBytes)
{
	int status;
	status = writeFloat64ArrayCmd(output, var, idx1start, idx1end, idx2start, idx2end, data, false, slice, remainingSlices, outBytes, inBytes);
	return status;
}

/*
 * Create a binary write command to write a 64-bit real array to the controller
 * Note: outBytes and inBytes are the number of bytes including the command header and suffix
 */
//  writeFloat64ArrayCmd(      buffer,          "APOS",             0,           7,             0,           0,        &data,         false,         0,     &remainingSlices,          &out,          &in)
int writeFloat64ArrayCmd(char *output, const char *var, int idx1start, int idx1end, int idx2start, int idx2end, double *data, bool checksum, int slice, int *remainingSlices, int *outBytes, int *inBytes)
{
	std::stringstream varst;
	int asciiVarSize;
	int cmdSize;
	bool multiPacket;
	int numDoubles;
	int maxDoublesPerPacket=0;
	int dataBytes;
	int numPackets;
	int packetDataBytes;
	
	// The number of doubles to be written to the specified variable
	numDoubles = (idx1end - idx1start + 1) * (idx2end - idx2start + 1);
	
	// The number of bytes to be written to the specified variable (not including the header and suffix)
	dataBytes = numDoubles * DOUBLE_DATA_SIZE;
	
	// The variable string part of the command
	varst << var << "(" << idx1start << "," << idx1end << ")(" << idx2start << "," << idx2end << ")";
	asciiVarSize = varst.str().size();
	
	/*
	 * The format of the command (and size of its components):
	 *   %1     - (2) - slice indicatior (only required if multiple packets need to be sent)
	 *   %>>    - (3) - start of binary write command
	 *   [08]   - (1) - data type - Real (8 bytes)
	 *   varst  - (variable)
	 *
	 * The format of the data:
	 *   /%     - (2) - Start of data
	 *   values - (variable) blocks of 8 byte data (64-bit values can't be split between slices)
	 *
	 * Overhead:
	 *   6 bytes - data+command fits in one packet
	 *   8 bytes - data+command requires multiple packets 
	 * 
	 */
	
	// Check if only one packet is needed
	if ((asciiVarSize + 6 + dataBytes) <= MAX_PACKET_DATA)
	{
		multiPacket = false;
		*remainingSlices = 0;
		// All the data fits into the packet
		packetDataBytes = dataBytes;
		cmdSize = asciiVarSize + 6 + packetDataBytes;
	}
	else
	{
		multiPacket = true;
		
		/*
		 * Calculate the number of doubles that will fit into a packet
		 */
		
		// Take the floorl because we can't split 64-bit values between packets
		maxDoublesPerPacket = floorl((MAX_PACKET_DATA - asciiVarSize - 8) / DOUBLE_DATA_SIZE);
		
		// Take the ceill because one last packet is needed for the remainder
		numPackets = ceill(numDoubles / maxDoublesPerPacket);
		
		// Slices are 0-indexed
		*remainingSlices = numPackets - slice - 1;
		
		// Determine how much data is in the current packet
		if (slice == (numPackets - 1))
		{
			// The last packet almost certainly has less data than the previous packets
			packetDataBytes = (numDoubles - slice * maxDoublesPerPacket) * DOUBLE_DATA_SIZE;
		}
		else
		{
			// All the packets other than the last one should be full
			packetDataBytes = maxDoublesPerPacket * DOUBLE_DATA_SIZE;
		}
		cmdSize = asciiVarSize + 8 + packetDataBytes;
	}
	
	/*
	 * Binary write double array format: 
	 *   if array < 1400:
	 *     [D3][F2][XX][XX]%>>[08]cmd/%data[D6]
	 *   else:
	 *     [D3][37][XX][XX]%>>[08]cmd/%data[D6]
	 *   where XX XX is the command length (little endian)
	 */
	
	// header
	output[0] = FRAME_START;
	if (multiPacket)
		output[1] = WRITE_LD_ARRAY_CMD;
	else
		output[1] = WRITE_D_ARRAY_CMD;
	output[2] = (cmdSize >> 0) & 0xFF;
	output[3] = (cmdSize >> 8) & 0xFF;
	// command
	strncpy(output+4, "%>>", 3);
	output[7] = DOUBLE_DATA_SIZE;
	strncpy(output+8, varst.str().c_str(), asciiVarSize);
	
	// data
	strncpy(output+8+asciiVarSize, "/%", 2);
	// The data offset should always be a multiple of the maxDoublesPerPacket
	memcpy(output+8+asciiVarSize+2, data+(slice*maxDoublesPerPacket*DOUBLE_DATA_SIZE), packetDataBytes);
	
	// end
	output[8+asciiVarSize+2+packetDataBytes] = FRAME_END;
	
	// The number of bytes in the binary write command
	*outBytes = 8+asciiVarSize+2+packetDataBytes+1;
	
	// the commands to write binary arrays always return two bytes
	*inBytes = 2;
	
	return packetDataBytes;
}
