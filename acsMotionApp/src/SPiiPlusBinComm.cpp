#include <string.h>
#include <sstream>

#include "SPiiPlusBinComm.h"

// readFloat64ArrayCmd(buffer, "MFLAGS", 0, 7)
int readFloat64ArrayCmd(char *output, const char *var, int idx1start, int idx1end, int *outBytes, int *inBytes)
{
	int status;
	status = readFloat64ArrayCmd(output, var, idx1start, idx1end, 0, 0, false, outBytes, inBytes);
	return status;
}

// readFloat64ArrayCmd(buffer, "MFLAGS", 0, 7, false)
int readFloat64ArrayCmd(char *output, const char *var, int idx1start, int idx1end, bool checksum, int *outBytes, int *inBytes)
{
	int status;
	status = readFloat64ArrayCmd(output, var, idx1start, idx1end, 0, 0, checksum, outBytes, inBytes);
	return status;
}

// readFloat64ArrayCmd(buffer, "MFLAGS", 0, 7, 0, 0)
int readFloat64ArrayCmd(char *output, const char *var, int idx1start, int idx1end, int idx2start, int idx2end, int *outBytes, int *inBytes)
{
	int status;
	status = readFloat64ArrayCmd(output, var, idx1start, idx1end, idx2start, idx2end, false, outBytes, inBytes);
	return status;
}

// readFloat64ArrayCmd(buffer, "MFLAGS", 0, 7, 0, 0, false)
/*
 * Create a binary read command to read a 64-bit real array from the controller
 * Note: outBytes and inBytes are the number of bytes including the command header and suffix
 */
int readFloat64ArrayCmd(char *output, const char *var, int idx1start, int idx1end, int idx2start, int idx2end, bool checksum, int *outBytes, int *inBytes)
{
	std::stringstream varst;
	int asciiVarSize;
	int cmdSize;
	
	// The number of bytes to be read from the specified variable, plus the header and suffix
	*inBytes = (idx1end - idx1start + 1) * (idx2end - idx2start + 1) * 8 + 5;
	
	varst << var << "(" << idx1start << "," << idx1end << ")(" << idx2start << "," << idx2end << ")";
	
	/*
	 * Binary query double array format: 
	 *   if array < 1400:
	 *     [D3][F0][XX][XX]%??[08]cmd[D6]
	 *   else:
	 *     [D3][41][XX][XX]%??[08]cmd[D6]
	 *   where XX XX is the command length (little endian)
	 * 
	 */
	
	asciiVarSize = varst.str().size();
	
	// header
	output[0] = FRAME_START;
	if (*inBytes > 1400)
		output[1] = READ_LD_ARRAY_CMD;
	else
		output[1] = READ_D_ARRAY_CMD;
	cmdSize = asciiVarSize+4;		// %?? + 0x8 + array-to-read
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
	
	return *outBytes;
}

/*
// readFloat64SliceCmd(buffer, 1, "MFLAGS", 0, 7, 0, 0, false)
int readFloat64SliceCmd(char *output,  int slice, char *var, int idx1start, int idx1end, int idx2start, int idx2end, bool checksum)
{

}
*/
