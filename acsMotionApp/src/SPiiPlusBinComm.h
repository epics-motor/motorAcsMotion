#define MAX_MESSAGE_LEN		256
#define MAX_PACKET_SIZE		1405
#define MAX_PACKET_DATA		1400
//
#define FRAME_START 		0xd3
#define FRAME_END 		0xd6
//
#define INT_DATA_SIZE 		0x04
#define DOUBLE_DATA_SIZE	0x08
//
#define READ_D_ARRAY_CMD	0xf0
#define READ_I_ARRAY_CMD	0xf1
#define READ_LD_ARRAY_CMD	0x41
#define READ_LD_SLICE_CMD	0x42
#define READ_LD_END_CMD		0x43
#define READ_LI_ARRAY_CMD	0x44
#define READ_LI_SLICE_CMD	0x45
#define SLICE_AVAILABLE		0x80
//
#define WRITE_I_ARRAY_CMD	0xf3
#define WRITE_D_ARRAY_CMD	0xf2
#define WRITE_LD_ARRAY_CMD	0x37
#define WRITE_LD_SLICE_CMD	0x38
#define WRITE_LD_END_CMD	0x39
#define WRITE_LI_ARRAY_CMD	0x3A
#define WRITE_LI_SLICE_CMD	0x3B

int readFloat64ArrayCmd(char *output, const char *var, int idx1start, int idx1end, int *outBytes, int *inBytes, int *dataBytes);
int readFloat64ArrayCmd(char *output, const char *var, int idx1start, int idx1end, bool checksum, int *outBytes, int *inBytes, int *dataBytes);
int readFloat64ArrayCmd(char *output, const char *var, int idx1start, int idx1end, int idx2start, int idx2end, int *outBytes, int *inBytes, int *dataBytes);
int readFloat64ArrayCmd(char *output, const char *var, int idx1start, int idx1end, int idx2start, int idx2end, bool checksum, int *outBytes, int *inBytes, int *dataBytes);

int readFloat64SliceCmd(char *output, int slice, const char *var, int idx1start, int idx1end, int idx2start, int idx2end, int *outBytes, int *inBytes, int *dataBytes);
int readFloat64SliceCmd(char *output, int slice, const char *var, int idx1start, int idx1end, int idx2start, int idx2end, bool checksum, int *outBytes, int *inBytes, int *dataBytes);

int readInt32ArrayCmd(char *output, const char *var, int idx1start, int idx1end, int *outBytes, int *inBytes, int *dataBytes);
int readInt32ArrayCmd(char *output, const char *var, int idx1start, int idx1end, bool checksum, int *outBytes, int *inBytes, int *dataBytes);
int readInt32ArrayCmd(char *output, const char *var, int idx1start, int idx1end, int idx2start, int idx2end, int *outBytes, int *inBytes, int *dataBytes);
int readInt32ArrayCmd(char *output, const char *var, int idx1start, int idx1end, int idx2start, int idx2end, bool checksum, int *outBytes, int *inBytes, int *dataBytes);

int readInt32SliceCmd(char *output, int slice, const char *var, int idx1start, int idx1end, int idx2start, int idx2end, int *outBytes, int *inBytes, int *dataBytes);
int readInt32SliceCmd(char *output, int slice, const char *var, int idx1start, int idx1end, int idx2start, int idx2end, bool checksum, int *outBytes, int *inBytes, int *dataBytes);
