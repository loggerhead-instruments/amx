// AMX data file

#define ULONG unsigned long
#define DFORM_LONG 4
#define DFORM_I24 3
#define DFORM_SHORT 2
#define DFORM_FLOAT32 5
#define SID_MAX 4

struct TIME_HEAD
{
	byte 	sec;  
	byte 	minute;  
	byte 	hour;  
	byte 	day;  
	byte 	mday;  
	byte 	month;  
	byte 	year;  
	short	tzOffset; //offset from GMT
};

struct SENSOR{
    char chipName[10]; // name of sensor e.g. MPU9250
    ULONG nChan;       //number of channels used (e.g. MPU9250 might have 9 for accel, mag, and gyro)
    char name[12][10]; //name of each channel (e.g. accelX, gyroZ). Max of 12 channels per chip.
    char units[12][10];// units of each channel (e.g. g, mGauss, degPerSec)
    float cal[12];     //calibration coefficient for each channel when multiplied by this value get data in specified units
};

struct DF_HEAD
{
	ULONG Version; // firmware version
  	ULONG UserID;  //tag type
  	TIME_HEAD RecStartTime;
};

struct SID_SPEC
{
	ULONG	SID;
	ULONG 	nBytes;	  // Size in bytes of this record (excluding header)
	ULONG	numChan;  // Number of expected channels in store
	SENSOR  sensor;	  // used to encode what data are saved: bitmask bit (accel3, mag3, gyro3, press, temperature, mic)
	ULONG	dForm;	  // short, long, or I24
	ULONG	period;	  // Sample period us
	ULONG 	recPts;   // Number of points per channel per sid_rec buffer
};

struct SID_REC
{
	ULONG	nSID;			      // This is record ID
	unsigned long long resvd;     // reserved to potentially store samples to get timestamp of block or timestamp
};
