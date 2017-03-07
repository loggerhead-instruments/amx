// AMX data file

#define STR_MAX 8

__declspec(align(1)) typedef struct
{
	UINT8 	sec;
	UINT8 	minute;
	UINT8 	hour;
	UINT8 	day;
	UINT8 	month;
	UINT8 NU[3];
	INT16 year;
	INT16 tzOffset; //offset from GMT 
}AMX_TIME;

__declspec(align(1)) typedef struct 
{
	char chipName[STR_MAX]; // name of sensor e.g. MPU9250
	INT16 nChan;            //number of channels used (e.g. MPU9250 might have 9 for accel, mag, and gyro)
	INT16 NU;
	char name[12][STR_MAX]; //name of each channel (e.g. accelX, gyroZ). Max of 12 channels per chip.
	char units[12][STR_MAX];// units of each channel (e.g. g, mGauss, degPerSec)
	float cal[12];          //calibration coefficient for each channel when multiplied by this value get data in specified units
}AMX_SENSOR;

__declspec(align(1)) typedef struct
{
	ULONG Version; // firmware version
	ULONG UserID;  //tag type
	AMX_TIME RecStartTime;
	float voltage;
}AMX_DF;

__declspec(align(1)) typedef struct
{
	char	SID[STR_MAX];
	INT16 sidType; // 0 = raw, 1 = summary  histogram
	INT16 NU;
	ULONG 	nSamples;	  // Size in samples of this record (excluding header)
	AMX_SENSOR  sensor;	  // used to encode what data are saved: bitmask bit (accel3, mag3, gyro3, press, temperature, mic)
	ULONG	dForm;	  // short, long, or I24
	float	srate;	  // Sample rate (Hz)
}AMX_SID_SPEC;

__declspec(align(1)) typedef struct
{
	UINT32 nSID;			      // This is record ID
	UINT32 NU[3];     // reserved to potentially store samples to get timestamp of block or timestamp
}AMX_SID_REC;