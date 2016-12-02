// AMX data file

#define ULONG unsigned long
#define DFORM_LONG 4
#define DFORM_I24 3
#define DFORM_SHORT 2
#define DFORM_FLOAT32 5
#define SID_MAX 5
#define SENSOR_MAX 5

#define RAW_SID 0
#define HISTOGRAM_SID 1
#define STR_MAX 8

struct TIME_HEAD
{
	byte 	sec;  
	byte 	minute;  
	byte 	hour;  
	byte 	day;  
	byte 	month;  
    byte NU[3];
	int16_t year;  
	int16_t	tzOffset; //offset from GMT 
};

struct SENSOR{
  char chipName[STR_MAX]; // name of sensor e.g. MPU9250
  uint16_t nChan;       //number of channels used (e.g. MPU9250 might have 9 for accel, mag, and gyro)
  uint16_t NU;
  char name[12][STR_MAX]; //name of each channel (e.g. accelX, gyroZ). Max of 12 channels per chip.
  char units[12][STR_MAX];// units of each channel (e.g. g, mGauss, degPerSec)
  float cal[12];     //calibration coefficient for each channel when multiplied by this value get data in specified units
};

struct DF_HEAD
{
    ULONG Version; // firmware version
    ULONG UserID;  //tag type
    TIME_HEAD RecStartTime;
    float voltage;
};

struct SID_SPEC
{
	char	SID[STR_MAX];
  uint16_t sidType; // 0 = raw, 1 = summary  histogram
  uint16_t NU;
	ULONG 	nSamples;	  // Size in samples of this record (excluding header)
	SENSOR  sensor;	  // used to encode what data are saved: bitmask bit (accel3, mag3, gyro3, press, temperature, mic)
	ULONG	dForm;	  // short, long, or I24
	float	srate;	  // Sample rate (Hz)
};

struct SID_REC
{
	uint32_t	nSID;			      // This is record ID
	uint32_t NU[3];     // reserved to potentially store samples to get timestamp of block or timestamp
};
