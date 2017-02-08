/****************************************************
 	Data record file begins with DF_HEAD followed
	by 1 or more SID_SPEC headers.  Then a SID_SPEC
	header with all zeros.  Then Begin the data sections
	with each record begining with a SID_REC followed
	by 0 to nBytes in data. 

	While not identical to TDTs data tank format the
	ADR data file targets this format so structures 
	will be similar. 
******************************************************/


//DM removed NU[16]
typedef struct 
{
	BYTE 	sec;  
	BYTE 	min;  
	BYTE 	hour;  
	BYTE 	day;  
	BYTE 	mday;  
	BYTE 	month;  
	BYTE 	year;  
	BYTE	NU;  
}TIME_HEAD;
typedef struct 
{
	ULONG Version;  
  	ULONG UserID;
  	TIME_HEAD RecStartTime;
}DF_HEAD;

typedef struct 
{
	ULONG Version;  
  	ULONG UserID;
  	TIME_HEAD RecStartTime;
	float Lat;
	float Lon;
	float depth;
	float DSGcal;
	float hydroCal;
	float lpFilt;
}DF_HEAD2;
// DM added RECPTS and RECINT
typedef struct 
{
	char	SID[4];
	ULONG 	nBytes;			// Size in bytes of this record (excluding header)
	ULONG	NumChan;		// Number of expected channels in store
	ULONG	StoreType;		// See TTank stuff
	ULONG	CircType;		// Circuit type they might generate this store in OpenEx (1-5)
	ULONG	DForm;			// See TTank stuff
	ULONG	SP256;			// Sample period (us) x 256
	ULONG 	RECPTS;			// rec points =0 for continuous; otherwise stutter
	ULONG	RECINT;		    // interval between rec pts; 
}SID_SPEC;

typedef struct 
{
	BYTE	nSID;			// This is record I/D
	BYTE	Chan;			// Channel index modifying SID of other user value or lower half of value
	UINT64	TS256;			// Timestamp in usecs * 256 since block record started
}SID_REC;


// These constants extradcted from OpenEx headers
#define EVTYPE_STRON	0x00000101
#define EVTYPE_STROFF	0x00000102
#define EVTYPE_SCALER	0x00000201
#define EVTYPE_STREAM	0x00008101
#define EVTYPE_SNIP		0x00008201
#define EVTYPE_MARK		0x00008801
#define EVTYPE_HASDATA	0x00008000

#define DFORM_FLOAT		0
#define DFORM_LONG		1
#define DFORM_SHORT		2
#define DFORM_BYTE		3
#define DFORM_DOUBLE	4
#define DFORM_QWORD		5


//RealTime.c
