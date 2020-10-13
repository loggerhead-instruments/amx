//
// AMX Wav
//
// Loggerhead Instruments
// 2020
// David Mann
// 
// Modified from PJRC audio code
// http://www.pjrc.com/store/teensy3_audio.html
//
// Compile with 96 MHz Fastest

// Modified by WMXZ 15-05-2018 for SdFS anf multiple sampling frequencies

char codeVersion[12] = "2020-10-13";
static boolean printDiags = 1;  // 1: serial print diagnostics; 0: no diagnostics

#define MQ 100 // to be used with LHI record queue (modified local version)

#include "SdFat.h"
#include "sdios.h"
#include "FreeStack.h"
#include "LHI_record_queue.h"
#include "control_sgtl5000.h"

//#if USE_SDFS==0
//  #include "input_i2s.h"
////  #include "LHI_record_queue.h"
////  #include "control_sgtl5000.h"
//#else
  #include <Audio.h>  //this also includes SD.h from lines 89 & 90
//#endif
//#include <Wire.h>
#include <i2c_t3.h>  // MS5837 pressure sensor doesn't work with Wire.h
#include <SPI.h>

#include <Snooze.h>  //using https://github.com/duff2013/Snooze; uncomment line 62 #define USE_HIBERNATE

#include <TimeLib.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h> // modify so calls i2c_t3 not Wire.h
#include <Adafruit_FeatherOLED.h>
#include <EEPROM.h>

#define CPU_RESTART_ADDR (uint32_t *)0xE000ED0C
#define CPU_RESTART_VAL 0x5FA0004
#define CPU_RESTART (*CPU_RESTART_ADDR = CPU_RESTART_VAL);

#define OLED_RESET -1
Adafruit_FeatherOLED display = Adafruit_FeatherOLED();
#define BOTTOM 25
#define displayLine1 0
#define displayLine2 8
#define displayLine3 16
#define displayLine4 25

// set this to the hardware serial port you wish to use
#define HWSERIAL Serial1

#define NREC 32 // increase disk buffer to speed up disk access

static uint8_t myID[8];

unsigned long baud = 115200;

#define SECONDS_IN_MINUTE 60
#define SECONDS_IN_HOUR 3600
#define SECONDS_IN_DAY 86400
#define SECONDS_IN_YEAR 31536000
#define SECONDS_IN_LEAP 31622400

#define MODE_NORMAL 0
#define MODE_DIEL 1

// GUItool: begin automatically generated code
AudioInputI2S            i2s2;           //xy=105,63
LHIRecordQueue           queue1;         //xy=281,63
AudioConnection          patchCord1(i2s2, 0, queue1, 0);
AudioControlSGTL5000     sgtl5000_1;     //xy=265,212
// GUItool: end automatically generated code

const int myInput = AUDIO_INPUT_LINEIN;
unsigned int gainSetting = 4; //default gain setting; can be overridden in setup file
int noDC = 0; // 0 = freezeDC offset; 1 = remove DC offset

// Pin Assignments
#define hydroPowPin 21
#define STOP 20
#define displayPow 20
#define ledGreen 17
#define vSense A14  // moved to Pin 21 (A7) for X1

// Pins used by audio shield
// https://www.pjrc.com/store/teensy3_audio.html
// MEMCS 6
// MOSI 7
// BCLK 9
// SDCS 10
// MCLK 11
// MISO 12
// RX 13
// SCLK 14
// VOL 15
// SDA 18
// SCL 19
// TX 22
// LRCLK 23

// Remember which mode we're doing
int mode = 0;  // 0=stopped, 1=recording, 2=playing
time_t startTime;
time_t stopTime;
time_t t;
time_t burnTime;
byte startHour, startMinute, endHour, endMinute; //used in Diel mode

boolean audioFlag = 1;

boolean LEDSON=1;
boolean introperiod=1;  //flag for introductory period; used for keeping LED on for a little while

int32_t lhi_fsamps[9] = {8000, 16000, 32000, 44100, 48000, 96000, 200000, 250000, 300000};
#define I_SAMP 4

float audio_srate = lhi_fsamps[I_SAMP];//44100.0;
int isf = I_SAMP;

//WMXZ float audioIntervalSec = 256.0 / audio_srate; //buffer interval in seconds
//WMXZ unsigned int audioIntervalCount = 0;
float gainDb;

int recMode = MODE_NORMAL;
long rec_dur = 10;
long rec_int = 30;
int wakeahead = 5;  //wake from snooze to give hydrophone and camera time to power up
int snooze_hour;
int snooze_minute;
int snooze_second;
int buf_count;
long nbufs_per_file;
boolean settingsChanged = 0;


// Pressure/Temp
byte Tbuff[3];
byte Pbuff[3];
int pressureSensor = 0; // flag to indicate which pressure sensor
volatile float pressure_mbar, temperature, depth, pressureOffset_mbar;
volatile boolean togglePress; //flag to toggle conversion of pressure and temperature

// Select which MS58xx sensor is used on board to correctly calculate pressure in mBar
#define MS5837_30bar

#ifdef MS5837_02bar
  #define MS58xx_constant 327680.0
  #define pressAddress 0x76
#endif
#ifdef MS5837_30bar
  #define MS58xx_constant 8192.0
  #define pressAddress 0x76
#endif

//Pressure and temp calibration coefficients
uint16_t PSENS; //pressure sensitivity C1
uint16_t POFF;  //Pressure offset C2
uint16_t TCSENS; //Temp coefficient of pressure sensitivity C3
uint16_t TCOFF; //Temp coefficient of pressure offset C4
uint16_t TREF;  //Ref temperature C5
uint16_t TEMPSENS; //Temperature sensitivity coefficient C6

long file_count;
char filename[40];
char dirname[20];
int folderMonth;

SnoozeAlarm alarm;
SnoozeAudio snooze_audio;
SnoozeBlock config_teensy32(snooze_audio, alarm);


// SD_FAT_TYPE = 0 for SdFat/File as defined in SdFatConfig.h,
// 1 for FAT16/FAT32, 2 for exFAT, 3 for FAT16/FAT32 and exFAT.
#define SD_FAT_TYPE 3
const uint8_t SD_CS_PIN = 10;
#define SPI_CLOCK SD_SCK_MHZ(50)

// Try to select the best SD card configuration.
#if HAS_SDIO_CLASS
#define SD_CONFIG SdioConfig(FIFO_SDIO)
#elif ENABLE_DEDICATED_SPI
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SPI_CLOCK)
#else  // HAS_SDIO_CLASS
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, SHARED_SPI, SPI_CLOCK)
#endif  // HAS_SDIO_CLASS

// Set PRE_ALLOCATE true to pre-allocate file clusters.
const bool PRE_ALLOCATE = true;

// Set SKIP_FIRST_LATENCY true if the first read/write to the SD can
// be avoid by writing a file header or reading the first record.
const bool SKIP_FIRST_LATENCY = true;

#if SD_FAT_TYPE == 0
SdFat sd;
File file;
#elif SD_FAT_TYPE == 1
SdFat32 sd;
File32 file;
#elif SD_FAT_TYPE == 2
SdExFat sd;
ExFile file;
#elif SD_FAT_TYPE == 3
SdFs sd;
FsFile file;
#else  // SD_FAT_TYPE
#error Invalid SD_FAT_TYPE
#endif  // SD_FAT_TYPE

typedef struct {
    char    rId[4];
    unsigned int rLen;
    char    wId[4];
    char    fId[4];
    unsigned int    fLen;
    unsigned short nFormatTag;
    unsigned short nChannels;
    unsigned int nSamplesPerSec;
    unsigned int nAvgBytesPerSec;
    unsigned short nBlockAlign;
    unsigned short  nBitsPerSamples;
    char    dId[4];
    unsigned int    dLen;
} HdrStruct;

HdrStruct wav_hdr;

unsigned char prev_dtr = 0;

struct TIME_HEAD
{
  byte  sec;  
  byte  minute;  
  byte  hour;  
  byte  day;  
  byte  month;  
    byte NU[3];
  int16_t year;  
  int16_t tzOffset; //offset from GMT 
};

void setup() {
  read_myID();
  
  Serial.begin(baud);
  delay(500);

  Serial.println(RTC_TSR);
  Serial.println(RTC_TSR);
  Serial.println(RTC_TSR);

  RTC_CR = 0; // disable RTC
  delay(100);
  Serial.println(RTC_CR,HEX);
  // change capacitance to 26 pF (12.5 pF load capacitance)
  RTC_CR = RTC_CR_SC16P | RTC_CR_SC8P | RTC_CR_SC2P; 
  delay(100);
  RTC_CR = RTC_CR_SC16P | RTC_CR_SC8P | RTC_CR_SC2P | RTC_CR_OSCE;
  delay(100);

  Serial.println(RTC_SR,HEX);
  Serial.println(RTC_CR,HEX);
  Serial.println(RTC_LR,HEX);

  Serial.println(RTC_TSR);
  delay(1000);
  Serial.println(RTC_TSR);
  delay(1000);
  Serial.println(RTC_TSR);
  
  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_400);
  Wire.setDefaultTimeout(10000);

  pinMode(hydroPowPin, OUTPUT);
  pinMode(STOP, INPUT_PULLUP);
  pinMode(vSense, INPUT);
  analogReference(DEFAULT);
  digitalWrite(hydroPowPin, HIGH);

  setSyncProvider(getTeensy3Time); //use Teensy RTC to keep time
  t = getTeensy3Time();
  if (t < 1451606400) Teensy3Clock.set(1451606400);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  //initialize display
  delay(140);
  cDisplay();
  display.println("Loggerhead");
  display.display();

  sensorInit();
  
  pinMode(ledGreen, OUTPUT);
  digitalWrite(ledGreen, HIGH);

  cDisplay();
  display.println("Loggerhead");
  display.display();
  
  // Initialize the SD card
  SPI.setMOSI(7);
  SPI.setSCK(14);
  if (!(sd.begin(10))) {
    // stop here if no SD card, but print a message
    Serial.println("Unable to access the SD card");
    
    while (1) {
      cDisplay();
      display.println("SD error");
      displayClock(displayLine4, getTeensy3Time());
      display.display();
      delay(1000);  
      //resetFunc();
    }
  }
  //SdFile::dateTimeCallback(file_date_time);

  readEEPROM();
  LoadScript(); // secret settings accessible from the card
  writeEEPROM();
  
  // Audio connections require memory, and the record queue
  // uses this memory to buffer incoming audio.
  // initialize now to estimate DC offset during setup
  AudioMemory(MQ+10);
  
  audio_srate = lhi_fsamps[isf];
//WMXZ  audioIntervalSec = 256.0 / audio_srate; //buffer interval in seconds

  AudioInit(isf); // this calls Wire.begin() in control_sgtl5000.cpp

  fileHeader();
  
  cDisplay();

  int roundSeconds = 10;//modulo to nearest x seconds
  if(rec_int > 60) roundSeconds = 60;
  if(rec_int > 300) roundSeconds = 300;
  
  t = getTeensy3Time();
  startTime = t;
  //startTime = getTeensy3Time();
  startTime -= startTime % roundSeconds;  
  startTime += roundSeconds; //move forward
  stopTime = startTime + rec_dur;  // this will be set on start of recording
  
 // if (recMode==MODE_DIEL) checkDielTime();  
  
  nbufs_per_file = (long) (ceil(((rec_dur * audio_srate / 256.0) / (float) NREC)) * (float) NREC);
  long ss = rec_int - wakeahead;
  if (ss<0) ss=0;
  snooze_hour = floor(ss/3600);
  ss -= snooze_hour * 3600;
  snooze_minute = floor(ss/60);
  ss -= snooze_minute * 60;
  snooze_second = ss;
  Serial.print("Snooze HH MM SS ");
  Serial.print(snooze_hour);
  Serial.print(snooze_minute);
  Serial.println(snooze_second);

  Serial.print("rec dur ");
  Serial.println(rec_dur);
  Serial.print("rec int ");
  Serial.println(rec_int);
  Serial.print("Current Time: ");
  printTime(t);
  Serial.print("Start Time: ");
  printTime(startTime);
  
  // Sleep here if won't start for 60 s
  long time_to_first_rec = startTime - t;
  Serial.print("Time to first record ");
  Serial.println(time_to_first_rec);
  
  mode = 0;

  // create first folder to hold data
  folderMonth = -1;  //set to -1 so when first file made will create directory
}

//
// MAIN LOOP
//

int recLoopCount;  //for debugging when does not start record
  
void loop() {
  // Standby mode
  if(mode == 0)
  {
      t = getTeensy3Time();
      cDisplay();
      displaySettings();
      displayClock(displayLine4, t);
      display.display();
      //
      //static uint32_t to; if(t >to) Serial.println(t); to=t;
      //

      if(t >= startTime){      // time to start?
        if(noDC==0) {
          audio_freeze_adc_hp(); // this will lower the DC offset voltage, and reduce noise
          noDC = -1;
        }
        Serial.println("Record Start.");
        
        stopTime = startTime + rec_dur;
        startTime = stopTime + rec_int;
      //  if (recMode==MODE_DIEL) checkDielTime();

        Serial.print("Current Time: ");
        printTime(getTeensy3Time());
        Serial.print("Stop Time: ");
        printTime(stopTime);
        Serial.print("Next Start:");
        printTime(startTime);
        digitalWrite(ledGreen, LOW);
        mode = 1;
        displayOff();
        startRecording();
      }
  }


  // Record mode
  if (mode == 1) {
    continueRecording();  // download data  

//  if(printDiags){
//        if (queue1.getQueue_dropped() > 0){
//      Serial.println(queue1.getQueue_dropped());
//    }
//  }
    if(digitalRead(STOP)==0){
      delay(10); // simple deBounce
        if(digitalRead(STOP)==0){
            // stop recording
            digitalWrite(ledGreen, HIGH);
            queue1.end();
            // update wav file header
            wav_hdr.rLen = 36 + buf_count * 256 * 2;
            wav_hdr.dLen = buf_count * 256 * 2;
            file.seek(0);
            file.write((uint8_t *)&wav_hdr, 44);
            file.close();
            displayOn();  //initialize display
            delay(100);
            cDisplay();
            display.println("Stopped");
            display.setTextSize(1);
            display.println("Safe to turn off");
            display.display();
            delay(58000); // if don't power off, restart
            FileInit();
            digitalWrite(ledGreen, LOW);
        }
    }
    
    
    if(buf_count >= nbufs_per_file){       // time to stop?
      if(rec_int == 0){
        file.close();
        FileInit();  // make a new file
        buf_count = 0;
        if(printDiags) {
          Serial.print("Audio Mem: ");
          Serial.println(AudioMemoryUsageMax());
        }
      }
      else{
        stopRecording();
        long ss = startTime - getTeensy3Time() - wakeahead;
        if (ss<0) ss=0;
        snooze_hour = floor(ss/3600);
        ss -= snooze_hour * 3600;
        snooze_minute = floor(ss/60);
        ss -= snooze_minute * 60;
        snooze_second = ss;
        
        if( (snooze_hour * 3600) + (snooze_minute * 60) + snooze_second >=10){
            digitalWrite(hydroPowPin, LOW); //hydrophone off
            audio_power_down();  // when this is activated, seems to occassionally have trouble restarting; no LRCLK signal or RX on Teensy

            if(printDiags){
              Serial.print("Snooze HH MM SS ");
              Serial.print(snooze_hour);
              Serial.print(snooze_minute);
              Serial.println(snooze_second);
              Serial.flush(); // make sure empty so doesn't prematurely wake
            }           
            delay(100);

            alarm.setRtcTimer(snooze_hour, snooze_minute, snooze_second); // to be compatible with new snooze library
            Snooze.sleep(config_teensy32); 

            /// ... Sleeping ....
            
            // Waking up


            digitalWrite(hydroPowPin, HIGH); // hydrophone on
            delay(300);  // give time for Serial to reconnect to USB
            audio_power_up();  // when use audio_power_down() before sleeping, does not always get LRCLK. This did not fix.  
         }
        Serial.println("Display");
        displayOn();  //initialize display
        mode = 0;  // standby mode
      }
    }
  }
  asm("wfi"); // reduce power between interrupts
}

void startRecording() {
  if (printDiags)  Serial.println("startRecording");
  FileInit();
  buf_count = 0;
  queue1.begin();
  if (printDiags)  Serial.println("Queue Begin");
}


byte buffer[NREC*512];
void continueRecording() {
  if (queue1.available() >= NREC*2) {
    // Fetch 2 blocks (or multiples) from the audio library and copy
    // into a 512 byte buffer.  micro SD disk access
    // is most efficient when full (or multiple of) 512 byte sector size
    // writes are used.
    digitalWrite(ledGreen, HIGH);
    for(int ii=0;ii<NREC;ii++){ 
      byte *ptr = buffer+ii*512;
      memcpy(ptr, queue1.readBuffer(), 256);
      queue1.freeBuffer();
      memcpy(ptr+256, queue1.readBuffer(), 256);
      queue1.freeBuffer();
    }
    digitalWrite(ledGreen, LOW);
    if(file.write(buffer, NREC*512)==-1) resetFunc(); //audio to .wav file
      
    buf_count += NREC;
//WMXZ    audioIntervalCount += NREC;
    
//    if(printDiags){
//      Serial.print(".");
//   }
  }
}

void stopRecording() {
  if (printDiags) Serial.println("stopRecording");
  int maxblocks = AudioMemoryUsageMax();
  if (printDiags) Serial.print("Audio Memory Max");
  if (printDiags) Serial.println(maxblocks);
  byte buffer[512];
  queue1.end();
  //queue1.clear();
  AudioMemoryUsageMaxReset();
  //file.timestamp(T_WRITE,(uint16_t) year(t),month(t),day(t),hour(t),minute(t),second);
  file.close();
  delay(100);
}


void sdInit(){
     if (!(sd.begin(SD_CONFIG))) {
    // stop here if no SD card, but print a message
    Serial.println("Unable to access the SD card");
    
    cDisplay();
    display.println("SD error. Restart.");
    displayClock(displayLine4, getTeensy3Time());
    display.display();
    delay(1000);
  }
}


void FileInit()
{
   t = getTeensy3Time();


// MS5803 start temperature conversion
  if(pressureSensor==1){ 
    readPress();   
    updateTemp();
  }
   
   if (folderMonth != month(t)){
    if(printDiags) Serial.println("New Folder");
    folderMonth = month(t);
    sprintf(dirname, "/%04d-%02d", year(t), folderMonth);
    #if USE_SDFS==1
      FsDateTime::callback = file_date_time;
    #else
      SdFile::dateTimeCallback(file_date_time);
    #endif
    sd.mkdir(dirname);
   }
   pinMode(vSense, INPUT);  // get ready to read voltage

   // open file 
   sd.chdir(dirname);
   sprintf(filename,"%04d%02d%02dT%02d%02d%02d.wav", year(t), month(t), day(t), hour(t), minute(t), second(t));  //filename is DDHHMMSS


   // log file
  #if USE_SDFS==1
    FsDateTime::callback = file_date_time;
  #else
    SdFile::dateTimeCallback(file_date_time);
  #endif

   float voltage = readVoltage();

  // MS5803 pressure and temperature
  if (pressureSensor==1){
      readTemp(); 
      updatePress();
      calcPressTemp();
  }
  // Keller PA7LD pressure and temperature
  if (pressureSensor==2){
    kellerRead();
    kellerConvert();  // start conversion for next reading
  }

   
  sd.chdir(); // only to be sure to start from root
  #if USE_SDFS==1
    if(file.open("LOG.CSV",  O_CREAT | O_APPEND | O_WRITE)){
  #else
    if(file.open("LOG.CSV",  O_CREAT | O_APPEND | O_WRITE)){
  #endif
      file.print(filename);
      file.print(',');
      for(int n=0; n<8; n++){
        file.print(myID[n]);
      }
      file.print(',');
      file.print(gainDb); 
      file.print(',');
      file.print(voltage); 
      file.print(',');
      file.println(codeVersion);
      file.print(',');
      file.print(pressure_mbar); 
      file.print(',');
      file.print(depth); 
      file.print(',');
      file.print(temperature); 


      
      if(voltage < 3.0){
        file.println("Stopping because Voltage less than 3.0 V");
        file.close();  
        // low voltage hang but keep checking voltage
        while(readVoltage() < 3.3){
            delay(30000);
        }
        resetFunc();  // just reset so can start with timing reset
      }
      file.close();
   }
   else{
    if(printDiags) Serial.print("Log open fail.");
    resetFunc();
   }

    
   sd.chdir(dirname);
   file.open(filename, O_WRITE | O_CREAT | O_EXCL);
   Serial.println(filename);
   
   while (!file){
    file_count += 1;
    sprintf(filename,"F%06d.wav",file_count); //if can't open just use count
    sd.chdir(dirname);
    file = sd.open(filename, O_WRITE | O_CREAT | O_EXCL);
    Serial.println(filename);
    delay(10);
    if(file_count>1000) resetFunc(); // give up after many tries
   }


    //intialize .wav file header
    sprintf(wav_hdr.rId,"RIFF");
    wav_hdr.rLen=36;
    sprintf(wav_hdr.wId,"WAVE");
    sprintf(wav_hdr.fId,"fmt ");
    wav_hdr.fLen=0x10;
    wav_hdr.nFormatTag=1;
    wav_hdr.nChannels=1;
    wav_hdr.nSamplesPerSec=audio_srate;
    wav_hdr.nAvgBytesPerSec=audio_srate*2;
    wav_hdr.nBlockAlign=2;
    wav_hdr.nBitsPerSamples=16;
    sprintf(wav_hdr.dId,"data");
    wav_hdr.rLen = 36 + nbufs_per_file * 256 * 2;
    wav_hdr.dLen = nbufs_per_file * 256 * 2;
  
    file.write((uint8_t *)&wav_hdr, 44);

  Serial.print("Buffers: ");
  Serial.println(nbufs_per_file);
}

void fileHeader(){

   sd.chdir(); // only to be sure to star from root
#if USE_SDFS==1
  if(file.open("LOG.CSV",  O_CREAT | O_APPEND | O_WRITE)){
#else
  if(file.open("LOG.CSV",  O_CREAT | O_APPEND | O_WRITE)){
#endif
      file.println("filename,ID,gain (dB),Voltage,Version, pressure mBar,depth,temp");
      file.close();
  }
}

//This function returns the date and time for SD card file access and modify time. One needs to call in setup() to register this callback function: SdFile::dateTimeCallback(file_date_time);
void file_date_time(uint16_t* date, uint16_t* time) 
{
  t = getTeensy3Time();
  #if USE_SDFS==1
    *date=FS_DATE(year(t),month(t),day(t));
    *time=FS_TIME(hour(t),minute(t),second(t));
  #else
    *date=FAT_DATE(year(t),month(t),day(t));
    *time=FAT_TIME(hour(t),minute(t),second(t));
  #endif
}

void AudioInit(int ifs){
 // Instead of using audio library enable; do custom so only power up what is needed in sgtl5000_LHI
  I2S_modification(lhi_fsamps[ifs], 16);
  audio_enable(ifs);
}

void calcGain(){
    switch(gainSetting){
    case 0: gainDb = -20 * log10(3.12 / 2.0); break;
    case 1: gainDb = -20 * log10(2.63 / 2.0); break;
    case 2: gainDb = -20 * log10(2.22 / 2.0); break;
    case 3: gainDb = -20 * log10(1.87 / 2.0); break;
    case 4: gainDb = -20 * log10(1.58 / 2.0); break;
    case 5: gainDb = -20 * log10(1.33 / 2.0); break;
    case 6: gainDb = -20 * log10(1.11 / 2.0); break;
    case 7: gainDb = -20 * log10(0.94 / 2.0); break;
    case 8: gainDb = -20 * log10(0.79 / 2.0); break;
    case 9: gainDb = -20 * log10(0.67 / 2.0); break;
    case 10: gainDb = -20 * log10(0.56 / 2.0); break;
    case 11: gainDb = -20 * log10(0.48 / 2.0); break;
    case 12: gainDb = -20 * log10(0.40 / 2.0); break;
    case 13: gainDb = -20 * log10(0.34 / 2.0); break;
    case 14: gainDb = -20 * log10(0.29 / 2.0); break;
    case 15: gainDb = -20 * log10(0.24 / 2.0); break;
  }
}


time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}

unsigned long processSyncMessage() {
  unsigned long pctime = 0L;
  const unsigned long DEFAULT_TIME = 1451606400; // Jan 1 2016
} 
  
// Calculates Accurate UNIX Time Based on RTC Timestamp
unsigned long RTCToUNIXTime(TIME_HEAD *tm){
    int i;
    unsigned const char DaysInMonth[] = {31,28,31,30,31,30,31,31,30,31,30,31};
    unsigned long Ticks = 0;

    long yearsSince = tm->year + 30; // Years since 1970
    long numLeaps = yearsSince >> 2; // yearsSince / 4 truncated

    if((!(tm->year%4)) && (tm->month>2))
            Ticks+=SECONDS_IN_DAY;  //dm 8/9/2012  If current year is leap, add one day

    // Calculate Year Ticks
    Ticks += (yearsSince-numLeaps)*SECONDS_IN_YEAR;
    Ticks += numLeaps * SECONDS_IN_LEAP;

    // Calculate Month Ticks
    for(i=0; i < tm->month-1; i++){
         Ticks += DaysInMonth[i] * SECONDS_IN_DAY;
    }

    // Calculate Day Ticks
    Ticks += (tm->day - 1) * SECONDS_IN_DAY;

    // Calculate Time Ticks CHANGES ARE HERE
    Ticks += (unsigned long)tm->hour * SECONDS_IN_HOUR;
    Ticks += (unsigned long)tm->minute * SECONDS_IN_MINUTE;
    Ticks += tm->sec;

    return Ticks;
}

void resetFunc(void){
  EEPROM.write(20, 1); // reset indicator register
  CPU_RESTART
}


void read_EE(uint8_t word, uint8_t *buf, uint8_t offset)  {
  noInterrupts();
  FTFL_FCCOB0 = 0x41;             // Selects the READONCE command
  FTFL_FCCOB1 = word;             // read the given word of read once area

  // launch command and wait until complete
  FTFL_FSTAT = FTFL_FSTAT_CCIF;
  while(!(FTFL_FSTAT & FTFL_FSTAT_CCIF))
    ;
  *(buf+offset+0) = FTFL_FCCOB4;
  *(buf+offset+1) = FTFL_FCCOB5;       
  *(buf+offset+2) = FTFL_FCCOB6;       
  *(buf+offset+3) = FTFL_FCCOB7;       
  interrupts();
}

    
void read_myID() {
  read_EE(0xe,myID,0); // should be 04 E9 E5 xx, this being PJRC's registered OUI
  read_EE(0xf,myID,4); // xx xx xx xx
}

float readVoltage(){
   float  voltage = 0;
   float vDivider = 2.1; //when using 3.3 V ref R9 100K
   //float vDivider = 4.5;  // when using 1.2 V ref R9 301K
   float vRef = 3.3;
   pinMode(vSense, INPUT);  // get ready to read voltage
   if (vRef==1.2) analogReference(INTERNAL); //1.2V ref more stable than 3.3 according to PJRC
   int navg = 32;
   for(int n = 0; n<navg; n++){
    voltage += (float) analogRead(vSense);
   }
   voltage = vDivider * vRef * voltage / 1024.0 / navg;  
   pinMode(vSense, OUTPUT);  // done reading voltage
   return voltage;
}

void sensorInit(){
  // Pressure--auto identify which if any is present
  cDisplay();

  // Keller
  int nAvg = 11;
  float pressureSum;
  if(kellerInit()) {
    pressureSensor = 2;   // 2 if present
    Serial.println("Keller Pressure Detected");
    kellerConvert();
    delay(20);
    kellerRead();
    for(int n=1; n<nAvg; n++){
      kellerConvert();
      delay(20);
      kellerRead();
      delay(100);
      
      pressureSum+= pressure_mbar;
      pressureOffset_mbar = pressureSum / n;

      cDisplay();
      display.println("Press Deep");
      display.print("Offset mBar:"); display.println(pressureOffset_mbar);
      display.print("Depth:"); display.println(depth);
      display.print("Temp:"); display.println(temperature);
      display.display();
    }
  }

  // Measurement Specialties
  if(pressInit()){
    pressureSensor = 1;
    Serial.println("MS Pressure Detected");
    updatePress();
    delay(50);
    readPress();
    updateTemp();
    delay(50);
    readTemp();
    for(int n=1; n<nAvg; n++){
      updatePress();
      delay(50);
      readPress();
      updateTemp();
      delay(50);
      readTemp();
      calcPressTemp();
      pressureSum+= pressure_mbar;
      pressureOffset_mbar = pressureSum / n;
      delay(100);

      cDisplay();
      display.println("MS Pressure");
      display.print("Offset mBar:"); display.println(pressureOffset_mbar);
      display.print("Depth:"); display.println(depth);
      display.print("Temp:"); display.println(temperature);
      display.display(); 
    }
    calcPressTemp();
    pressureOffset_mbar = pressureOffset_mbar / nAvg;
  }
  
  Serial.print("Pressure (mBar): "); Serial.println(pressure_mbar);
  Serial.print("Depth: "); Serial.println(depth);
  Serial.print("Temperature: "); Serial.println(temperature);


  if(pressureSensor==0) {
    cDisplay();
    display.println("Pressure");
    display.println("None Detected");
    display.display();
  }
  delay(5000);
}
