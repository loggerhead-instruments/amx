//
// AMX--Audio, Motion, Light, Pressure datalogger
//
// Loggerhead Instruments
// 2016
// David Mann
// 
// Modified from PJRC audio code
// http://www.pjrc.com/store/teensy3_audio.html
//
// Loggerhead AMX board is required for accelerometer, mageter, gyroscope, RGB light, pressure, and temperature sensors
//

// Note: Need to change Pressure/Temperature coefficient for MS5801 1 Bar versus 30 Bar sensor


#include <Audio.h>  //this also includes SD.h from lines 89 & 90
#include <analyze_fft256.h>
//#include <Wire.h>
#include <i2c_t3.h>  //https://github.com/nox771/i2c_t3
#include <SPI.h>
//#include <SdFat.h>
#include "amx32.h"
#include <Snooze.h>  //using https://github.com/duff2013/Snooze; uncomment line 62 #define USE_HIBERNATE
#include <TimeLib.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h> // modify so calls i2c_t3 not Wire.h
#include <Adafruit_FeatherOLED.h>
#include <EEPROM.h>
#include <TimerOne.h>

#define CPU_RESTART_ADDR (uint32_t *)0xE000ED0C
#define CPU_RESTART_VAL 0x5FA0004
#define CPU_RESTART (*CPU_RESTART_ADDR = CPU_RESTART_VAL);

#define OLED_RESET 4

#define displayLine1 0
#define displayLine2 8
#define displayLine3 16
#define displayLine4 25
Adafruit_FeatherOLED display = Adafruit_FeatherOLED();
#define BOTTOM 25

// set this to the hardware serial port you wish to use
#define HWSERIAL Serial1  // Rx: 0; Tx: 1

#define SPYCAM 1
#define FLYCAM 2
#define UCAM 3

// Select which MS58xx sensor is used on board to correctly calculate pressure in mBar
//#define MS5803_01bar 32768.0
//#define MS5803_30bar 819.2

#define MS5837_30bar

#ifdef MS5837_02bar
  #define MS58xx_constant 327680.0
  #define pressAddress 0x76
#endif
#ifdef MS5837_30bar
  #define MS58xx_constant 8192.0
  #define pressAddress 0x76
#endif

// 
// Dev settings
//
static boolean printDiags = 0;  // 1: serial print diagnostics; 0: no diagnostics 2=verbose
int dd = 1; //display on
long rec_dur = 3600; // seconds; default = 3600s
long rec_int = 0;

int nPlayBackFiles = 5; // number of playback files
int minPlayBackInterval = 120; // keep playbacks from being closer than x seconds
int longestPlayback = 30; // longest file for playback, used to power down playback board
float playBackDepthThreshold = 10.0; // tag must go deeper than this depth to trigger threshold
float ascentDepthTrigger = 5.0; // after exceed playBackDepthThreshold, must ascend this amount to trigger playback
float playBackResetDepth = 2.0; // tag needs to come back above this depth before next playback can happen
int maxPlayBacks = 20; // maximum number of times to play

int simulateDepth = 0;
float depthProfile[] = {0.0, 12.0, 1.0, 12.0, 4.0, 3.0, 10.0, 20.0, 50.0, 10.0, 0.0, 1.0, 40.0, 11.0, 5.0, 2.0, 0.0, 50.0, 2.0, 4.0,
                      0.0, 12.0, 1.0, 20.0, 4.0, 3.0, 10.0, 20.0, 50.0, 10.0, 0.0, 1.0, 12.0, 11.0, 5.0, 2.0, 0.0, 60.0, 2.0, 4.0,
                      0.0, 12.0, 1.0, 20.0, 4.0, 3.0, 10.0, 20.0, 50.0, 10.0, 0.0, 1.0, 12.0, 11.0, 5.0, 2.0, 0.0, 70.0, 2.0, 4.0}; //simulated depth profile; one value per minute


int camType = SPYCAM; // when on continuously cameras make a new file every 10 minutes

int camFlag = 0;
boolean camWave = 0; // one flag to swtich all settings to use camera control and wav files (camWave = 1)

float max_cam_hours_rec = 10.0; // turn off camera after max_cam_hours_rec to save power; SPYCAM gets ~10 hours with 32 GB card--depends on compression
byte fileType = 1; //0=wav, 1=amx
int moduloSeconds = 60; // round to nearest start time

float depthThreshold = 1.0; //depth threshold is given as a positive depth (e.g. 2: if depth < 2 m VHF will go on)
int saltThreshold = 10; // if voltage difference with digital out ON - digital out OFF is less than this turn off LED
//
//
//

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
AudioAnalyzeFFT256       fft256_1; 
AudioRecordQueue         queue1;         //xy=281,63
AudioConnection          patchCord1(i2s2, 0, queue1, 0);
AudioConnection          patchCord2(i2s2, 0, fft256_1, 0);
AudioControlSGTL5000     sgtl5000_1;     //xy=265,212
// GUItool: end automatically generated code

const int myInput = AUDIO_INPUT_LINEIN;
float gainDb;
float hydroCal = -180;
int noDC = 0; // 0 = freezeDC offset; 1 = remove DC offset

// Pin Assignments
#define CAM_TRIG 4
#define hydroPowPin 21
#define VHF 8
#define displayPow 20
#define SALT A11
#define saltSIG 3
#define ledGreen 17
#define BURN 5
#define AUDIO_AMP 2
#define usbSense 6
#define vSense A14  // moved to Pin 21 for X1
#define PLAY_POW 16
#define STOP 20

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
time_t playTime;
byte startHour, startMinute, endHour, endMinute; //used in Diel mode

boolean imuFlag = 1;
boolean rgbFlag = 1;
int burnFlag = 0; // set by setup.txt file if use BW or BM
byte pressure_sensor = 0; //0=none, 1=MS5802, 2=Keller PA7LD; autorecognized 
boolean audioFlag = 1;
boolean CAMON = 0;

boolean briteFlag = 0; // bright LED
long burnMinutes = 0;
int burnLog = 0; //burn status for log file

volatile boolean LEDSON = 1;
boolean introperiod=1;  //flag for introductory period; used for keeping LED on for a little while

int update_rate = 100;  // rate (Hz) at which interrupt to read RGB and P/T sensors will run, so sensor_srate needs to <= update_rate
float sensor_srate = 1.0;
float imu_srate = 100.0;
float audio_srate = 44100.0;
int toneDetect = 0; // counter for detecting burn tone
int refBin = 50; //reference signal bin
int toneBin = 58; // tone signal bin

int accel_scale = 16; //full scale on accelerometer [2, 4, 8, 16] (example cmd code: AS 8)


// Playback
int playNow = 0;
int trackNumber = 0;
int playBackDepthExceeded = 0;
float maxDepth;  
int nPlayed = 0;

float audioIntervalSec = 256.0 / audio_srate; //buffer interval in seconds
unsigned int audioIntervalCount = 0;
int systemGain = 4; // SG in script file

int recMode = MODE_NORMAL;

int wakeahead = 20;  //wake from snooze to give hydrophone and camera time to power up
int snooze_hour;
int snooze_minute;
int snooze_second;
volatile long buf_count;
float total_hour_recorded = 0.0;
long nbufs_per_file;
boolean settingsChanged = 0;

long file_count;
char filename[25];
char dirname[8];
int folderMonth;
//SnoozeBlock snooze_config;
SnoozeAlarm alarm;
SnoozeAudio snooze_audio;
SnoozeBlock config_teensy32(snooze_audio, alarm);

// The file where data is recorded
File frec;

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
unsigned int rms;

// Header for amx files
DF_HEAD dfh;
SID_SPEC sidSpec[SID_MAX];
SID_REC sidRec[SID_MAX];
SENSOR sensor[SENSOR_MAX]; //structure to hold sensor specifications. e.g. MPU9250, MS5803, PA47LD, ISL29125

unsigned char prev_dtr = 0;

// IMU
int FIFOpts;
#define IMUBUFFERSIZE 1800 // used this length because it is divisible by 18 bytes (e.g. A*3,M*3,G*3);
volatile byte imuBuffer[IMUBUFFERSIZE]; // buffer used to store IMU sensor data before writes in bytes
volatile byte time2writeIMU=0; 
volatile int IMUCounter = 0;
volatile int bufferposIMU = 0;
int halfbufIMU = IMUBUFFERSIZE/2;
volatile boolean firstwrittenIMU;
volatile uint8_t imuTempBuffer[20];
int16_t accel_x;
int16_t accel_y;
int16_t accel_z;
int16_t mag_x;
int16_t mag_y;
int16_t mag_z;
int16_t gyro_x;
int16_t gyro_y;
int16_t gyro_z;
float gyro_temp;
int magXoffset;
int magYoffset;
int magZoffset;
float pitch, roll, yaw;

// RGB
int16_t islRed;
int16_t islBlue;
int16_t islGreen;

// Pressure/Temp
byte Tbuff[3];
byte Pbuff[3];
volatile float pressure_mbar, temperature, depth;
volatile boolean togglePress; //flag to toggle conversion of pressure and temperature

//Pressure and temp calibration coefficients
uint16_t PSENS; //pressure sensitivity C1
uint16_t POFF;  //Pressure offset C2
uint16_t TCSENS; //Temp coefficient of pressure sensitivity C3
uint16_t TCOFF; //Temp coefficient of pressure offset C4
uint16_t TREF;  //Ref temperature C5
uint16_t TEMPSENS; //Temperature sensitivity coefficient C6

// Pressure, Temp double buffer
#define PTBUFFERSIZE 40
volatile float PTbuffer[PTBUFFERSIZE];
volatile byte time2writePT = 0; 
volatile int ptCounter = 0;
volatile byte bufferposPT=0;
byte halfbufPT = PTBUFFERSIZE/2;
volatile boolean firstwrittenPT;

// RGB buffer
#define RGBBUFFERSIZE 120
volatile byte RGBbuffer[RGBBUFFERSIZE];
volatile byte time2writeRGB=0; 
volatile int RGBCounter = 0;
volatile byte bufferposRGB=0;
byte halfbufRGB = RGBBUFFERSIZE/2;
volatile boolean firstwrittenRGB;

#define HWSERIAL Serial1

IntervalTimer slaveTimer;

void setup() {
  dfh.Version = 1000;
  dfh.UserID = 5555;

  if (camWave){
    imuFlag = 0;
    rgbFlag = 0;
    audioFlag = 1;
    camFlag = 1;
    briteFlag = 1;
    fileType = 0; // 0 = wav
  }

  read_myID();
  
  Serial.begin(baud);
  HWSERIAL.begin(9600); //playback

  delay(500);
 // Wire.begin();
  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_400);
  Wire.setDefaultTimeout(10000);

  displayOn();
  cDisplay();
  display.println("Loggerhead");
  display.display();

  // Initialize the SD card
  SPI.setMOSI(7);
  SPI.setSCK(14);
  if (!(SD.begin(10))) {
    // stop here if no SD card, but print a message
    Serial.println("Unable to access the SD card");
    
    while (1) {
      cDisplay();
      display.println("SD error");
      display.display();
      for (int flashMe=0; flashMe<3; flashMe++){
      delay(100);
      digitalWrite(ledGreen, HIGH);
      delay(100);
      digitalWrite(ledGreen, LOW);
      }
      delay(400);
    }
  }

  LoadScript();
  logFileHeader();

  sensorInit(); // initialize and test sensors
  setGain();

  if(printDiags > 0){
      Serial.print("YY-MM-DD HH:MM:SS ");
      // show 3 ticks to know crystal is working
      for (int n=0; n<3; n++){
        printTime(getTeensy3Time());
        delay(1000);
      }
   }

  pinMode(usbSense, OUTPUT);
  digitalWrite(usbSense, LOW); // make sure no pull-up
  pinMode(usbSense, INPUT);
  delay(500);    

  Serial.println("Loggerhead");
  //display.println("USB <->");
  //display.display();
  // Check for external USB connection to microSD
  digitalWrite(ledGreen, HIGH);
  digitalWrite(hydroPowPin, HIGH);

  if(!printDiags){ 
   while(digitalRead(usbSense)){
      pinMode(usbSense, OUTPUT);
      digitalWrite(usbSense, LOW); // forces low if USB power pulled
      pinMode(usbSense, INPUT);
      delay(500);
    }
  }

  
  setSyncProvider(getTeensy3Time); //use Teensy RTC to keep time


// ULONG newtime;
 
  // Power down USB if not using Serial monitor
  if (printDiags==0){
    //  usbDisable();
  }
  
//
//  cDisplay();
//  display.println("Loggerhead");
//  display.display();
  
 
  //SdFile::dateTimeCallback(file_date_time);

  
  mpuInit(1);; // update MPU with new settings
  setupDataStructures();

  //cDisplay();

  t = getTeensy3Time();

  if(burnFlag==2){
    burnTime = t + (burnMinutes * 60);
  }

  startTime = t + 60; // make sure have at least 1 minute
  startTime -= startTime % moduloSeconds;  //modulo to nearest 5 minutes
  startTime += moduloSeconds; //move forward
 
  stopTime = startTime + rec_dur;  // this will be set on start of recording
  
  if (recMode==MODE_DIEL) checkDielTime();  
  
  nbufs_per_file = (long) (rec_dur * audio_srate / 256.0);
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

  // Audio connections require memory, and the record queue
  // uses this memory to buffer incoming audio.
  AudioMemory(100);
  AudioInit(); // this calls Wire.begin() in control_sgtl5000.cpp
 // fft256_1.averageTogether(160); // number of FFTs to average together
  
  digitalWrite(hydroPowPin, HIGH);
  if (camFlag) cam_wake();
  mode = 0;

  // create first folder to hold data
  folderMonth = -1;  //set to -1 so when first file made will create directory
  
  //if (fileType) Timer1.initialize(1000000 / update_rate); // initialize with 100 ms period when update_rate = 10 Hz

}

//
// MAIN LOOP
//

int recLoopCount;  //for debugging when does not start record

void loop() {
  // if plug in USB power--get out of main loop
  if(!printDiags){
    if (digitalRead(usbSense)){  
        if (mode==1) stopRecording();
        resetFunc();
      }
  }

  t = getTeensy3Time();
  if((t >= burnTime) & (burnFlag>0)){
     digitalWrite(BURN, HIGH);  // burn on
     digitalWrite(VHF, HIGH);   // VHF on
     digitalWrite(ledGreen, LOW);
     burnLog = 1;

    // if burning and voltage is low; stop recording
     if(readVoltage() < 3.6){  
       if (mode == 1) stopRecording();
       if (camFlag) {
        cam_stop();
        delay(100);
       }
       frec.close();
       audio_power_down();
       digitalWrite(hydroPowPin, LOW); //hydrophone off
       playOff(); // power down playback board
        
       while(1){
          alarm.setAlarm(0, 2, 0);  // sleep for 2 minutes
          Snooze.sleep(config_teensy32);
  
          // ... asleep ...
          for(int n = 0; n<10; n++){
            digitalWrite(ledGreen, HIGH);
            delay(200);
            digitalWrite(ledGreen, LOW);
            delay(100);
          }
       }
     }
  }
  
  // Standby mode
  if(mode == 0)
  {
      if(dd){
        cDisplay();
        displaySettings();
        displayClock(BOTTOM, t);
        display.display();
      }
      
      if((t >= startTime - 4) & CAMON==1 & (camType==SPYCAM)){ //start camera 4 seconds before to give a chance to get going
        if (camFlag)  cam_start();
      }
      if(t >= startTime){      // time to start?
        Serial.println("Record Start.");
        cDisplay();
        display.display();

        if(noDC==0) {
          audio_freeze_adc_hp(); // this will lower the DC offset voltage, and reduce noise
          noDC = -1;
        }
        
        stopTime = startTime + rec_dur;
        startTime = stopTime + rec_int;
        if (recMode==MODE_DIEL) checkDielTime();

        Serial.print("Current Time: ");
        printTime(getTeensy3Time());
        Serial.print("Stop Time: ");
        printTime(stopTime);
        Serial.print("Next Start:");
        printTime(startTime);

        //convert pressure and temperature for first reading
        updateTemp();
        displayOff();

        mode = 1;
        startRecording();
      }
      delay(100);
  }


  // Record mode
  if (mode == 1) {
    continueRecording();  // download data  

    // Check if stop button pressed
    if(digitalRead(STOP)==0){
      delay(10); // simple deBounce
      if(digitalRead(STOP)==0){
        stopRecording();
        digitalWrite(ledGreen, HIGH);
        if(dd){
          displayOn();
          delay(1000);
          cDisplay();
          display.println("Stopped");
          display.println();
          display.print("Safe to turn off");
          display.display();
        }
        delay(58000); // if don't power off in 60s, restart
        if(dd){
          cDisplay();
          display.print("Restarting");
          display.display();
          delay(2000);
          displayOff();
        }
        FileInit();
        digitalWrite(ledGreen, LOW);
      }
    }

    // check for acoustic release signal
    // must be 10 reads of 9991.4 Hz 12 dB greater than bin 50
    // binsize = 44100/256 = 172.265625 Hz
    // bin 50 = 8613.28
    // bin 58 = 9991.4 Hz
    // center of bin 58 = 10077.5

    /*
    if(fft256_1.available()){
      float n1, n2, n3;
      n1 = fft256_1.read(refBin);
      n2 = fft256_1.read(toneBin);
      if(n2 > 0.000001){
          if(n1> 0.000001){
            if((n2/n1) > 4) toneDetect += 1;
          }
          else
          toneDetect += 1;
      }
      else
      toneDetect = 0;
      
      if(toneDetect > 10) {
        digitalWrite(BURN, HIGH); // burn on
        burnLog = 2;
      }
      
      if(printDiags==1){
        Serial.print("FFT: ");
        for (int i=50; i<66; i++){ //bin 58 = 9991.4 Hz
          float n = fft256_1.read(i);
          if( n > 0.000001) {
            Serial.print(20*log10(n));
            Serial.print(" ");
          }
          else
            Serial.print(" - ");
        }
        Serial.println();
      }
    }
    */
//    if(printDiags){  //this is to see if code still running when queue fails change to printDiags to hide
//      recLoopCount++;
//      if(recLoopCount>50){
//        recLoopCount = 0;
//        t = getTeensy3Time();
//        cDisplay();
//        if(rec_int > 0) {
//          display.println("Rec");
//          displayClock(stopTime, 20);
//        }
//        else{
//          display.println("Rec Contin");
//          display.setTextSize(1);
//          display.println(filename);
//        }
//        displayClock(t, BOTTOM);
//        display.display();
//      }
//    }

    // write IMU values to file
    if(time2writeIMU==1)
    {
      if(frec.write((uint8_t *) & sidRec[3],sizeof(SID_REC))==-1) resetFunc();
      if(frec.write((uint8_t *) & imuBuffer[0], halfbufIMU)==-1) resetFunc(); 
      time2writeIMU = 0;
      if ((LEDSON==1) & (introperiod==1)) digitalWrite(ledGreen, HIGH);  //LEDS on for first file) digitalWrite(ledGreen, HIGH);
    }
    if(time2writeIMU==2)
    {
      if(frec.write((uint8_t *) & sidRec[3],sizeof(SID_REC))==-1) resetFunc();
      if(frec.write((uint8_t *) & imuBuffer[halfbufIMU], halfbufIMU)==-1) resetFunc();     
      time2writeIMU = 0;
      if (LEDSON==1) digitalWrite(ledGreen, LOW);
    } 
    
    // write Pressure & Temperature to file
    if(time2writePT==1)
    { 
      if(frec.write((uint8_t *)&sidRec[1],sizeof(SID_REC))==-1) resetFunc();
      if(frec.write((uint8_t *)&PTbuffer[0], halfbufPT * 4)==-1) resetFunc(); 
      time2writePT = 0;
    }
    if(time2writePT==2)
    {
      if(frec.write((uint8_t *)&sidRec[1],sizeof(SID_REC))==-1) resetFunc();
      if(frec.write((uint8_t *)&PTbuffer[halfbufPT], halfbufPT * 4)==-1) resetFunc();     
      time2writePT = 0;
    }   
  
    // write RGB values to file
    if(time2writeRGB==1)
    {
      if(frec.write((uint8_t *)&sidRec[2],sizeof(SID_REC))==-1) resetFunc();
      if(frec.write((uint8_t *)&RGBbuffer[0], halfbufRGB)==-1) resetFunc(); 
      time2writeRGB = 0;
    }
    if(time2writeRGB==2)
    {
      if(frec.write((uint8_t *)&sidRec[2],sizeof(SID_REC))==-1) resetFunc();
      if(frec.write((uint8_t *)&RGBbuffer[halfbufRGB], halfbufRGB)==-1) resetFunc();     
      time2writeRGB = 0;
    } 

    if(nPlayBackFiles>0) checkPlay(); // checks if depth profile matches trigger for playback, sets flag and file number, plays

    
    
    if(buf_count >= nbufs_per_file){       // time to stop?
      
      total_hour_recorded += (float) rec_dur / 3600.0;
      if(total_hour_recorded > 1.0) introperiod = 0;  //LEDS on for first file
      if(rec_int == 0){
        if(printDiags > 0){
          Serial.print("Audio Memory Max");
          Serial.println(AudioMemoryUsageMax());
        }
        frec.close();

        FileInit();  // make a new file
        buf_count = 0;
      }
      else
      {
        stopRecording();
        if (camFlag) {
          cam_stop();
          delay(100);
        }
        long ss = startTime - getTeensy3Time() - wakeahead;
        if (ss<0) ss=0;
        snooze_hour = floor(ss/3600);
        ss -= snooze_hour * 3600;
        snooze_minute = floor(ss/60);
        ss -= snooze_minute * 60;
        snooze_second = ss;
        if((snooze_hour * 3600) + (snooze_minute * 60) + snooze_second >=10){
            digitalWrite(hydroPowPin, LOW); //hydrophone off
            if (imuFlag) mpuInit(0);  //gyro to sleep
            if (rgbFlag) islSleep(); // RGB light sensor
            audio_power_down();
            if (camFlag) cam_off();
//            cDisplay();
//            display.display();
//            delay(100);
//            display.ssd1306_command(SSD1306_DISPLAYOFF); 
            if(printDiags > 0){
              printTime(getTeensy3Time());
              Serial.print("Snooze HH MM SS ");
              Serial.print(snooze_hour);
              Serial.print(snooze_minute);
              Serial.println(snooze_second);
            }
            delay(100);
  
           // AudioNoInterrupts();
            
            //snooze_config.setAlarm(snooze_hour, snooze_minute, snooze_second);
            //delay(100);
            //Snooze.sleep( snooze_config );
            //Snooze.deepSleep(snooze_config);
            //Snooze.hibernate( snooze_config);
  
            alarm.setAlarm(snooze_hour, snooze_minute, snooze_second);
            Snooze.sleep(config_teensy32);
       
            /// ... Sleeping ....
            
            // Waking up
            
           // if (printDiags==0) usbDisable();
           // display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  //initialize display
           if(printDiags>0) printTime(getTeensy3Time());
            digitalWrite(hydroPowPin, HIGH); // hydrophone on 
          //  audio_enable();
          //  AudioInterrupts();
            audio_power_up();
            if (camFlag)  cam_wake();
            if (rgbFlag) islInit(); // RGB light sensor
            if (imuFlag) mpuInit(1);  //start gyro
            //sdInit();  //reinit SD because voltage can drop in hibernate
         }
        if(dd) displayOn();
        mode = 0;
      }
    }
  }
}


void startRecording() {
  Serial.println("startRecording");
  FileInit();
 // if (fileType) Timer1.attachInterrupt(sampleSensors);
  if (fileType) {
    slaveTimer.begin(sampleSensors, 1000000 / update_rate); 
    //slaveTimer.priority(200);
  }
  buf_count = 0;
  queue1.begin();
  Serial.println("Queue Begin");
}

void continueRecording() {
    byte buffer[512];
    // Fetch 2 blocks from the audio library and copy
    // into a 512 byte buffer.  The Arduino SD library
    // is most efficient when full 512 byte sector size
    // writes are used.
   // if(LEDSON | introperiod) digitalWrite(ledGreen,HIGH);
    if(queue1.available() >= 2) {
      buf_count += 1;
      audioIntervalCount += 1;
      memcpy(buffer, queue1.readBuffer(), 256);
      queue1.freeBuffer();
      memcpy(buffer+256, queue1.readBuffer(), 256);
      queue1.freeBuffer();
      if (fileType==0){
        frec.write(buffer, 512); //audio to .wav file
      }
      else{
        frec.write((uint8_t *)&sidRec[0],sizeof(SID_REC)); //audio to .amx file
        frec.write(buffer, 512); 
      }
    }

   // if(LEDSON | introperiod) digitalWrite(ledGreen,LOW);

    if(printDiags == 2){
      Serial.print(".");
   }
}

void stopRecording() {
  Serial.println("stopRecording");
  int maxblocks = AudioMemoryUsageMax();
  Serial.print("Audio Memory Max");
  Serial.println(maxblocks);
  byte buffer[512];
  queue1.end();
  digitalWrite(ledGreen, LOW);
  //queue1.clear();

  AudioMemoryUsageMaxReset();
  //if (fileType) Timer1.detachInterrupt();
  if (fileType) slaveTimer.end();
  
  // to do: add flush for PTbuffer and Gyro
  
  //frec.timestamp(T_WRITE,(uint16_t) year(t),month(t),day(t),hour(t),minute(t),second);
  frec.close();
  delay(100);
  //calcRMS();
  //Serial.println(rms);
}



void setupDataStructures(void){
  // setup sidSpec and sidSpec buffers...hard coded for now
  
  // audio
  strncpy(sensor[0].chipName, "SGTL5000", STR_MAX);
  sensor[0].nChan = 1;
  strncpy(sensor[0].name[0], "audio1", STR_MAX);
  strncpy(sensor[0].name[1], "audio2", STR_MAX);
  strncpy(sensor[0].name[2], "audio3", STR_MAX);
  strncpy(sensor[0].name[3], "audio4", STR_MAX);
  strncpy(sensor[0].units[0], "Pa", STR_MAX);
  strncpy(sensor[0].units[1], "Pa", STR_MAX);
  strncpy(sensor[0].units[2], "Pa", STR_MAX);
  strncpy(sensor[0].units[3], "Pa", STR_MAX);
  sensor[0].cal[0] = gainDb + hydroCal; // this needs to be set based on hydrophone sensitivity + chip gain
  sensor[0].cal[1] = gainDb + hydroCal;
  sensor[0].cal[2] = gainDb + hydroCal;
  sensor[0].cal[3] = gainDb + hydroCal;

  // Pressure/Temperature
  if(pressure_sensor == 1) {
    strncpy(sensor[1].chipName, "MS5803", STR_MAX);
    sensor[1].nChan = 2;
    strncpy(sensor[1].name[0], "pressure", STR_MAX);
    strncpy(sensor[1].name[1], "temp", STR_MAX);
    strncpy(sensor[1].units[0], "mBar", STR_MAX);
    strncpy(sensor[1].units[1], "degreesC", STR_MAX);
    sensor[1].cal[0] = 1.0;
    sensor[1].cal[1] = 1.0;
  }
  else{
    strncpy(sensor[1].chipName, "PA7LD", STR_MAX);
    sensor[1].nChan = 2;
    strncpy(sensor[1].name[0], "pressure", STR_MAX);
    strncpy(sensor[1].name[1], "temp", STR_MAX);
    strncpy(sensor[1].units[0], "mBar", STR_MAX);
    strncpy(sensor[1].units[1], "degreesC", STR_MAX);
    sensor[1].cal[0] = 1.0;
    sensor[1].cal[1] = 1.0;
  }

  
  // RGB light
  strncpy(sensor[2].chipName, "ISL29125", STR_MAX);
  sensor[2].nChan = 3;
  strncpy(sensor[2].name[0], "red", STR_MAX);
  strncpy(sensor[2].name[1], "green", STR_MAX);
  strncpy(sensor[2].name[2], "blue", STR_MAX);
  strncpy(sensor[2].units[0], "uWpercm2", STR_MAX);
  strncpy(sensor[2].units[1], "uWpercm2", STR_MAX);
  strncpy(sensor[2].units[2], "uWpercm2", STR_MAX);
  sensor[2].cal[0] = 20.0 / 65536.0;
  sensor[2].cal[1] = 18.0 / 65536.0;
  sensor[2].cal[2] = 30.0 / 65536.0;


  // IMU
  strncpy(sensor[3].chipName, "MPU9250", STR_MAX);
  sensor[3].nChan = 9;
  strncpy(sensor[3].name[0], "accelX", STR_MAX);
  strncpy(sensor[3].name[1], "accelY", STR_MAX);
  strncpy(sensor[3].name[2], "accelZ", STR_MAX);
  //strncpy(sensor[3].name[3], "temp-21C", STR_MAX);
  strncpy(sensor[3].name[3], "gyroX", STR_MAX);
  strncpy(sensor[3].name[4], "gyroY", STR_MAX);
  strncpy(sensor[3].name[5], "gyroZ", STR_MAX);
  strncpy(sensor[3].name[6], "magX", STR_MAX);
  strncpy(sensor[3].name[7], "magY", STR_MAX);
  strncpy(sensor[3].name[8], "magZ", STR_MAX);
  strncpy(sensor[3].units[0], "g", STR_MAX);
  strncpy(sensor[3].units[1], "g", STR_MAX);
  strncpy(sensor[3].units[2], "g", STR_MAX);
  //strncpy(sensor[3].units[3], "degreesC", STR_MAX);
  strncpy(sensor[3].units[3], "degPerS", STR_MAX);
  strncpy(sensor[3].units[4], "degPerS", STR_MAX);
  strncpy(sensor[3].units[5], "degPerS", STR_MAX);
  strncpy(sensor[3].units[6], "uT", STR_MAX);
  strncpy(sensor[3].units[7], "uT", STR_MAX);
  strncpy(sensor[3].units[8], "uT", STR_MAX);
  
  float accelFullRange = (float) accel_scale; //ACCEL_FS_SEL 2g(00), 4g(01), 8g(10), 16g(11)
  int gyroFullRange = 1000.0;  // FS_SEL 250deg/s (0), 500 (1), 1000(2), 2000 (3)
  int magFullRange = 4800.0;  // fixed
  
  sensor[3].cal[0] = accelFullRange / 32768.0;
  sensor[3].cal[1] = accelFullRange / 32768.0;
  sensor[3].cal[2] = accelFullRange / 32768.0;
  //sensor[3].cal[3] = 1.0 / 337.87;
  sensor[3].cal[3] = gyroFullRange / 32768.0;
  sensor[3].cal[4] = gyroFullRange / 32768.0;
  sensor[3].cal[5] = gyroFullRange / 32768.0;
  sensor[3].cal[6] = magFullRange / 32768.0;
  sensor[3].cal[7] = magFullRange / 32768.0;
  sensor[3].cal[8] = magFullRange / 32768.0;
}

int addSid(int i, char* sid,  unsigned int sidType, unsigned long nSamples, SENSOR sensor, unsigned long dForm, float srate)
{
  unsigned long nBytes;
//  memcpy(&_sid, sid, 5);
//
//  memset(&sidSpec[i], 0, sizeof(SID_SPEC));
//        nBytes<<1;  //multiply by two because halfbuf
//
//  switch(dForm)
//  {
//    case DFORM_SHORT:
//      nBytes = nElements * 2;
//      break;            
//    case DFORM_LONG:
//      nBytes = nElements * 4;  //32 bit values
//      break;            
//    case DFORM_I24:
//      nBytes = nElements * 3;  //24 bit values
//      break;
//    case DFORM_FLOAT32:
//      nBytes = nElements * 4;
//      break;
//  }

  strncpy(sidSpec[i].SID, sid, STR_MAX);
  sidSpec[i].sidType = sidType;
  sidSpec[i].nSamples = nSamples;
  sidSpec[i].dForm = dForm;
  sidSpec[i].srate = srate;
  sidSpec[i].sensor = sensor;  
  
  if(frec.write((uint8_t *)&sidSpec[i], sizeof(SID_SPEC))==-1)  resetFunc();

  sidRec[i].nSID = i;
  sidRec[i].NU[0] = 100; //put in something easy to find when searching raw file
  sidRec[i].NU[1] = 200;
  sidRec[i].NU[2] = 300; 
}


/*
void sdInit(){
     if (!(SD.begin(10))) {
    // stop here if no SD card, but print a message
    Serial.println("Unable to access the SD card");
    
    while (1) {
      cDisplay();
      display.println("SD error. Restart.");
      displayClock(getTeensy3Time(), BOTTOM);
      display.display();
      delay(1000);
      
    }
  }
}
*/

void logFileHeader(){
  if(File logFile = SD.open("LOG.CSV",  O_CREAT | O_APPEND | O_WRITE)){
      logFile.println("filename,ID,gain (dB),Voltage,Burn");
      logFile.close();
  }
}

void FileInit()
{
   t = getTeensy3Time();
   
   if (folderMonth != month(t)){
    if(printDiags > 0) Serial.println("New Folder");
    folderMonth = month(t);
    sprintf(dirname, "%04d-%02d", year(t), folderMonth);
    SdFile::dateTimeCallback(file_date_time);
    SD.mkdir(dirname);
   }

   // only audio save as wav file, otherwise save as AMX file
   
   // open file 
   if(fileType==0)
      sprintf(filename,"%s/%02d%02d%02d%02d.wav", dirname, day(t), hour(t), minute(t), second(t));  //filename is DDHHMM
    else
      sprintf(filename,"%s/%02d%02d%02d%02d.amx", dirname, day(t), hour(t), minute(t), second(t));  //filename is DDHHMM

   // log file
   SdFile::dateTimeCallback(file_date_time);

   float voltage = readVoltage();
   
   if(File logFile = SD.open("LOG.CSV",  O_CREAT | O_APPEND | O_WRITE)){
      logFile.print(filename);
      logFile.print(',');
      for(int n=0; n<8; n++){
        logFile.print(myID[n]);
      }
      
      logFile.print(',');
      logFile.print(gainDb); 
      
      logFile.print(',');
      logFile.print(voltage); 

      logFile.print(',');
      logFile.print(burnLog);
      
      logFile.println();

      if(((voltage < 3.76) | (total_hour_recorded > max_cam_hours_rec)) & camFlag) { //disable camera when power low or recorded more than 8 hours
        cam_stop();
        cam_off();
        
        camFlag = 0; 
        if(printDiags) Serial.println("Camera disabled");
        logFile.println("Camera stopped");
      }
      
      if(voltage < 3.0){
        logFile.println("Stopping because Voltage less than 3.0 V");
        logFile.close();  
        // low voltage hang but keep checking voltage
        while(readVoltage() < 3.0){
            delay(30000);
        }
      }
      logFile.close();
   }
   else{
    if(printDiags) Serial.print("Log open fail.");
    resetFunc();
   }
    
   frec = SD.open(filename, O_WRITE | O_CREAT | O_EXCL);

   if(printDiags > 0){
     Serial.println(filename);
     Serial.print("Hours rec:"); Serial.println(total_hour_recorded);
     Serial.print(voltage); Serial.println("V");
   }

   
   while (!frec){
    file_count += 1;
    if(fileType==0)
      sprintf(filename,"F%06d.wav",file_count); //if can't open just use count
      else
      sprintf(filename,"F%06d.amx",file_count); //if can't open just use count
    frec = SD.open(filename, O_WRITE | O_CREAT | O_EXCL);
    Serial.println(filename);
   }

   if(fileType==0){
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
    
      frec.write((uint8_t *)&wav_hdr, 44);
   }

   //amx file header
   dfh.voltage = voltage;

   if(fileType==1){
    // write DF_HEAD
    dfh.RecStartTime.sec = second();  
    dfh.RecStartTime.minute = minute();  
    dfh.RecStartTime.hour = hour();  
    dfh.RecStartTime.day = day();  
    dfh.RecStartTime.month = month();  
    dfh.RecStartTime.year = (int16_t) year();  
    dfh.RecStartTime.tzOffset = 0; //offset from GMT
    frec.write((uint8_t *) &dfh, sizeof(dfh));
    
    // write SID_SPEC depending on sensors chosen
    int sidCount=0;
    addSid(sidCount, "AUDIO", RAW_SID, 256, sensor[0], DFORM_SHORT, audio_srate);
    if (pressure_sensor>0) {
      sidCount++;
      addSid(sidCount, "PRTMP", RAW_SID, halfbufPT, sensor[1], DFORM_FLOAT32, sensor_srate);  
    }
    if (rgbFlag) {
      sidCount++;
      addSid(sidCount, "LIGHT", RAW_SID, halfbufRGB / 2, sensor[2], DFORM_SHORT, sensor_srate);
    }
    if (imuFlag) {
      sidCount++;
      addSid(sidCount, "3DAMG", RAW_SID, halfbufIMU / 2, sensor[3], DFORM_SHORT, imu_srate);
    }
    sidCount++;
    addSid(sidCount, "END", 0, 0, sensor[4], 0, 0);
  }
  if(printDiags > 0){
    Serial.print("Buffers: ");
    Serial.println(nbufs_per_file);
  }
}

//This function returns the date and time for SD card file access and modify time. One needs to call in setup() to register this callback function: SdFile::dateTimeCallback(file_date_time);
void file_date_time(uint16_t* date, uint16_t* time) 
{
  t = getTeensy3Time();
  *date=FAT_DATE(year(t),month(t),day(t));
  *time=FAT_TIME(hour(t),minute(t),second(t));
}


void AudioInit(){
    // Enable the audio shield, select input, and enable output
 // sgtl5000_1.enable();

 // Instead of using audio library enable; do custom so only power up what is needed in sgtl5000_LHI
  audio_enable();
 
  sgtl5000_1.inputSelect(myInput);
  sgtl5000_1.volume(0.0);
  sgtl5000_1.lineInLevel(systemGain);  //default = 4
  // CHIP_ANA_ADC_CTRL
// Actual measured full-scale peak-to-peak sine wave input for max signal
//  0: 3.12 Volts p-p
//  1: 2.63 Volts p-p
//  2: 2.22 Volts p-p
//  3: 1.87 Volts p-p
//  4: 1.58 Volts p-p
//  5: 1.33 Volts p-p
//  6: 1.11 Volts p-p
//  7: 0.94 Volts p-p
//  8: 0.79 Volts p-p (+8.06 dB)
//  9: 0.67 Volts p-p
// 10: 0.56 Volts p-p
// 11: 0.48 Volts p-p
// 12: 0.40 Volts p-p
// 13: 0.34 Volts p-p
// 14: 0.29 Volts p-p
// 15: 0.24 Volts p-p
  sgtl5000_1.autoVolumeDisable();
  sgtl5000_1.audioProcessorDisable();
}

void setGain(){
    sgtl5000_1.lineInLevel(systemGain);  //default = 4
  // CHIP_ANA_ADC_CTRL
  // Actual measured full-scale peak-to-peak sine wave input for max signal
  //  0: 3.12 Volts p-p
  //  1: 2.63 Volts p-p
  //  2: 2.22 Volts p-p
  //  3: 1.87 Volts p-p
  //  4: 1.58 Volts p-p (0.79 Vpeak)
  //  5: 1.33 Volts p-p
  //  6: 1.11 Volts p-p
  //  7: 0.94 Volts p-p
  //  8: 0.79 Volts p-p (+8.06 dB)
  //  9: 0.67 Volts p-p
  // 10: 0.56 Volts p-p
  // 11: 0.48 Volts p-p
  // 12: 0.40 Volts p-p
  // 13: 0.34 Volts p-p
  // 14: 0.29 Volts p-p
  // 15: 0.24 Volts p-p
  //sgtl5000_1.autoVolumeDisable();
 // sgtl5000_1.audioProcessorDisable();
  switch(systemGain){
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

void checkDielTime(){
  unsigned int startMinutes = (startHour * 60) + (startMinute);
  unsigned int endMinutes = (endHour * 60) + (endMinute );
  unsigned int startTimeMinutes =  (hour(startTime) * 60) + (minute(startTime));
  
  tmElements_t tmStart;
  tmStart.Year = year(startTime) - 1970;
  tmStart.Month = month(startTime);
  tmStart.Day = day(startTime);
  // check if next startTime is between startMinutes and endMinutes
  // e.g. 06:00 - 12:00 or 
  if(startMinutes<endMinutes){
     if ((startTimeMinutes < startMinutes) | (startTimeMinutes > endMinutes)){
       // set startTime to startHour startMinute
       tmStart.Hour = startHour;
       tmStart.Minute = startMinute;
       tmStart.Second = 0;
       startTime = makeTime(tmStart);
       Serial.print("New diel start:");
       printTime(startTime);
       if(startTime < getTeensy3Time()) startTime += SECS_PER_DAY;  // make sure after current time
       Serial.print("New diel start:");
       printTime(startTime);
       }
     }
  else{  // e.g. 23:00 - 06:00
    if((startTimeMinutes<startMinutes) & (startTimeMinutes>endMinutes)){
      // set startTime to startHour:startMinute
       tmStart.Hour = startHour;
       tmStart.Minute = startMinute;
       tmStart.Second = 0;
       startTime = makeTime(tmStart);
       Serial.print("New diel start:");
       printTime(startTime);
       if(startTime < getTeensy3Time()) startTime += SECS_PER_DAY;  // make sure after current time
       Serial.print("New diel start:");
       printTime(startTime);
    }
  }
}

unsigned long processSyncMessage() {
  unsigned long pctime = 0L;
  const unsigned long DEFAULT_TIME = 1451606400; // Jan 1 2016
} 

void sampleSensors(void){  //interrupt at update_rate
  ptCounter++;
  if(imuFlag) {
    readImu();
    incrementIMU();
    accel_x = (int16_t) ((int16_t)imuTempBuffer[0] << 8 | imuTempBuffer[1]);
  }


 // MS5803 start temperature conversion half-way through
  if((ptCounter>=(1.0 / sensor_srate) * update_rate / 2.0) & (pressure_sensor==1)  & togglePress){ 
    readPress();   
    updateTemp();
    togglePress = 0;
  }
  
  if(ptCounter>=(1.0 / sensor_srate) * update_rate){
      ptCounter = 0;
    //  int saltValOff = checkSalt(); // get value with saltSIG low
    //  digitalWrite(saltSIG, HIGH); // start signal for salt and let time reading sensors let it get high for checkSalt
      
      if (rgbFlag){
        islRead(); 
        incrementRGBbufpos(islRed);
        incrementRGBbufpos(islGreen);
        incrementRGBbufpos(islBlue);
      }

      // MS5803 pressure and temperature
      if (pressure_sensor==1){
          readTemp(); 
          updatePress();
          calcPressTemp();
          togglePress = 1;
          checkDepthVHF();
      }
      // Keller PA7LD pressure and temperature
      if (pressure_sensor==2){
        kellerRead();
        kellerConvert();  // start conversion for next reading
      }

      if(simulateDepth) depth = depthProfile[minute(t)];
      if(printDiags){
        Serial.print("D:");
        Serial.println(depth);
      }
      // MS5803 pressure and temperature
      if (pressure_sensor>0){
        PTbuffer[bufferposPT] = pressure_mbar;
        incrementPTbufpos();
        PTbuffer[bufferposPT] = temperature;
        incrementPTbufpos();
      }
   /*   int saltValOn = checkSalt();
      digitalWrite(saltSIG, LOW);
      if(abs(saltValOn - saltValOff) < saltThreshold){
        LEDSON = 1; //this makes them blink
      }
      else{
        digitalWrite(ledGreen, LOW);
        LEDSON = 0;
      }
      */
//      if(printDiags){
//      Serial.print("Off: ");
//      Serial.print(saltValOff);
//      Serial.print("  On: ");
//      Serial.println(saltValOn);
//    }
  }
}

// increment PTbuffer position by 1 sample. This does not check for overflow, because collected at a slow rate
void incrementPTbufpos(){
  bufferposPT++;
   if(bufferposPT==PTBUFFERSIZE)
   {
     bufferposPT=0;
     time2writePT=2;  // set flag to write second half
     firstwrittenPT=0; 
   }
 
  if((bufferposPT>=halfbufPT) & !firstwrittenPT)  //at end of first buffer
  {
    time2writePT=1; 
    firstwrittenPT=1;  //flag to prevent first half from being written more than once; reset when reach end of double buffer
  }
}

void incrementRGBbufpos(unsigned short val){
  RGBbuffer[bufferposRGB] = (uint8_t) val;
  bufferposRGB++;
  RGBbuffer[bufferposRGB] = (uint8_t) val>>8;
  bufferposRGB++;
  
   if(bufferposRGB==RGBBUFFERSIZE)
   {
     bufferposRGB = 0;
     time2writeRGB= 2;  // set flag to write second half
     firstwrittenRGB = 0; 
   }
 
  if((bufferposRGB>=halfbufRGB) & !firstwrittenRGB)  //at end of first buffer
  {
    time2writeRGB = 1; 
    firstwrittenRGB = 1;  //flag to prevent first half from being written more than once; reset when reach end of double buffer
  }
}

void incrementIMU(){
  for(int i=0; i<6; i++){
    imuBuffer[bufferposIMU] = (uint8_t) imuTempBuffer[i]; //accelerometer X,Y,Z
    bufferposIMU++;
  }
  // skipping IMU temperature in 6 and 7
  for(int i=8; i<20; i++){
    imuBuffer[bufferposIMU] = (uint8_t) imuTempBuffer[i]; //gyro and mag
    bufferposIMU++;
  }
  if(bufferposIMU==IMUBUFFERSIZE)
  {
    bufferposIMU = 0;
    time2writeIMU= 2;  // set flag to write second half
    firstwrittenIMU = 0; 
  }
  if((bufferposIMU>=halfbufIMU) & !firstwrittenIMU)  //at end of first buffer
  {
    time2writeIMU = 1; 
    firstwrittenIMU = 1;  //flag to prevent first half from being written more than once; reset when reach end of double buffer
  }
}
void resetFunc(void){
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
  pinMode(CAM_TRIG, OUTPUT);
  pinMode(hydroPowPin, OUTPUT);
  pinMode(displayPow, OUTPUT);
  pinMode(ledGreen, OUTPUT);
  pinMode(PLAY_POW, OUTPUT);
  pinMode(STOP, INPUT);

  pinMode(BURN, OUTPUT);
  pinMode(AUDIO_AMP, OUTPUT);
  //pinMode(SDSW, OUTPUT);
  pinMode(VHF, OUTPUT);
  pinMode(vSense, INPUT);
  pinMode(SALT, INPUT);
  pinMode(saltSIG, INPUT);
  analogReference(DEFAULT);

  digitalWrite(CAM_TRIG, HIGH);
  //digitalWrite(SDSW, HIGH); //low SD connected to microcontroller; HIGH SD connected to external pins
  digitalWrite(hydroPowPin, LOW);
  digitalWrite(displayPow, HIGH);  // also used as Salt output

  Serial.println("Sensor Init");

  // Digital IO
  digitalWrite(ledGreen, HIGH);
  digitalWrite(BURN, HIGH);
  digitalWrite(VHF, HIGH);
  // playback
  playBackOn();
  Serial.println("Playback On");
  
  delay(3000);
  
  digitalWrite(ledGreen, LOW);
  digitalWrite(BURN, LOW);
  digitalWrite(VHF, LOW);

  playTrackNumber(0);
  

  // IMU
  if(imuFlag){
    mpuInit(1);

    for(int i=0; i<50; i++){
      readImu();
      calcImu();
      euler();

      Serial.print("a/g/m/t:\t");
      Serial.print(accel_x); Serial.print("\t");
      Serial.print(accel_y); Serial.print("\t");
      Serial.print(accel_z); Serial.print("\t");
      Serial.print(gyro_x); Serial.print("\t");
      Serial.print(gyro_y); Serial.print("\t");
      Serial.print(gyro_z); Serial.print("\t");
      Serial.print(mag_x); Serial.print("\t");
      Serial.print(mag_y); Serial.print("\t");
      Serial.print(mag_z); Serial.print("\t");
      Serial.println(gyro_temp);

      cDisplay();
      display.println("IMU Init");
      if(i<15){
        display.print("A:");
        display.print( accel_x); display.print(" ");
        display.print( accel_y); display.print(" ");
        display.println( accel_z); 
        display.print("G:");
        display.print(gyro_x); display.print(" ");
        display.print(gyro_y); display.print(" ");
        display.println(gyro_z); 
        display.print("M:");
        display.print(mag_x); display.print(" ");
        display.print(mag_y); display.print(" ");
        display.print(mag_z); display.print(" ");
      }
      else{
        display.print("P: ");
        display.println(pitch);
        display.print("R: ");
        display.println(roll);
        display.print("Y: ");
        display.print(yaw);
      }
      display.display();
      delay(200);
    }
  }

  // RGB
  if(rgbFlag){
    islInit(); 
    for(int x=0; x<30; x++){
      islRead();
      Serial.print("R:"); Serial.println(islRed);
      Serial.print("G:"); Serial.println(islGreen);
      Serial.print("B:"); Serial.println(islBlue);
      cDisplay();
      display.println("Light Init");
      display.print("R:"); display.println(islRed);
      display.print("G:"); display.println(islGreen);
      display.print("B:"); display.println(islBlue);
      display.display();
      delay(200);
    }
  }
  
  
// Pressure--auto identify which if any is present
  cDisplay();
  
  pressure_sensor = 0;
  // Keller
  if(kellerInit()) {
    pressure_sensor = 2;   // 2 if present
    Serial.println("Keller Pressure Detected");
    kellerConvert();
    delay(20);
    kellerRead();
    delay(10);
    kellerConvert();
    delay(20);
    kellerRead();
    display.println("Press Deep");
  }

  // Measurement Specialties
  if(pressInit()){
    pressure_sensor = 1;
    Serial.println("MS Pressure Detected");
    display.println("MS Pressure");
    updatePress();
    delay(50);
    readPress();
    updateTemp();
    delay(50);
    readTemp();
    updatePress();
    delay(50);
    readPress();
    updateTemp();
    delay(50);
    readTemp();
    calcPressTemp();

  }
  
  Serial.print("Pressure (mBar): "); Serial.println(pressure_mbar);
  Serial.print("Depth: "); Serial.println(depth);
  Serial.print("Temperature: "); Serial.println(temperature);

  if(pressure_sensor==0) {
    display.println("Pressure");
    display.println("None Detected");
  }
  else{
      display.print("Press:"); display.println(pressure_mbar);
      display.print("Depth:"); display.println(depth);
      display.print("Temp:"); display.println(temperature);
  }
 
  display.display();
  
  if(simulateDepth) depth = 0;

  // playback
  playBackOff();
  Serial.println("Playback Off");  

  delay(3000);
}

void calcImu(){
  accel_x = (int16_t) (((int16_t)imuTempBuffer[0] << 8) | imuTempBuffer[1]);    
  accel_y = (int16_t) (((int16_t)imuTempBuffer[2] << 8) | imuTempBuffer[3]);   
  accel_z = (int16_t) -(((int16_t)imuTempBuffer[4] << 8) | imuTempBuffer[5]);    

  gyro_temp = (int16_t) (((int16_t)imuTempBuffer[6]) << 8 | imuTempBuffer[7]);   
 
  gyro_x = (int16_t) (((int16_t)imuTempBuffer[8] << 8) | imuTempBuffer[9]);  
  gyro_y = (int16_t) (((int16_t)imuTempBuffer[10] << 8) | imuTempBuffer[11]);   
  gyro_z = (int16_t) -(((int16_t)imuTempBuffer[12] << 8) | imuTempBuffer[13]);  
  
  mag_y = (int16_t)  (((int16_t)imuTempBuffer[14] << 8) | imuTempBuffer[15]);   
  mag_x = (int16_t)  (((int16_t)imuTempBuffer[16] << 8) | imuTempBuffer[17]);     
  mag_z = (int16_t) (((int16_t)imuTempBuffer[18] << 8) | imuTempBuffer[19]); 

// NED orientation

//  gyro_x = imu.gx;
//  gyro_y = imu.gy;
//  gyro_z = -imu.gz;
//  accel_x = imu.ax;
//  accel_y = imu.ay;
//  accel_z = -imu.az;
//  mag_x = imu.my - magYoffset;
//  mag_y = imu.mx - magXoffset;
//  mag_z = imu.mz - magZoffset;
  
}

void playOn(){
  digitalWrite(PLAY_POW, HIGH);
}

void playOff(){
  digitalWrite(PLAY_POW, LOW);
}



time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}


void cam_wake() {
  if(camFlag==SPYCAM){
   digitalWrite(CAM_TRIG, HIGH);  
   delay(3000);
  } 
  if(camFlag==FLYCAM){
    digitalWrite(CAM_TRIG, HIGH);
    delay(2000); //power on camera (if off)
    digitalWrite(CAM_TRIG, LOW);     
  } 
  CAMON = 1;   
}

void cam_start() {
  if(camFlag==SPYCAM){
    digitalWrite(CAM_TRIG, LOW);
    delay(1000);  // simulate  button press
    digitalWrite(CAM_TRIG, HIGH);  
  }
  else{
    digitalWrite(CAM_TRIG, HIGH);
    delay(500);  // simulate  button press
    digitalWrite(CAM_TRIG, LOW);  
  }     
  CAMON = 2;
}

void cam_stop(){
  if(camFlag==SPYCAM){
    digitalWrite(CAM_TRIG, LOW);
    delay(400);  // simulate  button press
    digitalWrite(CAM_TRIG, HIGH);  
    delay(6000); //give camera time to close file
  }
  else{
    digitalWrite(CAM_TRIG, HIGH);
    delay(100);  // simulate  button press
    digitalWrite(CAM_TRIG, LOW);  
  }
}

void cam_off() {
  if(camFlag==SPYCAM){
    delay(1000); //give last file chance to close
    digitalWrite(CAM_TRIG, LOW); //so doesn't draw power through trigger line
  }
  else{
    digitalWrite(CAM_TRIG, HIGH);
    delay(3000); //power down camera (if still on)
    digitalWrite(CAM_TRIG, LOW); 
  }        
  CAMON = 0;
}

void checkDepthVHF(){
  if((depth < depthThreshold) | burnLog==1) {
    digitalWrite(VHF, HIGH);
  }
  else{
    digitalWrite(VHF, LOW);
  }
}


int checkSalt(){
  return analogRead(SALT);
}


