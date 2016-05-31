//
// Record sound as .wav on a SD card
//
// Loggerhead Instruments
// 2016
// David Mann
// 
// Modified from PJRC code
// wav only parts will work with Teensy audio board
// http://www.pjrc.com/store/teensy3_audio.html
//
// Loggerhead AMX board is required for accelerometer, magnetometer, gyroscope, RGB light, pressure, and temperature sensors
//

/* To Do: 
 * 
 *  .AMX file if using motion sensors, audio only gets wav file
 * MPU9250 (buffer with interrupt)
 * 1 Hz Interrupt
 * -MS5803
 * -Keller Pressure sensor
 * -RGB light sensor (add sleep)
 * 
 * burn wire 1 & 2
 * play sound
 * 
 * UTC and timezone time stamps
 * 
 * Time stamp on file
 * Long filename (use UNIX time)
 * Power Savings:
 * All unused pins to output mode
 * Disable USB
*/

#include <SerialFlash.h>
#include <Audio.h>  //this also includes SD.h from lines 89 & 90
#include <Wire.h>
#include <SPI.h>
//#include <SdFat.h>
#include <datafile32.h>
#include <Snooze.h>
#include <TimeLib.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <EEPROM.h>

#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);
#define BOTTOM 55

// set this to the hardware serial port you wish to use
#define HWSERIAL Serial1

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
AudioRecordQueue         queue1;         //xy=281,63
AudioConnection          patchCord1(i2s2, 0, queue1, 0);
AudioControlSGTL5000     sgtl5000_1;     //xy=265,212
// GUItool: end automatically generated code

const int myInput = AUDIO_INPUT_LINEIN;

// Pin Assignments
const int CAM_POW = 1;
const int hydroPowPin = 2;
// OpenCam
//const int UP = 2;
//const int DOWN = 3;
//const int SELECT = 6;
//const int reset_pin = 4;

// AMX
const int UP = 4;
const int DOWN = 5;
const int SELECT = 8;
const int displayPow = 20;
const int LED_green = 17;
const int LED_red = 16;

// Pins used by audio shield
// https://www.pjrc.com/store/teensy3_audio.html
// MEMCS 6t
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
byte startHour, startMinute, endHour, endMinute; //used in Diel mode

boolean imuFlag = 0;
boolean pressureFlag = 0;
boolean audioFlag = 1;
boolean CAMON = 0;

float pressure_period = 1.0;
float imu_period = 0.1;
float audio_srate = 44100.0;
float audio_period = 1/audio_srate;

int recMode = MODE_NORMAL;
long rec_dur = 60;
long rec_int = 60;
int wakeahead = 12;  //wake from snooze to give hydrophone and camera time to power up
int snooze_hour;
int snooze_minute;
int snooze_second;
int buf_count;
long nbufs_per_file;
boolean settingsChanged = 0;

long file_count;
char filename[20];
SnoozeBlock snooze_config;

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
float hydroCal = -164;

unsigned char prev_dtr = 0;

// IMU
int FIFOpts;
#define BUFFERSIZE 140 // used this length because it is divisible by 20 bytes (e.g. A*3,M*3,G*3,T) and 14 (w/out mag)
byte buffer[BUFFERSIZE]; //Double buffer used to store IMU sensor data before writes in bytes
int16_t accel_x;
int16_t accel_y;
int16_t accel_z;
int16_t magnetom_x;
int16_t magnetom_y;
int16_t magnetom_z;
int16_t gyro_x;
int16_t gyro_y;
int16_t gyro_z;
float gyro_temp;

// RGB
int16_t islRed;
int16_t islBlue;
int16_t islGreen;

// Pressure/Temp
byte Tbuff[3];
byte Pbuff[3];

//Pressure and temp calibration coefficients
unsigned int PSENS; //pressure sensitivity
unsigned int POFF;  //Pressure offset
unsigned int TCSENS; //Temp coefficient of pressure sensitivity
unsigned int TCOFF; //Temp coefficient of pressure offset
unsigned int TREF;  //Ref temperature
unsigned int TEMPSENS; //Temperature sensitivity coefficient

void setup() {
  Serial.begin(baud);
  delay(1000);
  Wire.begin();

  pinMode(CAM_POW, OUTPUT);
  pinMode(hydroPowPin, OUTPUT);
  pinMode(displayPow, OUTPUT);
  pinMode(LED_green, OUTPUT);
  pinMode(LED_red, OUTPUT);
  
  digitalWrite(CAM_POW,  LOW);
  digitalWrite(hydroPowPin, LOW);
  digitalWrite(displayPow, HIGH);
  digitalWrite(LED_green, LOW);
  digitalWrite(LED_red, LOW);

  //setup display and controls
  pinMode(UP, INPUT);
  pinMode(DOWN, INPUT);
  pinMode(SELECT, INPUT);
  digitalWrite(UP, HIGH);
  digitalWrite(DOWN, HIGH);
  digitalWrite(SELECT, HIGH);

  // Audio connections require memory, and the record queue
  // uses this memory to buffer incoming audio.
  AudioMemory(150);
  AudioInit(); // this calls Wire.begin() in control_sgtl5000.cpp
  delay(1000);
  
  mpuInit(1);
  islInit(); // RGB light sensor

  setSyncProvider(getTeensy3Time); //use Teensy RTC to keep time
  t = getTeensy3Time();
  if (t < 1451606400) Teensy3Clock.set(1451606400);
  startTime = getTeensy3Time();
  stopTime = startTime + rec_dur;

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  //initialize display
  cDisplay();
  display.println("Loggerhead");
  Serial.println("Loggerhead");
  display.display();
  delay(1000);

  // Initialize the SD card
  SPI.setMOSI(7);
  SPI.setSCK(14);
  if (!(SD.begin(10))) {
    // stop here if no SD card, but print a message
    Serial.println("Unable to access the SD card");
    
    while (1) {
      cDisplay();
      display.println("SD error");
      displayClock(getTeensy3Time(), BOTTOM);
      display.display();
      delay(1000);
    }
  }
  SdFile::dateTimeCallback(file_date_time);

  //LoadScript();

  manualSettings();
  cDisplay();
  
  t = getTeensy3Time();
  if (startTime < t)
  {  
    //startTime = ((t + 60)/10) * 10;  //move ahead and round to nearest 10 s
    startTime = t + 5; //start in 5 s
    stopTime = startTime + rec_dur;  // this will be set on start of recording
    
  }
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
  
  if (time_to_first_rec > 60)
  {
      ss = time_to_first_rec - wakeahead;
      int nap_hour = floor(ss/3600);
      ss -= nap_hour * 3600;
      int nap_minute = floor(ss/60);
      ss -= nap_minute * 60;
      int nap_second = ss;
      cDisplay();
      display.println("Sleep...");
      display.display();
      delay(800);
      display.ssd1306_command(SSD1306_DISPLAYOFF); 
      Serial.print("Nap HH MM SS ");
      Serial.print(nap_hour);
      Serial.print(nap_minute);
      Serial.println(nap_second);     
      Serial.println("Going to sleep.");
      delay(2000);
      digitalWrite(displayPow, LOW);
      snooze_config.setAlarm(nap_hour, nap_minute, nap_second);
      Snooze.sleep( snooze_config );
      // ... sleeping ...
      digitalWrite(displayPow, HIGH);
      display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  //initialize display   
  }
        
  digitalWrite(hydroPowPin, HIGH);
  cam_wake();
  mode = 0;
}

//
// MAIN LOOP
//
void loop() {
 
  t = getTeensy3Time();
  
  // Standby mode
  if(mode == 0)
  {
      if(t >= startTime){      // time to start?
        Serial.println("Record Start.");
        
        stopTime = startTime + rec_dur;
        startTime = stopTime + rec_int;
        if (recMode==MODE_DIEL) checkDielTime();

        Serial.print("Current Time: ");
        printTime(getTeensy3Time());
        Serial.print("Stop Time: ");
        printTime(stopTime);
        Serial.print("Next Start:");
        printTime(startTime);

        cDisplay();
        display.println("Rec");
        display.setTextSize(1);
        display.print("Stop Time: ");
        displayClock(stopTime, 30);
        display.display();

        mode = 1;
        
        startRecording();
        cam_start();
      }
      else{
        cDisplay();
        display.println("Next Start");
        displayClock(startTime, 20);
        displayClock(getTeensy3Time(), BOTTOM);
        display.display();
      }
  }
  
  // Record mode
  if (mode == 1) {
    continueRecording();  // download data
    if(buf_count >= nbufs_per_file){       // time to stop?
      stopRecording();

      long ss = startTime - getTeensy3Time() - wakeahead;
      if (ss<0) ss=0;
      snooze_hour = floor(ss/3600);
      ss -= snooze_hour * 3600;
      snooze_minute = floor(ss/60);
      ss -= snooze_minute * 60;
      snooze_second = ss;

      if( snooze_hour + snooze_minute + snooze_second >=10){
          digitalWrite(hydroPowPin, LOW); //hydrophone off
          cam_off();
          cDisplay();
          display.display();
          delay(100);
          display.ssd1306_command(SSD1306_DISPLAYOFF); 
          Serial.print("Snooze HH MM SS ");
          Serial.print(snooze_hour);
          Serial.print(snooze_minute);
          Serial.println(snooze_second);
          delay(500);
          mpuInit(0);  //gyro to sleep
          snooze_config.setAlarm(snooze_hour, snooze_minute, snooze_second);
          Snooze.sleep( snooze_config );
          
          /// ... Sleeping ....
          
          // Waking up
          display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  //initialize display
          digitalWrite(hydroPowPin, HIGH); // hydrophone on
          cam_wake();
          }
      
      mpuInit(1);  //start gyro
      mode = 0;
    }
  }
}

void startRecording() {
  Serial.println("startRecording");
  FileInit();
  resetGyroFIFO();
  if (frec) {
    buf_count = 0;
    queue1.begin();
  }
  else
  {
    Serial.println("could not open file");
  }
}

void continueRecording() {
  if (queue1.available() >= 2) {
    byte buffer[512];
    // Fetch 2 blocks from the audio library and copy
    // into a 512 byte buffer.  The Arduino SD library
    // is most efficient when full 512 byte sector size
    // writes are used.
    digitalWrite(LED_green, HIGH);
    memcpy(buffer, queue1.readBuffer(), 256);
    queue1.freeBuffer();
    memcpy(buffer+256, queue1.readBuffer(), 256);
    queue1.freeBuffer();
    frec.write(buffer, 512);
    buf_count += 1;
    digitalWrite(LED_green, LOW);
    pollGyro();
  }
}

void stopRecording() {
  Serial.println("stopRecording");
  int maxblocks = AudioMemoryUsageMax();
  Serial.print("Audio Memory Max");
  Serial.println(maxblocks);
  byte buffer[512];
  queue1.end();
  while (queue1.available() > 0) {
    memcpy(buffer, queue1.readBuffer(), 256);
    queue1.freeBuffer();
    memcpy(buffer+256, queue1.readBuffer(), 256);
    queue1.freeBuffer();
  }
  AudioMemoryUsageMaxReset();
  //frec.timestamp(T_WRITE,(uint16_t) year(t),month(t),day(t),hour(t),minute(t),second);
  frec.close();
  delay(100);
  //calcRMS();
  //Serial.println(rms);
  digitalWrite(hydroPowPin, LOW);
}


void FileInit()
{
   t = getTeensy3Time();
   // open file 
   sprintf(filename,"%02d%02d%02d%02d.wav",day(t), hour(t), minute(t), second(t));  //filename is DDHHMM
   frec = SD.open(filename, O_WRITE | O_CREAT | O_EXCL);
   Serial.println(filename);
   delay(100);
   
   while (!frec){
    file_count += 1;
    sprintf(filename,"F%06d.wav",file_count); //if can't open just use count
    frec = SD.open(filename, O_WRITE | O_CREAT | O_EXCL);
    Serial.println(filename);
    delay(10);
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
  t = getTeensy3Time();

  frec.write((uint8_t *)&wav_hdr,44);
  Serial.print("Buffers: ");
  Serial.println(nbufs_per_file);
}

//This function returns the date and time for SD card file access and modify time. One needs to call in setup() to register this callback function: SdFile::dateTimeCallback(file_date_time);
void file_date_time(uint16_t* date, uint16_t* time) 
{
 /* t = getTeensy3Time();
  *date=FAT_DATE(year(t),month(t),day(t));
  *time=FAT_TIME(hour(t),minute(t),second(t));
  *
   */
}

void cam_wake() {
  digitalWrite(CAM_POW, HIGH);
  delay(2000); //power on camera (if off)
  digitalWrite(CAM_POW, LOW);      
  CAMON=1;   
}

void cam_start() {
  digitalWrite(CAM_POW, HIGH);
  delay(500);  // simulate Flywire button press
  digitalWrite(CAM_POW, LOW);          
}

void cam_off() {
  digitalWrite(CAM_POW, HIGH);
  delay(3000); //power down camera (if still on)
  digitalWrite(CAM_POW, LOW);           
  CAMON=0;
}

void AudioInit(){
    // Enable the audio shield, select input, and enable output
  sgtl5000_1.enable();
  sgtl5000_1.inputSelect(myInput);
  sgtl5000_1.volume(0.0);
  sgtl5000_1.lineInLevel(0);  //default = 8
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
    Ticks += (tm->mday - 1) * SECONDS_IN_DAY;

    // Calculate Time Ticks CHANGES ARE HERE
    Ticks += (ULONG)tm->hour * SECONDS_IN_HOUR;
    Ticks += (ULONG)tm->minute * SECONDS_IN_MINUTE;
    Ticks += tm->sec;

    return Ticks;
}




/* DISPLAY FUNCTIONS
 *  
 */
void printDigits(int digits){
  // utility function for digital clock display: prints preceding colon and leading 0
  display.print(":");
  printZero(digits);
  display.print(digits);
}

void printZero(int val){
  if(val<10) display.print('0');
}

#define noSet 0
#define setRecDur 1
#define setRecSleep 2
#define setYear 3
#define setMonth 4
#define setDay 5
#define setHour 6
#define setMinute 7
#define setSecond 8
#define setMode 9
#define setStartHour 10
#define setStartMinute 11
#define setEndHour 12
#define setEndMinute 13

void manualSettings(){
  boolean startRec = 0, startUp, startDown;
  readEEPROM();
  
  // make sure settings valid (if EEPROM corrupted or not set yet)
  if (rec_dur < 0 | rec_dur>100000) rec_dur = 60;
  if (rec_int<0 | rec_int>100000) rec_int = 60;
  if (startHour<0 | startHour>23) startHour = 0;
  if (startMinute<0 | startMinute>59) startMinute = 0;
  if (endHour<0 | endHour>23) endHour = 0;
  if (endMinute<0 | endMinute>59) endMinute = 0;
  if (recMode<0 | recMode>1) recMode = 0;
  
  while(startRec==0){
    static int curSetting = noSet;
    static int newYear, newMonth, newDay, newHour, newMinute, newSecond, oldYear, oldMonth, oldDay, oldHour, oldMinute, oldSecond;
    
    // Check for mode change
    boolean selectVal = digitalRead(SELECT);
    if(selectVal==0){
      curSetting += 1;
      if((recMode==MODE_NORMAL & curSetting>9) | (recMode==MODE_DIEL & curSetting>13)) curSetting = 0;
    }

    cDisplay();

    t = getTeensy3Time();
    switch (curSetting){
      case noSet:
        if (settingsChanged) {
          writeEEPROM();
          settingsChanged = 0;
        }
        display.print("UP+DN->Rec"); 
        // Check for start recording
        startUp = digitalRead(UP);
        startDown = digitalRead(DOWN);
        if(startUp==0 & startDown==0) {
          cDisplay();
          writeEEPROM(); //save settings
          display.print("Starting..");
          display.display();
          delay(1500);
          startRec = 1;  //start recording
        }
        break;
      case setRecDur:
        rec_dur = updateVal(rec_dur, 1, 3600);
        display.print("Rec:");
        display.print(rec_dur);
        display.println("s");
        break;
      case setRecSleep:
        rec_int = updateVal(rec_int, 0, 3600 * 24);
        display.print("Slp:");
        display.print(rec_int);
        display.println("s");
        break;
      case setYear:
        oldYear = year(t);
        newYear = updateVal(oldYear,2000, 2100);
        if(oldYear!=newYear) setTeensyTime(hour(t), minute(t), second(t), day(t), month(t), newYear);
        display.print("Year:");
        display.print(year(getTeensy3Time()));
        break;
      case setMonth:
        oldMonth = month(t);
        newMonth = updateVal(oldMonth, 1, 12);
        if(oldMonth != newMonth) setTeensyTime(hour(t), minute(t), second(t), day(t), newMonth, year(t));
        display.print("Month:");
        display.print(month(getTeensy3Time()));
        break;
      case setDay:
        oldDay = day(t);
        newDay = updateVal(oldDay, 1, 31);
        if(oldDay!=newDay) setTeensyTime(hour(t), minute(t), second(t), newDay, month(t), year(t));
        display.print("Day:");
        display.print(day(getTeensy3Time()));
        break;
      case setHour:
        oldHour = hour(t);
        newHour = updateVal(oldHour, 0, 23);
        if(oldHour!=newHour) setTeensyTime(newHour, minute(t), second(t), day(t), month(t), year(t));
        display.print("Hour:");
        display.print(hour(getTeensy3Time()));
        break;
      case setMinute:
        oldMinute = minute(t);
        newMinute = updateVal(oldMinute, 0, 59);
        if(oldMinute!=newMinute) setTeensyTime(hour(t), newMinute, second(t), day(t), month(t), year(t));
        display.print("Minute:");
        display.print(minute(getTeensy3Time()));
        break;
      case setSecond:
        oldSecond = second(t);
        newSecond = updateVal(oldSecond, 0, 59);
        if(oldSecond!=newSecond) setTeensyTime(hour(t), minute(t), newSecond, day(t), month(t), year(t));
        display.print("Second:");
        display.print(second(getTeensy3Time()));
        break;
      case setMode:
        display.print("Mode:");
        recMode = updateVal(recMode, 0, 1);
        if (recMode==MODE_NORMAL)  display.print("Norm");
        if (recMode==MODE_DIEL) display.print("Diel");
        break;
      case setStartHour:
        startHour = updateVal(startHour, 0, 23);
        display.print("Strt HH:");
        printZero(startHour);
        display.print(startHour);
        break;
      case setStartMinute:
        startMinute = updateVal(startMinute, 0, 59);
        display.print("Strt MM:");
        printZero(startMinute);
        display.print(startMinute);
        break;
      case setEndHour:
        endHour = updateVal(endHour, 0, 23);
        display.print("End HH:");
        printZero(endHour);
        display.print(endHour);
        break;
      case setEndMinute:
        endMinute = updateVal(endMinute, 0, 59);
        display.print("End MM:");
        printZero(endMinute);
        display.print(endMinute);
        break;
    }
    displaySettings();
    displayClock(getTeensy3Time(), BOTTOM);
    display.display();
    delay(200);
  }
}

void setTeensyTime(int hr, int mn, int sc, int dy, int mh, int yr){
  tmElements_t tm;
  tm.Year = yr - 1970;
  tm.Month = mh;
  tm.Day = dy;
  tm.Hour = hr;
  tm.Minute = mn;
  tm.Second = sc;
  time_t newtime;
  newtime = makeTime(tm); 
  Teensy3Clock.set(newtime); 
}
  
int updateVal(long curVal, long minVal, long maxVal){
  boolean upVal = digitalRead(UP);
  boolean downVal = digitalRead(DOWN);
  static int heldDown = 0;
  static int heldUp = 0;
  if(upVal==0){
    settingsChanged = 1;
      if (heldUp > 10) {
        curVal += 10;
        if (heldUp > 20) curVal += 90;
      }
      else curVal += 1;
      heldUp += 1;
    }
    else heldUp = 0;
    
    if(downVal==0){
      settingsChanged = 1;
      if (heldDown > 10) {
        curVal -= 10;
        if (heldDown > 20) curVal -= 90;
      }
      else
        curVal -= 1;
      heldDown += 1;
    }
    else heldDown = 0;

    if (curVal < minVal) curVal = maxVal;
    if (curVal > maxVal) curVal = minVal;
    return curVal;
}

void cDisplay(){
    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setTextSize(2);
    display.setCursor(0,0);
}

void displaySettings(){
  //t = Teensy3Clock.get();
  t = getTeensy3Time();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 18);
  display.print("Mode:");
  if (recMode==MODE_NORMAL) display.println("Normal");
  if (recMode==MODE_DIEL) {
    display.println("Diel");
  }
  display.print("Rec:");
  display.print(rec_dur);
  display.println("s");
  display.print("Sleep:");
  display.print(rec_int);
  display.println("s");
  if (recMode==MODE_DIEL) {
    display.print("Active: ");
    printZero(startHour);
    display.print(startHour);
    printDigits(startMinute);
    display.print("-");
    printZero(endHour);
    display.print(endHour);
    printDigits(endMinute);
    display.println();
  }
}

void displayClock(time_t t, int loc){
  display.setTextSize(1);
  display.setCursor(0,loc);
  display.print(year(t));
  display.print('-');
  display.print(month(t));
  display.print('-');
  display.print(day(t));
  display.print("  ");
  printZero(hour());
  display.print(hour(t));
  printDigits(minute(t));
  printDigits(second(t));
}

void printTime(time_t t){
  Serial.print(year(t));
  Serial.print('-');
  Serial.print(month(t));
  Serial.print('-');
  Serial.print(day(t));
  Serial.print(" ");
  Serial.print(hour(t));
  Serial.print(':');
  Serial.print(minute(t));
  Serial.print(':');
  Serial.println(second(t));
}

void readEEPROM(){
  rec_dur = readEEPROMlong(0);
  rec_int = readEEPROMlong(4);
  startHour = EEPROM.read(8);
  startMinute = EEPROM.read(9);
  endHour = EEPROM.read(10);
  endMinute = EEPROM.read(11);
  recMode = EEPROM.read(12);
}

union {
  byte b[4];
  long lval;
}u;

long readEEPROMlong(int address){
  u.b[0] = EEPROM.read(address);
  u.b[1] = EEPROM.read(address + 1);
  u.b[2] = EEPROM.read(address + 2);
  u.b[3] = EEPROM.read(address + 3);
  return u.lval;
}

void writeEEPROMlong(int address, long val){
  u.lval = val;
  EEPROM.write(address, u.b[0]);
  EEPROM.write(address + 1, u.b[1]);
  EEPROM.write(address + 2, u.b[2]);
  EEPROM.write(address + 3, u.b[3]);
}

void writeEEPROM(){
  writeEEPROMlong(0, rec_dur);  //long
  writeEEPROMlong(4, rec_int);  //long
  EEPROM.write(8, startHour); //byte
  EEPROM.write(9, startMinute); //byte
  EEPROM.write(10, endHour); //byte
  EEPROM.write(11, endMinute); //byte
  EEPROM.write(12, recMode); //byte
}

void pollGyro(){
  FIFOpts=getGyroFIFO();
  if(FIFOpts>BUFFERSIZE)  //once have enough data for a block, download and write to disk
  {
     Read_Gyro(BUFFERSIZE);  //download block from FIFO
     
    // print out first line of block
    // MSB byte first, then LSB, X,Y,Z
    accel_x = (int16_t) ((int16_t)buffer[0] << 8 | buffer[1]);    
    accel_y = (int16_t) ((int16_t)buffer[2] << 8 | buffer[3]);   
    accel_z = (int16_t) ((int16_t)buffer[4] << 8 | buffer[5]);    
    
    gyro_temp = (int16_t) (((int16_t)buffer[6]) << 8 | buffer[7]);   
   
    gyro_x = (int16_t)  (((int16_t)buffer[8] << 8) | buffer[9]);   
    gyro_y = (int16_t)  (((int16_t)buffer[10] << 8) | buffer[11]); 
    gyro_z = (int16_t)  (((int16_t)buffer[12] << 8) | buffer[13]);   
    
    magnetom_x = (int16_t)  (((int16_t)buffer[14] << 8) | buffer[15]);   
    magnetom_y = (int16_t)  (((int16_t)buffer[16] << 8) | buffer[17]);   
    magnetom_z = (int16_t)  (((int16_t)buffer[18] << 8) | buffer[19]);  

    Serial.print("a/g/m/t:\t");
    Serial.print( accel_x); Serial.print("\t");
    Serial.print( accel_y); Serial.print("\t");
    Serial.print( accel_z); Serial.print("\t");
    Serial.print(gyro_x); Serial.print("\t");
    Serial.print(gyro_y); Serial.print("\t");
    Serial.print(gyro_z); Serial.print("\t");
    Serial.print(magnetom_x); Serial.print("\t");
    Serial.print(magnetom_y); Serial.print("\t");
    Serial.print(magnetom_z); Serial.print("\t");
    Serial.println((float) gyro_temp/337.87+21);

    islRead();
    Serial.print("RGB:");Serial.print("\t");
    Serial.print(islRed); Serial.print("\t");
    Serial.print(islGreen); Serial.print("\t");
    Serial.println(islBlue); 
    
  // Write data buffer
//  digitalWrite(LED_GRN,HIGH);
//  file.write(&SidRec[0],sizeof(SID_REC));
//  file.write(buffer, BUFFERSIZE);
//  SidRec[0].samples+=(BUFFERSIZE/2);   
  }
}

