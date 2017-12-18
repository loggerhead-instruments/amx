#define PMTK_LOCUS_STARTLOG  "$PMTK185,0*22"
#define PMTK_LOCUS_STOPLOG "$PMTK185,1*23"
#define PMTK_LOCUS_STARTSTOPACK "$PMTK001,185,3*3C"
#define PMTK_LOCUS_QUERY_STATUS "$PMTK183*38"
#define PMTK_LOCUS_ERASE_FLASH "$PMTK184,1*22"
#define PMTK_LOCUS_DUMP "$PMTK622,1*29"
// turn on only the second sentence (GPRMC)
#define PMTK_SET_NMEA_OUTPUT_RMCONLY "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"
// turn on GPRMC and GGA
#define PMTK_SET_NMEA_OUTPUT_RMCGGA "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
// turn on ALL THE DATA
#define PMTK_SET_NMEA_OUTPUT_ALLDATA "$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
// turn off output
#define PMTK_SET_NMEA_OUTPUT_OFF "$PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"

// send any byte to wake from Standby
#define PMTK_STANDBY "$PMTK161,0*28"
#define PMTK_NORMAL "$PMTK225,0*2B"
#define PMTK_ALWAYSLOCATE_STANDBY "$PMTK225,8*23"
#define PMTK_ALWAYSLOCATE_BACKUP "$PMTK225,9*22"

#define maxChar 256
char gpsStream[maxChar];
int streamPos;
volatile boolean endGpsLog;

int gps(byte incomingByte){
  char temp2[2];
  char temp3[3];
  char temp5[5];
  char temp7[7];
  char temp12[12];
  // check for start of new message
  // if a $, start it at Pos 0, and continue until next G
  if(incomingByte=='$') {
   // Serial.print("String position:");
   // Serial.println(streamPos);
   // Serial.println(gpsStream);
    //process last message
    if(streamPos > 10){
      // OriginGPS
      // $GNRMC,134211.000,A,2715.5428,N,08228.7924,W,1.91,167.64,020816,,,A*62
      // Adafruit GPS
      // $GPRMC,222250.000,A,2716.6201,N,08227.4996,W,1.01,301.49,250117,,,A*7C
      char rmcCode[6 + 1];
      float rmcTime; //           225446       Time of fix 22:54:46 UTC
      char rmcValid[2]; //           A            Navigation receiver warning A = OK, V = warning
      float rmcLat; //           4916.45,N    Latitude 49 deg. 16.45 min North
      char rmcLatHem[2];
      float rmcLon; //           12311.12,W   Longitude 123 deg. 11.12 min West
      char rmcLonHem[2];
      float rmcSpeed; //           000.5        Speed over ground, Knots
      float rmcCourse;//           054.7        Course Made Good, True
      char rmcDate[6 + 1];//           191194       Date of fix  19 November 1994
      float rmcMag;//           020.3,E      Magnetic variation 20.3 deg East
      char rmcMagHem[2];
      char rmcChecksum[4 + 1]; //           *68          mandatory checksum

      // check for end of log dump  $PMTKLOX,2*47
      if(gpsStream[1]=='P' & gpsStream[2]=='M' &  gpsStream[3]=='T' &  gpsStream[4]=='K' &  gpsStream[5]=='L'  
      & gpsStream[6]=='O' &  gpsStream[7]=='X' &  gpsStream[8]==',' &  gpsStream[9]=='2' & gpsStream[10]=='*' 
      & gpsStream[11]=='4' & gpsStream[12]=='7'){
        endGpsLog = 1;
      }

      if(gpsStream[1]=='G' & gpsStream[2]=='P' &  gpsStream[3]=='R' &  gpsStream[4]=='M' &  gpsStream[5]=='C'){
       char temp[streamPos + 1];
       //char temp[100];
       const char s[2] = ",";
       char *token;

        gpsTimeout += 1;
            
        memcpy(&temp, &gpsStream, streamPos);
        //Serial.println(temp);s
        //testing with a known string 72 chars
        //strcpy(temp, "$GNRMC,134211.000,A,2715.5428,N,08228.7924,W,1.91,167.64,020816,43,W,A*62");  
        //Serial.println(temp);
        
        token = strtok(temp, s);
        sprintf(rmcCode, "%s", token);
        //Serial.println(rmcCode);

        token = strtok(NULL, s);
        sscanf(token, "%f", &rmcTime);
        sscanf(token, "%2d%2d%2d", &gpsHour, &gpsMinute, &gpsSecond);
        //Serial.println(rmcTime);

        token = strtok(NULL, s);
        sprintf(rmcValid, "%s", token);
        //Serial.println(rmcValid);

        token = strtok(NULL, s);
        sscanf(token, "%f", &rmcLat);
        //Serial.println(rmcLat, 6);

        token = strtok(NULL, s);
        sprintf(rmcLatHem, "%s", token);
        //Serial.println(rmcLatHem);

        token = strtok(NULL, s);
        sscanf(token, "%f", &rmcLon);
        //Serial.println(rmcLon, 6);

        token = strtok(NULL, s);
        sprintf(rmcLonHem, "%s", token);
        //Serial.println(rmcLonHem);
        
        token = strtok(NULL, s);
        sscanf(token, "%f", &rmcSpeed);
        //Serial.println(rmcSpeed);

        token = strtok(NULL, s);
        sscanf(token, "%f", &rmcCourse);
        //Serial.println(rmcCourse);  

        token = strtok(NULL, s);
        sprintf(rmcDate, "%s", token);
        sscanf(token, "%2d%2d%2d", &gpsDay, &gpsMonth, &gpsYear);
        //gpsYear += 2000;
//        Serial.println(rmcDate);
//        Serial.print("Day-Month-Year:");
//        Serial.print(gpsDay); Serial.print("-");
//        Serial.print(gpsMonth);  Serial.print("-");
//        Serial.println(gpsYear);

        token = strtok(NULL, s);
        sscanf(token, "%f", &rmcMag);
        //Serial.println(rmcMag);          

        token = strtok(NULL, s);
        sprintf(rmcMagHem, "%s", token);
        //Serial.println(rmcMagHem);    

        token = strtok(NULL, s);
        sprintf(rmcChecksum, "%s", token);
        //Serial.println(rmcChecksum);         

        if(rmcValid[0]=='A'){
           latitude = rmcLat;
           longitude = rmcLon;
           latHem = rmcLatHem[0];
           lonHem = rmcLonHem[0];
           goodGPS = 1;
        }
      }
    }
    // start new message here
    streamPos = 0;
  }
  gpsStream[streamPos] = incomingByte;
  streamPos++;
  if(streamPos >= maxChar) streamPos = 0;
}

void gpsStartLogger(){
  HWSERIAL.println(PMTK_LOCUS_STARTLOG);
  waitForGPS();
}

void gpsStopLogger(){
  HWSERIAL.println(PMTK_LOCUS_STOPLOG);
  waitForGPS();
}

void gpsEraseLogger(){
  HWSERIAL.println(PMTK_LOCUS_ERASE_FLASH);
  waitForGPS();
}

void gpsStatusLogger(){
  HWSERIAL.println(PMTK_LOCUS_QUERY_STATUS);
  waitForGPS();
}

void gpsSleep(){
  HWSERIAL.println(PMTK_STANDBY);
  HWSERIAL.flush();
}

void gpsHibernate(){
  HWSERIAL.println("$PMTK225,4*2F");
  HWSERIAL.flush();
}

void gpsWake(){
  HWSERIAL.println(".");
  HWSERIAL.flush();
}

void gpsSpewOff(){
  HWSERIAL.println(PMTK_SET_NMEA_OUTPUT_OFF);
}

void gpsSpewOn(){
  HWSERIAL.println(PMTK_SET_NMEA_OUTPUT_RMCONLY);
}

void waitForGPS(){
  for(int n=0; n<100; n++){
    delay(20);
    while (HWSERIAL.available() > 0) {    
        byte incomingByte = HWSERIAL.read();
        Serial.write(incomingByte);
    }
  }
}


int gpsDumpLogger(){
  // open file for storing data; append
  endGpsLog = 0;
   if(File logFile = SD.open("GPSLOG.CSV",  O_CREAT | O_APPEND | O_WRITE)){
    // serial number
      for(int n=0; n<8; n++){
        logFile.print(myID[n]);
      }

   HWSERIAL.println(PMTK_LOCUS_DUMP);
   int dumping = 1;
   while(endGpsLog==0){
    digitalWrite(ledGreen, LOW);
       while (HWSERIAL.available() > 0) {    
        digitalWrite(ledGreen, HIGH);
        byte incomingByte = HWSERIAL.read();
        gps(incomingByte);
        Serial.write(incomingByte);
        logFile.write(incomingByte);
    }
    if(gpsTimeout >= gpsTimeOutThreshold) return 0;
   }
      logFile.close();
   }
   else{
    if(printDiags) Serial.print("Log open fail.");
    return 0;
   }
   return 1;
}


