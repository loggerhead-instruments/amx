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

      if(gpsStream[1]=='G' & gpsStream[2]=='N' &  gpsStream[3]=='R' &  gpsStream[4]=='M' &  gpsStream[5]=='C'){
       char temp[streamPos + 1];
       //char temp[100];
       const char s[2] = ",";
       char *token;
            
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
        sscanf(token, "%2d%2d%2d", &gpsYear, &gpsMonth, &gpsDay);
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


        float tempLatitude, tempLongitude;
        if(rmcValid[0]=='A'){
           tempLatitude = rmcLat;
           tempLongitude = rmcLon;
           latHem = rmcLatHem[0];
           lonHem = rmcLonHem[0];
           if(latHem=='S') tempLatitude = -tempLatitude;
           if(lonHem=='W') tempLongitude = -tempLongitude;
           latitude = convertDegMinToDecDeg(tempLatitude);
           longitude = convertDegMinToDecDeg(tempLongitude);
           goodGPS = 1;
           if(printDiags) {
            Serial.print("Lt:");
            Serial.print(latitude);
            Serial.print(" Ln:");
            Serial.println(longitude);
           }
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


void gpsSleep(){
//  gpsSerial.println("$PMTK161,0*28");
//  gpsSerial.flush();
}

void gpsHibernate(){
//  gpsSerial.println("$PMTK225,4*2F");
//  gpsSerial.flush();
}

void gpsWake(){
//  gpsSerial.println(".");
//  gpsSerial.flush();
}

void gpsSpewOff(){
  //gpsSerial.println(PMTK_SET_NMEA_OUTPUT_OFF);
}

void gpsSpewOn(){
 // gpsSerial.println(PMTK_SET_NMEA_OUTPUT_RMCONLY);
}

void waitForGPS(){
  for(int n=0; n<100; n++){
    delay(20);
    while (gpsSerial.available() > 0) {    
        byte incomingByte = gpsSerial.read();
        Serial.write(incomingByte);
    }
  }
}

//int gpsDumpLogger(){
//  // open file for storing data; append
//  endGpsLog = 0;
//   gpsSerial.println(PMTK_LOCUS_DUMP);
//   int dumping = 1;
//   while(endGpsLog==0){
//       while (gpsSerial.available() > 0) {    
//        byte incomingByte = gpsSerial.read();
//        gps(incomingByte);
//        Serial.write(incomingByte);
//       }
//    if(gpsTimeout >= gpsTimeOutThreshold) return 0;
//   }
//   return 1;
//}

double convertDegMinToDecDeg(float degMin) {
  double min = 0.0;
  double decDeg = 0.0;
 
  //get the minutes, fmod() requires double
  min = fmod((double)degMin, 100.0);
 
  //rebuild coordinates in decimal degrees
  degMin = (int) ( degMin / 100 );
  decDeg = degMin + ( min / 60 );
 
  return decDeg;
}



void gpsGetTimeLatLon(){
    // get GPS
  int incomingByte;
  long gpsTimeOutStart = millis();

  goodGPS = 0;
  
  // can't display GPS data here, because display slows things down too much
  while((!goodGPS) & (millis()-gpsTimeOutStart<gpsTimeOutThreshold)){
    while (gpsSerial.available() > 0) {    
        incomingByte = gpsSerial.read();
        Serial.write(incomingByte);
        gps(incomingByte);  // parse incoming GPS data
    }
  }

  Serial.print("GPS search time:");
  Serial.println(millis()-gpsTimeOutStart);
  
  Serial.print("Good GPS:");
  Serial.println(goodGPS);
}
