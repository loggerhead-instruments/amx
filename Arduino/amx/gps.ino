#define maxChar 256
char gpsStream[maxChar];
int streamPos;

int gpsYear = 2000, gpsMonth = 1, gpsDay = 1, gpsHour = 0, gpsMinute = 0, gpsSecond = 0;

void gps(byte incomingByte){
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
      // $GNGGA,003549.000,2716.6206,N,08227.4920,W,2,12,0.93,33.0,M,-28.8,M,,*7E
      // for some reason using strcmp misses some codes; so we will just do 1 char at a time
      //   memcpy(&temp5[0], &gpsStream[1], 5);
      //   char testStr[5];
      //   strcpy(testStr, "GNGGA");
      
      //if(strcmp(temp5, testStr) == 0){
 /*     if(gpsStream[1]=='G' & gpsStream[2]=='N' &  gpsStream[3]=='G' &  gpsStream[4]=='G' &  gpsStream[5]=='A'){
        // Serial.println("got one");
         // check response status
        
         int curPos = 7;
         memcpy(&temp2[0], &gpsStream[curPos], 2);
         sscanf(temp2, "%d", &gpsHour);
         curPos += 2;
         memcpy(&temp2[0], &gpsStream[curPos], 2);
         sscanf(temp2, "%d", &gpsMinute); 
         curPos += 2;
         memcpy(&temp2[0], &gpsStream[curPos], 2);
         sscanf(temp2, "%d", &gpsSecond);

         gpsTime.sec = gpsSecond;
         gpsTime.minute = gpsMinute;
         gpsTime.hour = gpsHour;

         curPos = 18;
         memcpy(&temp2[0], &gpsStream[curPos], 2);
         curPos += 2;
         int latDeg;
         float latMin;
         float tempVal;
         sscanf(temp2, "%d", &latDeg);
         memcpy(&temp7[0], &gpsStream[curPos], 7);
         sscanf(temp7, "%f", &latMin);
         tempVal = (double) latDeg + (latMin / 60.0);
         if (tempVal>0.0 & tempVal<=90.0){
          latitude = tempVal;
          curPos = 28;
          memcpy(&latHem, &gpsStream[curPos], 1);
         }
         
         curPos = 30;
         memcpy(&temp3[0], &gpsStream[curPos], 3);
         curPos += 3;
         int lonDeg;
         float lonMin;
         sscanf(temp3, "%d", &lonDeg);
         memcpy(&temp7[0], &gpsStream[curPos], 7);
         sscanf(temp7, "%f", &lonMin);
         tempVal = (double) lonDeg + (lonMin / 60.0);
         if (tempVal>0.0 & tempVal<=180.0){
          longitude = tempVal;
          curPos = 41;
          memcpy(&lonHem, &gpsStream[curPos], 1);
         }
     }
*/
      
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

      if(gpsStream[1]=='G' & gpsStream[2]=='P' &  gpsStream[3]=='R' &  gpsStream[4]=='M' &  gpsStream[5]=='C'){
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
        sscanf(token, "%2d%2d%2d", &gpsDay, &gpsMonth, &gpsYear);
        gpsYear += 2000;
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
           gpsTime.sec = gpsSecond;
           gpsTime.minute = gpsMinute;
           gpsTime.hour = gpsHour;
           gpsTime.day = gpsDay;
           gpsTime.month = gpsMonth;
           gpsTime.year = gpsYear-2000;
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



