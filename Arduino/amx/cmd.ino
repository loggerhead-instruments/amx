#define CMD(a,b) ( a + (b << 8))
#define TRUE 1
#define FALSE 0

int ProcCmd(char *pCmd)
{
	short *pCV;
	short n;
	long lv1;
	char s[22];
        unsigned int tday;
        unsigned int tmonth;
        unsigned int tyear;
        unsigned int thour;
        unsigned int tmin;
        unsigned int tsec;

	pCV = (short*)pCmd;

	n = strlen(pCmd);
	if(n<2) 
          return TRUE;

	switch(*pCV)
	{                     
    // IMU
		case ('I' + ('M'<<8)):
    {
      sscanf(&pCmd[3],"%d",&lv1);
      imu_srate = lv1;
      fileType = 1;
      imuFlag = 1;
      break;
    }

    // Keller Pressure and Temperature
    case ('K' + ('P'<<8)):
    {
      pressure_sensor = 2;
      fileType = 1;
      break;
    }

    // Measurement Specialities Pressure and Temperature
    case ('M' + ('P'<<8)):
    {
      pressure_sensor = 1;
      fileType = 1;
      break;
    }

    // RGB light sensor setup
    case ('L' + ('S'<<8)):
    {
      rgbFlag = 1;
      fileType = 1;
      break;
    }

    // Enable bright LED
    case ('B' + ('L'<<8)):
    {
      briteFlag = 1;
    }
    
    // Disable LEDS
    case ('L' + ('D'<<8)):
    {
        LEDSON=0;
        break;
    }

		// Set of Real Time Clock
		case ('T' + ('M'<<8)):
		{
         //set time
         sscanf(&pCmd[3],"%d-%d-%d %d:%d:%d",&tyear,&tmonth,&tday,&thour,&tmin,&tsec);
         TIME_HEAD NewTime;
         NewTime.sec = tsec;
         NewTime.minute = tmin;
         NewTime.hour = thour;
         NewTime.day = tday;
         NewTime.month = tmonth;
         NewTime.year = tyear-2000;
         ULONG newtime=RTCToUNIXTime(&NewTime);  //get new time in seconds
         startTime=RTCToUNIXTime(&NewTime);
         Teensy3Clock.set(newtime); 
         Serial.print("Clock Set: ");
         Serial.println(newtime);
         break;
      }

    case ('B' + ('W'<<8)):
    {
         //set time
         sscanf(&pCmd[3],"%d-%d-%d %d:%d:%d",&tyear,&tmonth,&tday,&thour,&tmin,&tsec);
         TIME_HEAD NewTime;
         NewTime.sec = tsec;
         NewTime.minute = tmin;
         NewTime.hour = thour;
         NewTime.day = tday;
         NewTime.month = tmonth;
         NewTime.year = tyear-2000;
         ULONG newtime=RTCToUNIXTime(&NewTime);  //get new time in seconds
         burnTime=RTCToUNIXTime(&NewTime);
         break;
      }
      
      case ('R' + ('D'<<8)):
      {
        sscanf(&pCmd[3],"%d",&lv1);
        rec_dur = lv1;
        break;
      }
      
      case ('R' + ('I'<<8)):
      {
        sscanf(&pCmd[3],"%d",&lv1);
        rec_int = lv1;
        break;
      } 
      
      case ('S' + ('R'<<8)):
      {
        //start time
         sscanf(&pCmd[3],"%d-%d-%d %d:%d:%d",&tyear,&tmonth,&tday,&thour,&tmin,&tsec);
         TIME_HEAD NewTime;
         NewTime.sec = tsec;
         NewTime.minute = tmin;
         NewTime.hour = thour;
         NewTime.day = tday;
         NewTime.month = tmonth;
         NewTime.year = tyear-2000;
         startTime=RTCToUNIXTime(&NewTime);
         Serial.print("Start Record Set: ");
         Serial.println(startTime);
         break;
      } 
	}	
	return TRUE;
}

boolean LoadScript()
{
  char s[30];
  char c;
  short i;

  File file;
  unsigned long TM_byte;
  int comment_TM = 0;

  // Read card setup.txt file to set date and time, recording interval
  file=SD.open("setup.txt");
 if(file)
 {
   do{
      	i = 0;
      	s[i] = 0;
        do{
            c = file.read();
	          if(c!='\r') s[i++] = c;
            if(c=='T') 
            {
              TM_byte = file.position() - 1;
              comment_TM = 1;
            }
            if(i>29) break;
	        }while(c!='\n');
      	  s[--i] = 0;
          if(s[0] != '/' && i>1)
          {
            ProcCmd(s);
      	  }
      }while(file.available());
      file.close();  
      
      // comment out TM line if it exists
      if (comment_TM)
      {
        Serial.print("Comment TM ");
        Serial.println(TM_byte);
        file = SD.open("setup.txt", FILE_WRITE);
        file.seek(TM_byte);
        file.print("//");
        file.close();
      }
      
  }
  else
  {   
    Serial.println("setup.txt not opened");
    display.println("no setup file");
    return 0;
  }
 return 1;	
}



