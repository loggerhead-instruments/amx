int PressAddress = 0x77;


void Press_Init()
{
  int i = 0;
  byte buff[2];
  int bytesread;
  
  // Reset so PROM is loaded
  Wire.beginTransmission(PressAddress);
  Wire.send(0x1E);  //Reset Command
  bytesread=Wire.endTransmission();
  //Serial.println("Pressure Reset 0=success");
  //Serial.println(bytesread);  
    
  delay(5);  //reset needs at least 2.8 ms
  
  // Read and store calibration coefficients
  Wire.beginTransmission(PressAddress);
  Wire.send(0xA2);  //PROM Read Pressure Sensitivity
  Wire.endTransmission();
  
  bytesread=Wire.requestFrom(PressAddress, 2);    // request 2 bytes from device
//  Serial.println("PROM Bytes Available");
//  Serial.println(bytesread);
  
  while(Wire.available())   // ((Wire.available())&&(i<6))
  { 
    buff[i] = Wire.receive();  // receive one byte
    i++;
  }
  PSENS=(unsigned int) (buff[0]<<8)|buff[1]; //pressure sensitivity  MSB first
    
  i=0;
  Wire.beginTransmission(PressAddress);
  Wire.send(0xA4);  //PROM
  Wire.endTransmission();
  Wire.requestFrom(PressAddress, 2);    // request 2 bytes from device
  while(Wire.available())   // ((Wire.available())&&(i<6))
  { 
    buff[i] = Wire.receive();  // receive one byte
    i++;
  }
  POFF=(buff[0]<<8)|buff[1];  //Pressure offset
 
  i=0;
  Wire.beginTransmission(PressAddress);
  Wire.send(0xA6);  //PROM
  Wire.endTransmission();
  Wire.requestFrom(PressAddress, 2);    // request 2 bytes from device
  while(Wire.available())   // ((Wire.available())&&(i<6))
  { 
    buff[i] = Wire.receive();  // receive one byte
    i++;
  }
  TCSENS=(buff[0]<<8)|buff[1];  //

  i=0;
  Wire.beginTransmission(PressAddress);
  Wire.send(0xA8);  //PROM
  Wire.endTransmission();
  Wire.requestFrom(PressAddress, 2);    // request 2 bytes from device
  while(Wire.available())   // ((Wire.available())&&(i<6))
  { 
    buff[i] = Wire.receive();  // receive one byte
    i++;
  }

  TCOFF=(buff[0]<<8)|buff[1];  //Temp coefficient of pressure offset
  
  i=0;
  Wire.beginTransmission(PressAddress);
  Wire.send(0xAA);  //PROM
  Wire.endTransmission();
  Wire.requestFrom(PressAddress, 2);    // request 2 bytes from device
  while(Wire.available())   // ((Wire.available())&&(i<6))
  { 
    buff[i] = Wire.receive();  // receive one byte
    i++;
  }

  TREF=(buff[0]<<8)|buff[1];  //Ref temperature
  
  i=0;
  Wire.beginTransmission(PressAddress);
  Wire.send(0xAC);  //PROM
  Wire.endTransmission();
  Wire.requestFrom(PressAddress, 2);    // request 2 bytes from device
  while(Wire.available())   // ((Wire.available())&&(i<6))
  { 
    buff[i] = Wire.receive();  // receive one byte
    i++;
  }

  TEMPSENS =(buff[0]<<8)|buff[1];  //Temperature sensitivity coefficient  

}

void Update_Press()
{
    Wire.beginTransmission(PressAddress);
    Wire.send(0x48);  //Initiate pressure conversion OSR-4096
    Wire.endTransmission();

}

void Update_Temp()
{
   Wire.beginTransmission(PressAddress);
   Wire.send(0x58); //Initiate Temperature conversion OSR=4096
   Wire.endTransmission();
}

void Read_Press()
{
  int i = 0;
  
  Wire.beginTransmission(PressAddress);
  Wire.send(0x00);  //ADC Read Command
  Wire.endTransmission();
  
  Wire.requestFrom(PressAddress, 3);    // request 3 bytes from device
  while(Wire.available())   // ((Wire.available())&&(i<6))
  { 
    Pbuff[i] = Wire.receive();  // receive one byte
    i++;
  }
}


void Read_Temp()
{
  int i = 0;
 
  Wire.beginTransmission(PressAddress);
  Wire.send(0x00);  //ADC Read Command
  Wire.endTransmission();
  
  Wire.requestFrom(PressAddress, 3);    // request 3 bytes from device
  i=0;
  while(Wire.available())   // ((Wire.available())&&(i<6))
  { 
    Tbuff[i] = Wire.receive();  // receive one byte
    i++;
  }

}
