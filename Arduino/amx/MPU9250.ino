int CompassAddress = 0x0C;  //0x0C internal compass on 9150
int GyroAddress = 0x68;

#define AKM_REG_WHOAMI      (0x00)

#define AKM_REG_ST1         (0x02)
#define AKM_REG_HXL         (0x03)
#define AKM_REG_ST2         (0x09)

#define AKM_REG_CNTL        (0x0A)
#define AKM_REG_ASTC        (0x0C)
#define AKM_REG_ASAX        (0x10)
#define AKM_REG_ASAY        (0x11)
#define AKM_REG_ASAZ        (0x12)

#define AKM_DATA_READY      (0x01)
#define AKM_DATA_OVERRUN    (0x02)
#define AKM_OVERFLOW        (0x80)
#define AKM_DATA_ERROR      (0x40)

#define AKM_BIT_SELF_TEST   (0x40)

#define SUPPORTS_AK89xx_HIGH_SENS   (0x10)
#define AKM_POWER_DOWN          (0x00 | SUPPORTS_AK89xx_HIGH_SENS)
#define AKM_SINGLE_MEASUREMENT  (0x01 | SUPPORTS_AK89xx_HIGH_SENS)
#define AKM_FUSE_ROM_ACCESS     (0x0F | SUPPORTS_AK89xx_HIGH_SENS)
#define AKM_MODE_SELF_TEST      (0x08 | SUPPORTS_AK89xx_HIGH_SENS)

#define AKM_WHOAMI      (0x48)

#define BIT_I2C_READ        (0x80)
#define BIT_SLAVE_BYTE_SW   (0x40)
#define BIT_SLAVE_GROUP     (0x10)
#define BIT_SLAVE_EN        (0x80)
#define BIT_I2C_MST_VDDIO   (0x80)

int mpuInit(boolean mode)
{
  int ecode;
   if (printDiags) Serial.print("Gyro Init\n");
   if(mode==0)
  {
     ecode = I2Cwrite(GyroAddress, 0x6B, 0x40);  //Sleep mode, internal 8 MHz oscillator  //another mode is cycle where it wakes up periodically to take a value
     return ecode;
  }

    //set clock source
    ecode = I2Cwrite(GyroAddress, 0x6B, 0x01);  //everything awake; clock from X gyro reference
  
    // set gyro range
    I2Cwrite(GyroAddress, 0x1B, 0x10);  // 0x10 +/- 1000 deg/s ; 0x18 +/-2000 deg/s
    
    // set accel range
    I2Cwrite(GyroAddress, 0x1C, 0x18); // +/- 16 g
    
    // configure frame sync and LPF
    //I2Cwrite(GyroAddress, 0x1A, 0x08 | 0x05);  //Frame sync to Temperature LSB; DLPF 10 Hz; causes Gyro to sample at 1 kHz
    I2Cwrite(GyroAddress, 0x1A, 0x05);  //no frame sync; DLPF 10 Hz; causes Gyro to sample at 1 kHz
    
    // set sample rate divider
    I2Cwrite(GyroAddress, 0x19, 0x31);  //  0x31=49=>20Hz; divide 1 kHz/(1+9)=100 Hz sample rate for all sensors
    
    // enable FIFO
    I2Cwrite(GyroAddress, 0x23, 0xF9);  //enable temp, gyro, accel, slave 0
   // I2Cwrite(GyroAddress, 0x23, 0xF8);  //enable temp, gyro, accel

    // setup compass
    setup_compass();

    // User Control
    I2Cwrite(GyroAddress, 0x6A, 0x07); // reset FIFO
    I2Cwrite(GyroAddress, 0x6A, 0x60); // FIFO enabled, Master Mode enabled

  // I2Cwrite(GyroAddress, 25, CompassAddress);  // set to write to compass
  // I2Cwrite(GyroAddress, 26, 0);  // set address of compass register to write
  // I2Cwrite(GyroAddress, 27, 0x91);  // enable and transfer byte from register 99, sending address first
   
/*
   I2Cwrite(GyroAddress, 0x24, 0x0D); // set to single-master control; 400 kHz I2C rate     
   I2Cwrite(GyroAddress, 0x23, 0x00);  //FIFO enable
   I2Cwrite(GyroAddress, 0x25, 0x80 | CompassAddress);  // set slave 0 to read compass
   I2Cwrite(GyroAddress, 0x26, 0x03);  // set address of compass register to read
   I2Cwrite(GyroAddress, 0x27, 0xD0 | 0x06);  // enable read of 6 bytes, with byte swapping
*/
  if (printDiags) Serial.print(ecode);
   return ecode;
}

void resetGyroFIFO(){
    I2Cwrite(GyroAddress, 0x6A, 0x07); // reset FIFO
    I2Cwrite(GyroAddress, 0x6A, 0x60); // FIFO enabled, Master Mode enabled
}

byte I2Cwrite(byte addr, byte reg, byte val)
{
  Wire.beginTransmission(addr);  
  Wire.write(reg);  // gyro scale, sample rate and LPF
  Wire.write(val);  
  byte ecode=Wire.endTransmission(); //end transmission
  if (printDiags) Serial.print(ecode);
  delay(5);
  return ecode;
}

void Read_Gyro(int numbytestoread)
{
//  int i = 0;
//  Wire.beginTransmission(GyroAddress); 
//  Wire.write(0x74);        //sends address to read from  0x3B is direct read; 0x74 is FIFO
//  Wire.endTransmission(); //end transmission
 /*
  int numblocks=numbytestoread/14;  //because wire will only get 32 or fewer bytes
  for(int n=0; n<numblocks; n++)
  {
     Wire.requestFrom(GyroAddress, 14);    // request 20 bytes from device
    while(Wire.available())   // ((Wire.available())&&(i<6))
    { 
      buffer[i] = Wire.read();  // receive one byte
      i++;
    }
  }
  */

  // reading one byte at a time
  for(int n=0; n<numbytestoread; n++)
  {
    Wire.beginTransmission(GyroAddress); 
    Wire.write(0x74);        //sends address to read from  0x3B is direct read; 0x74 is FIFO
    Wire.endTransmission(); //end transmission
    Wire.requestFrom(GyroAddress, 1);    // request 1 bytes from device
    if(Wire.available())   // ((Wire.available())&&(i<6))
    { 
      imuBuffer[n] = Wire.read();  // receive one byte
    }
  }
}

int getImuFifo()
{
 int fifopts; 
  // read FIFO size
//  Wire.beginTransmission(GyroAddress); 
//  Wire.write(0x3B);        //sends address to read from
//  Wire.endTransmission(); //end transmission
    
  Wire.beginTransmission(GyroAddress); 
  Wire.write(0x72);        //sends address to read from
  Wire.endTransmission(); //end transmission
  Wire.requestFrom(GyroAddress, 2);    // request 6 bytes from device
  
  byte FIFO_CNT[2];
  int i=0;
 
  while(Wire.available())   // ((Wire.available())&&(i<6))
  { 
    FIFO_CNT[i]= Wire.read();  // receive one byte
    i++;
  }
  fifopts=FIFO_CNT[0]<<8|FIFO_CNT[1];
   
 return fifopts; 
}

 /* This initialization is similar to the one in ak8975.c. */
int setup_compass(void)
{
   byte data;
   /* Set up master mode, master clock, and ES bit. */
    data = 0x40;
    if (I2Cwrite(GyroAddress, 0x24, data))
        return -6;

    /* Slave 0 reads from AKM data registers. */
    data = BIT_I2C_READ | CompassAddress;
    if (I2Cwrite(GyroAddress, 0x25, data))
        return -7;

    /* Compass reads start at this register. */
    data = AKM_REG_HXL;
    if (I2Cwrite(GyroAddress, 0x26, data))
        return -8;

    /* Enable slave 0, 6-byte reads. */
    data = BIT_SLAVE_EN  | BIT_SLAVE_GROUP | BIT_SLAVE_BYTE_SW | 6;
    //data= BIT_SLAVE_EN | 8;
    if (I2Cwrite(GyroAddress, 0x27, data))
        return -9;

    /* Slave 1 changes AKM measurement mode. */
    data = CompassAddress;
    if (I2Cwrite(GyroAddress, 0x28, data))
        return -10;

    /* AKM measurement mode register. */
    data = AKM_REG_CNTL;
    if (I2Cwrite(GyroAddress, 0x29, data))
        return -11;

    /* Enable slave 1, 1-byte writes. */
    data = BIT_SLAVE_EN | 1;
    if (I2Cwrite(GyroAddress, 0x2A, data))
        return -12;

    /* Set slave 1 data. */
    data = AKM_SINGLE_MEASUREMENT;
    if (I2Cwrite(GyroAddress, 0x64, data))
        return -13;

    /* Trigger slave 0 and slave 1 actions at each sample. */
    data = 0x03;
    if (I2Cwrite(GyroAddress, 0x67, data))
        return -14;

    /* For the MPU9150, the auxiliary I2C bus needs to be set to VDD. */
    data = BIT_I2C_MST_VDDIO;
    if (I2Cwrite(GyroAddress, 0x01, data))
        return -15;
        
    return 0;
}

