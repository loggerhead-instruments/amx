// Keller Pressure sensor

int kellerAddress = 0x40;
float pAt16384 = 0.0; //minimum pressure (bar)
float pAt49152 = 200.0; // maximum pressure (bar)

// read values of pressure sensor
void kellerInit(){
  if (printDiags) Serial.println("Keller");
  byte temp[2];
  int i = 0;
   Wire.beginTransmission(kellerAddress);
   Wire.send(0x00);  
   Wire.endTransmission();
   delay(2);
   Wire.requestFrom(kellerAddress, 2); 
   delay(2);
   while(Wire.available())  
   { 
    temp[i] = Wire.receive();  // receive one byte
    i++;
    if (printDiags) Serial.println(temp[i]);
   } 
   
}

void kellerConvert(){
   Wire.beginTransmission(kellerAddress);
   Wire.send(0xAC);  //Initiate pressure conversion takes >4 ms
   Wire.endTransmission();
}

void kellerRead(){
  int i = 0;
  byte temp[5];
  
  Wire.requestFrom(kellerAddress, 5);    // request 5 bytes from device; status; pressure(2); temperature (2)
  while(Wire.available())  
  { 
    temp[i] = Wire.receive();  // receive one byte
    i++;
  } 
  float pressure = (float) ((uint16_t) temp[1] << 8 | (uint16_t) temp[2]);
  float milliBar = ((pressure - 16384.0) * (pAt49152 - pAt16384) / 32768.0 + pAt16384) * 1000.0;
  depth = -(1010.0 - milliBar) / 1000.0;
  
  uint16_t tU16 = ((uint16_t) temp[3] << 8 | (uint16_t) temp[4]);
  temperature = (float) ((tU16 >> 4) - 24) * 0.05 - 50.0;
}

