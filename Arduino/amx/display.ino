
/* DISPLAY FUNCTIONS
 *  
 */

void displayOn(){
  //display.ssd1306_command(SSD1306_DISPLAYON);
  display.init();
  display.setBatteryVisible(true);
}

void displayOff(){
  display.ssd1306_command(SSD1306_DISPLAYOFF);
}

void cDisplay(){
  display.clearDisplay();
  readVoltage();
  displayBattery();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0,0);
}

void displaySettings(){
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, displayLine1);
  if(mode==0) {
    display.print("Standby ");
    display.print(startTime - t);
    display.println("s");
  }
  if(mode==1) display.print("Running ");
  display.setCursor(0, displayLine2);

  display.print(audio_srate, 0);
  display.print(" Hz");
  
  display.setCursor(0, displayLine3);
  display.print("Rec ");
  display.print(rec_dur);
  display.print("s");
  display.print("  Sleep ");
  display.print(rec_int);
  display.println("s");
}

void displayBattery(){
  float voltage = readVoltage();
  display.setBattery(voltage);
  display.renderBattery();
}

void displayClock(int loc, time_t t){
  display.setTextSize(1);
  display.setCursor(0,loc);
  display.print(year(t));
  display.print('-');
  display.print(month(t));
  display.print('-');
  display.print(day(t));
  display.print("  ");
  printZero(hour(t));
  display.print(hour(t));
  printDigits(minute(t));
  printDigits(second(t));
}

void printDigits(int digits){
  // utility function for digital clock display: prints preceding colon and leading 0
  display.print(":");
  printZero(digits);
  display.print(digits);
}

void printZero(int val){
  if(val<10) display.print("0");
}


// 
//void printDigits(int digits){
//  // utility function for digital clock display: prints preceding colon and leading 0
//  display.print(":");
//  printZero(digits);
//  display.print(digits);
//}
//
//void printZero(int val){
//  if(val<10) display.print('0');
//}

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
