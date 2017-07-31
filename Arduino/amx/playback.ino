// Playback with Adafruit sound board

void playBackOn(){
  gpsOn();
}

void playBackOff(){
  gpsOff();
}

// use saltSig (SS) as reset pin
void playBackReset(){
  digitalWrite(saltSIG, LOW);
  pinMode(saltSIG, OUTPUT);
  delay(10);
  pinMode(saltSIG, INPUT);
  delay(1000);
  getPlaybackResponse();
}

void listPlaybackFiles(){
  HWSERIAL.println('L');
  getPlaybackResponse();
}

void playTrackNumber(int n){
  HWSERIAL.write('#');
  HWSERIAL.println(n);
  if(printDiags) {
    Serial.print("Track: "); 
    Serial.println(n);
  }

}

void setPlaybackVolumeUp(){
  HWSERIAL.println('+');
  HWSERIAL.flush();
  getPlaybackResponse();
}

void setPlaybackVolumeDown(){
  HWSERIAL.println('-');
  HWSERIAL.flush();
  getPlaybackResponse();
}

void getPlaybackResponse(){
  delay(1);
  while(HWSERIAL.available()){
    Serial.print(HWSERIAL.read());
  }
}

