void calcRMS(){
  // load file; HWSERIAL out  first n bytes
  float rms_float;
  int wavbuffer[256];
  unsigned long sumsquares = 0;
  frec = SD.open(filename);
  if(frec){
    Serial.println(filename);
    frec.seek(sizeof(wav_hdr));  // skip wav header
    for (long i = 0; i<nbufs_per_file; i++){
      frec.read(&wavbuffer, 512);
      for (int k = 0; k<256; k++){
        sumsquares += (wavbuffer[k] * wavbuffer[k]);
      }
    }
    frec.close();
    rms_float = (20.0 * log10((float) sumsquares / ( (float) nbufs_per_file * 256.0 * 32768.0))) - hydroCal;
    rms = (unsigned int) rms_float;
    Serial.println(rms_float, DEC);
  }
  else{
    Serial.println("Unable to open file to calc RMS");
  }
}
