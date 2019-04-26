#define CAM_TRIG 4

void setup() {
  pinMode(CAM_TRIG, OUTPUT);
  digitalWrite(CAM_TRIG, LOW);

}

void loop() {
  for(int mSec = 10; mSec<1000; mSec+=10){
    cam_start(mSec);
    Serial.print("Cam Start: ");
    Serial.println(mSec);
    delay(10000);
    cam_start(mSec);
    Serial.print("Cam Stop: ");
    Serial.println(mSec);
    delay(10000);
  }
  
  

}


void cam_start(int mSec) {
  digitalWrite(CAM_TRIG, HIGH);
  delay(mSec);  // simulate  button press
  digitalWrite(CAM_TRIG, LOW);  
}


