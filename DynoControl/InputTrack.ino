/*

*/

float track;

void setup() {
  
  Serial.begin(9600);
  Serial.setTimeout(3000000);
  analogWrite(DAC0, 0);
  analogWriteResolution(12);
}

void loop() {
  
  track = analogRead(A0);
  analogWrite(DAC0, track)
  
}
