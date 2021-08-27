/*

*/
float v0 = 1;
float vgrad = 1;
float voltage = 0;
float rpm = 0;
float bgrad = 1;
float value;
float max;


void setup() {
  
  pinMode(3, OUTPUT);
  Serial.begin(9600);
  Serial.setTimeout(3000000);
}

void loop() {
  
  //voltage range between 0.55-2.75volts, 2.2 volts. Analog write resolution 0-4095. 
  //need to relate rpm request to input voltage. 
  // 0 rpm voltage = v0
  // gradient of speed voltage curve = vgrad
  // voltage to bit = bgrad

  Serial.println("Select procedure:");
  while(Serial.available()== 0) {
    
  }
  
  if(Serial.parseInt() == "rpm" or "RPM") {
    
    
    Serial.println("Request RPM?"); //Prompt User for Input
  while (Serial.available() == 0) {
    // Wait for User to Input Data
  }
  rpm = Serial.parseInt(); //Read the data the user has input
  voltage = (rpm/vgrad) + v0;
  value = bgrad*(voltage);
  analogWrite(3, value); 
  }
  
  if(Serial.parseInt() == "Ramp up" or "ramp up") {
    Serial.println("Ramp to Input number?");
    //Will need conversion from rpm to input value
    max = Serial.parseInt();
    // need something to convert rpm to voltage signal
    for (int i = 0; i <= max; i++) {
    analogWrite(DAC0, i);
    delay(10);
  }

}