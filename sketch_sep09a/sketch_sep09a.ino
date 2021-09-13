#include <Servo.h>
int input;
float value;
float sinSignal;
Servo myServo;

#include <buffer.h>
#include <crc.h>
#include <datatypes.h>
#include <VescUart.h>

VescUart UART2;

void setup() {
  // put your setup code here, to run once:
  Serial.setTimeout(100000);
  Serial.begin(9600);
  Serial1.begin(19200);
  analogWriteResolution(12);
  analogReadResolution(12);
  myServo.attach(7);
  myServo.writeMicroseconds(1500);
  UART2.setSerialPort(&Serial1);
}



/*void loop() {

  Serial.println("Enter a value between -100 and 100");
  value = Serial.parseInt();
  if( value>1000 or value<-1000){
    Serial.println("Value out of range");
  }
  else {
    input = 2048 + (value);
    Serial.println(input);
    analogWrite(7, input);  
  }
}
*/


void loop() {
  Serial.println("Enter a value between -1000 and 1000");
  value = Serial.parseInt();
  if( value>1000 or value<-1000){
    Serial.println("Value out of range");
  }
  else {
    input = 2048 + (value);
    Serial.println(input);
    if(value > 0) {
      for(int i = 0; i <= value; i++) {
        analogWrite(7, 2048 + i);
 
      }
    }
    if(value <= 0) {
      for(int a = 0; a <= abs(value); a=a+1){
        sinSignal = value*abs(sin((10*a/(abs(value)))*(3.14159)));
        //analogWrite(7, 2048 + sinSignal);
        //delay(200);
        //Serial.println(sinSignal);
        myServo.writeMicroseconds(1500+(sinSignal));
        UART2.setBrakeCurrent(sinSignal/value);
        delay(35);
      }
     //analogWrite(7,0);
    }
    
  }
}
