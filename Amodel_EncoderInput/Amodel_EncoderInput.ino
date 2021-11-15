
#include <VescUart.h>
VescUart UART2;
#include "variables.h"
unsigned long LastTime1, Time;
                                
void setup() {

setupThings();
pinMode(50, OUTPUT);
pinMode(A0, INPUT);
}



void loop(){
 
  getMotorSpeedEncoder(&motor);
  getMotorAccelEncoder(&motor);
  getFWspeed(&motor, &boat, &systemParam, &oar);
  MimicHeavyFlyWheel(&motor, &boat, &systemParam, &oar);
  //setMotorTorque(&motor);
  boat.transducerForce = -40-((664.0-analogRead(A1))/23.0)*9.81;
  
//  static int z = 0;
//  z++;
//  if(z==5){
//
//    Serial.print(motor.accel);
//    Serial.print(" ");
//    Serial.print(motor.radPerS);
//    Serial.print(" ");
//    Serial.println(motor.dutyRPM);
//    z = 0;
//  }
  CollectDataForPoly();
  PolyFit(&poly);
  static int rt = 0;
  rt++
  if(rt = 100){
    Serial.print(millis());
  }
 
}
  
