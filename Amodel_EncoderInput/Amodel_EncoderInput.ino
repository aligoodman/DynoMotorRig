
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
  setMotorTorque(&motor);
  boat.transducerForce = -40-((664.0-analogRead(A1))/23.0)*9.81;
  
  //CollectDataForPoly();
  //PolyFit(&poly, &motor);
//  static int tf = 0;
//  tf++;
//  if(tf == 1000){
//    Serial.println(motor.accel);
//    tf = 0;
//  }
}
  
