
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
  //getMotorAccelEncoderQUICK(&motor);
  getFWspeed(&motor, &boat, &systemParam, &oar);
  MimicHeavyFlyWheel(&motor, &boat, &systemParam, &oar);
  //getSpoonForce(&boat, &oar, &motor, &systemParam);
  //getSpoonForceSpring(&motor, &boat, &systemParam, &oar);
  //getHandleForce(&boat, &oar);
  //getMotorTorque(&boat, &motor, &systemParam, &oar);
  //setMotorTorque(&motor);
  boat.transducerForce = -40-((664.0-analogRead(A1))/23.0)*9.81;
  //Serial.println(Count);
  //Serial.print(" ");
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
//  Serial.print(ticks);
//  Serial.print(" ");
//  Serial.print(poly.A*pow(Ticks[9][0],2) + poly.B*Ticks[9][0] + poly.C);
//  Serial.print(" ");
//  Serial.print(2*poly.A*Ticks[9][0] + poly.B);
//  Serial.print(" ");
  Serial.println(2*poly.A);
  
//  Serial.print(" ");
//  Serial.print(Polyb);
//  Serial.print(" ");
//  Serial.println(Polyc);

 
}
  
