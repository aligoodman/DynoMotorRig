//initialise UART stuff for the vesc
#include <VescUart.h>
VescUart UART2;
#include "variables.h"


void setup() {

setupThings();

}



void loop(){
  setDynoSpeed(&motor);
  getSpoonForce(&boat, &oar, &motor, &systemParam);
  getHandleForce(&boat, &oar);
  getMotorTorque(&boat, &motor, &systemParam, &oar);
  getBoatSpeed(&boat, &motor, &systemParam);
  //setMotorTorque(&motor);
  //investigateHandlePosition(&motor);
  Serial.print(boat.relSpeed);
  Serial.print(" ");
  //Serial.print((motor.RPM)/400);
  //Serial.print(" ");
  Serial.println(boat.deflectionVel);
  
  
  
  
  
  
  

}
