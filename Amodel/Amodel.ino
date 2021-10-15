#include <SimpleFOC.h>
#include <Filters.h>
#include <VescUart.h>
VescUart UART2;
#include "variables.h"

LowPassFilter filter = LowPassFilter(0.4);                                   



void setup() {

setupThings();

}



void loop(){
  //setDynoSpeed(&motor);
  getSpoonForce(&boat, &oar, &motor, &systemParam);
  //getHandleSpeed(&boat, &oar, &motor, &systemParam);
  getHandleForce(&boat, &oar);
  getMotorTorque(&boat, &motor, &systemParam, &oar);
  getBoatSpeed(&boat, &motor, &systemParam);
  setMotorTorque(&motor);
  //investigateHandlePosition(&motor);
  //Serial.print(boat.spoonForce);
  //Serial.print(" ");
  //Serial.println(filter(boat.spoonForce));
  //Serial.print(" ");
  //Serial.println(boat.deflectionVel);
  
  
  
  
  
  
  

}
