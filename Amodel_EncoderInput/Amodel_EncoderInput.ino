
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
  //getSpoonForce(&boat, &oar, &motor, &systemParam);
  getSpoonForceSpring(&motor, &boat, &systemParam, &oar);
  getHandleForce(&boat, &oar);
  getMotorTorque(&boat, &motor, &systemParam, &oar);
  setMotorTorque(&motor);
  Serial.println(motor.current);
  //Serial.print(" ");
  //Serial.println(boat.spoonSpeed);

 
}
  
