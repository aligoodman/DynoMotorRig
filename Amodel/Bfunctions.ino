
//get RPM over UART. 
void getMotorSpeed(MOTOR_STATE_T* ms, SYSTEM_PARAM_T * sp){
  float prevRadPerS = ms->radPerS;
  float prevTime = ms->RPMtime;
  while(1) {
    if ( UART2.getVescValues() ) {
      ms->RPM = -(UART2.data.rpm/7);
      ms->RPMtime = micros();
      ms->radPerS = 3.14159*2*(ms->RPM/60);
      break;
    }
    else {
      Serial.println("Fail");
    }
  } 
  ms->accel = ((ms->radPerS - prevRadPerS)/((ms->RPMtime - prevTime)/1000000));
  ms->revs  += (ms->radPerS*(ms->RPMtime - prevTime)/1000000)/sp->motorHandleRatio  ; 
}

//trying to do deflection by integrating up the amount of revolutions and correlating that to handle position, 
// from there we give spoon position as handleposition + deflection. Unsuprisingly it yielded no improved results. 
void getHandleSpeed(BOAT_STATE_T* bs, OAR_T* oar, MOTOR_STATE_T* ms, SYSTEM_PARAM_T* sp){
 
  getMotorSpeed(ms, sp);
  float nowTime = micros();
  float handleAngle = ms->revs;
  float prevRelSpeed = bs->relSpeed;
  //bs->deflection = -(bs->spoonForce*oar->stiffness)/oar->outboardLength;
  float spoonAngle = handleAngle + bs->deflection;
  bs->spoonSpeed = oar->outboardLength*(spoonAngle - bs->spoonAngle)/((nowTime - bs->deflTimer)/1000000);
  
  if (bs->spoonSpeed > bs->boatSpeed){
    bs->bladeDepth = 1;
  }
  else if(bs->spoonSpeed <= bs->boatSpeed){
    bs->bladeDepth = 0;
  }
  bs->relSpeed = bs->spoonSpeed - bs->boatSpeed;
  float relSpeedAccel = (bs->relSpeed - prevRelSpeed)/((nowTime - bs->deflTimer)/1000000);
  bs->spoonAngle = spoonAngle;
  bs->spoonForce = bs->bladeDepth*((oar->surfaceConstant)*bs->relSpeed*abs(bs->relSpeed));
  bs->deflTimer = nowTime;
}




//calculate the force on the rowing blade, include effects of blade profile, depth in water
void getSpoonForce(BOAT_STATE_T* bs, OAR_T* oar, MOTOR_STATE_T* ms, SYSTEM_PARAM_T* sp){ 
  //get handle speed from motor speed
  getMotorSpeed(ms, sp); 
  bs->handleSpeed = (ms->RPM/sp->motorHandleRatio)/60; //in m/s
  bs->handleAccel = (ms->accel/(2*3.14159))/sp->motorHandleRatio; //in m/s^2
  bs->spoonSpeed = bs->handleSpeed*(oar->outboardLength/oar->inboardLength); //in m/s
  //set blade depth to define the catch and finish
  if (bs->spoonSpeed  > 0){
    bs->bladeDepth = 1;
  }
  else if(bs->spoonSpeed  <= 0){ 
    bs->bladeDepth = 0;
  }
  
  //take a time for dt calculations 
  float timeNow = micros();
  
  //
  if((ms->radPerS*(6.33/14) > FWspeed)){
    Uncouple = false;
  }
  
  /* not using relative speed at the moment
  float relSpeed = ((bs->spoonSpeed) - bs->boatSpeed); // + bs->deflectionVel);
  float relAccel = (relSpeed - bs->relSpeed)/((timeNow - bs->deflTimer)/1000000); 
  bs->relSpeed = relSpeed; */
  
    
    filter.Tf = 0.1;
    bs->handleAccel= filter(bs->handleAccel);
    
    
    float accelComp = bs->handleAccel*(sp->cIn)*pow(70,2); 
    float velComp = bs->handleSpeed*abs(bs->handleSpeed)*(sp->cDamp));
    Serial.print(accelComp);
    Serial.print(" ");
    Serial.println(velComp);
    
      
    if(Uncouple == false){
      FWspeed = ms->radPerS*(6.33/14);
      virHandSpeed = bs->handleSpeed;
      bs->spoonForce = velComp + accelComp;
    }
    
    float FWvelComp = virHandSpeed * abs(VirHandSpeed) * (sp->cDamp);
    
    if((FWvelComp/70.67)/(sp->cIn) <= ms->motorAccel*(6.33/14) and Uncouple == false){
      Uncouple = true;
    }
    
    float FWvelComp = virHandSpeed * abs(VirHandSpeed) * (sp->cDamp);
    
    if(Uncouple == true){
      bs->spoonForce = 0;
      FWspeed = FWspeed - (FWvelComp/70.67)/(sp->cIn)*((timeNow-bs->deflTimer)/1000000));
      virHandSpeed = FWspeed/(70.67);
    }
    //Serial.println(FWspeed);

    

    
  //bs->spoonForce = bs->bladeDepth*(oar->surfaceConstant)*bs->relSpeed*abs(bs->relSpeed) + bs->handleAccel*sp->cIn;

  //need constrain to stop decel force working against you. 
    bs->spoonForce = constrain(bs->spoonForce, 0, 0);
   
//  float deflection = -bs->spoonForce*oar->stiffness;
//  bs->deflectionVel = (deflection - bs->deflection)/((timeNow - bs->deflTimer)/1000000);
//  //bs->deflectionVel = constrain(bs->deflectionVel, -0.25, 0.25);
//  bs->deflection = deflection;
    bs->deflTimer = timeNow;

}



void getBoatSpeed(BOAT_STATE_T* bs, MOTOR_STATE_T* ms, SYSTEM_PARAM_T* sp){

//energy into the system
  float energyIn = 0;
  float energyOut = 0;
  float instantPower;
  float prevTime = bs->bstimer;
  float netEnergy;
  bs->bstimer = micros();
  
  if (bs->bladeDepth == 1){
    
    instantPower = abs(bs->handleSpeed*bs->handleForce);
    energyIn = instantPower*((bs->bstimer-prevTime)/1000000);
  }

//energy out of the system (speed squared*drag constant*distance)
  energyOut = pow(bs->boatSpeed, 2)*sp->hullDragConstant*(bs->boatSpeed*((bs->bstimer - prevTime)/1000000));
  netEnergy = energyIn - energyOut;
  bs->boatEnergy += netEnergy;
  bs->boatEnergy = constrain(bs->boatEnergy, 0, 1000000);

  //final component accounts for the moving mass of the rower effecting the boat speed
  bs->boatSpeed = sqrt((bs->boatEnergy *2)/(sp->mass)); // bs->handleSpeed*0.8;
  bs->boatSpeed = constrain(bs->boatSpeed, 0, 15) ;
}


//calculate the force on the blade handle, incldue effects of blade stiffness, blade length, blade inertia etc
void getHandleForce(BOAT_STATE_T* bs, OAR_T* oar){
  //for now we will have direct scaling
  bs->handleForce = bs->spoonForce;
}


//calculate desired motor torque from blade torque, accounting for rig dynamics
void getMotorTorque(BOAT_STATE_T* bs, MOTOR_STATE_T* ms, SYSTEM_PARAM_T* sp, OAR_T* oar){
  //handle force is in newtons so need to convert this to NM
    ms->current = 1 + ((bs->handleForce)/(sp->motorHandleRatio*2*3.14159))/(sp->motorKt); // ms->accel*0.003;
  
  //ms->current = filter(ms->current);
}

//set the motor torque
void setMotorTorque(MOTOR_STATE_T* ms){

  ms->current = constrain(ms->current, -15, 15);
  UART2.setCurrent(ms->current);
}

void investigateHandlePosition(MOTOR_STATE_T* ms) {
  ms->revs  += ms->RPM*(micros() - ms->RPMtime); 
  
}

void getMotorAccel(MOTOR_STATE_T* ms, SYSTEM_PARAM_T* sp ){
  getMotorSpeed(ms, sp);
  prevRPMtime = ms->RPMtime;
  prevRadPerS = ms->radPerS;
  getMotorSpeed(ms, sp);
  ms->accel = ((ms->radPerS - prevRadPerS)/((ms->RPMtime - prevRPMtime)/1000000));
  
}

void setDynoSpeed(MOTOR_STATE_T* ms){
  myIndex = int((millis() - startTime)/20);
  if (myIndex > 153){
    startTime = millis();
    ms->revs = 0;
    Serial.println(i);
    i = 0;
  }
  i += 1;
  if(speedProfile[myIndex][0]>0){
    analogWrite(DAC1, 2030 + (speedProfile[myIndex][0])*5);
  }
  else {
    analogWrite(DAC1, 2030 + (speedProfile[myIndex][0])*5);
  }

}

void setupThings(){
  // UART things for vesc
  Serial1.begin(115200);  
  while (!Serial) {;}
  UART2.setSerialPort(&Serial1);
  
  //raise resolution to 12 bit. 
  analogReadResolution(12);
  analogWriteResolution(12);
  
  Serial.begin(9600);
  
  digitalWrite(DAC0, HIGH);
  analogWrite(DAC1, 2050);
  
  startTime = millis();
}
