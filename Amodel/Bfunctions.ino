//get RPM over UART. 
void getMotorSpeed(MOTOR_STATE_T* ms){
  float prevRadPerS = ms->radPerS;
  float prevTime = ms->RPMtime;
  while(1) {
    if ( UART2.getVescValues() ) {
      ms->RPM = (UART2.data.rpm/7);
      ms->RPMtime = micros();
      ms->radPerS = 3.14159*2*(ms->RPM/60);
      break;
    }
    else {
      Serial.println("Fail");
    }
    ms->accel = ((ms->radPerS - prevRadPerS)/((ms->RPMtime - prevRPMtime)/1000000));
  } 
}


//calculate the force on the rowing blade, include effects of blade profile, depth in water
void getSpoonForce(BOAT_STATE_T* bs, OAR_T* oar, MOTOR_STATE_T* ms, SYSTEM_PARAM_T* sp){
  //get handle speed from motor speed
  getMotorSpeed(ms);
  bs->handleRadPerS = ms->radPerS/sp->motorHandleRatio;
  bs->spoonRadPerS = bs->handleRadPerS;
  //set blade depth to define the catch and finish
  if (ms->radPerS> bs->boatSpeed*sp->motorHandleRatio/oar->outboardLength){
    bs->bladeDepth = 1;
  }
  else if(ms->radPerS <= bs->boatSpeed*sp->motorHandleRatio/oar->outboardLength){
    bs->bladeDepth = 0;
  }

  //get relative speed, oar force and deflection from previous values
  float timeNow = micros();
  float relativeSpeed = bs->bladeDepth*((bs->spoonRadPerS*oar->outboardLength) - bs->boatSpeed); // bs->deflectionVel);
  bs->relSpeed = relativeSpeed;
  bs->spoonForce = (oar->surfaceConstant)*relativeSpeed*abs(relativeSpeed);
  float deflection = -bs->spoonForce*oar->stiffness;
  bs->deflectionVel = (deflection - bs->deflection)/((timeNow - bs->deflTimer)/1000000);
  //bs->deflectionVel = constrain(bs->deflectionVel, -0.25, 0.25);
  bs->deflection = deflection;
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
    
    instantPower = abs((ms->radPerS*(ms->current)*sp->motorKt));
    energyIn = instantPower*((bs->bstimer-prevTime)/1000000);
  }

//energy out of the system (speed squared*drag constant*distance)
  energyOut = pow(bs->boatSpeed, 2)*sp->hullDragConstant*(bs->boatSpeed*((bs->bstimer - prevTime)/1000000));
  netEnergy = energyIn - energyOut;
  bs->boatEnergy += netEnergy;
  bs->boatEnergy = constrain(bs->boatEnergy, 0, 1000000);

  //final component accounts for the moving mass of the rower effecting the boat speed
  bs->boatSpeed = sqrt((bs->boatEnergy*2)/(sp->mass)); //- bs->handleRadPerS*1.15*0.8;
  bs->boatSpeed = constrain(bs->boatSpeed, 0, 15);
}


//calculate the force on the blade handle, incldue effects of blade stiffness, blade length, blade inertia etc
void getHandleForce(BOAT_STATE_T* bs, OAR_T* oar){
  //for now we will have direct scaling
  bs->handleForce = bs->spoonForce*(oar->outboardLength/oar->inboardLength);
}


//calculate desired motor torque from blade torque, accounting for rig dynamics
void getMotorTorque(BOAT_STATE_T* bs, MOTOR_STATE_T* ms, SYSTEM_PARAM_T* sp, OAR_T* oar){
  //handle force is in newtons so need to convert this to NM
  if (bs->bladeDepth==1){
    ms->current = -(((bs->handleForce)*(oar->inboardLength))/(sp->motorHandleRatio))/(sp->motorKt); // ms->accel*0.003;
  }
  if (bs->bladeDepth == 0){
    ms->current = ms->accel*0.003;
  }
  //ms->current = ms->current;
}

//set the motor torque
void setMotorTorque(MOTOR_STATE_T* ms){
  ms->current = constrain(ms->current, -40, 4);
  UART2.setCurrent(ms->current*0.7);
}

void investigateHandlePosition(MOTOR_STATE_T* ms) {
  if(myIndex == 0){
    
  }
  ms->revs  += ms->RPM*(micros() - ms->RPMtime); 
}

void getMotorAccel(MOTOR_STATE_T* ms){
  getMotorSpeed(ms);
  prevRPMtime = ms->RPMtime;
  prevRadPerS = ms->radPerS;
  getMotorSpeed(ms);
  ms->accel = ((ms->radPerS - prevRadPerS)/((ms->RPMtime - prevRPMtime)/1000000));
  
}

void setDynoSpeed(MOTOR_STATE_T* ms){
  myIndex = int((millis() - startTime)/20);
  if (myIndex > 153){
    startTime = millis();
    ms->revs = 0;
  }
  analogWrite(DAC1, 2050 + (speedProfile[myIndex][1])*4.5);

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
