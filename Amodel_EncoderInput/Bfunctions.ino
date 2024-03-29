
enum VESC_MASK_E:uint32_t{
  VESC_MASK_tempMosfet=((uint32_t)1 << 0), // 2 bytes - mc_interface_temp_fet_filtered()
  VESC_MASK_tempMotor=((uint32_t)1 << 1),  // 2 bytes - mc_interface_temp_motor_filtered()
  VESC_MASK_avgMotorCurrent=((uint32_t)1 << 2), // 4 bytes - mc_interface_read_reset_avg_motor_current()
  VESC_MASK_avgInputCurrent=((uint32_t)1 << 3), // 4 bytes - mc_interface_read_reset_avg_input_current()
  VESC_MASK_dutyCycleNow=((uint32_t)1 << 6),  // 2 bytes - mc_interface_get_duty_cycle_now()
  VESC_MASK_rpm=((uint32_t)1 << 7),   // 4 bytes - mc_interface_get_rpm()
  VESC_MASK_inpVoltage=((uint32_t)1 << 8),    // 2 bytes - GET_INPUT_VOLTAGE()
  VESC_MASK_ampHours=((uint32_t)1 << 9), // 4 bytes - mc_interface_get_amp_hours(false)
  VESC_MASK_ampHoursCharged=((uint32_t)1 << 10), // 4 bytes - mc_interface_get_amp_hours_charged(false)
  VESC_MASK_wattHours=((uint32_t)1 << 11), // 4 bytes - mc_interface_get_watt_hours(false)
  VESC_MASK_wattHoursCharged=((uint32_t)1 << 12), // 4 bytes - mc_interface_get_watt_hours_charged(false)
  VESC_MASK_tachometer=((uint32_t)1 << 13), // 4 bytes - mc_interface_get_tachometer_value(false)
  VESC_MASK_tachometerAbs=((uint32_t)1 << 14), // 4 bytes - mc_interface_get_tachometer_abs_value(false)
  VESC_MASK_error=((uint32_t)1 << 15), // 1 byte  - mc_interface_get_fault()
  VESC_MASK_pidPos=((uint32_t)1 << 16), // 4 bytes - mc_interface_get_pid_pos_now()
  VESC_MASK_id=((uint32_t)1 << 17),  // 1 byte  - app_get_configuration()->controller_id 
      // additionally, vedder has:
      // 18th value float16[3] = {NTC_TEMP_MOS1,NTC_TEMP_MOS2,NTC_TEMP_MOS3}
      // 19th value float32 = mc_interface_read_reset_avg_vd()
      // 20th value float32 = mc_interface_read_reset_avg_vd()
};

//if we are using the string pot
void GetPotSpeed(MOTOR_STATE_T* ms){

      PotReading = analogRead(A0);
      for(int z = 0; z<10; z++){
        potReadings[z][0] = potReadings[z+1][0];
        potReadings[z][1] = potReadings[z+1][1];
      }
      potReadings[10][0] = PotReading;
      potReadings[10][1] = (float)(ms->RPMtime - ms->prevRPMtime)*1e-6;

      float AvTime = 0;
      for(int z = 0; z<11; z++){
        AvTime += (potReadings[z][1])/11;
      }
      float PotSpeedSimple = (potReadings[1][0] - potReadings[0][0])/potReadings[1][1];
      float PotSpeed = (2.032/4095)*(-2*potReadings[0][0] + 25*potReadings[1][0] -150*potReadings[2][0] + 600*potReadings[3][0] - 2100*potReadings[4][0] + 2100*potReadings[6][0] - 600*potReadings[7][0] + 150*potReadings[8][0] - 25*potReadings[9][0] + 2*potReadings[10][0])/(2520*AvTime);
      PotSpeed = PotSpeed;
      Serial.println(PotSpeed);
}


//if we want speed and accel data over UART
void getMotorSpeedUART(MOTOR_STATE_T* ms, SYSTEM_PARAM_T* sp, BOAT_STATE_T* bs){
  bs->transducerForce = ((664.0-analogRead(A1))/23.0)*9.81;
  float prevRadPerS = ms->radPerS;
  unsigned long prevTime = ms->RPMtime;
  ms->prevRPMtime = ms->RPMtime;
  float prevDuty = ms->duty;
  while(1) {
    if ( UART2.getVescValuesSelective((uint32_t)(VESC_MASK_rpm | VESC_MASK_dutyCycleNow)) ) {
      ms->duty = -UART2.data.dutyCycleNow;
      ms->RPM = -UART2.data.rpm/7;
      ms->RPMtime = micros();
      ms->radPerS = 3.14159*2*(ms->RPM/60);
      if( abs(ms->RPM) < 4000){
        break;
      }
    }  
  }
  ms->dutyRPM = ms->duty*2500;
  ms->dutyAccel = 261.8*((ms->duty - prevDuty)/(((float)(ms->RPMtime - prevTime))/1000000.0));
  ms->accel = (((ms->radPerS - prevRadPerS)/(((float)(ms->RPMtime - prevTime))/1000000.0)));
}



//calculate spoon force, this was what was used a while ago and I dont use it anymore, at the moment we use MimicHeavyFlywheel()
void getSpoonForce(BOAT_STATE_T* bs, OAR_T* oar, MOTOR_STATE_T* ms, SYSTEM_PARAM_T* sp){ 
  
  //set blade depth to define the catch and finish
  if (ms->RPM > 0){
    bs->bladeDepth = 1;
  }
  else if(ms->RPM  <= 0){ 
    bs->bladeDepth = 0;
  }
  
  //take a time for dt calculations 
  unsigned long timeNow = micros();
    
  if((ms->radPerS*(10.55/14) > bs->FWspeed)){
    Uncouple = false;
  }

  
   
  float accelComp =(ms->accel*(10.55/14))*(sp->cIn); 
  float velComp = ms->radPerS*abs(ms->radPerS)*(sp->cDamp)*1e-6*pow((10.55/14),2);
  
  if(Uncouple == false){
    bs->spoonForce = (70.67)*(accelComp + velComp);
    bs->FWspeed = ms->radPerS*(10.55/14);
  }
  
  float FWvelTorque = bs->FWspeed * abs(bs->FWspeed) * (sp->cDamp) * (1e-6);
    
  if((FWvelTorque)/(sp->cIn) <= - ms->accel*(10.55/14) and Uncouple == false){
    Uncouple = true;
  
  }
    
  if(Uncouple == true){
    SoftStart = 0;
    bs->spoonForce = 0;
    bs->FWspeed = bs->FWspeed - ((FWvelTorque)/(sp->cIn))*(((float)(timeNow-bs->deflTimer))/1000000.0);
  }

  
  bs->spoonForce = bs->spoonForce*bs->bladeDepth;
  bs->spoonForce = constrain(bs->spoonForce, 0, 1000);  
  bs->deflTimer = timeNow;

}

//currently used spoonforce calculator
void MimicHeavyFlyWheel(MOTOR_STATE_T* ms, BOAT_STATE_T* bs, SYSTEM_PARAM_T* sp, OAR_T* oar){
 
  float Inertia = (sp->cIn)*(pow((ms->radPerS/(FWengageSpeed)),8));
  Inertia = constrain(Inertia, 0, (sp->cIn));
  float accelComp_  = ms->accel*Inertia; 
  float velComp_ = ms->radPerS*abs(ms->radPerS)*sp->cDamp*1e-6;
  bs->spoonForce = (70.67)*((accelComp_) + velComp_);
  if(ms->radPerS <= 0){
    bs->spoonForce = 0;
  }
  bs->spoonForce = constrain(bs->spoonForce, 0, 1000);
  ms->current = ((bs->spoonForce)/(sp->motorHandleRatio*2*3.14159))/(sp->motorKt);
}

//tracks the flywheel speed so we know when to couple and uncouple etc
void getFWspeed(MOTOR_STATE_T* ms, BOAT_STATE_T* bs, SYSTEM_PARAM_T* sp, OAR_T* oar){
  if(ms->radPerS > bs->FWspeed){
    Uncouple = false;
  }
   if(Uncouple == false){
    bs->FWspeed = ms->radPerS;
  }
  float FWvelTorque = bs->FWspeed * bs->FWspeed * (sp->cDamp) * (1e-6);   
  if((FWvelTorque)/(sp->cIn) <= - ms->accel and Uncouple == false){
    Uncouple = true; 
  }   
  if(Uncouple == true){
    bs->FWspeed = bs->FWspeed - ((FWvelTorque)/(sp->cIn))*dt;
    FWengageSpeed = bs->FWspeed;
  } 
}


//get handle force from spoon force, on ergo mode they are the same
void getHandleForce(BOAT_STATE_T* bs, OAR_T* oar){
  bs->handleForce = bs->spoonForce;
  //bs->handleForce = bs->spoonForce*(oar->inboardLength/oar->outboardLength);
}



//dont use this at the moment, was used with GetSpoonForce(). 
void getMotorTorque(BOAT_STATE_T* bs, MOTOR_STATE_T* ms, SYSTEM_PARAM_T* sp, OAR_T* oar){
    float s;
    
    if(ms->RPM>10){
      s = 1.0;
    }
    else if(ms->RPM<-10){
      s = -1.0;
    }
    else if(abs(ms->RPM)<10){
      s = 0.0;
    }
    
    ms->InertialOffsetTorque = (ms->accel*0.0005 + s*0.05);
    ms->InertialOffsetTorque = constrain(ms->InertialOffsetTorque, -0.6, 0.6);
    
    if( Uncouple == false){
      ms->current = ((bs->handleForce)/(sp->motorHandleRatio*2*3.14159))/(sp->motorKt);
    }
    if ( Uncouple == true){
      ms->current = -ms->InertialOffsetTorque/sp->motorKt; 
    } 


    
}

//sends current demand to motor, separate function so I can have it on or off without effecting everything else
void setMotorTorque(MOTOR_STATE_T* ms){
  ms->current = constrain(ms->current, -80, 80);
  UART2.setCurrent((ms->current));
}

//used in dyno rig
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

//get motor speed fromt the encoder using the speed observer from vedders code
void getMotorSpeedEncoder(MOTOR_STATE_T* ms) {
  if(PeriodA == 0){
    PeriodA = 100000000;
  }
  ms->dutyRPM = ((42000000/PeriodA)/(1024))*3.14159*2;
  phase = (ticks/4096)*2*M_PI;
  while((micros()-LastTime)<(ExecuteTime - 5)){
    
  }
  TimeNow = micros();
  dt = (TimeNow - LastTime)/1000000.0;
  delta_theta = phase-phase_var;
  phase_var += (ms->radPerS + Kp*delta_theta)*dt;
  ms->radPerS += (Ki*delta_theta)*dt;
  ms->RPM = ms->radPerS*(60/(2*M_PI));
  LastTime = TimeNow;
  OldSpeed = ms->radPerS;
}


//get accel data from encoder using vedders code
void getMotorAccelEncoder(MOTOR_STATE_T* ms){

  TimeNowAccel = micros();
  dt = (TimeNowAccel - LastTimeAccel)/1000000.0;
  delta_speed = ms->radPerS - Speed_var;
  Speed_var += (ms->accel + KpA*delta_speed)*dt;
  ms->accel += (KiA*delta_speed)*dt;
  LastTimeAccel = TimeNow;


}

//setup stuff
void setupThings(){
  // UART things for vesc
  pinMode(A6, INPUT_PULLUP);
  pinMode(A7, INPUT_PULLUP);
  Serial1.begin(460800);  
  while (!Serial) {;}
  UART2.setSerialPort(&Serial1);
  
  //raise resolution to 12 bit. 
  analogReadResolution(12);
  analogWriteResolution(12);
  pinMode(A1, INPUT);
  
  Serial.begin(115200);
  
  digitalWrite(DAC0, HIGH);
  analogWrite(DAC1, 2050);
  
  startTime = millis();

//interrupt things
  pinMode(46, INPUT_PULLUP);
  pinMode(44, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(46), CountTicksA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(44), CountTicksB, CHANGE);

//set the constant time values and calculate the constant parts of the least squares curve fit
  for(int tr = 0; tr<Length; tr++){
    Ticks[tr][0] = (ExecuteTime/1000000.0)*tr;
  }
  Sxx = getSxx();
  Sxx2 = getSxx2();
  Sx2x2 = getSx2x2();
  Sx2 = getSx2();
  Sx = getSx();

  Denom = (Sxx*Sx2x2 - pow(Sxx2,2));
}



//-------------------------------------least squares encoder fitting---------------------------------

//store the ticks data into an array for curve fit
void CollectDataForPoly(){
  for(int z = 0; z<(Length - 1); z++){
        Ticks[z][1] = Ticks[z+1][1];
      }
  Ticks[Length - 1][1] = ticks;

}

//all this stuff is for calculating the curve fit
float getSxp(int xorder, int yorder){
  float S = 0;
  for(int v = 0; v<Length; v++){
    S += pow(Ticks[v][0], xorder)*pow(Ticks[v][1], yorder);
  }
  return S;
}

float getSxx(){
  float Sxx = getSxp(2, 0) - (pow(getSxp(1,0),2)/Length);
  return Sxx;
}

float getSxy(){
  float Sxy = getSxp(1,1) - (Sx*getSxp(0,1))/Length;
  return Sxy;
}

float getSxx2(){
  float Sxx2 = getSxp(3,0) - (getSxp(1,0)*getSxp(2,0)/Length);
  return Sxx2;
}

float getSx2y(){
  float Sx2y = getSxp(2,1) - Sx2*getSxp(0,1)/Length;
  return Sx2y;
}

float getSx2x2(){
  float Sx2x2 = getSxp(4,0) - pow(getSxp(2, 0),2)/Length;
  return Sx2x2;
}

float getSy(){
  float Sy = getSxp(0, 1);
  return Sy;
}

float getSx2(){
 float Sx2 = getSxp(2,0);
 return Sx2;
}

float getSx(){
  float Sx = getSxp(1, 0);
  return Sx;
}


void PolyFit(PolyData* PD, MOTOR_STATE_T* ms){

  float Sxy = getSxy();
  float Sx2y = getSx2y();
  float Sy = getSy();

  PD->A = (Sx2y*Sxx - Sxy*Sxx2)/Denom;
  PD->B = (Sxy*Sx2x2 - Sx2y*Sxx2)/Denom;
  PD->C = Sy/Length - PD->B*(Sx/Length) - PD->A*Sx2/Length;

  ms->POLYradPerS = (2*poly.A*Ticks[Length - 1][0] + PD->B)*2*3.14159/4096;
  ms->POLYaccel = (2*poly.A)*2*3.14159/4096; 
} 

//-------------------------- Our interrupt functions -------------------------------------------------------------

void CountTicksA () {
  if(digitalRead(46)){
    if(digitalRead(44)){
      ticks--;
    }
    else{
      ticks++;
    }
  }
  else{
    if(digitalRead(44)){
      ticks++;
    }
    else{
      ticks--;
    }
  }
}


void CountTicksB () {
  if(digitalRead(44)){
    if(digitalRead(46)){
      ticks++;
    }
    else{
      ticks--;
    }
  }
  else{
    if(digitalRead(46)){
      ticks--;
    }
    else{
      ticks++;
    }
  }
}
