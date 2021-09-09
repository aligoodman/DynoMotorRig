#include <buffer.h>
#include <crc.h>
#include <datatypes.h>
#include <VescUart.h>

/* All variables*/
float current;
int count;
float Torque;
float zero;
int inByte = 0;
float sumSpeed;
float avSpeed;
float currentScale;


VescUart UART2;

void setup() {
  /** Initiate VescUart class */

  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  analogReadResolution(12);
  analogWriteResolution(12);
  Serial.begin(9600);
  Serial.setTimeout(3);
  pinMode(41, OUTPUT);

  /** Setup UART port (Serial1 on Atmega32u4) */
  Serial1.begin(19200);  
  while (!Serial) {;}

  /** Define which ports to use as UART */
  UART2.setSerialPort(&Serial1);
}

void loop() {
  if(inByte == 0) {
    
      Serial.setTimeout(300000);
      Serial.println("Select Case");
      inByte = Serial.parseInt();
      Serial.println(inByte);
      Serial.setTimeout(3);
    
  }
    
  switch(inByte) {

/*Declare current value*/
    case 1:
      current = 0;
      Serial.println("Enter Current");
      while(current == 0) {
        current = Serial.parseFloat();
      }
      
      Serial.println(current);
      inByte = 0;
      break;
      
/*Test case*/
    case 2:
      digitalWrite(41, LOW);
      delay(500);
      inByte = 3;
      break;

/*Test case*/
    case 3:
      digitalWrite(41, HIGH);
      delay(500);
      inByte = 2;
      break;
      
/*Ramp current to value set from case 1*/
    case 4:
      Serial.println("Enter any value to exit");
      for( float c = 0; c <= current; c += 0.1){
        UART2.setBrakeCurrent(c);
        if(Serial.parseInt() != 0) {
          break;
        } 
        delay(10);
      }
      while(1) {
        UART2.setBrakeCurrent(current);
        if(Serial.parseInt() != 0) {
          break;
        }  
      }
      inByte = 0;
      break;
      
/*Get torque and vesc data one time*/
     case 5:
       UART2.setBrakeCurrent(current);
       Torque = 0;
       count = 0;
       for(int i = 0; i<= 100000; i++){
         Torque +=analogRead(A0);
         count += 1;
       }
       UART2.setBrakeCurrent(current);
       Torque = ((((Torque/count)-zero)/4096)/2.5)*20*3.3;
       if ( UART2.getVescValues() ) {

        Serial.print("RPM, ");
        Serial.println(UART2.data.rpm/11);
        Serial.print("Input Voltage, ");
        Serial.println(UART2.data.inpVoltage);
        Serial.print("Input Current, ");
        Serial.println(UART2.data.avgInputCurrent);
        Serial.print("Motor Current, ");
        Serial.println(UART2.data.avgMotorCurrent);
        Serial.print("Torque, ");
        Serial.println(Torque);
        inByte = 8;

       }
       else {
        Serial.println("Failed to get data!");
        inByte = 5;
       }
      UART2.setBrakeCurrent(current);
      break;

/*Get torque and vesc data repetativly*/
     case 6:
       while(1) {
        if(Serial.parseInt() != 0) {
           break;
        }
        UART2.setBrakeCurrent(current);
        Torque = 0;
        count = 0;
        for(int i = 0; i<= 100000; i++){
          Torque +=analogRead(A0);
           count += 1;
        }
        UART2.setBrakeCurrent(current);
        Torque = ((((Torque/count)-zero)/4096)/2.5)*20*3.3;
        if ( UART2.getVescValues() ) {

          Serial.print("RPM, ");
          Serial.println(UART2.data.rpm/7);
          Serial.print("Input Voltage, ");
          Serial.println(UART2.data.inpVoltage);
          Serial.print("Input Current, ");
          Serial.println(UART2.data.avgInputCurrent);
          Serial.print("Motor Current, ");
          Serial.println(UART2.data.avgMotorCurrent);
          Serial.print("Torque, ");
          Serial.println(Torque);

        }
        else {
          Serial.println("Failed to get data!");
        }
        UART2.setBrakeCurrent(current);
        if(Serial.parseInt() != 0) {
          break;
        }
      }
      inByte = 0;
      break;

/*Get Torque zero when motor is stationary*/
      case 7:
        Torque = 0;
        count = 0;
        for(int i = 0; i<= 10000; i = i + 1){
          Torque +=analogRead(A0);
          count += 1;
        }
        Torque = Torque/count;
        zero = Torque;
        Serial.println("Zero is: ");
        Serial.print(zero);
        inByte = 0;
        break;
      
/* Ramp torque down rather than a hard stop*/      
      case 8:
        for( float c = current; c >= 0; c -= 0.1){
          UART2.setBrakeCurrent(c);
          delay(10);
        }
      current = 0;
      inByte = 0;
      break;

/* want to investigate the frequency response of the motor*/
      case 9:
        digitalWrite(41, HIGH);
        delay(3000);
        UART2.setBrakeCurrent(current);
        delay(5000);
        current = 0;
        UART2.setBrakeCurrent(current);
        inByte = 0;
        break;
        
/*Have a go at a torque tracking given a speed input*/
      case 10:
        sumSpeed = 0;
        current = 0;
        while(1) {
        //average over 10 readings
          sumSpeed=0;
          for(int b = 0; b <=10; ++b) {
            sumSpeed += analogRead(A1);
            if(Serial.parseInt() != 0) {
              inByte = 8;
              break;
            }
            delay(5);
          }
          Serial.println(analogRead(A1));
          avSpeed = (sumSpeed/11);
          Serial.println(avSpeed);
          currentScale = 0.5;
          current = avSpeed;
          
          analogWrite(DAC0,current);
          //UART2.setBrakeCurrent(current);
          if(Serial.parseInt() != 0) {
            inByte = 8;
            break;
          }
          break;       
        }

/* want to see how quickly we can pull data from the vesc (mainly rpm)*/
      case 11:
        while(1) {
            if ( UART2.getVescValues() ) {
              float rpm = UART2.data.rpm/11;
              current = rpm*0.005;
              UART2.setBrakeCurrent(current);
              Serial.println(rpm);
            }
            else {
              Serial.println("fail");
            }
        }
      
    }
}
