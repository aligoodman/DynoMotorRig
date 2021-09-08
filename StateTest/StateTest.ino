#include <buffer.h>
#include <crc.h>
#include <datatypes.h>
#include <VescUart.h>

/* All variables*/
float current;
int count;
float Torque;
float zero;


VescUart UART2;

void setup() {
  /** Initiate VescUart class */

  pinMode(A0, INPUT);
  pinMode(A0, INPUT);
  analogReadResolution(12);
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

  if (Serial.available() > 0) {

    int inByte = Serial.read();

    
  switch(inByte) {

/*Declare current value*/
    case '1':
      current = 0;
      Serial.println("Enter Current");
      while(current == 0) {
        current = Serial.parseFloat();
      }
      
      Serial.println(current);
      break;
      
/*Test case*/
    case '2':
      digitalWrite(41, LOW);
      delay(500);
      inByte = '3';

/*Test case*/
    case '3':
      digitalWrite(41, HIGH);
      delay(500);
      inByte = '2';
      
/*Ramp current to value set from case 1*/
    case '4':
      Serial.println("Enter any value to exit");
      for( float c = 0; c <= current; c += 0.1){
        UART2.setCurrent(c);
        delay(10);
      }
      while(1) {
        UART2.setCurrent(current);
        if(Serial.parseInt() != 0) {
          break;
        }  
      }
      break;
      
/*Default case, set motor at desired current*/

    case default:
      
      
/*Get torque and vesc data one time*/
     case '5':
       UART2.setCurrent(current);
       Torque = 0;
       count = 0;
       for(int i = 0; i<= 100000; i++){
         Torque +=analogRead(A0);
         count += 1;
       }
       UART2.setCurrent(current);
       Torque = ((((Torque/count)-zero)/4096)/1.25)*20*3.3;
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
      UART2.setCurrent(current);
      break;

/*Get torque and vesc data repetativly*/
     case '6':
       while(1) {
        if(Serial.parseInt() != 0) {
           break;
        }
        UART2.setCurrent(current);
        Torque = 0;
        count = 0;
        for(int i = 0; i<= 100000; i++){
          Torque +=analogRead(A0);
           count += 1;
        }
        UART2.setCurrent(current);
        Torque = ((((Torque/count)-zero)/4096)/1.25)*20*3.3;
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
        UART2.setCurrent(current);
        if(Serial.parseInt() != 0) {
           break;
        }
      }
      break;

/*Get Torque zero when motor is stationary*/
      case '7':
        Torque = 0;
        count = 0;
        for(int i = 0; i<= 10000; i = i + 1){
          Torque +=analogRead(A0);
          count += 1;
        }
        Torque = Torque/count;
        zero = Torque;
        Serial.println(zero);
        break;
    }
  }
}
