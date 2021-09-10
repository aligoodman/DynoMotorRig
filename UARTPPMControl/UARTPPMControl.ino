#include <buffer.h>
#include <crc.h>
#include <datatypes.h>
#include <VescUart.h>

VescUart UART2;

float current;
float rpm;
void setup() {

  analogReadResolution(12);
  analogWriteResolution(12);

  Serial1.begin(19200);  
  while (!Serial) {;}

  /** Define which ports to use as UART */
  UART2.setSerialPort(&Serial1);
}

void(loop){
  if ( UART2.getVescValues() ) {
    rpm = UART2.data.rpm/11;
    current = rpm*0.005;
    UART2.setBrakeCurrent(current);
    Serial.println(rpm);
  }
  else {
    Serial.println("fail");
  }
  current = rpm*500;
  analogWrite(7,current);

}
