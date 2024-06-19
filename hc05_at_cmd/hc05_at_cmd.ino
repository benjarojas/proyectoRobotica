#include <SoftwareSerial.h>

// Script para enviar comandos AT al módulo HC-05
// HC-05 debe estar en modo configuración!

// Arduino RX (A4) <- Module TX
// Arduino TX (A5) -> Module RX
SoftwareSerial BT(18,19); // A4, A5

void setup(){
  Serial.begin(9600);
  BT.begin(38400);

  Serial.println("Ready for AT commands");
}

void loop(){
  if(BT.available())
    Serial.write(BT.read());
  
  if(Serial.available())
    BT.write(Serial.read());
}
