#include <SoftwareSerial.h>
SoftwareSerial BT1(3, 2); // RX | TX

String ordenes[] =
{ "AT+CIOBAUD=9600",
  "END"                 // Para reconocer el fin de los comandos AT
};

void setup()
{ Serial.begin(115200);
  BT1.begin(115200);
  int index = 0;
  while (ordenes[index] != "END")
    {
      BT1.println(ordenes[index++]);
    }
  Serial.end();
  BT1.end();
  Serial.begin(9600);
  BT1.begin(9600);
}

void loop()
{ String B = "." ;
  if (BT1.available())
  { char c = BT1.read() ;
    Serial.print(c);
  }
  if (Serial.available())
  { char c = Serial.read();
    BT1.print(c);
  }
}
