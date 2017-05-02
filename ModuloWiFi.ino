#include <SoftwareSerial.h>
SoftwareSerial BT1(3, 2); // RX | TX

String ordenes1[] =
{ "AT+CIOBAUD=9600",
  "END"                 // Para reconocer el fin de los comandos AT
};

void setup()
{ Serial.begin(115200);
  BT1.begin(115200);
 int index = 0;
  while (ordenes1[index] != "END")
    {
      BT1.println(ordenes1[index++]);
    }
  Serial.end();
  BT1.end();
  Serial.begin(9600);
  BT1.begin(9600);
}

void escribir(String text)
{BT1.print("AT+CIPSEND=0,");  
 BT1.println(text.length());
 if (BT1.find(">"))             // Si se recibe el mensaje
 {Serial.println(text);
  BT1.println(text);            //mandamos el mensaje por el wifi 
  delay(10);
  while ( BT1.available() > 0 ) 
  {if (  BT1.find("SEND OK") )  //buscamos "ok" y luego salimos
   break; 
  }
 }
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
  servidor(); 
}
int i=0;
void servidor(void) 
    { //escribir("<!DOCTYPE HTML>");
      escribir("<html>");                                                  //una pagina web necesita esto <HTML> y </HTML> es el inicio y fin del documento
      escribir("<head><title>Prueba arduino esp8266 html :)</title>");    //nombre de la pestaña que llevara la pagina
      escribir("<meta http-equiv=\"refresh\" content=\"3\"></head>");    //tiempo para refrescar la pagina web
      escribir("<body><h1> Conteo de Ejemplo "+String(i)+"</h1>"); 
      escribir("</html>");
      i++;
      delay(1);
      BT1.println("AT+CIPCLOSE=0");
      //delay(500);
    } 
