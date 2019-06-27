//#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <LiquidCrystal.h>

bool locked = false;
bool LoRaconnected = false;
int LoRaByte = 0;

const float pi =  3.14159265359;

//SoftwareSerial LoRa(10, 11);//10 = Rx, 11 = Tx
SoftwareSerial LoRa(5, 6);//5 = Rx, 6 = Tx
//char List[10] = "abcdefghijk";

void setup()
{
  Serial.begin(115200);
  Serial.println("starting...");
  //ss.begin(GPSBaud);
  LoRa.begin(9600);

  //lcd.begin(16, 2);
  //lcd.print("LoRa Echo Tester");

  //lcd.setCursor(0, 1);//row2
  //lcd.print("msg: ");
  
  //LoRa.print("startup");
  
}

void loop()
{

  for (int i = 0 ; i < 100; i++){
      LoRa.write(i);
      //LoRa.write(List[i]);
      delay(5000);
      //if (LoRa.available() > 0) {
                // read the incoming byte:
                //LoRaByte = Serial.read();
      //}
      //if (int(LoRaByte) == 0) i = -1;
      //delay(4500);
  }

}
