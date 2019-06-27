#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <LiquidCrystal.h>
/*
   This sample sketch demonstrates the normal use of a TinyGPS++ (TinyGPSPlus) object.
   It requires the use of SoftwareSerial, and assumes that you have a
   4800-baud serial GPS device hooked up on pins 4(rx) and 3(tx).
*/
static const int RXPin = 8, TXPin = 9;
static const uint32_t GPSBaud = 9600;

float latitudeNOW = 0;
float longitudeNOW = 0;

float latitudeTHEN = 0;
float longitudeTHEN = 0;

//UCT
float latitudeORIGIN = -33.958381;
float longitudeORIGIN = 18.459545;

float testfloatA = 1223334445;
float testfloatB = 6677788889;

char testmsg[20] = "thisisatestmessage";
char data_in[20];

//home
//float latitudeORIGIN = -34.0250155;
//float longitudeORIGIN = 18.4627513;

bool locked = false;
bool LoRaconnected = false;
int duration = 0;

char data_prev = 0;
uint8_t error_counter = 0;

const float pi =  3.14159265359;

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);
SoftwareSerial LoRa(10, 11);//10 = Rx, 11 = Tx



// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(6, 7, 5, 4, 3, 2);

void setup()
{
  Serial.begin(115200);
  ss.begin(GPSBaud);
  LoRa.begin(9600);

  lcd.begin(16, 2);
  lcd.print("LoRa Echo Tester");

  lcd.setCursor(0, 1);//row2
  lcd.print("msg: ");

  LoRa.print("startup");

}

void loop()
{
  //LoRa.print("temp?");
  long starttime = millis();
  LoRa.listen();
  Serial.println("waiting for LoRa response");
  while (((millis() - starttime) < 5000) && (LoRa.available() < 1)) { //wait for LoRa response. only wait 5seconds max

    //Serial.println(LoRa.available());
    //Serial.println(millis()-starttime);

  }

  if (LoRa.available() > 0) {
    //lcd.clear();
    lcd.setCursor(0, 0);//row1
    lcd.print("LoRa Echo Tester");
    lcd.setCursor(0, 1);//row2
    lcd.print("msg: ");
    int inbytes = LoRa.available();
    for (int i = 0 ; i < inbytes ; i++ ) {
      data_in[i] = LoRa.read();
      if ((char)data_in[i] == 1) { // 1
        Serial.println("resetting error count...");
        lcd.setCursor(0, 0);//row1
        lcd.print("LoRa Echo Tester");
        lcd.setCursor(0, 1);//row2
        lcd.print("msg:    err:   ");
        error_counter = 0;
        data_prev = 0; // 0
      }
      if ((char)data_in[i] != (char)data_prev + 1) { //if the new data is not 1 more than previous data
        //lcd.clear();
        if (((char)data_in[i] > -128) && ((char)data_in[i] < 256)) { // range for char / uint8_t
          uint8_t temp = (uint8_t)data_in[i] - (uint8_t)data_prev - 1; // number o errors is new - old - 1


          Serial.print("temp:"); Serial.println(temp);
          Serial.print("data_in[1]"); Serial.println(data_in[1]);
          Serial.print("data_prev"); Serial.println(data_prev);

          error_counter = error_counter + temp;
          Serial.print("error_counter:"); Serial.println((uint8_t)error_counter);

          lcd.setCursor(0, 0);//row1
          lcd.print("LoRa Echo Tester");
          lcd.setCursor(8, 1);//row2
          lcd.print("err:");
          //lcd.setCursor(12, 1);//row2
          //lcd.print((error_counter+1-1));

          lcd.setCursor(12, 1);//hundreds
          int h0 = error_counter / 100;
          lcd.print((char) (h0 + 48));

          lcd.setCursor(13, 1); // tens
          int t0 = (error_counter - h0 * 100) / 10;
          lcd.print((char) (t0 + 48));

          lcd.setCursor(14, 1); //units
          int u0 = error_counter - h0 * 100 - t0 * 10;
          lcd.print((char) (u0 + 48));
          //
          Serial.print("number: "); Serial.print(h0); Serial.print(t0); Serial.println(u0);
          Serial.println("--------------------------------------");
        }
        else {
          Serial.println("garbage");
        }
      }
      else {
        Serial.println("no error");
      }
      data_prev = data_in[i];
      Serial.print("default: "); Serial.println(data_in[i]);
      Serial.print("uint8_t: "); Serial.println((uint8_t) data_in[i]);

      uint8_t number = (uint8_t) data_in[i];
      //lcd.setCursor(4+i, 1);
      //lcd.print((char)(data_in[i]+1-1));
      lcd.setCursor(4 + i, 1); // hundreds
      int h = number / 100;
      lcd.print((char) (h + 48));

      lcd.setCursor(5 + i, 1); // tens
      int t = (number - h * 100) / 10;
      lcd.print((char) (t + 48));

      lcd.setCursor(6 + i, 1); //units
      int u = number - h * 100 - t * 10;
      lcd.print((char) (u + 48));
      //
      Serial.print("number: "); Serial.print(h); Serial.print(t); Serial.println(u);
      Serial.println("--------------------------------------");
      //lcd.setCursor(10, 1);
      //lcd.print((char)223);
      //lcd.print("C");
      //Serial.println((char)(data_in[i]+1-1));
    }
    LoRa.flush();
    //send the message back:
    LoRa.print(data_in);
  }
  else {
    Serial.println("LoRa timeout");
    //LoRa.print("timeout");
    //LoRa.listen();
  }

}


