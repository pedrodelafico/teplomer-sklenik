// teplomer do skleniku pro mereni a zaznam teploty a vlhkosti vzuchu
// kombinovany senzor DHT11
// 4 digit LCD display s radicem TM1637
// Adam Destensky 31.07.2022 | adam.destensky@seznam.cz

/* zpojeni
Arduino    Senzor/deska
D2    ->   tlacitko (pomocna deska) - exINT0
D3    ->   RED LED (pomocna deska)
D4    ->   GREEN LED1 (pomocna deska)
D5    ->   GREEN LED2 (pomocna deska)
D6    ->   DIO (LCD modul)
D7    ->   CLK (LCD modul)
D8    ->   DATA (DHT11)
*/

#include <Arduino.h>
#include <DHT.h>
#include <Adafruit_Sensor.h>
#include <TM1637Display.h>

//------------------------------- nastaveni programu ----------------------------------
#define delay_interval 5000                   // cas pro automaticke stridani udaju na LCD v ms
#define typDHT DHT11                          // definice pouzivaneho DHT senzoru (DHT11 nebo DHT22)
//---------------------------- konec nastaveni programu -------------------------------


#define BUTTON 2                              // tlacitko na pomocne desce
#define R_LED 3                               // cervena LED na pomocne desce
#define G_LED1 4                              // zelena LED1 na pomocne desce
#define G_LED2 5                              // zelena LED2 na pomocne desce
#define DIO 6                                 // piny pro LCD
#define CLK 7
#define pinDHT 8                              // datovy pin pro data z senzoru DHT11
#define L_LED 13                              // vestavena LED

DHT mojeDHT(pinDHT, typDHT);                  // inicializace DHT senzoru
TM1637Display displej(CLK,DIO);               // inicializace LCD
float TEP;                                    // promenne pro ulozeni teploty a vlhkosti
float VLH;
int TEMP_INT;
int VLH_INT;
// napis ERR2 pro indikaci poruchy cidla DHT11
const uint8_t ERR2[] = {
  SEG_A | SEG_D | SEG_E | SEG_F | SEG_G,      // pismeno "E"
  SEG_E | SEG_G,                              // pismeno "r"
  SEG_E | SEG_G,                              // pismeno "r"
  SEG_A | SEG_B | SEG_D | SEG_E | SEG_G       // cislice "2"
};
// Symbol "°C" pro zobrazeni teploty na LCD
const uint8_t celsius[] = {
  SEG_A | SEG_B | SEG_F | SEG_G,              // znak "°"
  SEG_A | SEG_D | SEG_E | SEG_F               // pismeno "C"
};
// Symbol "rh" pro zobrazeni vlhkosti na LCD
const uint8_t rh[] = {
  SEG_E | SEG_G,                              // pismeno "r"
  SEG_C | SEG_E | SEG_F | SEG_G               // pismeno "h"
};


void setup() {
  // nastaveni I/O
  pinMode(BUTTON,INPUT);                      // tlacitko vybaveno externim pull-down => prima logika
  pinMode(R_LED,OUTPUT);                      // LED diody
  pinMode(G_LED1,OUTPUT);
  pinMode(G_LED2,OUTPUT);
  pinMode(L_LED,OUTPUT);
  pinMode(DIO,OUTPUT);                        // LCD display
  pinMode(CLK,OUTPUT);
  pinMode(pinDHT,INPUT);                      // teplomer DHT11
  // pocatecni stav I/O
  digitalWrite(R_LED,HIGH);                  // probliknuti vsech diod
  digitalWrite(G_LED1,HIGH);
  digitalWrite(G_LED2,HIGH);
  digitalWrite(L_LED,HIGH); 
  mojeDHT.begin();                            // zahajeni komunikace s DHT
  Serial.begin(9600);                         // otevreni serioveho kanalu pro monitorovani
  displej.setBrightness(8);                  // nastaveni svitivosti LCD (8(min) - 15(max))
  delay(500);
  digitalWrite(R_LED,LOW);                  
  digitalWrite(G_LED1,LOW);
  digitalWrite(G_LED2,LOW);
  digitalWrite(L_LED,LOW); 
}

void loop() {
  // pomoci funkcí readTemperature a readHumidity nacteme
  // do promenných TEP a VLH informace o teplote a vlhkosti,
  // ctení trvá cca 250 ms
  digitalWrite(L_LED,HIGH);
  TEP = mojeDHT.readTemperature();
  VLH = mojeDHT.readHumidity();
  digitalWrite(L_LED,LOW);

  if (isnan(TEP) || isnan(VLH)) {                 // kontrola, jestli jsou nactené hodnoty cisla pomocí funkce isnan
    Serial.print("porucha senzoru: DHT");            // pokud ne, vypis chybu na seriovy terminal
    Serial.println(typDHT);
    displej.setSegments(ERR2);                    // a vypise chybu "Err2" na LCD display
  }else{
    Serial.print("teplota: ");                    // vypis namerenych hodnot na seriovy terminal
    Serial.print(TEP);
    Serial.println(" C");
    Serial.print("vlhkost: ");
    Serial.print(VLH);
    Serial.println(" %");
    TEMP_INT=TEP;                                 // prevod float hodnot na integer
    VLH_INT=VLH;
    if (TEMP_INT < 0) {
      displej.showNumberDec(TEMP_INT, false, 3, 0);  // pokud je teplota mensi nez 0°C uprav vvypis na LCD pro znamenko "-"
      displej.setSegments(celsius, 1, 3);
    }else{
      displej.showNumberDec(TEMP_INT, false, 2, 0);  // pokud je teplota 0°C a vyssi ponecha standardni vystup na LCD
      displej.setSegments(celsius, 2, 2);
    }
    delay(delay_interval);
    if (VLH_INT > 99) {                              
      displej.showNumberDec(VLH_INT, false, 3, 0);   // pokud je vlhkost vetsi nez 99%, uprav vypis na LCD
      displej.setSegments(rh, 1, 3);
    }else{
      displej.showNumberDec(VLH_INT, false, 2, 0);   // pokud je vlhkost 99% a nizsi ponecha standardni vystup na LCD
      displej.setSegments(rh, 2, 2);
    }
    delay(delay_interval);
  }


  delay(1000);  //cekani na dalsi beh loop smycky
}