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
#include <EEPROM.h>

//------------------------------- nastaveni programu ----------------------------------
#define delay_interval 5000                       // cas pro automaticke stridani udaju na LCD v ms
#define typDHT DHT11                              // definice pouzivaneho DHT senzoru (DHT11 nebo DHT22)
unsigned long INTERVAL = 86400000;                // doba v ms, kdy si ma pamatovat min/max teploty
//---------------------------- konec nastaveni programu -------------------------------


#define BUTTON 2                                  // tlacitko na pomocne desce
#define R_LED 3                                   // cervena LED na pomocne desce
#define G_LED1 4                                  // zelena LED1 na pomocne desce
#define G_LED2 5                                  // zelena LED2 na pomocne desce
#define DIO 6                                     // piny pro LCD
#define CLK 7
#define pinDHT 8                                  // datovy pin pro data z senzoru DHT11
#define L_LED 13                                  // vestavena LED

DHT mojeDHT(pinDHT, typDHT);                      // inicializace DHT senzoru
TM1637Display displej(CLK,DIO);                   // inicializace LCD
void zapis_LCD(int teplota_vlhkost, int value);   // deklarace funkce pro zapis na LCD
void extint_button();                             // deklarace funkce pro obsluhu tlacitka z preruseni
float TEP;                                        // promenne pro ulozeni teploty a vlhkosti
float VLH;
volatile int CURRENT_TEMP_INT;                    // promenne pro ulozeni aktualni teploty a vlhkosti (volatile kvuli preruseni)
int CURRENT_VLH_INT;
int LOW_TEMP_ALL_TIME;
int HIGH_TEMP_ALL_TIME;
int LOW_TEMP_INTERVAL;
int HIGH_TEMP_INTERVAL;
unsigned long TIME_STAMP_MIN_TEMP = 0;            // casovy okamzik, kdy zapise novou min teplotu
unsigned long TIME_STAMP_MAX_TEMP = 0;            // casovy okamzik, kdy zapise novou max teplotu
bool FIRST_MEASUREMENT;                           // promenna, ktera indikuje prvni pruchod mereni po startu MCU
volatile bool UNDERVOLT_ACCU;                     // promenna pro rezim detekce podpeti "Accu"
volatile byte CURRENT_STATE;                      // promenna, podle ktere se v programu pozna co se zrovna vypisuje (volatile kvuli preruseni)
const uint8_t ERR2[] = {                          // napis "ERR2" pro indikaci poruchy cidla DHT
  SEG_A | SEG_D | SEG_E | SEG_F | SEG_G,          // pismeno "E"
  SEG_E | SEG_G,                                  // pismeno "r"
  SEG_E | SEG_G,                                  // pismeno "r"
  SEG_A | SEG_B | SEG_D | SEG_E | SEG_G           // cislice "2"
};
const uint8_t celsius[] = {                       // Symbol "°C" pro zobrazeni teploty na LCD
  SEG_A | SEG_B | SEG_F | SEG_G,                  // znak "°"
  SEG_A | SEG_D | SEG_E | SEG_F                   // pismeno "C"
};
const uint8_t rh[] = {                            // Symbol "rh" pro zobrazeni vlhkosti na LCD
  SEG_E | SEG_G,                                  // pismeno "r"
  SEG_C | SEG_E | SEG_F | SEG_G                   // pismeno "h"
};
const uint8_t accu[] = {                          // napis "Accu" pro indikaci rezimu hlidani napeti
  SEG_A | SEG_B | SEG_C | SEG_E | SEG_F | SEG_G,  // pismeno "A"
  SEG_D | SEG_E | SEG_G,                          // pismeno "c"
  SEG_D | SEG_E | SEG_G,                          // pismeno "c"
  SEG_C | SEG_D | SEG_E                           // pismeno "u"
};
const uint8_t lipo[] = {                          // napis "LiPo" pro indikaci rezimu hlidani napeti
  SEG_D | SEG_E | SEG_F,                          // pismeno "L"
  SEG_E,                                          // pismeno "i"
  SEG_A | SEG_B | SEG_E | SEG_F| SEG_G,           // pismeno "P"
  SEG_C | SEG_D | SEG_E | SEG_G                   // pismeno "o"
};
// same carky na LCD pro potvrzeni smazani EEPROM "----"
const uint8_t erase[] = {SEG_G, SEG_G, SEG_G, SEG_G};  // znaky "----"
                             




void setup() {
  CURRENT_STATE = 99;                          // 99 = setup sequence
  // nastaveni I/O
  pinMode(BUTTON,INPUT);                       // tlacitko vybaveno externim pull-down => prima logika
  pinMode(R_LED,OUTPUT);                       // LED diody
  pinMode(G_LED1,OUTPUT);
  pinMode(G_LED2,OUTPUT);
  pinMode(L_LED,OUTPUT);
  pinMode(DIO,OUTPUT);                         // LCD display
  pinMode(CLK,OUTPUT);
  pinMode(pinDHT,INPUT);                       // teplomer DHT11
  // pocatecni stav I/O
  digitalWrite(R_LED,HIGH);                    // probliknuti vsech diod
  digitalWrite(G_LED1,HIGH);
  digitalWrite(G_LED2,HIGH);
  digitalWrite(L_LED,HIGH); 
  mojeDHT.begin();                             // zahajeni komunikace s DHT
  Serial.begin(9600);                          // otevreni serioveho kanalu pro monitorovani
  displej.setBrightness(8);                    // nastaveni svitivosti LCD (8(min) - 15(max))
  delay(500);
  digitalWrite(R_LED,LOW);                  
  digitalWrite(G_LED1,LOW);
  digitalWrite(G_LED2,LOW);
  digitalWrite(L_LED,LOW);
  LOW_TEMP_ALL_TIME = EEPROM.read(0);          // nacti z EEPROM nejnizsi doposud dosazenou teplotu
  HIGH_TEMP_ALL_TIME = EEPROM.read(1);         // nacti z EEPROM nejvyssi doposud dosazenou teplotu
  FIRST_MEASUREMENT = true;
  UNDERVOLT_ACCU = EEPROM.read(2);             // nacti z EEPROM rezim detekce podpeti
  attachInterrupt(0, extint_button, RISING);   // nastaveni externiho preruseni na pin D2 co bude volat funkci button
}

void loop() {
  // pomoci funkcí readTemperature a readHumidity nacteme
  // do promenných TEP a VLH informace o teplote a vlhkosti,
  // ctení trvá cca 250 ms
  digitalWrite(L_LED,HIGH);
  CURRENT_STATE = 0;                                      // 0 = measurement sequence
  TEP = mojeDHT.readTemperature();
  VLH = mojeDHT.readHumidity();
  digitalWrite(L_LED,LOW);

  if (isnan(TEP) || isnan(VLH)) {                         // kontrola, jestli jsou nactené hodnoty cisla pomocí funkce isnan
    Serial.print("porucha senzoru: DHT");                 // pokud ne, vypis chybu na seriovy terminal
    Serial.println(typDHT);
    displej.setSegments(ERR2);                            // a vypise chybu "Err2" na LCD display
    return;
  }
  Serial.print("teplota: ");                              // vypis namerenych hodnot na seriovy terminal
  Serial.print(TEP);
  Serial.println(" C");
  Serial.print("vlhkost: ");
  Serial.print(VLH);
  Serial.println(" %");
  CURRENT_TEMP_INT=TEP;                                   // prevod float hodnot na integer
  CURRENT_VLH_INT=VLH;
  zapis_LCD(0, CURRENT_TEMP_INT);
  CURRENT_STATE = 10;                                     // 10 = vypis aktualni teploty
  delay(delay_interval);
  zapis_LCD(1, CURRENT_VLH_INT);
  CURRENT_STATE = 11;                                     // 11 = vypis aktualni vlhkosti
  delay(delay_interval);
  CURRENT_STATE = 255;

// porovnani teploty s minimem a maximem za zvoleny casovy interval a jejich vypis na LCD
if(FIRST_MEASUREMENT == true){                            // jen pri prvnim pruchodu inicializuj a zapis aktualni teplotu
  LOW_TEMP_INTERVAL = CURRENT_TEMP_INT;                   // at je odkud se odpichnout
  HIGH_TEMP_INTERVAL = CURRENT_TEMP_INT;                  // dalsimi pruchody si uz system automaticky postupne nastavi nova
  FIRST_MEASUREMENT = false;                              // lokalni minima a maxima
}
if (CURRENT_TEMP_INT < LOW_TEMP_INTERVAL){                // pokud byla podkrocena minimalni teplota za casovy interval (24h)
  LOW_TEMP_INTERVAL = CURRENT_TEMP_INT;                   // prepis promennou v RAM
  TIME_STAMP_MIN_TEMP = millis();                         // zapis cas teto udalosti za ucelem ochranny teto hodnoty po zvoleny interval
}else{                                                    // pokud teplota podkrocena nebyla
  if ((unsigned long)(millis() - TIME_STAMP_MIN_TEMP) >= INTERVAL){    // uplynulo uz alespon 24h od posledniho prepisu ?
    LOW_TEMP_INTERVAL = CURRENT_TEMP_INT;                // prepis aktualni hodnotou, protoze puvodni hodnota uz byla stejne starsi jak 24h
    TIME_STAMP_MIN_TEMP = millis();                      // zapis cas teto udalosti za ucelem ochranny teto hodnoty po zvoleny interval
  }
}
if (CURRENT_TEMP_INT > HIGH_TEMP_INTERVAL){              // pokud byla prekrocena maximalni teplota za casovy interval (24h)
  HIGH_TEMP_INTERVAL = CURRENT_TEMP_INT;                 // prepis promennou v RAM
  TIME_STAMP_MAX_TEMP = millis();                        // zapis cas teto udalosti za ucelem ochranny teto hodnoty po zvoleny interval
}else{                                                   // pokud teplota prekrocena nebyla
  if ((unsigned long)(millis() - TIME_STAMP_MAX_TEMP) >= INTERVAL){    // uplynulo uz alespon 24h od posledniho prepisu ?
    HIGH_TEMP_INTERVAL = CURRENT_TEMP_INT;               // prepis aktualni hodnotou, protoze puvodni hodnota uz byla stejne starsi jak 24h
    TIME_STAMP_MAX_TEMP = millis();                      // zapis cas teto udalosti za ucelem ochranny teto hodnoty po zvoleny interval
  }
}
  digitalWrite(G_LED1,HIGH);
  zapis_LCD(0,LOW_TEMP_INTERVAL);
  CURRENT_STATE = 20;                                     // 20 = vypis nejnizsi teploty za INTERVAL
  delay(delay_interval);
  zapis_LCD(0,HIGH_TEMP_INTERVAL);
  CURRENT_STATE = 21;                                     // 21 = vypis nejvyssi teploty za INTERVAL
  delay(delay_interval);
  CURRENT_STATE = 255;
  digitalWrite(G_LED1,LOW);

// porovnani teploty s absolutnim minimem a maximem, ktere jsou ulozeny v EEPROM a jejich vypis na LCD
  if (CURRENT_TEMP_INT < LOW_TEMP_ALL_TIME){               // pokud je prekonana historicky nejnizsi teplota
    EEPROM.write(0, CURRENT_TEMP_INT);                     // zapis ji do pameti EEPROM na adresu 0
    LOW_TEMP_ALL_TIME = CURRENT_TEMP_INT;                  // zapis ji do RAM pro dalsi beh programu do restartu
  }
  if (CURRENT_TEMP_INT > HIGH_TEMP_ALL_TIME){              // pokud je prekonana historicky nejvyssi teplota
    EEPROM.write(1, CURRENT_TEMP_INT);                     // zapis ji do pameti EEPROM na adresu 1
    HIGH_TEMP_ALL_TIME = CURRENT_TEMP_INT;                 // zapis ji do RAM pro dalsi beh programu do restartu
  }
  digitalWrite(G_LED2,HIGH);
  zapis_LCD(0,LOW_TEMP_ALL_TIME);
  CURRENT_STATE = 30;                                     // 30 = vypis absolutne nizke teploty z EEPROM
  delay(delay_interval);
  zapis_LCD(0,HIGH_TEMP_ALL_TIME);
  CURRENT_STATE = 31;                                     // 31 = vypis absolutne vysoke teploty z EEPROM
  delay(delay_interval);
  CURRENT_STATE = 255;
  digitalWrite(G_LED2,LOW);

  
  delay(1000);  //cekani na dalsi beh loop smycky
}


void extint_button(){
  detachInterrupt(0);                           // deaktivuje dalsi preruseni
  if(digitalRead(BUTTON) == HIGH){              // je stale stisknuto tlacitko ?
    switch (CURRENT_STATE){
      case 11:                                  // zrovna se vypisovala aktualni vlhkost
        UNDERVOLT_ACCU = !UNDERVOLT_ACCU;       // zmen rezim detekce podpeti
        EEPROM.write(2, UNDERVOLT_ACCU);        // zapis novy rezim do EEPROM
        if(UNDERVOLT_ACCU == true){
          displej.setSegments(accu, 4, 0);      // vypis na LCD "Accu"
        }else{
          displej.setSegments(lipo, 4, 0);      // vypis na LCD "LiPo"
        }
      break;      
      case 20:                                  // zrovna se vypisovala nejnizsi teplota za INTERVAL
        displej.setSegments(erase, 4, 0);       // vypis na LCD "----"
        LOW_TEMP_INTERVAL = CURRENT_TEMP_INT;   // znovu inicializuj nejnizsi teplotu za INTERVAL podle aktualni
      break; 
      case 21:                                  // zrovna se vypisovala nejvyssi teplota za INTERVAL
        displej.setSegments(erase, 4, 0);       // vypis na LCD "----"
        HIGH_TEMP_INTERVAL = CURRENT_TEMP_INT;  // znovu inicializuj nejnizsi teplotu za INTERVAL podle aktualni
      break;      
      case 30:                                   // zrovna se vypisovala obrazovka cislo 2 - vypis min/max teplot za zvoleny interval
        displej.setSegments(erase, 4, 0);        // vypis na LCD "----"
        EEPROM.write(0, CURRENT_TEMP_INT);       // znovu inicializuj nejnizsi absolutni teplotu z EEPROM podle aktualni
      break;
      case 31:                                   // zrovna se vypisovala obrazovka cislo 2 - vypis min/max teplot za zvoleny interval
        displej.setSegments(erase, 4, 0);        // vypis na LCD "----"
        EEPROM.write(1, CURRENT_TEMP_INT);       // znovu inicializuj nejvyssi absolutni teplotu z EEPROM podle aktualni
      break;
    default:
      break;
    }
  }
  attachInterrupt(0, extint_button, RISING);    // znovu aktivuje preruseni
}


// funkce pro zapis hodnoty na LCD
// sama urpavi format vypisu v pripade zaporne teploty a nebo vlhkosti 100%
// vstupni parametr teplota_vlhkost = 0 -> zapisuje teplotu
//                                  = 1 -> zapisuje vlhkost
void zapis_LCD(int teplota_vlhkost, int value){
  if (teplota_vlhkost == 0){                        // bude zapisovat teplotu
    if (value < 0) {                           
      displej.showNumberDec(value, false, 3, 0);    // pokud je teplota mensi nez 0°C uprav vvypis na LCD pro znamenko "-"
      displej.setSegments(celsius, 1, 3);
    }else{
      displej.showNumberDec(value, false, 2, 0);    // pokud je teplota 0°C a vyssi ponecha standardni vystup na LCD
      displej.setSegments(celsius, 2, 2);      
    }
  }else if (teplota_vlhkost == 1){                  // bude zapisovat vlhkost
     if (value > 99) {                              
      displej.showNumberDec(value, false, 3, 0);   // pokud je vlhkost vetsi nez 99%, uprav vypis na LCD
      displej.setSegments(rh, 1, 3);
    }else{
      displej.showNumberDec(value, false, 2, 0);   // pokud je vlhkost 99% a nizsi ponecha standardni vystup na LCD
      displej.setSegments(rh, 2, 2);
    }  
  }else{
    displej.clear();                               // pokud prisel nesmyslny vstupni parametr smaze LCD
  }
  return;
}