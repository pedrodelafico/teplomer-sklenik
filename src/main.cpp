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
int I_TEP;
int I_VLH;
uint8_t vypisLCD[] = { 0, 0, 0, 0 };             // pole hodnot pro jednotlive pozice displeje
// napis ERR2 pro indikaci poruchy cidla DHT11
const uint8_t ERR2[] = {
  SEG_A | SEG_D | SEG_E | SEG_F | SEG_G,      // pismeno "E"
  SEG_E | SEG_G,                              // pismeno "r"
  SEG_E | SEG_G,                              // pismeno "r"
  SEG_A | SEG_B | SEG_D | SEG_E | SEG_G       // cislice "2"
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
  displej.setBrightness(10);                  // nastaveni svitivosti LCD (8(min) - 15(max))
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
    Serial.print("porucha senzoru: ");            // pokud ne, vypis chybu na seriovy terminal
    Serial.println(typDHT);
    displej.setSegments(ERR2);                    // a vypise chybu "Err2" na LCD display
  }else{
    Serial.print("teplota: ");                    // vypis namerenych hodnot na seriovy terminal
    Serial.print(TEP);
    Serial.println(" C");
    Serial.print("vlhkost: ");
    Serial.print(VLH);
    Serial.println(" %");
    I_TEP=TEP;
    I_VLH=VLH;
    // zpracovani vysledku a zobrazeni teploty 
    vypisLCD[0]=120;                                 // pismeno "t"
    vypisLCD[1]=121+128;                             // pismeno "E" s dvojteckou (to je hodnota 128)
    vypisLCD[2]=displej.encodeDigit(I_TEP/10);
    vypisLCD[3]=displej.encodeDigit(I_TEP%10);
    displej.setSegments(vypisLCD);
    delay(delay_interval);
    // zpracovani vysledku a zobrazeni vlhkosti 
    vypisLCD[0]=80;                                  // pismeno "r"
    vypisLCD[1]=116+128;                             // pismeno "h" s dvojteckou (to je hodnota 128)
    vypisLCD[2]=displej.encodeDigit(I_VLH/10);
    vypisLCD[3]=displej.encodeDigit(I_VLH%10);
    displej.setSegments(vypisLCD);
    delay(delay_interval);
  }


  delay(1000);  //cekani na dalsi beh loop smycky
}