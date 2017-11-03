/*
Wii Balance Board                http://wiibrew.org/wiki/Wii_Balance_Board
Lectura y gestión a través de un display ST7735 de las células de carga de la Wii Balance Board
mostrándose el peso y el equilibrio.
Es necesario extraer el modulo bluetooth, retirar varias resistencias y conectar el los puntos
donde tenemos las senyales de DOUT1 y DOUT2 , SCLK, MUX, ON/OFF, pulsador y led (imagen ajunta para más detalle)

Celulas de carga de la Wii Balance Board

      3                 1

            frontal

      4    pulsador     2

 
// ADS1222 con CLK a 8Mhz/32->250khz
// Tenemos dos ADS1222 con dos canales, en cada cambio de canal tenemos que esperas 10us para 8Mhz y realizar 4 lecturas 
// para asegurarnos que la lectura es estable, después de los 24 pulsos para leer los bits realizaremos un pulso más para
// asegurar que el pin DRDY/DOUT pase a estado alto, para pasar al modo espera se debe de mantener a nivel alto SCLK cuando
// DRDY/DOUT pasa a nivel bajo

 Radioelf - 29 Octubre 2017
 http://radioelf.blogspot.com.es/
 
  Copyright (c) 2017 Radioelf.  All rights reserved.

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.
  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.

----------------------------------------------------------------------------------------
Funciones de las librerías TFT:
tft.setRotation(1)                             orientación pantalla, 1->horizontal, 0-> vertical
tft,drawBitmap (x,y,ancho,alto,array,escala)   escribe el array (imagen)
tft.setCursor (x0,y0,)                         posiciona el cursor en las posición X0, Y0
tft.print("texto")                             escribe el texto indicado
tft.print(X, DEC)                              escribe el valor de las variable x en decimal
tft.setTextSize(3)                             tamaño del texto a 3,  1 = 5x8, 2 = 10x16 3 = 15x24 
tft.setTextColor(color)                        color del texto
tft.setTextWrap(true)                          true-> sigue en la siguiente linea
tft.fillScreen(color)                          borra la pantalla con el color indicado
tft.fillRect (x0,y0, W6, H4, color)            rellena un recuadro de color del punto x0,y0 a con una anchura de 6 y una altura de 4
tft.drawPixel(x0,y0, color)                    pinta el pixel indicado
tft.drawFastHLine(x0,y0, L, color)             dibuja una línea horizontal de la longitud L
tft.drawFastVLine(x0,y0, H, color)             dibuja una línea vertical de la longitud H
tft.fillCircle(x, y, radio, color)             dibuja un circulo relleno
tft.drawCircle(x, y, radio, color)             dibuja el contorno de un circulo
tft.drawRect(X0, Y0, X1, Y1,color)             dibuja un rectángulo
int X = tft.width(),                           devuelve la anchura de la pantalla 
int X = tft.height(),                          devuelve la altura

128*160 
El punto (0,0) es la esquina superior izquierda y el punto (160,128) la esquina inferior derecha. (vertical)
con tft.setRotation(0); 
    Y->          
  X0-Y0         X0-Y128
  X *-------------*
  | |             |
  V //           //
    |             |
    *-------------*               
  X160-Y0       X160-Y128  

color:
ST7735_BLACK   0x0000
ST7735_BLUE    0x001F
ST7735_RED     0xF800
ST7735_GREEN   0x07E0
ST7735_CYAN    0x07FF
ST7735_MAGENTA 0xF81F
ST7735_YELLOW  0xFFE0
ST7735_WHITE   0xFFFF
ST7735_GRAY    0xCCCC
ST7735_ORANGE  0xFA60 

*******************************************************************************************/
#include "LowPower.h"                     // https://github.com/rocketscream/Low-Power
#include <Adafruit_GFX.h>                 // Core graphics library V 1.0.1 https://github.com/adafruit/Adafruit-GFX-Library
#include <Adafruit_ST7735.h>              // Hardware-specific library, driver IC:  ST7735
#include <SPI.h>

#include "wiiLOGO.h"
//#define debug

// definición de los pines para control de la pantalla (bus SPI)
//#define BLpin       vcc                 // retroiluminación 
#define dc            6                   // D/C -D6
#define rst           7                   // reset del display -D7
#define cs            8                   // Selección LCD -D8
// SPI por hardware
//#define miso        12                  // SDO, entrada de datos del master -D12
//#define mosi        11                  // SDI, salida de datos del master -D11
//#define sclk        13                  // SCK, pulso sincronización -D13

// Creamos una instancia del objeto Adafruit_ST7735 que llamamos  tft
Adafruit_ST7735 tft = Adafruit_ST7735(cs, dc, rst);  // hardware SPI

#define ST7735_GREY    0xCCCC              // color gris NO incluido en librería
#define ST7735_ORANGE  0xFA60              // color naranja NO incluido en librería

#define sclk    2                          // pin SCLK ADS1222 1 y 2, entrada de reloj de serie -D2
#define Dout1   3                          // pin Dout ADS1222 1, DRDY/DOUT drdy-> nivel bajo datos a leer listos, salida de datos por flaco positivo de slck, MSB primero -D3
#define Dout2   4                          // pin Dout ADS1222 2, DRDY/DOUT drdy-> nivel bajo datos a leer listos, salida de datos por flaco positivo de slck, MSB primero -D4
#define mux     5                          // pin MUX ADS1222 1 y 2, selección de la entrada analógica 0->AINP1+ AINN1- 1->AINP2+ AINN2 -D5

#define PULSADOR_wii 14                     // -A0
#define LED_wii      15                     // -A1
#define ON_wii       16                     // -A2

#ifdef debug
  #define Vcc 3.2
#endif

String digitos_act="000.00", digitos_ant="999.99";
int cursor_x =80, cursor_y =64, x_cursor, y_cursor, color =0x0000, color_ant =0xFFFF;
unsigned int espera_peso =0;
unsigned long sensor [4] ={0, 0, 0, 0}, sensor_cal[4] ={0, 0, 0, 0};
float peso[4] ={0.0, 0.0, 0.0, 0.0}, peso_total_act =0.0, peso_total_ant =150.0;
double factor[4] ={0.0, 0.0, 0.0, 0.0};

//*************************************************************
// Inicializa pantalla                           
//*************************************************************
void inicia_tft(){
  tft.initR(INITR_BLACKTAB);
  tft.setRotation(1);                                               // 0 - vertical, 1 - horizontal
  tft.setTextWrap(true);                                            // si el texto no cabe lo pasamos a la siguiente línea
  
  tft.fillScreen(ST7735_BLUE);                                      // rellenar de gris, color de fondo
  tft.drawBitmap(0,0,stationLOGO1,160,128,ST7735_YELLOW);           // icono en pantalla completa
  tft.setTextSize(1);                                               // tamanyo texto
  tft.setTextColor(ST7735_BLACK);                                   // color texto
  tft.setCursor(35,45);                                             // posición texto  
  tft.print("Iniciando...");
  tft.setTextSize(2);
  tft.setTextColor(ST7735_RED);                                    
  tft.setCursor(35,75);
  tft.print("!ESPERA!");
  delay(3000); 
  }
//*************************************************************
// Escritura de texto en display
//*************************************************************
void testdrawtext(char *text, uint16_t color){
   tft.setCursor(0, 0);
   tft.setTextColor(color);
   tft.setTextWrap(true);                                            
   tft.print(text);
}
//##########################################################################################
// Inicializa ADS1222
void iniADS1222(void){
byte x, n;
  digitalWrite(mux, HIGH);
  for (n =0; n <3; n++){                                          // realizamos 3 lecturas completas
    delay (10);
    for(x =0; x <24; x++){                                        // 24 pulsos-> 24 bits
      digitalWrite(sclk, HIGH);                                   // HIGH pulse, algo más de 33us
      delayMicroseconds(30);  
      digitalWrite(sclk, LOW);                                    // LOW pulse, algo más de 33us
      delayMicroseconds(30);                          
    }
  }
  delay (10);
  while (digitalRead(Dout1) + digitalRead(Dout2)){}               // esperamos datos preparados 
  for(x =0; x <26; x++){                                          // auto-calibrado 26 pulsos-> 24bits + 2  On calibración
    digitalWrite(sclk, HIGH); 
    delayMicroseconds(5);  
    digitalWrite(sclk, LOW); 
    delayMicroseconds(5);
   }
   #ifdef debug
    Serial.println("Auto-calibrado..");
   #endif
   while (digitalRead(Dout1) + digitalRead(Dout2)){}              // esperamos fin calibracion                           
   #ifdef debug
    Serial.println("Inicializando");
   #endif
} 

// ADS1222 leemos (se leen los 20 bits MSB, solo son efectivos los 20 bits de más peso)
void read_ads1222(bool canal){
byte x, n =0, z =2;
  if (canal){
    n =1;
    z =3;
  }
  digitalWrite(mux, canal);                                        // selecionamos el canal
  delayMicroseconds(8);
  do{
    delayMicroseconds(2);
  } while (digitalRead(Dout1) + digitalRead(Dout2));               // esperamos datos listos, Dout1 y Dout2 
  
  for(x =23; x >=4; x--){                                          // del bit 23 al 3
    digitalWrite(sclk, HIGH);
    digitalRead(Dout1) ? bitWrite(sensor[n], x, 1): bitWrite(sensor[n], x, 0);// algo más de 16us, leemos 0 y 1 
    digitalRead(Dout2) ? bitWrite(sensor[z], x, 1): bitWrite(sensor[z], x, 0);// algo más de 16us, leemos 2 y 3
    digitalWrite(sclk, LOW);
    delayMicroseconds(30);
  }
  for (x =0; x <5; x++){                                           // realizamos 5 pulsos, bits del 3 al 0 + pulso 25 -> forzamos Dout1 y Dout2 a 1
  digitalWrite(sclk, HIGH);
  delayMicroseconds(30);                                                 
  digitalWrite(sclk, LOW);
  delayMicroseconds(30);
  }
}
#ifdef debug
// Leemos y obtenemos la tension, convertir coplemento 2's 
// si adc > 2^23->8388608, es un valor negativo, Vref = VCC/2 
void calcular_mV(void){
double mv[4];
  if (bitRead(sensor[0], 23)) 
    mv[0] = (double) (sensor[0] - 16777216.0)/8388.607 * Vcc;
  else
    mv[0] = (double) sensor[0] / 8388.607 * Vcc;
    
  if (bitRead(sensor[1], 23)) 
    mv[1] = (double) (sensor[1] - 16777216.0)/8388.607 * Vcc;
  else
    mv[1] = (double) sensor[1] / 8388.607 * Vcc;
    
  if (bitRead(sensor[2], 23)) 
    mv[2] = (double) (sensor[2] - 16777216.0)/8388.607 * Vcc;
  else
    mv[2] = (double) sensor[2] / 8388.607 * Vcc;
    
  if (bitRead(sensor[3], 23)) 
    mv[3] = (double) (sensor[3]- 16777216.0)/8388.607 * Vcc;
  else
    mv[3] = (double) sensor[3]/ 8388.607 * Vcc;
    
  Serial.print ("Tension 1: ");Serial.print((String) mv[0]); Serial.println(" mv."); 
  Serial.print ("Tension 2: ");Serial.print((String) mv[1]); Serial.println(" mv.");
  Serial.print ("Tension 3: ");Serial.print((String) mv[2]); Serial.println(" mv.");
  Serial.print ("Tension 4: ");Serial.print((String) mv[3]); Serial.println(" mv.");
}
#endif
bool calibracion_cero(void){
double x;
  read_ads1222(false);                                // leemos canal 1 de ADS1222 1 y 2 
  delay(250);
  read_ads1222(true);                                 // leemos canal 2 de ADS1222 1 y 2
  if (bitRead(sensor[0], 23) || bitRead(sensor[1], 23) || bitRead(sensor[2], 23) || bitRead(sensor[3], 23)) return false; // valor negativo
  if (sensor[0] >3000000 || sensor[1] >650000 || sensor[2] >900000 || sensor[3] >2500000) return false; // máximo peso para calibración cero
  sensor_cal[0] = sensor[0];
  sensor_cal[1] = sensor[1];
  sensor_cal[2] = sensor[2];
  sensor_cal[3] = sensor[3];
  x = (double) 8388607 - sensor[0];
  factor[0] =(double) 500.0 / x, 9;
  x = 8388607 - sensor[1];
  factor[1] =(double) 500.0 / x, 9;
  x = 8388607 - sensor[2];
  factor[2] =(double) 500.0 / x, 9;
  x = 8388607 - sensor[3];
  factor[3] =(double) 500.0 / x, 9;
  return true;
}
// Realizamos calculos para obtener el peso en Kg.
void calcular (void){
long sensor_comp[4] ={0, 0, 0, 0};
  sensor_comp[0] =(long) sensor[0] - sensor_cal[0];
  if (sensor_comp[0] <= 0)
    peso[0] =0.0;
  else  {
    peso[0] =(float) sensor_comp[0] * factor[0];
  }
  sensor_comp[1] =(long) sensor[1] - sensor_cal[1];
  if (sensor_comp[1] <= 0)
    peso[1] =0.0;
  else  {
    peso[1] =(float) sensor_comp[1] * factor[1];
  }
  sensor_comp[2] =(long) sensor[2] - sensor_cal[2];
  if (sensor_comp[2] <= 0)
    peso[2] =0.0;
  else  
    peso[2] =(float) sensor_comp[2] * factor[2];
    
  sensor_comp[3] =(long) sensor[3] - sensor_cal[3];
  if (sensor_comp[3] <= 0)
    peso[3] =0.0;
  else  
    peso[3] =(float) sensor_comp[3] * factor[3];
  
  peso_total_act =(float) peso[0] + peso[1] + peso[2] + peso[3];
}
//  Gestionamos senyal OFF-ON placa Wii
void off_on(void){
  tft.fillScreen(ST7735_WHITE);
  tft.setCursor(20,60);
  tft.setTextColor(ST7735_RED);
  tft.setTextSize(3);
  tft.print("-ADIOS-");
  delay (2500);
  digitalWrite(ON_wii, LOW);                              // OFF, apagamos
  digitalWrite(LED_wii, LOW);
  tft.fillScreen(ST7735_BLACK);
  while(digitalRead(PULSADOR_wii)){                       // esperamos pulsador
    LowPower.idle(SLEEP_2S, ADC_OFF, TIMER2_OFF, TIMER1_OFF, TIMER0_OFF, 
                SPI_OFF, USART0_OFF, TWI_OFF);
  }  
  digitalWrite(LED_wii, HIGH);                            // ON, encendemos a traves de reiniciar
  espera_peso =0;
  peso_total_ant = peso_total_act+1;
  asm volatile ("  jmp 0");                               // reiniciamos, salto a la dirección 0 
}
// configuración inicial
void setup() {
byte ciclo =0;
  pinMode(ON_wii, OUTPUT);                           
  digitalWrite(ON_wii, HIGH);                             // encendemos controladore WII y TFT                                          
  pinMode(mux, OUTPUT);                           
  digitalWrite(mux, LOW);  
  pinMode(sclk, OUTPUT);                           
  digitalWrite(sclk, LOW); 
  pinMode(LED_wii, OUTPUT);
  digitalWrite(LED_wii, HIGH);  
  pinMode(PULSADOR_wii, INPUT_PULLUP);   
  pinMode(Dout1, INPUT);
  pinMode(Dout2, INPUT);          
  
  SPI.setClockDivider(SPI_CLOCK_DIV4);                     // SPI ON, 4 MHz (half speed), MSBFIRST, SPI_MODE0
  #ifdef debug
    Serial.begin(115200);
  #endif
  digitalWrite(LED_wii, HIGH);
  inicia_tft();
  delay(50);
  iniADS1222();
  delay(200);

  tft.fillRect (34, 44, 91, 61, ST7735_YELLOW);            // borramos textos anterior
  tft.setTextSize(1);                                      // tamanyo texto
  tft.setTextColor(ST7735_BLACK);                          // color texto
  tft.setCursor(35,45);                                    // posición texto  
  tft.setTextSize(2);
  delay(250);
  if (calibracion_cero()){
    tft.print("Peparado");
  }else{
    tft.setTextColor(ST7735_RED);
    tft.print("!ERROR!");
    while (ciclo <10){
      delay(250);
      digitalRead(LED_wii) ? digitalWrite(LED_wii, LOW): digitalWrite(LED_wii, HIGH);
      ciclo ++;
    }
    off_on(); 
  }
  while (ciclo <6){
    if (ciclo %2 ==0)
      tft.setTextColor(ST7735_RED);
    else
      tft.setTextColor(ST7735_GREEN);                                    
    tft.setCursor(35,75);
    tft.print("Puedes");
    tft.setCursor(50,95);
    tft.print("subir"); 
    delay(500); 
    digitalRead(LED_wii) ? digitalWrite(LED_wii, LOW): digitalWrite(LED_wii, HIGH);
    ciclo ++;
  }
  digitalWrite(LED_wii, LOW); 
}
// **-MAIN-**
void loop() {
byte media =0;
float media_peso = 0.0;
  if (espera_peso <51){
    while (media <4){                                   // ciclos de lectura 0, 1, 2, 3
      delay(100);
      read_ads1222(false);
      delay(25);
      read_ads1222(true);
      calcular();
      media_peso = media_peso + peso_total_act;
      media++;
    }
    peso_total_act =media_peso /4.0;                    // obtenemos la media de las 4 últimas lecturas
  }else{
    delay(50);
    read_ads1222(false);
    delay(50);
    read_ads1222(true);
    calcular();
  }
  if (peso_total_ant != peso_total_act && espera_peso <51){
     digitalWrite(LED_wii, HIGH);
    if (espera_peso ==0){
      tft.fillScreen(ST7735_BLACK);                     // borrar, rellenar de negro 
      tft.setCursor(18,0);
      tft.setTextColor(ST7735_GREEN);
      tft.setTextSize(3);
      tft.print("PESANDO"); 
      tft.setCursor(95,72);
      tft.setTextColor(ST7735_WHITE);
      tft.print(",");
      tft.setCursor(62,100);
      tft.setTextColor(ST7735_RED);
      tft.print("Kg");
    }
    if (peso_total_act >25) color =0xCCCC;
    else if (peso_total_act >50) color = 0xFFE0;
    else if (peso_total_act >75) color = 0xFA60;
    else if (peso_total_act >100) color = 0xF800;
    else color =0x07FF;
    
    digitos_act = String(peso_total_act);
    if (peso_total_act <10)
      digitos_act = "00" + digitos_act;
    else if (peso_total_act <100)
      digitos_act = "0" + digitos_act;
    tft.setTextSize(6);
    if (digitos_act.substring(0, 1) != digitos_ant.substring(0, 1) || color_ant !=color){ // centenas
        tft.setCursor(0,50);
        tft.setTextColor(ST7735_BLACK);
        tft.print(digitos_ant.substring(0, 1));
        tft.setCursor(0,50);
        tft.setTextColor(color);
        tft.print(digitos_act.substring(0, 1));
    }
    if (digitos_act.substring(1, 2) != digitos_ant.substring(1, 2) || color_ant !=color){ // decenas
        tft.setCursor(35,50);
        tft.setTextColor(ST7735_BLACK);
        tft.print(digitos_ant.substring(1, 2));
        tft.setCursor(35,50);
        tft.setTextColor(color);
        tft.print(digitos_act.substring(1, 2));
    }
    if (digitos_act.substring(2, 3) != digitos_ant.substring(2, 3) || color_ant !=color){ // unidades
        tft.setCursor(70,50);
        tft.setTextColor(ST7735_BLACK);
        tft.print(digitos_ant.substring(2, 3));
        tft.setCursor(70,50);
        tft.setTextColor(color);
        tft.print(digitos_act.substring(2, 3));
    }
    color_ant =color;
    // coma (se dibuja en el inicio del ciclo)
    tft.setTextSize(4);
    if (digitos_act.substring(4, 5) != digitos_ant.substring(4, 5)){   // décimas    
        tft.setCursor(110,55);
        tft.setTextColor(ST7735_BLACK);
        tft.print(digitos_ant.substring(4, 5));
        tft.setCursor(110,55);
        tft.setTextColor(ST7735_BLUE);
        tft.print(digitos_act.substring(4, 5));
    }
    if (digitos_act.substring(5, 6) != digitos_ant.substring(5, 6)){    // centésimas
        tft.setCursor(131,55);
        tft.setTextColor(ST7735_BLACK);
        tft.print(digitos_ant.substring(5, 6));
        tft.setCursor(131,55);
        tft.setTextColor(ST7735_BLUE);
        tft.print(digitos_act.substring(5, 6));
    }
    digitos_ant =digitos_act;
    peso_total_ant = peso_total_act;
    espera_peso ++;
  }
  if (espera_peso >50 && peso_total_act >25) {                           // iniciamos secuencia monitorización equilibrio
    if (espera_peso ==51){
      tft.fillScreen(ST7735_WHITE);
      tft.setTextSize(2);
      tft.setTextColor(ST7735_BLACK);
      espera_peso++;
    }
    tft.fillRect (125, 0, 40, 15, ST7735_WHITE);                          // borramos valor peso 0
    tft.setCursor(125, 0);                                                // esquina superior derecha
    tft.setTextColor(ST7735_BLACK);
    tft.print(byte (peso[0]));
    tft.fillRect (0, 115, 40, 15, ST7735_WHITE);                          // borramos valor peso 1
    tft.setCursor(0, 115);                                                // esquina inferior izquierda
    tft.setTextColor(ST7735_BLACK);
    tft.print(byte (peso[3]));
    tft.fillRect (0, 0, 40, 15, ST7735_WHITE);                            // borramos valor peso 2
    tft.setCursor(0,0);                                                   // esquina superior izquierda
    tft.setTextColor(ST7735_BLACK);
    tft.print(byte (peso[2]));
    tft.fillRect (125, 115, 40, 15, ST7735_WHITE);                        // borramos valor peso 3
    tft.setCursor(125, 115);                                              // esquina inferior derecha
    tft.setTextColor(ST7735_BLACK);
    tft.print(byte (peso[1]));
    if (peso[2] +peso[0] > peso[3] +peso[1]){
      if (peso[2] > peso[0])
        cursor_x =(int (80 - (peso[2] - peso[0])));
      else
        cursor_x =(int (80 + (peso[0] - peso[2])));
    }else{
      if (peso[3] > peso[1])
        cursor_x =(int (80 - (peso[3] - peso[1])));
    else
        cursor_x =(int (80 + (peso[1] - peso[3])));
    }
    if (peso[2] +peso[3] > peso[0] +peso[1]){
      if (peso[2] > peso[3])
        cursor_y =(int (64 - (peso[2] - peso[3])));
      else
        cursor_y =(int (64 + (peso[3] - peso[2])));
    }else{
      if (peso[0] > peso[1])
        cursor_y =(int (64 - (peso[0] - peso[1])));
      else
        cursor_y =(int (64 + (peso[1] - peso[0])));
    }
    if (cursor_x <10) cursor_x =10;
    if (cursor_x >124) cursor_x =124;
    if (cursor_y <10) cursor_y =10;
    if (cursor_y >114) cursor_y =114;  
    tft.fillCircle (x_cursor, y_cursor, 10, ST7735_WHITE);
    tft.fillCircle (cursor_x, cursor_y, 10, ST7735_GREEN);
    x_cursor =cursor_x;
    y_cursor =cursor_y;
    tft.drawCircle(80, 64, 15, ST7735_RED);
    tft.drawCircle(80, 64, 16, ST7735_RED);
    espera_peso++;
    if (espera_peso %2) digitalRead(LED_wii) ? digitalWrite(LED_wii, LOW): digitalWrite(LED_wii, HIGH);
  }else{
    if (espera_peso >50) off_on();      
  }  
  if (espera_peso ==250) off_on();                                  // apagamos y pasamos a bajo consumo

#ifdef debug
  if ((espera_peso %3) ==0){
    Serial.println("Lecturas:");
    Serial.print("Sensor 1: "); Serial.println(sensor[0]);
    Serial.print("Sensor 2: "); Serial.println(sensor[1]);
    Serial.print("Sensor 3: "); Serial.println(sensor[2]);
    Serial.print("Sensor 4: "); Serial.println(sensor[3]);
    calcular_mV();
  
    Serial.println("Peso:");
    Serial.print(String (peso[0])); Serial.println(" Kg.");
    Serial.print(String (peso[1])); Serial.println(" Kg.");
    Serial.print(String (peso[2])); Serial.println(" Kg.");
    Serial.print(String (peso[3])); Serial.println(" Kg.");
  }
 #endif
}

