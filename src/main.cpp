/**
 * Name:     Hanging Gardens
 * Autor:    Alfonso J. Fernández Álvarez
 * Date:     20/07/2024
 *
 * ESP32 WROOVER pinout:
 *          _______________
 *         |      USB      |
 * RST BTN-|EN          D23|-MOSI
 *         |VP          D22|-SCL
 *         |VN          TX0|
 *         |D34         RX0|
 *         |D35         D21|-SDA
 *   DHT22-|D32         D19|-MISO
 *         |D33         D18|-SCK
 *         |D25          D5|-CS
 *         |D26         TX2|-ENCODER CLK
 *         |D27         RX2|-ENCODER DT
 *         |D14          D4|-ENCODER SW
 *         |D12          D2|
 *         |D13         D15|
 *         |SD2          D0|
 *         |SD3         SD1|
 *         |CMD         SD0|
 *     GND-|GND         CLK|
 *      5V-|VIN         3V3|-3V3
 *         |_______________|
 *
*/
 
 
/**  
 * LIBRERIAS NECESARIAS PARA EL FUNCIONAMIENTO DEL CODIGO 
 */

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <WiFi.h>
#include "Adafruit_Sensor.h"        // https://github.com/adafruit/Adafruit_Sensor
#include "Adafruit_ADS1X15.h"      // https://github.com/adafruit/Adafruit_ADS1X15
#include "DHT.h"                  // https://github.com/adafruit/DHT-sensor-library
#include "RTClib.h"              // https://github.com/adafruit/RTClib.git
#include "LiquidCrystal_I2C.h"  // https://github.com/marcoschwartz/LiquidCrystal_I2C
#include "PCF8574.h"           // https://github.com/xreef/PCF8574_library
#include "WiFiManager.h"      // https://github.com/tzapu/WiFiManager
#include "ArduinoJson.h"     // https://github.com/bblanchon/ArduinoJson

#define BLYNK_TEMPLATE_ID "TMPL5RalpQ0jN"
#define BLYNK_TEMPLATE_NAME "Hanguing Garden"
#define BLYNK_AUTH_TOKEN "yxV5iZ4gta92R9rDKPqD0EsGJRG6I3Mk"
#include "BlynkSimpleEsp32.h"          // https://github.com/blynkkk/blynk-library

/**
* MACROS, CONSTANTES, ENUMERADORES, ESTRUCTURAS Y VARIABLES GLOBALES
*/
// Valor de tiempo (en microsegundos)
uint64_t tNow=0,tPrevLog=0,tPrevLCDUpdate=0,tPrevBtn=0, tPrevWatering=0, tPrevNutADose=0, tPrevNutBDose=0;
uint64_t tPrevSample=0, tPrevSensorUpdate=0, tPrevBlynkUpdate=0;
#define PIN_SPI_CS 5 // The ESP32 pin GPIO5
#define DS3231_I2C_ADDRESS 0x68 // Dirección del módulo RTC
#define COUNT(x) sizeof(x)/sizeof(*x)                   // Macro para contar el numero de elementos de un array 
const int channelPinA = 17;                             //CLK Pin in encoder module
const int channelPinB = 16;                             //DT Pin in encoder module
const int buttonPin = 4;                                // SW Pin in encoder module
const byte rowsLCD    = 4;                              // Filas del LCD
const byte columnsLCD = 20;                             // Columnas del LCD

const int tThreshold = 5000; // 5ms
const int tDebounce = 50000; // 50 ms
const int tLCDRefresh = 5000000; // 5s
const int min_2_us =  60000000; // pasa de minutos a microsegundos
const byte Nut_flow_rate = 10; // 10 ml/min
bool sys_error1=false; //Network error
bool sys_error2=false; //SD card error
bool sys_error3=false; //I2C error
bool EncoderTurn = false;
bool EncoderCW = true;
bool buttonState = LOW;                // the current reading from the input pin
bool lastButtonState = LOW;           // the previous reading from the input pin
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
volatile bool ButtonInterruption = false;
bool WaterPump_is_running = false;
bool NutAPump_is_running =  false;
bool NutBPump_is_running =  false;
bool NutAPump_is_waiting =  false;
bool NutBPump_is_waiting =  false;
bool GrowLight_is_active =  false;
bool NutA_LVL=0;
bool NutB_LVL=0;
bool LowNutSol=0;
byte LastScreen=0;
byte LightLevel=0;
byte water_LVL=0;
byte iARROW     = 0;                              // ID icono flecha
byte bARROW[]   = {                               // Bits icono flecha
  0x00,
  0x04,
  0x06,
  0x1F,
  0x06,
  0x04,
  0x00,
  0x00
};
 
enum Screen{ Menu1, Flag,Flag1, Number }; // Enumerador con los distintos tipos de submenus disponibles
 
const char *txmenu[] = {        // Los textos del menu principal, la longitud maxima = columnsLCD-1, rellenar caracteres sobrantes con espacios.
    "Change Date/Hour   ",
    "min TDS            ",
    "max TDS            ",
    "Watering period    ",
    "Watering duty cycle",
    "Nutrient A dose    ",
    "Nutrient B dose    ",
    "Dosing period      ",
    "DataLog period     ",
    "LCD Backlight      ",
    "Photoperiod        ",
    "Change dawn time   ",
    "Devices connected  ",
    "Reset day counter  ",
    "Save and exit      ",
    "Exit               "
};
const byte imenu = COUNT(txmenu);   // Numero de items/opciones del menu principal
 
const char *txRTCmenu[] = {       // Los textos del menu RTC, la longitud maxima = columnsLCD-1, rellenar caracteres sobrantes con espacios.
    "Change Year        ",
    "Change Month       ",
    "Change Day         ",
    "Change Hour        ",
    "Change Minutes     ",
    "Exit               "
};
const byte iRTCmenu = COUNT(txRTCmenu);                       // Numero de items/opciones del menu RTC
 
enum menu1{ OFF, Auto_OFF ,ON };                         // Enumerador de las opciones disponibles del submenu 2 (tienen que seguir el mismo orden que los textos)
const char *txmenu1[] = {                              // Textos del submenu 1, longitud maxima = columnsLCD-2, rellenar caracteres sobrantes con espacios
    "   Always OFF     ",
    "  Auto Turn OFF   ",
    "   Always ON      ",
};
const byte imenu1 = COUNT(txmenu1);                       // Numero de items/opciones del menu principal

/* ESTRUCTURAS MEMORIA */
struct MYCONFIG{    // Estructura con la configuración que se almacenaran en la memoria SD
    int initialized;
    int minTDS;                // The minimum TDS allowed in the nutrient solution
    int maxTDS;               // The maximum TDS allowed in the nutrient solution
    int WateringPeriod;      // The time period of the water pump action
    int WateringDutyCicle;  // Ratio on which the water pump is ON in a period
    int NutADose;          // Volume of nutrient A in each dose 
    int NutBDose;         // Volume of nutrient B in each dose
    int DosingPeriod;    // The time period that the peristatic pumps waits after dosing
    int DatalogPeriod;  // The time period for each datalog in the SD
    int Brightness;    // Config for the backlight of the LCD
    int Photoperiod;  // The time that the crop need to be in a luminous environment
    int DawnTime;    // Hour where the photoperiod of the day starts (and the day in daycounter)
    int Slave2;     // Config for the availability of the second slave device
};
struct MYVALUES{   // Estructura con las variables que se almacenaran en la memoria SD
  int Daycounter; // Days transcurred in the current 
  int LightTime;
  int CurrentDay;
};
union MEMORY{       // Estructura UNION para facilitar la lectura y escritura de la memoria
    MYCONFIG d;
    MYVALUES v;
}
memory;
const char* ConfigFilename = "/config.txt"; //<- SD library uses 8.3 filenames


// Sensor variables

float temperature, humidity;
float TDS, pH, WaterTemp, LDR;
#define SCOUNT 20 // sum of sample point
#define BCOEFF  3380 //B-constante: 3380K -/+ 1% 
#define SERIESRESISTOR 10000 // Resistencia en serie con el termistor
#define THERMISTORNOMINAL 10000 // Resistencia nominal del sensor NTC
#define TEMPERATURENOMINAL 25.0
int analogBufferTDS[SCOUNT]; //Buffer para las medidas de TDS
int analogBufferpH[SCOUNT]; // Buffer para las medidas de pH
int analogBufferNTC[SCOUNT]; // Buffer para las medidas del termistor NTC sumergible
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0,copyIndex = 0;

// Generic variables
int i=0;


/**  
 * OBJETOS DE LAS LIBRERÍAS 
 */
LiquidCrystal_I2C lcd(0x27, 20, 4); // Creates I2C LCD Object With (Address=0x27, Cols=20, Rows=4) 
PCF8574 I2C_PIN1(0x20);   // PCF8574 del Slave 1
PCF8574 I2C_PIN2(0x24); // PCF8574 del Slave 2
DHT my_sensor(33, DHT22); 
RTC_DS3231 rtc;
WiFiManager wm;
File DataFile;
Adafruit_ADS1115 ADC1;
Adafruit_ADS1115 ADC2;
/**  
 *  Rutina de interrupción del botón incorporado en el encoder
 */
void IRAM_ATTR ButtonPress() {
  tNow=esp_timer_get_time(); //Resolución 1us limite: +200 años vs millis() cuyo limite es 50 días
  if ((tNow - lastDebounceTime) > tDebounce) {
    Serial.println("Button pressed!");
    ButtonInterruption = true;
    // reset the debouncing timer
    lastDebounceTime = tNow;
  }
}
/**  
 *  Rutina de interrupción del pin A (CLK) del encoder sensible a ambos flancos (precisión doble)
 */
void IRAM_ATTR doEncode() // Double precision with one interrupt 
{
  Serial.print("Encoder");
  EncoderTurn = true;
  tNow=esp_timer_get_time(); //Resolución 1us limite: +200 años vs millis() cuyo limite es 50 días
  if (tNow > tPrevBtn + tThreshold)
  {
    if (digitalRead(channelPinA) != digitalRead(channelPinB)) // counterclockwise turn
    {
      Serial.println("CW turn!");
      EncoderCW = true;
    }
    else // clockwise turn
    {
      Serial.println("CCW turn!");
      EncoderCW = false;
    }
    tPrevBtn = tNow;
  }
}

/**  
 *  Añade un 0 a los valores menores a la decena para tener un formato consistente de dos cifras
 */
String DateFormat(String x){
    if(x=="0"||x=="1"||x=="2"||x=="3"||x=="4"||x=="5"
    ||x=="6"||x=="7"||x=="8"||x=="9")x="0"+ x;
    return x;
}
/**  
 *  Convierte la fecha y hora del RTC en una cadena de texto
 */
String GetDate() {
  String anno, dia, mes, Fecha;
  DateTime now = rtc.now();
  anno = now.year();
  mes = now.month();
  mes=DateFormat(mes);
  dia = now.day();
  dia=DateFormat(dia);
  Fecha = dia + "/" + mes + "/" + anno;
  return Fecha;
}
String GetHour(){
  String horas, minutos,segundos,Hora;
  DateTime now = rtc.now();
  horas = now.hour();
  horas=DateFormat(horas);
  minutos = now.minute();
  minutos= DateFormat(minutos);
  segundos = now.second();
  segundos = DateFormat(segundos);
  Hora = horas + ":" + minutos+ ":" + segundos;
  return Hora;
}


void DATALOG(){
  if((tNow-tPrevLog) >= (memory.d.DatalogPeriod*min_2_us) ){ 
   DataFile = SD.open("/DATALOG.csv", FILE_WRITE);
    if (DataFile) {
      if(DataFile.size()==0)DataFile.println("Date,Hour,Day,AmbTemp,AmbRH,WaterTemp,TDS,pH,LightLevel,WaterPump,NutAPump,NutBPump,GrowLight");
      DataFile.seek(DataFile.size());
      DataFile.print(GetDate());
      DataFile.print(",");
      DataFile.print(GetHour());
      DataFile.print(",");
      DataFile.print(memory.v.Daycounter);
      DataFile.print(",");     
      DataFile.print(temperature);
      DataFile.print(",");
      DataFile.print(humidity);
      DataFile.print(",");
      DataFile.print(WaterTemp);
      DataFile.print(",");
      DataFile.print(TDS);
      DataFile.print(",");
      DataFile.print(pH); 
      DataFile.print(",");
      DataFile.print(LightLevel);
      DataFile.print(",");
      DataFile.print(WaterPump_is_running);
      DataFile.print(",");
      DataFile.print(NutAPump_is_running);
      DataFile.print(",");
      DataFile.print(NutBPump_is_running);
      DataFile.print(",");
      DataFile.println(GrowLight_is_active);
      DataFile.close();
      tPrevLog=tNow;
    }
    else{
      Serial.println("error opening DATALOG.csv");
      sys_error2=true;
    }
  }
}


/** 
* Guarda en la memoria SD la configuración que hay cargada en la FLASH dentro de union MEMORY()
*/
void saveConfig(){
  // Delete existing file, otherwise the configuration is appended to the file
  if(SD.exists(ConfigFilename))SD.remove(ConfigFilename);
    // Open file for writing
  File file = SD.open(ConfigFilename, FILE_WRITE);
  if (!file) {
    Serial.println(F("Failed to create file"));
    sys_error2=true;
    return;
  }
  else sys_error2=false;
  // Allocate a temporary JsonDocument
  JsonDocument doc;
  // Set the values in the document
  doc["minTDS"]=memory.d.minTDS;
  doc["maxTDS"]=memory.d.maxTDS;
  doc["WateringPeriod"]=memory.d.WateringPeriod;
  doc["WateringDutyCycle"]=memory.d.WateringDutyCicle;
  doc["NutADose"]=memory.d.NutADose;
  doc["NutBDose"]=memory.d.NutBDose;
  doc["DosingPeriod"]=memory.d.DosingPeriod;
  doc["DatalogPeriod"]=memory.d.DatalogPeriod;
  doc["Brightness"]=memory.d.Brightness;
  doc["Photoperiod"]=memory.d.Photoperiod;
  doc["DawnTime"]=memory.d.DawnTime;
  doc["Slave2"]=memory.d.Slave2;
  

    // Serialize JSON to file
  if (serializeJson(doc, file) == 0) {
    Serial.println(F("Failed to write to file"));
    sys_error2=true;
  }
  else sys_error2=false;
  // Close the file
  file.close();

}

/** 
* Carga en la memoria FLASH la onfiguración que hay almacenada en el archivo de config.txt en la SD
*/
void loadConfig(){
    File file= SD.open(ConfigFilename);
    // Allocate a temporary JsonDocument
    JsonDocument doc;
    // Deserialize the JSON document
    DeserializationError error = deserializeJson(doc, file);
    if (error){
      Serial.println(F("Failed to read file, using default configuration"));
      memory.d.initialized=1;
      memory.d.minTDS=750;
      memory.d.maxTDS=1500;
      memory.d.WateringPeriod=10;
      memory.d.WateringDutyCicle=10;
      memory.d.NutADose=30;
      memory.d.NutBDose=30;
      memory.d.DosingPeriod=30;
      memory.d.DatalogPeriod=15;
      memory.d.Brightness=3;
      memory.d.Photoperiod=12;
      memory.d.DawnTime=8;
      memory.d.Slave2=0;
    }
    memory.d.initialized=1;
    memory.d.minTDS=doc["minTDS"];
    memory.d.maxTDS=doc["maxTDS"];
    memory.d.WateringPeriod=doc["WateringPeriod"];
    memory.d.WateringDutyCicle=doc["WateringDutyCycle"];
    memory.d.NutADose=doc["NutADose"];
    memory.d.NutBDose=doc["NutBDose"];
    memory.d.DosingPeriod=doc["DosingPeriod"];
    memory.d.DatalogPeriod=doc["DatalogPeriod"];
    memory.d.Brightness=doc["Brightness"];
    memory.d.Photoperiod=doc["Photoperiod"];
    memory.d.DawnTime=doc["DawnTime"];
    memory.d.Slave2=doc["Slave2"];

}

/** 
* Guarda en la memoria SD las variables que hay cargadas en la FLASH dentro de union MEMORY()
*/
void saveValues(){
    // Delete existing file, otherwise the configuration is appended to the file
  if(SD.exists("/values.txt"))SD.remove("/values.txt");
    // Open file for writing
  File file = SD.open("/values.txt", FILE_WRITE);
  if (!file) {
    Serial.println(F("Failed to create file"));
    sys_error2=true;
    return;
  }
  else sys_error2=false;
  // Allocate a temporary JsonDocument
  JsonDocument doc;
  doc["Daycounter"]=memory.v.Daycounter;
  doc["LightTime"]=memory.v.LightTime;
  doc["CurrentDay"]=memory.v.CurrentDay;
  if (serializeJson(doc, file) == 0) {
    Serial.println(F("Failed to write to file"));
    sys_error2=true;
  }
  else sys_error2=false;
  // Close the file
  file.close();
}
/** 
* Carga en la memoria FLASH las variables que hay almacenadas en el archivo de values.txt en la SD
*/
void loadValues(){
    File file= SD.open("/values.txt");
    // Allocate a temporary JsonDocument
    JsonDocument doc;
    // Deserialize the JSON document
    DeserializationError error = deserializeJson(doc, file);
    if (error){
      Serial.println(F("Failed to read file, using default values"));
      memory.v.CurrentDay=0;
      memory.v.Daycounter=0;
      memory.v.LightTime=0;
    }
    memory.v.CurrentDay=doc["CurrentDay"];
    memory.v.Daycounter=doc["Daycounter"];
    memory.v.LightTime=doc["LightTime"];
}
/**
 * MUESTRA EL SUBMENU EN EL LCD.
 *
 * @param menuID    ID del menu principal para usarlo como titulo del submenu
 * @param screen    Segun el tipo, se representara el submenu de una forma u otra.
 * @param value     Puntero a la variable que almacena el dato, y que se modificara.
 * @param minValue  Valor minimo que puede tener la variable.
 * @param maxValue  Valor maximo que puede tener la variable.
 * @param stepValue Resolución del cambio del valor de la variable
 */
void openSubMenu( byte menuID, Screen screen, int *value, int minValue, int maxValue, int stepValue ){
    boolean exitSubMenu = false;
    boolean forcePrint  = true;
 
    lcd.clear();
 
    while( !exitSubMenu )
    {
        if( ButtonInterruption)
        {
          ButtonInterruption=false;
          exitSubMenu = true;
        }
        else if( EncoderTurn && !EncoderCW)
        {
          (*value) = (*value)-stepValue;
          if(*value<minValue) *value=maxValue;
        }
        else if( EncoderTurn && EncoderCW)
        {
            (*value) = (*value)+stepValue;
            if(*value>maxValue) *value= minValue;
        }
 
 
        if( !exitSubMenu && (forcePrint|| EncoderTurn)  )
        {
            forcePrint = false;
            EncoderTurn=false;
            //ButtonInterruption=false;
 
            lcd.setCursor(0,0);
            lcd.print(txmenu[menuID]);
 
            lcd.setCursor(0,1);
            lcd.print("<");
            lcd.setCursor(columnsLCD-1,1);
            lcd.print(">");
 
            if( screen == Screen::Menu1 )
            {
                lcd.setCursor(1,1);
                lcd.print(txmenu1[*value]);
            }
            else if( screen == Screen::Flag )
            {
                lcd.setCursor(1, 1);
                lcd.print(*value == 0 ? "    One Device    " : "    Two Devices   ");
            }
            else if( screen == Screen::Flag1 )
            {
                lcd.setCursor(1, 1);
                lcd.print(*value == 1 ? "        YES       " : "         NO       ");
            }
            else if( screen == Screen::Number )
            {
                lcd.setCursor(columnsLCD/2-1, 1);
                lcd.print(*value);
                lcd.print(" ");
                lcd.setCursor(columnsLCD-3,2);
                if(menuID==1||menuID==2) lcd.print("ppm");
                if(menuID==3||menuID==7||menuID==8) lcd.print("min");
                if(menuID==4) lcd.print("%  ");
                if(menuID==5||menuID==6) lcd.print("ml ");
                if(menuID==10) lcd.print("h  ");

            }
        }
 
    }
 
    lcd.clear();
}

void openRTCSubMenu(int id,int minValue, int maxValue){
    boolean exitSubMenu = false;
    boolean forcePrint  = true;
    int aux=0;
    lcd.clear();
 
    while( !exitSubMenu )
    {
        if( ButtonInterruption)
        {
          ButtonInterruption=false;
          exitSubMenu = true;
        }
        else if( EncoderTurn && !EncoderCW)
        {
          aux--;
          if(aux<minValue) aux=maxValue;
        }
        else if( EncoderTurn && EncoderCW)
        {
            aux++;
            if(aux>maxValue) aux= minValue;
        }
 
 
        if( !exitSubMenu && (forcePrint|| EncoderTurn)  )
        {
            forcePrint = false;
            EncoderTurn=false;
            //ButtonInterruption=false;
 
            lcd.setCursor(0,0);
            lcd.print(txRTCmenu[id]);
 
            lcd.setCursor(0,1);
            lcd.print("<");
            lcd.setCursor(columnsLCD-1,1);
            lcd.print(">");
            lcd.setCursor(columnsLCD/2-1, 1);
            lcd.print(aux);
            lcd.print(" ");
            
        }
      
    }
    DateTime now = rtc.now();
    int y = (id==0)? (aux-2000) : now.year();
    int mon= (id==1)? aux : now.month();
    int d= (id==2)? aux : now.day(); 
    int h= (id==3)? aux : now.hour();
    int min= (id==4)? aux : now.minute();
   //Fijar a fecha y hora específica. 
    rtc.adjust(DateTime(y,mon,d,h,min,0));

}

void openRTCMenu(){

    byte idxRTCMenu       = 0;
    boolean exitRTCMenu   = false;
    boolean forcePrint = true;
    EncoderTurn=false;
    ButtonInterruption=false;
    lcd.clear();
 
    while( !exitRTCMenu )
    {
 
        if(EncoderTurn && !EncoderCW)
        {
          //EncoderTurn=false;
          if(idxRTCMenu>=1)idxRTCMenu--;
          else idxRTCMenu=iRTCmenu;
        }
        else if(EncoderTurn && EncoderCW)
        {
          //EncoderTurn=false;
          if(idxRTCMenu<iRTCmenu)idxRTCMenu++;
          else idxRTCMenu=0;
        }
        else if(ButtonInterruption)
        {
        ButtonInterruption=false;
        switch( idxRTCMenu )
            {
                case 0: openRTCSubMenu(idxRTCMenu,2024,2099); break;  // Año
                case 1: openRTCSubMenu(idxRTCMenu,1,12); break;  // Mes
                case 2: openRTCSubMenu(idxRTCMenu,1,31); break;  // Día
                case 3: openRTCSubMenu(idxRTCMenu,0,23); break; // Hora
                case 4: openRTCSubMenu(idxRTCMenu,0,59); break;  // Minutos
                case 5:  exitRTCMenu = true;  break;   //Salir y guardar
            }
            forcePrint = true;
        }
 
 
        if( !exitRTCMenu && (forcePrint || EncoderTurn))
        {
            forcePrint = false;
            EncoderTurn=false;
 
            static const byte endFor1 = (iRTCmenu+rowsLCD-1)/rowsLCD;
            int graphMenu     = 0;
 
            for( int i=1 ; i<=endFor1 ; i++ )
            {
                if( idxRTCMenu < i*rowsLCD )
                {
                    graphMenu = (i-1) * rowsLCD;
                    break;
                }
            }
 
            byte endFor2 = graphMenu+rowsLCD;
 
            for( int i=graphMenu, j=0; i< endFor2 ; i++, j++ )
            {
                lcd.setCursor(1, j);
                lcd.print( (i<iRTCmenu) ? txRTCmenu[i] : "                    " );
            }
 
            for( int i=0 ; i<rowsLCD ; i++ )
            {
                lcd.setCursor(0, i);
                lcd.print(" ");
            }
            lcd.setCursor(0, idxRTCMenu % rowsLCD );
            lcd.write(iARROW);
        }
    }
 
    lcd.clear();

}
/**
 *  MUESTRA EL MENU PRINCIPAL EN EL LCD.
 */
void openMenu()
{
    byte idxMenu       = 0;
    boolean exitMenu   = false;
    boolean forcePrint = true;
    EncoderTurn=false;
    ButtonInterruption=false;
    lcd.clear();
 
    while( !exitMenu )
    {
 
        if(EncoderTurn && !EncoderCW)
        {
          //EncoderTurn=false;
          if(idxMenu>=1)idxMenu--;
          else idxMenu=imenu;
        }
        else if(EncoderTurn && EncoderCW)
        {
          //EncoderTurn=false;
          if(idxMenu<imenu)idxMenu++;
          else idxMenu=0;
        }
        else if(ButtonInterruption)
        {
          ButtonInterruption=false;

          switch( idxMenu )
            {
                case 0: openRTCMenu();   break; // Menú Hora
                case 1: openSubMenu( idxMenu, Screen::Number, &memory.d.minTDS,    550, 2500, 50         ); break;
                case 2: openSubMenu( idxMenu, Screen::Number, &memory.d.maxTDS,    850, 3500, 50       ); break;
                case 3: openSubMenu( idxMenu, Screen::Number,   &memory.d.WateringPeriod, 0, 120, 5      ); break;
                case 4: openSubMenu( idxMenu, Screen::Number,   &memory.d.WateringDutyCicle, 5, 100, 5  ); break;
                case 5: openSubMenu( idxMenu, Screen::Number,  &memory.d.NutADose,       0, 100, 5      ); break;
                case 6: openSubMenu( idxMenu, Screen::Number, &memory.d.NutBDose,        0, 100, 5      ); break;
                case 7: openSubMenu( idxMenu, Screen::Number, &memory.d.DosingPeriod,    5, 120, 5      ); break;
                case 8: openSubMenu( idxMenu, Screen::Number, &memory.d.DatalogPeriod,   1, 60, 1       ); break;
                case 9: openSubMenu( idxMenu, Screen::Menu1, &memory.d.Brightness,       0, 2, 1        ); break;
                case 10: openSubMenu( idxMenu, Screen::Number, &memory.d.Photoperiod,    0, 24, 1       ); break;
                case 11: openSubMenu( idxMenu, Screen::Number, &memory.d.DawnTime,       1, 24, 1       ); break;
                case 12: openSubMenu( idxMenu, Screen::Flag, &memory.d.Slave2,           0, 1, 1        ); break;
                case 13: openSubMenu(idxMenu, Screen::Flag1, &memory.v.Daycounter,    1, memory.v.Daycounter,(memory.v.Daycounter-1) ); break;
                case 14:  exitMenu = true; saveConfig(); saveValues(); break;   //Salir y guardar
                case 15:  exitMenu = true; loadConfig(); loadValues(); break;                //Salir y cancelar cambios
            }
            forcePrint = true;
        }
 
 
        if( !exitMenu && (forcePrint || EncoderTurn))
        {
            forcePrint = false;
            EncoderTurn=false;
 
            static const byte endFor1 = (imenu+rowsLCD-1)/rowsLCD;
            int graphMenu     = 0;
 
            for( int i=1 ; i<=endFor1 ; i++ )
            {
                if( idxMenu < i*rowsLCD )
                {
                    graphMenu = (i-1) * rowsLCD;
                    break;
                }
            }
 
            byte endFor2 = graphMenu+rowsLCD;
 
            for( int i=graphMenu, j=0; i< endFor2 ; i++, j++ )
            {
                lcd.setCursor(1, j);
                lcd.print( (i<imenu) ? txmenu[i] : "                    " );
            }
 
            for( int i=0 ; i<rowsLCD ; i++ )
            {
                lcd.setCursor(0, i);
                lcd.print(" ");
            }
            lcd.setCursor(0, idxMenu % rowsLCD );
            lcd.write(iARROW);
        }
    }
 
    lcd.clear();
}

/**
 * INICIO Y CONFIGURACION DEL PROGRAMA
 */
void setup() {
 
// Inicilización de la comunicación serial
  Serial.begin(115200);
  
// Inicialización del protocolo I2C
  Wire.begin(21, 22);

// Inicialización del sensor DHT22
  my_sensor.begin();

// Inicialización de la LCD
  lcd.init();
  lcd.backlight();
  lcd.createChar(iARROW,bARROW);
  lcd.home();

  // Imprime la informacion del proyecto:
  lcd.setCursor(0,0); lcd.print("   Hanging Gardens  ");
  lcd.setCursor(0,1); lcd.print("      Project       ");
  lcd.setCursor(0,2); lcd.print("     Ver.  1.0      ");
    

  u_int16_t date[6];

  if (!rtc.begin()) {
    Serial.println(F("Couldn't find RTC"));
    while (1) Serial.println(F("Couldn't find RTC"));
  }

// En caso que el módulo RTC perdiera por un instante la energía de la pila:
// Se configura como fecha y hora la de compilación del programa
  if (rtc.lostPower()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  DateTime now = rtc.now();
  uint8_t CurrentDay= now.day(); // Almacena en la memoria volatil el día del RTC 

  lcd.setCursor(0,3);
  lcd.print(".");
  i++;
  delay(150);
// Inicialización de la tarjeta SD
  while (!SD.begin(PIN_SPI_CS)) { // SD CARD error
      Serial.println(F("SD CARD FAILED, OR NOT PRESENT!"));
      delay(500);
    }
  Serial.println(F("SD CARD INITIALIZED."));

  lcd.print(".");
  i++;
  delay(150);

  // Carga la configuracion de la SD, y la configura la primera vez:
  loadConfig();
  // Carga los valores almacenados en la SD
  loadValues();

  lcd.print(".");
  i++;
  delay(150);
 
  I2C_PIN1.pinMode(P0,OUTPUT);
  I2C_PIN1.pinMode(P1,OUTPUT);
  I2C_PIN1.pinMode(P2,OUTPUT);
  I2C_PIN1.pinMode(P3,OUTPUT);
  I2C_PIN1.pinMode(P7, INPUT);
  I2C_PIN1.pinMode(P6, INPUT);
  I2C_PIN1.pinMode(P5, INPUT);
  I2C_PIN1.pinMode(P4, INPUT);
  I2C_PIN1.digitalWrite(P0,LOW); // NutA Pump OFF
  I2C_PIN1.digitalWrite(P1,LOW); // NutB Pump OFF
  I2C_PIN1.digitalWrite(P2,LOW); // Water Pump OFF
  I2C_PIN1.digitalWrite(P3,LOW); // Grow Light OFF
  if(memory.d.Slave2 == 1){
    I2C_PIN2.pinMode(P0,OUTPUT);
    I2C_PIN2.pinMode(P1,OUTPUT);
    I2C_PIN2.pinMode(P2,OUTPUT);
    I2C_PIN2.pinMode(P3,OUTPUT);
    I2C_PIN2.pinMode(P7, INPUT);
    I2C_PIN2.pinMode(P6, INPUT);
    I2C_PIN2.pinMode(P5, INPUT);
    I2C_PIN2.pinMode(P4, INPUT);
    I2C_PIN2.digitalWrite(P0,LOW); // NutA Pump OFF
    I2C_PIN2.digitalWrite(P1,LOW); // NutB Pump OFF
    I2C_PIN2.digitalWrite(P2,LOW); // Water Pump OFF
    I2C_PIN2.digitalWrite(P3,LOW); // Grow Light OFF
  }

 lcd.print(".");
 i++; 
 delay(150);

  // Configuración del encoder
  pinMode(channelPinA, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(channelPinA), doEncode, CHANGE);
  pinMode(buttonPin, INPUT_PULLUP); // Configure the pin as an input with internal pull-up resistor
  attachInterrupt(digitalPinToInterrupt(buttonPin),ButtonPress, RISING); // Configure the interrupt

  lcd.print(".");
  i++;
  delay(150);

  WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP
  //WiFiManager, Local intialization. Once its business is done, there is no need to keep it around
  
  pinMode(2,OUTPUT); // pin LED
  bool res;
  wm.setConfigPortalBlocking(false);
  res = wm.autoConnect("HanguingGardenAP","WfMB4by10n"); // password protected ap
      if(!res) {
        Serial.println("Failed to connect");
        wm.startConfigPortal("OnDemandAP");
    } 

  else {
      //if you get here you have connected to the WiFi    
    Serial.println("WiFi Connected");  
    lcd.print(".");
    i++;
    delay(150);
  }
  lcd.print(".");
  i++;
  delay(150);
  
  Blynk.config(BLYNK_AUTH_TOKEN); //Una vez conectado a WiFi se conecta a Blynk
  Blynk.connect(); 
  lcd.print(".");
  i++;
  delay(150);

  ADC1.setGain(GAIN_ONE); // +/- 4.096V  1 bit = 0.125mV 
  ADC2.setGain(GAIN_ONE);
  //float adc_resolution = 65536.0; // 16 bits
  
  if(!ADC1.begin(0x48)) sys_error3=true;                // Iniciar el ADS1115
  if(memory.d.Slave2 && !ADC2.begin(0x49)) sys_error3=true; 

  lcd.print(".");
  i++;
  delay(150);

  for(i; i<20;i++)    {
        lcd.print(".");
        delay(50);
    }
  lcd.clear();
  i=0;

}

void MainScreen(){

  if((tNow-tPrevLCDUpdate)>tLCDRefresh){
  tPrevLCDUpdate=tNow;
  switch (memory.d.Brightness){
  case 0: lcd.noBacklight(); break;
  case 1: if(LightLevel<=2||EncoderTurn){lcd.backlight();EncoderTurn=false;} else lcd.noBacklight(); break;
  case 2: lcd.backlight(); break;
  }
  if(memory.d.Slave2){
  lcd.setCursor(0,0); lcd.print("                    ");
  lcd.setCursor(0,1); lcd.print("                    ");
  }
  else{
      lcd.setCursor(0,0); lcd.print(" My Hanging Gardens ");
      lcd.setCursor(0,1); lcd.print("        Day:");
      lcd.setCursor(12,1); lcd.print(memory.v.Daycounter);
      lcd.setCursor(13,1); lcd.print("     ");
  }
  lcd.setCursor(0,2); lcd.print("                    ");
  lcd.setCursor(0,3); lcd.print("                    ");
  if(LastScreen==3 && (sys_error1)){
    i=1;
    lcd.setCursor(0,0); lcd.print("******WARNINGS******");
    if(sys_error1){
      lcd.setCursor(0,i); lcd.print("err1:Network error ");
      i++;
    }
    if(sys_error2){
      lcd.setCursor(0,i); lcd.print("err2:SD card error ");
      i++;
    }
    if(sys_error3){
      lcd.setCursor(0,i); lcd.print("err3:I2C addr error ");
      i++;
    }
    LastScreen=0;
  }
  else if(LastScreen==3||LastScreen==0){
    if(memory.d.Slave2){
    lcd.setCursor(0,0); lcd.print(" My Hanging Gardens ");
    lcd.setCursor(0,1); lcd.print("        Day:");
    lcd.setCursor(12,1); lcd.print(memory.v.Daycounter);
    }
    lcd.setCursor(0,2); lcd.print(GetDate()+"  "+ GetHour());
    lcd.setCursor(0,3);
      lcd.print(temperature);     // print the temperature
      lcd.print(" ");
      lcd.print((char)223);      // print ° character
      lcd.print("C");
      lcd.print("  RH:");
      lcd.print(humidity);     // print the humidity
      lcd.print(" %");
      
      LastScreen=1;
  }
  else if(LastScreen==1){
    //lcd.setCursor(0,0); //Reservado para 2 slave
    //lcd.setCursor(0,1); //Reservado para 2 slave
    if(memory.d.Slave2){}
    else{
      lcd.setCursor(0,2); lcd.print("Nut A:");
      lcd.setCursor(6,2); lcd.print(NutA_LVL==1? "OK ":"LOW");
      lcd.setCursor(11,2);lcd.print("Nut B:");
      lcd.setCursor(17,2); lcd.print(NutB_LVL==1? "OK":"LOW"); 
      lcd.setCursor(0,3); 
      if(water_LVL==2)lcd.print(" Water Level: High  ");
      if(water_LVL==1)lcd.print(" Water Level: Mid   ");
      if(water_LVL==0)lcd.print(" Water Level: LOW!  ");
      LastScreen=2;
    }
  }
  else if(LastScreen==2){
    //lcd.setCursor(0,0); //Reservado para 2 slave
    //lcd.setCursor(0,1); //Reservado para 2 slave
    lcd.setCursor(0,2); lcd.print("TDS:");
    lcd.setCursor(4,2); lcd.print(TDS);
    if(TDS<1000) {
      lcd.setCursor(7,2); lcd.print("  ");
    }
    else lcd.setCursor(8,2); lcd.print(" ");
    lcd.setCursor(11,2); lcd.print("pH:"); lcd.print(pH);
    lcd.setCursor(0,3); lcd.print("Light Level:");
    switch(LightLevel){
    case 0:lcd.setCursor(12,3); lcd.print("Dark    ");
    case 1:lcd.setCursor(12,3); lcd.print("Dim     ");
    case 2:lcd.setCursor(12,3); lcd.print("Bright  ");
    case 3:lcd.setCursor(12,3); lcd.print("Sunlight");
    }
    LastScreen=3;
  }

  }  
}

void WaterPump(){
  if((tNow-tPrevWatering)>= (memory.d.WateringPeriod*min_2_us) && !WaterPump_is_running){ 

      if(water_LVL!=0){
        WaterPump_is_running=true;
        I2C_PIN1.digitalWrite(P2,HIGH);
        tPrevWatering=tNow;
      } 
      else Serial.println("The Water Level in the device 1 is too low");
      if(memory.d.Slave2){
        if(water_LVL!=0){
          WaterPump_is_running=true;
          I2C_PIN2.digitalWrite(P2,HIGH);
          tPrevWatering=tNow;
        } 
        else Serial.println("The Water Level in the device 2 is too low");
      }

      
  }
  else if(WaterPump_is_running && (tNow-tPrevWatering)>=((memory.d.WateringPeriod*min_2_us)*(memory.d.WateringDutyCicle/100))){
    WaterPump_is_running=false;
    I2C_PIN1.digitalWrite(P2,LOW);
    if(memory.d.Slave2) I2C_PIN2.digitalWrite(P2,LOW);  
    tPrevWatering=tNow;
  }
}

void NutrientPumps(){
  // Lógica de la bomba peristáltica del nutriente A
  if((tNow-tPrevNutADose)>= (memory.d.NutADose*(1/Nut_flow_rate)*min_2_us) && !NutAPump_is_waiting){
      if(!NutAPump_is_running && LowNutSol){
        if(NutA_LVL!=0){
          NutAPump_is_running=true;
          I2C_PIN1.digitalWrite(P0,HIGH);
          tPrevNutADose=tNow;
        } 
        else Serial.println("The Nutrient A Level in the device 1 is too low");
      }
      if(NutAPump_is_running){ // Se fija un tiempo en el que la bomba peristáltica está apagada
        NutAPump_is_running=false;
        I2C_PIN1.digitalWrite(P0,LOW);
        NutAPump_is_waiting=true;
        tPrevNutADose=tNow;
      }
    }
    // Lógica de la bomba peristáltica del nutriente B
    if((tNow-tPrevNutBDose)>= (memory.d.NutBDose*(1/Nut_flow_rate)*min_2_us) && !NutBPump_is_waiting){
      if(!NutBPump_is_running && LowNutSol){
        if(NutB_LVL!=0){
          NutBPump_is_running=true;
          I2C_PIN1.digitalWrite(P1,HIGH);
          tPrevNutBDose=tNow;
        } 
        else Serial.println("The Nutrient B Level in the device 1 is too low");
      }
      if(NutBPump_is_running){ // Se fija un tiempo en el que la bomba peristáltica está apagada
        NutBPump_is_running=false;
        I2C_PIN1.digitalWrite(P1,LOW);
        NutBPump_is_waiting=true;
        tPrevNutBDose=tNow;
      }
    }
  if((NutAPump_is_waiting && NutBPump_is_waiting) && (tNow-(max(tPrevNutADose,tPrevNutBDose)))>=(memory.d.DosingPeriod*min_2_us)){
      NutAPump_is_waiting=false;
      NutBPump_is_waiting=false;
  }
}

void GrowLight(){
  DateTime now = rtc.now();
  if(memory.v.CurrentDay!=now.day() && memory.d.DawnTime<=now.hour()){
    memory.v.Daycounter++;
    memory.v.LightTime=0;
    memory.v.CurrentDay=now.day();
    saveValues();
  }
  if(LightLevel<2 && memory.v.LightTime<memory.d.Photoperiod && !GrowLight_is_active){
    GrowLight_is_active=true;
    I2C_PIN1.digitalWrite(P3,HIGH); 
  }
  else if(GrowLight_is_active){
    GrowLight_is_active=false;
    I2C_PIN1.digitalWrite(P3,LOW);
  }
}
/**
 * Median filter algorithm (bubble algorithm) implementation in the TDS probe example code by dfrobot:
 * https://wiki.dfrobot.com/Gravity__Analog_TDS_Sensor___Meter_For_Arduino_SKU__SEN0244
 */
int getMedianNum(int bArray[], int iFilterLen) {
    int bTab[iFilterLen];
    // Copia los elementos de bArray a bTab
    for (byte i = 0; i < iFilterLen; i++)
        bTab[i] = bArray[i];
    
    int i, j, bTemp;
    // Ordena la matriz bTab
    for (j = 0; j < iFilterLen - 1; j++) {
        for (i = 0; i < iFilterLen - j - 1; i++) {
            if (bTab[i] > bTab[i + 1]) {
                // Intercambia los elementos si están en el orden incorrecto
                bTemp = bTab[i];
                bTab[i] = bTab[i + 1];
                bTab[i + 1] = bTemp;
            }
        }
    }
    // Calcula la mediana
    if ((iFilterLen & 1) > 0)
        // Longitud impar: toma el valor medio
        bTemp = bTab[(iFilterLen - 1) / 2];
    else
        // Longitud par: toma el promedio de los dos valores medios
        bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
    
    // Devuelve la mediana
    return bTemp;
}
/**
 * Return the average of the array elements 
 */
int avgArray(int Array[],int size){ 
  int temp=0;
  for(i=0; i<size; i++){
    temp+=Array[i];
  }
  temp=temp/size;
  return temp;
}
void Sensors(){
  float factorEscala = 0.125; // El factor de escala del ADC es de 0,125mV
   if(tNow-tPrevSample > 50000)     //every 50 milliseconds,read the analog sample from the ADC
   {
     tPrevSample=tNow;
     analogBufferTDS[analogBufferIndex] = ADC1.readADC_SingleEnded(0);    //Lectura del pin A0 del ADS1115 correspondiente al sensor TDS
     analogBufferpH[analogBufferIndex] = ADC1.readADC_SingleEnded(1);    //Lectura del pin A1 del ADS1115 correspondiente al sensor pH
     analogBufferNTC[analogBufferIndex]= ADC1.readADC_SingleEnded(2);    //Lectura del pin A2 del ADS1115 correspondiente al sensor NTC
     analogBufferIndex++;
     if(analogBufferIndex == SCOUNT) 
         analogBufferIndex = 0;
   }
   if(tNow-tPrevSensorUpdate > 1000000) { // Every 1 second update the sensors value
      tPrevSensorUpdate=tNow;
    /* DIGITAL SENSORS*/
      if(I2C_PIN1.digitalRead(P4) && I2C_PIN1.digitalRead(P5)) water_LVL=2;
      else if(I2C_PIN1.digitalRead(P5)) water_LVL=1;
      else water_LVL=0;
      if(I2C_PIN1.digitalRead(P6))NutA_LVL=1;
      if(I2C_PIN1.digitalRead(P7))NutB_LVL=1;
      temperature= my_sensor.readTemperature();
      humidity = my_sensor.readHumidity();
    /* ANALOG SENSORS*/
    //WaterTemp
    float R4_mV=factorEscala*avgArray(analogBufferNTC,SCOUNT);
    float NTC_mV=3300-R4_mV;
    float NTC_Ohm= SERIESRESISTOR*(NTC_mV-R4_mV);
    //Symplified Steinhart-Hart equation (B parameter equation) 
    WaterTemp = NTC_Ohm / THERMISTORNOMINAL;     // (R/Ro)
    WaterTemp = log(WaterTemp);                  // ln(R/Ro)
    WaterTemp /= BCOEFF;                   // 1/B * ln(R/Ro)
    WaterTemp += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
    WaterTemp = 1.0 / WaterTemp;                 // Invert
    WaterTemp -= 273.15;                         // convert absolute temp to C    
    //TDS (corrected with WaterTemp)
    for(copyIndex=0;copyIndex<SCOUNT;copyIndex++)
     analogBufferTemp[copyIndex]= analogBufferTDS[copyIndex];
    float filtVoltage = getMedianNum(analogBufferTemp,SCOUNT) *factorEscala/1000 ; // read the analog value more stable by the median filtering algorithm, and convert to voltage value
    float compensationCoefficient=1.0+0.02*(WaterTemp-25.0); //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
    float compensationVoltage=filtVoltage/compensationCoefficient; //temperature compensation
    TDS=(133.42*compensationVoltage*compensationVoltage*compensationVoltage - 255.86*compensationVoltage*compensationVoltage + 857.39*compensationVoltage)*0.5; //convert voltage value to tds value
    if(TDS<(memory.d.minTDS+((memory.d.maxTDS-memory.d.minTDS)/2))) LowNutSol=true;
    else LowNutSol=false;
    //LDR (only 1 measurement per second because of its response time)
    float R5_mV=(ADC1.readADC_SingleEnded(3)*factorEscala);
    float LDR_mV=3300-R5_mV;
    float R5=3.3; // resistencia en serie con el LDR
    float LDR_kOhm=R5*(LDR_mV/R5_mV);
    if (LDR_kOhm>=10.0)    LightLevel=0;
    else if (LDR_kOhm>=1.5)LightLevel=1;
    else if (LDR_kOhm>=0.5)LightLevel=2;
    else if (LDR_kOhm>=0.1)LightLevel=3;
    //pH
    for(copyIndex=0;copyIndex<SCOUNT;copyIndex++)
     analogBufferTemp[copyIndex]= analogBufferTDS[copyIndex];
    filtVoltage = getMedianNum(analogBufferTemp,SCOUNT)*factorEscala/1000 ; // read the analog value more stable by the median filtering algorithm, and convert to voltage value
    pH=7 + ((1.66 - filtVoltage) / 0.18); // The pH sensor voltage range is reduced from [0V,+5V] to [0V,+3V3] via voltage divider
   }  
}
void BlynkUpdate(){
  
  if((tNow-tPrevBlynkUpdate)>1000000){ //Cada segundo
    Blynk.run();
    // You can send any value at any time.
    // Please don't send more that 10 values per second.
    Blynk.virtualWrite(V0, temperature);
    Blynk.virtualWrite(V1, humidity);
    Blynk.virtualWrite(V2, TDS);  
    Blynk.virtualWrite(V3, pH);
    if(water_LVL==0)Blynk.logEvent("water_level_low");
    if(NutA_LVL==0)Blynk.logEvent("nut_a_level_low");
    if(NutB_LVL==0)Blynk.logEvent("nut_b_level_low");
  }
}
void loop(){
  tNow=esp_timer_get_time(); //Resolución 1us limite: +200 años vs millis() cuyo limite es 50 días
  if(WiFi.status() != WL_CONNECTED)
  {
    wm.process();
    sys_error1=true;
  }
  else sys_error1=false;
  //Check Button Interruption
  if(ButtonInterruption)
  {
    openMenu();
    ButtonInterruption = false;
  }
  else MainScreen();
  WaterPump();
  NutrientPumps();
  GrowLight();
  Sensors();
  DATALOG();
  BlynkUpdate();
}