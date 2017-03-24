/*************************************************************************************
 
  Ioquir Sotile
 
  Sotile Homebrew  v0.00j

  Notas da subversão / Subversion Notes:
  working:
  - digital termometer
  - input using a potentiometer, not a rotary encoder
  - display ok, but paralel 
  - all steps are fine.
  - rgb led 
  - rotary encoder
  - portuguese characters in messages
  not working/implemented or implementing:
  - customize mash and boil profiles
  - ESC button
  - use interrupts for OK and ESC buttons
  -  q (not working)
  
  
 
   Auxiliar no processo de fabricação de cerveja caseira
   Curitiba - Paraná - Brasil
   Início da programação start of project: 26/out/2015.
 
  This program is designed to help home brewing proccess
 
  Prepared to work with a 4x20 display.
  For English messages, uncomment #define ENGLISH
  -- not yet prepared for fahrenheit . Sorry! See DisplayTemperature function.

 www.zonamaker.com.br/simbolos-personalizados-no-display-lcd-com-i2c/

 some sources:
 http://www.pjrc.com/teensy/td_libs_OneWire.html
 
**************************************************************************************/

#define On    1
#define Off   0

//#define TEST    1     //uncomment for code test
#define ENGLISH 1  //uncomment for english messages. If not, it will be BR portuguese.
#define COMMON_ANODE //  comment if using RGB common catode LEDs
//#define POTENTIOMETER //uncomment if using a 5K potentiometer to input values and menus, instead will use a rotary encoder


// display inicial definitions
/* if you use a 20x4 with parallel interface, you shoud set:
* #include <LiquidCrystal.h>
* LiquidCrystal lcd(7, 6, 5, 4, 3, 2);           // select the pins used on the LCD panel
*/
// for a display with a serial interface adapter:
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x3F,20,4);  // set the LCD address to 0x27 for a 20 chars and 4 line display

// temperature sensor initial definitions
#include <OneWire.h>

OneWire ONE_WIRE_BUS(8); // select the digital pin used to connect sensor(s)

/*-----( Declare Variables )-----*/
// special characters
byte Cedilha[8] = {0x0,0x0, 0xe,0x11,0x10,0x15,0xe,0x8};  // character ç
byte ATil   [8] = {0xd,0x16,0xe,0x1,0xf,0x11,0xf}; //character ã
byte AAgudo [8] = {0x2,0x4,0xe,0x1,0xf,0x11,0xf};  //character á
//byte Thermo [8] = {B00100,B01010,B01010,B01110,B01110,B11111,B11111,B01110}; //icon for termometer

// #constants
#define Choose     0
#define Prepare    1
#define Mash       2
#define Lauther    3
#define Boil       4
#define Cool       5
#define Whirlpool  6

#define BoilUp     0
#define BoilOn     1
#define BoilOk     2
#define BoilRest   3

const int ClarificationTime   = 10; // time recirculating the worth before Lauther
const int CoolToTemperature   = 25; // final temperature of cooling step
const int Recirculate         =  0;
const int WhirlpoolTime       = 5;
const int Sparge              =  1;
const int TemperatureTolerance= 1;

// PINS digital

#define Encoder0PinA    0
#define Encoder0PinB    1 // Interrupt pin in NANO
#define CancelButton    2 // Interrupt pin in NANO
#define OkButton        3
// parallel lcd       7,6,5,4,3,2    // only if you use a parallel display, note the conflicting ports!!
// free                 4
// free                 5
// free                 6
// free                 7
//      ONE_WIRE_BUS    8 // definitiron above. Here only for documentation
#define Buzzer          9
// free                10
#define RGBLedRedPin   11
#define RGBLedGreenPin 12
#define RGBLedBluePin  13
   
// PINS analogic
#define SensorTemp      0
#define Potentiometer   1

// vars

boolean WaitForOkButton, OkButtonWasPressed,
      HeatIsOn, YellowAlert, RedAlert;
byte  ActualStage,      PreviousStage,
      ActualMashStep,   ActualBoilStep, 
      HopNumber,
      Option, LastOption, 
      Red, Green, Blue,
      present = 0, type_s, data[12], addr[8],
      Encoder0PinALast;
    
int   Minutes, Seconds, Aux, i;      
unsigned long Now, HalfSecondAgo,  TimeStageStarted, TimeMashStepStarted, ActualBoilStepStarted;
            
float ActualTemperature;

struct StructSteps {
   String Name;
   int    InitialTemperature;
   int    NumberOfSteps;
   int    Temperature[4]; // 3 steps plus mashout
   int    Time[4];        // 3 steps plus mashout
};

struct StructBoil {
   int TotalTime;
   int NumberOfHops;
   int TimeOfHop[4];  // max of 4 hoppings during boil
};

struct StructSteps StepProfile;
struct StructBoil  BoilProfile;


void CleanDisplay(){
   //for(j = 0; j  < 4; j++)
   //    for (i = 0; i  < 20; i++)
   //       lcd.setCursor(i,j); lcd.print(" ");
   // code was not working, substituted by:
   lcd.setCursor( 0,0); lcd.print (F("                    "));
   lcd.setCursor( 0,1); lcd.print (F("                    "));
   lcd.setCursor( 0,2); lcd.print (F("                    "));
   lcd.setCursor( 0,3); lcd.print (F("                    "));
}

int Message(int Which, int B){
   for(i = 0; i  < 20; i++){
      lcd.setCursor(i,3); 
      lcd.print(" ");
   }
  
   lcd.setCursor(0,3);  
#ifdef ENGLISH
  switch (Which) {
      case  0 : lcd.print(F("                    ")); break; // F function loads strings into flash rather than in SRAM
      case  1 : lcd.print(F("  ** warm up!!!  ** ")); break;
      case  2 : lcd.print(F("  Recirculate wort  ")); break;
      case  3 : lcd.print(F("    Sparge  (OK)    ")); break;
      case  4 : lcd.print(F("    Do whirlpool    ")); break;
      case  5 : lcd.print(F("   Heat the wather  ")); break;
      case  6 : lcd.print(F("  Add grains  (OK)  ")); break;
      case  7 : lcd.print(F(" Mash complete (OK) ")); break;
      case  8 : lcd.print(F("Warm up til ramp(OK)")); break;
      case  9 : lcd.print(F("     Recirculate    ")); break;
      case 10 : lcd.print(F("Heat to boiling!(OK)")); break;
      case 11 : lcd.print(F(" Boil completed (OK)")); break;     
      case 12 : lcd.print(F(" Add hop "           )); break;                       
      case 13 : lcd.print(F(" Add whirflock (OK) ")); break;                        
      case 14 : { lcd.print(F("  Rest for ")); 
                  lcd.print(ClarificationTime);
                  lcd.print(F(" min.  ")); }          break;
      case 15 : lcd.print(F("    Cool the wort   ")); break;
      case 16 : lcd.print(F(" Remove chiller (OK)")); break;    
      case 17 : lcd.print(F(" Press (OK) to start")); break;    
  }
#else 
  switch (Which) {
      case  0 : lcd.print(F("                    ")); break;
      case  1 : lcd.print(F("  ** Aquecer!!!  ** ")); break;
      case  2 : lcd.print(F("  Recircular mosto  ")); break;
      case  3 : {
                lcd.print(F(" Lavar os graos (OK)"));
                lcd.setCursor(12,3); lcd.write(char(1)); break;  // writes an "ã"
      }
      case  4 : lcd.print(F("  Fazer whirlpool   ")); break;
      case  5 : lcd.print(F("  Aquecer a agua    ")); 
                lcd.setCursor(12,3); lcd.write(char(2)); break;  // writes an "á"
      case  6 : {
                lcd.print(F("Adicionar graos (OK)")); 
                lcd.setCursor(12,3); lcd.write(char(1)); break;  // writes an "ã"
      }
      case  7 : lcd.print(F(" Mash completo (OK) ")); break;
      case  8 : lcd.print(F("Aquecer p/ rampa(OK)")); break;
      case  9 : lcd.print(F("     Recircular     ")); break;
      case 10 : lcd.print(F("Aquecer p/ferver(OK)")); break;
      case 11 : lcd.print(F(" Fim da fervura (OK)")); break;            
      case 12 : lcd.print(F("Ad.lupulo "          )); break;                        
      case 13 : lcd.print(F("Junte whirflock (OK)")); break;                        
      case 14 : lcd.print(F(" Descansar 10 min.  ")); break;
      case 15 : lcd.print(F("  Resfriar o mosto  ")); break;
      case 16 : lcd.print(F("Retirar chiller (OK)")); break;
      case 17 : lcd.print(F("   Pressione (OK)   ")); break;    
  }
#endif

  if (B) {
       tone(Buzzer, 1500);
       delay(150);
       noTone(Buzzer);
   }
}


void ShowAlert (){
/* RGB common catode for display
*   R    G    B
*  255,   0,   0 = red
*    0, 255,   0 = green
*    0,   0, 255 = blue
*  255, 255,   0 = yellow
*   80,   0,  80 = purple
*    0, 255, 255 = aqua
*/
   if (RedAlert) {
       Red   = 255;
       Green =   0;
       Blue  =   0;
   } else if (YellowAlert) {  // blue seems better than yellow!
       Red   =   0;
       Green =   0;
       Blue  = 255;
   } else if ((ActualStage ==Choose) || (WaitForOkButton && !OkButtonWasPressed)) { // waiting for an input = aqua color
         Red   =   0;
         Green = 255;
         Blue  = 255;
   } else { // green color 
         Red   =   0;
         Green = 255;
         Blue  =   0;
   }
   #ifdef COMMON_ANODE
   //  for common anode, we need to make digital pin near to 0, so the color is:
   //  color = 255 - color;
      Red   = (255 - Red)  ;
      Green = (255 - Green);
      Blue  = (255 - Blue) ;
   #endif
   analogWrite(RGBLedRedPin  , Red  );
   analogWrite(RGBLedGreenPin, Green);
   analogWrite(RGBLedBluePin , Blue );  
}
  
void TurnMotor(int OnOff){
    // to be developed
}

void TurnHeat(int OnOff) {
  // to be adapted
  if (OnOff == On) {
     Message( 1, true); //" ** warm up!!!  **"
     HeatIsOn    = true;
     YellowAlert = true;
  } else {
     Message ( 0, false); // cleans message
     HeatIsOn  = false;
   YellowAlert = false;
  }
}

void TurnPump (int Witch, int OnOff){
  switch (Witch) {
    case Recirculate :
       if (OnOff = On) {
          Message( 2, false); //"Recirculate wort"
          YellowAlert = true;
       }
       else               
          Message( 0, false);  // cleans message
        YellowAlert = false;
       break;
    case Sparge    :
       if (OnOff = On) {
           Message( 3, false); //"Sparge  (OK)"
           YellowAlert = true;
       } else                  {
           Message( 0, false);  // cleans message
           YellowAlert = false;
       }
       break;
    case Whirlpool    :
       if (OnOff = On) {
           Message( 4, false); //"Do whirlpool"
           YellowAlert = true;
       } else {              
           Message( 0, false);  // cleans message
           YellowAlert = false;
       }
       break;
  }
}

void MessageStage(int Which){
   for(i = 0; i  < 20; i++){ // 20 = lcd rows
      lcd.setCursor(i,0); 
      lcd.print(" ");
   }
   lcd.setCursor(0,0);
#ifdef ENGLISH
   switch (Which) {
      case Choose:   { lcd.print(F("Stage: Choose"      )); break; }  // translate here
      case Prepare:  { lcd.print(F("Stage: Prepare"     )); break; }
      case Mash:     { lcd.print(F("Stage: Mash"        ));
                       lcd.setCursor(15,0);  lcd.print("1/");
                       lcd.print(StepProfile.NumberOfSteps);
                       break; }
      case Lauther:  { lcd.print(F("Stage: Sparge"      )); break; }
      case Boil:     { lcd.print(F("Stage: Boil  "      ));
                       lcd.print(BoilProfile.TotalTime);
                       lcd.print("\'");
                       break; }
      case Cool:     { lcd.print(F("Stage: Cooling"     )); break; }
      case Whirlpool:{ lcd.print(F("Stage: Whirlpool"   )); break; }
    }
#else
   switch (Which) {
      case Choose:   { lcd.print(F("Etapa: Escolha"     )); break; }
      case Prepare:  { lcd.print(F("Etapa: Preparacao " ));                       
                       lcd.setCursor(14,0); 
                       lcd.print(char(0));           // writea a  "ç"
                       lcd.print(char(1)); break; }  // writes an "ã"
      case Mash:     { lcd.print(F("Etapa: Mostura"     ));
                       lcd.setCursor(15,0);  lcd.print("1/");
                       lcd.setCursor(17,0);  lcd.print(StepProfile.NumberOfSteps);
                       break; }
      case Lauther:  { lcd.print(F("Etapa: Filtragem"   )); break; }
      case Boil:     { lcd.print(F("Etapa: Fervura  "   ));
                       lcd.print(BoilProfile.TotalTime);
                       lcd.print("\'");
                       break; }    
      case Cool:     { lcd.print(F("Etapa: Resfriamento")); break; }
      case Whirlpool:{ lcd.print(F("Etapa: Whirlpool"   )); break; }
   }
#endif
}

void DisplayATime(int X, int Min, int Sec){
  if (Min >= 60) { // if time is more than one hour, shows using format hh:mm (or 12h00 if ENGLISH is not deffined)
     Aux = Min   / 60; // calculate hours
     lcd.setCursor(X ,1); 
     if (Aux < 10)  
        lcd.print("  ");
     lcd.print(Aux);  // prints hours
     Aux = Min   % 60; // calculate minutes that are not full hours
#ifdef ENGLISH
     lcd.print(":");
#else         
     lcd.print("h");
#endif
     lcd.setCursor(X+3,1); 
     if ((int)Aux < 10) 
        lcd.print("0");        
     lcd.print(Aux);
  } else {         // else shows using format  mm'ss
     if (Min >= 10)  lcd.setCursor(X  ,1);
        else         lcd.setCursor(X+1,1);
     lcd.print(Min); lcd.print("\'");
     if ((int)Sec < 10) 
        lcd.print("0");
     lcd.print(Sec);
  }
}

void DisplayTimes(){
   // displays total time:
 
   Now     = millis()/1000;
   Minutes =  (int) (Now / 60);
   Seconds =  (int) (Now-(Minutes*60));
   DisplayATime(0, Minutes, Seconds);
   Minutes =  (int) ((Now-TimeStageStarted)        / 60);
   Seconds =  (int) ((Now-TimeStageStarted)-(Minutes*60));
   DisplayATime(6, Minutes, Seconds);
    
   switch (ActualStage) {
      case Mash :
         if ((StepProfile.NumberOfSteps  == 1) && (ActualMashStep ==1)) {
            lcd.setCursor(11,1); lcd.print("/");  lcd.print(StepProfile.Time[0]); lcd.print("\'");
         } else {
            if (ActualMashStep <= StepProfile.NumberOfSteps +1) {
               Minutes =  (int) ((Now-TimeMashStepStarted)         /60 );
               Seconds =  (int) ((Now-TimeMashStepStarted)-(Minutes*60));
               DisplayATime(12, Minutes, Seconds);     
               lcd.setCursor(17,1); lcd.print("/"); lcd.print(StepProfile.Time[ActualMashStep-1]);
            }
         }
         break;
    
      case Boil :
         //if (WaitForOkButton == false) {
            Minutes =  (int) ((Now-ActualBoilStepStarted)        / 60) ;
            Seconds =  (int) ((Now-ActualBoilStepStarted)-(Minutes*60));
            DisplayATime(12,Minutes, Seconds);        
         //}
         break;       
   }
}

float GetONE_WIRE_BUSTemperature(){
  ONE_WIRE_BUS.reset();
  ONE_WIRE_BUS.select(addr);
  ONE_WIRE_BUS.write(0x44, 1);  // start conversion, with parasite power on at the end
  delay(850);         // maybe 750ms is enough, maybe not
  
  ONE_WIRE_BUS.reset();
  ONE_WIRE_BUS.select(addr);    
  ONE_WIRE_BUS.write(0xBE);     // Read Scratchpad

  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ONE_WIRE_BUS.read();
    //Serial.print(data[i], HEX);
    //Serial.print(" ");
  }

  // Convert the data to actual temperature because the result is a 16 bit signed integer, it should
  // be stored to an "int16_t" type, which is always 16 bits  even when compiled on a 32 bit processor.
  int16_t raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {                      // "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
  }
  return (float)raw / 16.0;
}

void DisplayTemperature(){
  Now     = millis()/1000;
  if (Now - HalfSecondAgo > 0.3){
     HalfSecondAgo = Now;
#ifdef TEST    // A 5K potentiometer is used to mimic the sensor
     Aux = analogRead (SensorTemp);
     ActualTemperature = (map((Aux),800,0, 10,100));
#else       
     ActualTemperature = GetONE_WIRE_BUSTemperature();  // a digital read, first sensor in the bus.   
#endif
     /* All code is to work with Celsius
      * If you want, change sensors funcion by sensors.getTempFByIndex(0) to read Fahreinheit
      * and adapt code if necessary.
      */
     lcd.setCursor( 0,2); 
     lcd.print("temp.: "); lcd.print(ActualTemperature,1); lcd.write(B11011111); lcd.print("C ");
     switch (ActualStage) {
        case Prepare :
                lcd.print("/ "); lcd.print(StepProfile.InitialTemperature); lcd.write(B11011111); lcd.print("C");
                if ((ActualTemperature) > StepProfile.InitialTemperature+TemperatureTolerance)
                     RedAlert = true ;
                else RedAlert = false;
                if ((ActualTemperature) < StepProfile.InitialTemperature-TemperatureTolerance)
                     YellowAlert = true ;
                else YellowAlert = false;
                break;
        case Mash :
                if (WaitForOkButton) {
                   if (ActualMashStep <= StepProfile.NumberOfSteps) {
                      lcd.setCursor(16,2); lcd.print(StepProfile.Temperature[ActualMashStep]);
                      if ((ActualTemperature)  > StepProfile.Temperature[ActualMashStep]+TemperatureTolerance)
                           RedAlert = true ;
                      else RedAlert = false;
                      if ((ActualTemperature)  < StepProfile.Temperature[ActualMashStep]-TemperatureTolerance)
                           YellowAlert = true ;
                      else YellowAlert = false;
                   } else {
                      lcd.setCursor(16,2); lcd.print(StepProfile.Temperature[ActualMashStep-2]);
                      if ((ActualTemperature)  > 76+TemperatureTolerance)
                           RedAlert = true ;
                      else RedAlert = false;
                      if ((ActualTemperature)  < 76-TemperatureTolerance)
                           YellowAlert = true ;
                      else YellowAlert = false;
                   }
                } else {
                      lcd.setCursor(16,2); lcd.print(StepProfile.Temperature[ActualMashStep-1]);
                      if ((ActualTemperature)  > StepProfile.Temperature[ActualMashStep-1]+TemperatureTolerance)
                           RedAlert = true ;
                      else RedAlert = false;
                      if ((ActualTemperature)  < StepProfile.Temperature[ActualMashStep-1]-TemperatureTolerance)
                           YellowAlert = true ;
                      else YellowAlert = false;
                    }
                lcd.setCursor(18,2); lcd.write(B11011111); lcd.print("C");
                lcd.setCursor(13,2); lcd.print(" / ");
                break;
        case Lauther :
                lcd.setCursor(13,2); lcd.print(" / 76"); lcd.write(B11011111); lcd.print("C");
                if ((ActualTemperature) > 76+TemperatureTolerance)
                     RedAlert = true ;
                else RedAlert = false;
                if ((ActualTemperature) < 76-TemperatureTolerance)
                           YellowAlert = true ;
                      else YellowAlert = false;
                break;
        case Boil :
                   if ((ActualTemperature) < 94) // assuming that it will boil less than 100ºC
                          YellowAlert = true ;
                    else  YellowAlert = false;
                break;
        case Cool :   
                lcd.setCursor(13,2); lcd.print(" / "); lcd.print(CoolToTemperature); lcd.write(B11011111); lcd.print("C");
                break;
     }
  } 
}

int InputOption(int Min, int Max, int Encoder0Pos) {
#ifdef POTENTIOMETER  // using a potentiometer
     // Actual has no use when reading a potentiometer.
     Aux = analogRead (Potentiometer);
     return (map((Aux),5,890, Min, Max));  // converts potentiometer read to value, adjusted for a 5K linear potentiometer
#else
     // http://henrysbench.capnfatz.com/henrys-bench/arduino-sensors-and-input/keyes-ky-040-arduino-rotary-encoder-user-manual/
     i = digitalRead(Encoder0PinA);
     if ((Encoder0PinALast == LOW) && (i == HIGH)) {
        if (digitalRead(Encoder0PinB) == LOW) {
           if (Encoder0Pos > Min)  
              Encoder0Pos--;
           else                 
              Encoder0Pos = Max;
        } else {
            if (Encoder0Pos < Max)  
              Encoder0Pos++;
           else                 
              Encoder0Pos = Min;
        }
      } 
     Encoder0PinALast = i;     
     return(Encoder0Pos);
#endif
}

void ProcessChooseSteps(){
   const int NumberOfProfiles = 7;
  
    /*                                          Protein rest      Saccharification  Saccharification  mash out
      type              steps body    ºC wather ºC target time    ºC target time    ºC target time    ºC target time
    0 temperature mash  1     light   71                          64        75                        76        10
    1 temperature mash  1     medium  73                          67        60                        76        10
    2 temperature mash  1     full    76                          69        40                        76        10
    3 temperature mash  2     light   54        50        30      64        75                        76        10
    4 temperature mash  2     medium  54        50        30      67        45                        76        10
    5 temperature mash  2     full    54        50        30      69        30                        76        10
    6 temperature mash  3     medium  54        50        20      62        20      67        50      76        10
   
     7 OPTIONS of steps. If you increase  NumberOfProfiles, do not forget to initialize variables!!!!
   */

   Option     =  0;
   LastOption = -2;
   CleanDisplay;   
   Encoder0PinALast = LOW;
   
   lcd.setCursor( 0,0);
#ifdef ENGLISH
   lcd.print(F("Select mash profile:"));
#else        
   lcd.print(F("Selecione o perfil: "));
#endif
   do {
      Option = InputOption(1, NumberOfProfiles, Option);
      
      // steps template - position: 01234561789ABCDEFGHIJ
      //                            50C/20'+64C/30'+70/30
      if (Option != LastOption) {
         switch (Option) {
            case 1 :
               lcd.setCursor( 0,1); lcd.print(F("TM 1 step light body"));
               lcd.setCursor( 0,2); lcd.print(F("64C/75'             "));  
               break;
            case 2 :
               lcd.setCursor( 0,1); lcd.print(F("TM 1 step medium bdy"));
               lcd.setCursor( 0,2); lcd.print(F("67C/60'             "));  
               break;
            case 3 :
               lcd.setCursor( 0,1); lcd.print(F("TM 1 step full body "));
               lcd.setCursor( 0,2); lcd.print(F("69C/40'             "));  
               break;
            case 4 :
               lcd.setCursor( 0,1); lcd.print(F("TM 2 step light body"));
               lcd.setCursor( 0,2); lcd.print(F("50C/30'+64C/75'     "));  
               break;
            case 5 :
               lcd.setCursor( 0,1); lcd.print(F("TM 2 step medium bdy"));
               lcd.setCursor( 0,2); lcd.print(F("50C/30'+67C/45'     "));  
               break;
            case 6 :
               lcd.setCursor( 0,1); lcd.print(F("TM 2 step full body "));
               lcd.setCursor( 0,2); lcd.print(F("50C/30'+69C/30'     "));  
               break;
            case 7 :
               lcd.setCursor( 0,1); lcd.print(F("TM 3 step medium bdy"));
               lcd.setCursor( 0,2); lcd.print(F("50C/20'+62C/20+67/50"));  
               break;
         }
         //lcd.setCursor( 2,2); lcd.print("C/");  
         lcd.setCursor( 3,3); lcd.print(Option); lcd.print("/"); lcd.print(NumberOfProfiles); lcd.print(" Ok?");
         // Aux = digitalRead(OkButton);
      }
      LastOption = Option;
      Aux = digitalRead (OkButton);
    } while (Aux); // OK button pressed goes to false/low/zero

    // initiale fields
    StepProfile.Temperature[0] =  0;  StepProfile.Time[0] =  0;
    StepProfile.Temperature[1] =  0;  StepProfile.Time[1] =  0;
    StepProfile.Temperature[2] =  0;  StepProfile.Time[2] =  0;
    StepProfile.Temperature[3] =  0;  StepProfile.Time[3] =  0;
  
    switch (Option) {
        case 1 : {
           StepProfile.Name = F("TM 1 step light body");
           StepProfile.NumberOfSteps = 1;
#ifdef TEST
           StepProfile.InitialTemperature = 30;  // just for initial tests.
           StepProfile.Time[0] = 1;
#else
           StepProfile.InitialTemperature = 68;
           StepProfile.Time[0] = 75;
#endif        
           StepProfile.Temperature[0] = 64;
           break;
        }
        case 2 : {
           StepProfile.Name = F("TM 1 step medium bdy");
           StepProfile.NumberOfSteps = 1;
           StepProfile.InitialTemperature = 71;
           StepProfile.Temperature[0] = 67;  StepProfile.Time[0] = 60;
           break;
        }
        case 3 : {
           StepProfile.Name = F("TM 1 step full body ");
           StepProfile.NumberOfSteps = 1;
           StepProfile.InitialTemperature = 73;
           StepProfile.Temperature[0] = 69;  StepProfile.Time[0] = 40;
           break;
        }
        case 4 :
           StepProfile.Name = F("TM 2 step light body");
           StepProfile.NumberOfSteps = 2;
           StepProfile.InitialTemperature = 54;
           StepProfile.Temperature[0] = 50;  StepProfile.Time[0] = 30;
           StepProfile.Temperature[1] = 64;  StepProfile.Time[1] = 75;
           break;
        case 5 :
           StepProfile.Name = F("TM 2 step medium bdy");
           StepProfile.NumberOfSteps = 2;
           StepProfile.InitialTemperature = 54;
           StepProfile.Temperature[0] = 50;  StepProfile.Time[0] = 30;
           StepProfile.Temperature[1] = 67;  StepProfile.Time[1] = 45;
           break;
        case 6 :
           StepProfile.Name = F("TM 2 step full body ");
           StepProfile.NumberOfSteps = 2;
           StepProfile.InitialTemperature = 54;
           StepProfile.Temperature[0] = 50;  StepProfile.Time[0] = 30;
           StepProfile.Temperature[1] = 69;  StepProfile.Time[1] = 30;
           break;
        case 7 :
        case 8 :
           StepProfile.Name = F("TM 3 step medium bdy");
           StepProfile.NumberOfSteps = 3;
#ifdef TEST
           StepProfile.InitialTemperature = 30;  // just for initial tests.
           StepProfile.Temperature[0] = 50;  StepProfile.Time[0] = 1;
           StepProfile.Temperature[1] = 62;  StepProfile.Time[1] = 2;
           StepProfile.Temperature[2] = 67;  StepProfile.Time[2] = 1;         
#else
           StepProfile.InitialTemperature = 54;
           StepProfile.Temperature[0] = 50;  StepProfile.Time[0] = 20;
           StepProfile.Temperature[1] = 62;  StepProfile.Time[1] = 25;
           StepProfile.Temperature[2] = 67;  StepProfile.Time[2] = 50;         
#endif
           break;
      
     }
     // set mash out temperature and time:
#ifdef TEST   
     StepProfile.Temperature[StepProfile.NumberOfSteps] = 76;  StepProfile.Time[StepProfile.NumberOfSteps] = 2;   
#else
     StepProfile.Temperature[StepProfile.NumberOfSteps] = 76;  StepProfile.Time[StepProfile.NumberOfSteps] = 10;   
#endif                  
}

void ProcessChooseBoil(){
   const int NumberOfBoilProfiles = 18;
 
   Option     =  0;
   LastOption = -2;
   Encoder0PinALast = LOW;

   CleanDisplay;
   lcd.setCursor( 0,0);
#ifdef ENGLISH     
   lcd.print(F("Select boil profile"));
#else              
   lcd.print(F("Selecione fervura: "));
#endif
   do {
      Option = InputOption(1, NumberOfBoilProfiles, Option);
      // steps template - position: 01234561789ABCDEFGHIJ
      //                            50C/20'+64C/30'+70/30
      if (Option != LastOption) {
         switch (Option) {
            case 1 :
               lcd.setCursor( 0,1); lcd.print(F(" 60 min, 1 hops     "));
               lcd.setCursor( 0,2); lcd.print(F(" 60'                "));  
               break;
            case 2 :
               lcd.setCursor( 0,1); lcd.print(F(" 60 min, 2 hops     "));
               lcd.setCursor( 0,2); lcd.print(F(" 60'   0'           "));  
               break;
            case 3 :
               lcd.setCursor( 0,1); lcd.print(F(" 60 min, 2 hops     "));
               lcd.setCursor( 0,2); lcd.print(F(" 60'   5'           "));  
               break;
            case 4 :
               lcd.setCursor( 0,1); lcd.print(F(" 60 min, 3 hops     "));
               lcd.setCursor( 0,2); lcd.print(F(" 60'  10'   0'      "));  
               break;
            case 5 :
               lcd.setCursor( 0,1); lcd.print(F(" 60 min, 3 hops     "));
               lcd.setCursor( 0,2); lcd.print(F(" 60'  10'   5'      "));  
               break;
            case 6 :
               lcd.setCursor( 0,1); lcd.print(F(" 60 min, 3 hops     "));
               lcd.setCursor( 0,2); lcd.print(F(" 60'  20'   0'      "));  
               break;
            case 7 :
               lcd.setCursor( 0,1); lcd.print(F(" 60 min, 3 hops     "));
               lcd.setCursor( 0,2); lcd.print(F(" 60'  20'   5'      "));  
               break;
            case 8 :
               lcd.setCursor( 0,1); lcd.print(F(" 60 min, 3 hops     "));
               lcd.setCursor( 0,2); lcd.print(F(" 60'  20'   10'     "));  
               break;
            case 9 :
               lcd.setCursor( 0,1); lcd.print(F(" 75 min, 2 hops     "));
               lcd.setCursor( 0,2); lcd.print(F(" 60'  15'           "));  
               break;
            case 10 :
               lcd.setCursor( 0,1); lcd.print(F(" 75 min, 2 hops     "));
               lcd.setCursor( 0,2); lcd.print(F(" 60'  20'           "));  
               break;
            case 11:
               lcd.setCursor( 0,1); lcd.print(F(" 75 min, 3 hops     "));
               lcd.setCursor( 0,2); lcd.print(F(" 60'  20'   0'      "));  
               break;
            case 12 :
               lcd.setCursor( 0,1); lcd.print(F(" 90 min, 2 hops     "));
               lcd.setCursor( 0,2); lcd.print(F(" 60'  15'           "));  
               break;
            case 13 :
               lcd.setCursor( 0,1); lcd.print(F(" 90 min, 2 hops     "));
               lcd.setCursor( 0,2); lcd.print(F(" 60'  20'           "));  
               break;
            case 14 :
               lcd.setCursor( 0,1); lcd.print(F(" 90 min, 3 hops     "));
               lcd.setCursor( 0,2); lcd.print(F(" 60'  20'   0'      "));  
               break;
            case 15 :
               lcd.setCursor( 0,1); lcd.print(F("120 min, 2 hops     "));
               lcd.setCursor( 0,2); lcd.print(F(" 60'  15'           "));  
               break;
            case 16 :
               lcd.setCursor( 0,1); lcd.print(F("120 min, 2 hops     "));
               lcd.setCursor( 0,2); lcd.print(F(" 60'  20'           "));  
               break;
            case 17 :
               lcd.setCursor( 0,1); lcd.print(F("120 min, 3 hops     "));
               lcd.setCursor( 0,2); lcd.print(F(" 60'  20'   0'      "));  
               break;
            case 18 :
               lcd.setCursor( 0,1); lcd.print(F("120 min, 4 hops     "));
               lcd.setCursor( 0,2); lcd.print(F(" 60'  20'   15'  0' "));  
               break;
         }
         if (Option < 10) {
           lcd.setCursor( 2,3); lcd.print(" ");
           //lcd.setCursor( 3,3);
         } else
           lcd.setCursor( 2,3);
         lcd.print(Option);
         //lcd.setCursor( 4,3);
         lcd.print("/");
         //lcd.setCursor( 5,3);
         lcd.print(NumberOfBoilProfiles);
         lcd.print(" Ok?");       
      }
      LastOption = Option;
      Aux = digitalRead (OkButton);
    } while (Aux); // OK button pressed goes to false/low/zero

    // to avoid repeated code
    BoilProfile.TimeOfHop[1] = -1; // -1 indicates no hop at this time
    BoilProfile.TimeOfHop[2] = -1;
    BoilProfile.TimeOfHop[3] = -1;
    switch (Option) {
        case 1 :
            //lcd.setCursor( 0,1); lcd.print(" 60 min, 1 hops     ");
            //lcd.setCursor( 0,2); lcd.print(" 60'                ");  
            BoilProfile.TotalTime    =  60;
            BoilProfile.NumberOfHops =   1;
            BoilProfile.TimeOfHop[0] =  60;
           break;
        case 2 :
            //lcd.setCursor( 0,1); lcd.print(" 60 min, 2 hops     ");
            //lcd.setCursor( 0,2); lcd.print(" 60'   0'           ");  
            BoilProfile.TotalTime    =   60;
            BoilProfile.NumberOfHops =    2;
            BoilProfile.TimeOfHop[0] =   60;
            BoilProfile.TimeOfHop[1] =    0;
           break;
        case 3 :
            //lcd.setCursor( 0,1); lcd.print(" 60 min, 2 hops     ");
            //lcd.setCursor( 0,2); lcd.print(" 60'   5'           ");  
            BoilProfile.TotalTime    =   60;
            BoilProfile.NumberOfHops =    2;
            BoilProfile.TimeOfHop[0] =   60;
            BoilProfile.TimeOfHop[1] =    5;
           break;
        case 4 :
            //lcd.setCursor( 0,1); lcd.print(" 60 min, 3 hops     ");
            //lcd.setCursor( 0,2); lcd.print(" 60'  10'   0'      ");  
            BoilProfile.TotalTime    =   60;
            BoilProfile.NumberOfHops =    3;
            BoilProfile.TimeOfHop[0] =   60;
            BoilProfile.TimeOfHop[1] =   10;
            BoilProfile.TimeOfHop[2] =    0;
           break;
        case 5 :
            //lcd.setCursor( 0,1); lcd.print(" 60 min, 3 hops     ");
            //lcd.setCursor( 0,2); lcd.print(" 60'  10'   5'      ");  
            BoilProfile.TotalTime    =   60;
            BoilProfile.NumberOfHops =    3;
            BoilProfile.TimeOfHop[0] =   60;
            BoilProfile.TimeOfHop[1] =   10;
            BoilProfile.TimeOfHop[2] =    5;
           break;
        case 6 :
            //lcd.setCursor( 0,1); lcd.print(" 60 min, 3 hops     ");
            //lcd.setCursor( 0,2); lcd.print(" 60'  20'   0'      ");  
            BoilProfile.TotalTime    =   60;
            BoilProfile.NumberOfHops =    3;
            BoilProfile.TimeOfHop[0] =   60;
            BoilProfile.TimeOfHop[1] =   20;
            BoilProfile.TimeOfHop[2] =    0;
           break;
        case 7 :
            //lcd.setCursor( 0,1); lcd.print(" 60 min, 3 hops     ");
            //lcd.setCursor( 0,2); lcd.print(" 60'  20'   5'      ");  
            BoilProfile.TotalTime    =   60;
            BoilProfile.NumberOfHops =    3;
            BoilProfile.TimeOfHop[0] =   60;
            BoilProfile.TimeOfHop[1] =   20;
            BoilProfile.TimeOfHop[2] =    5;
           break;
        case 8 :
            //lcd.setCursor( 0,1); lcd.print(" 60 min, 3 hops     ");
            //lcd.setCursor( 0,2); lcd.print(" 60'  20'   10'     ");  
            BoilProfile.TotalTime    =   60;
            BoilProfile.NumberOfHops =    3;
            BoilProfile.TimeOfHop[0] =   60;
            BoilProfile.TimeOfHop[1] =   20;
            BoilProfile.TimeOfHop[2] =   10;
           break;
        case 9 :
            //lcd.setCursor( 0,1); lcd.print(" 75 min, 2 hops     ");
            //lcd.setCursor( 0,2); lcd.print(" 60'  15'           ");  
            BoilProfile.TotalTime    =   75;
            BoilProfile.NumberOfHops =    2;
            BoilProfile.TimeOfHop[0] =   60;
            BoilProfile.TimeOfHop[1] =   15;
           break;
        case 10 :
            //lcd.setCursor( 0,1); lcd.print(" 75 min, 2 hops     ");
            //lcd.setCursor( 0,2); lcd.print(" 60'  20'           ");  
            BoilProfile.TotalTime    =   75;
            BoilProfile.NumberOfHops =    2;
            BoilProfile.TimeOfHop[0] =   60;
            BoilProfile.TimeOfHop[1] =   20;
           break;
        case 11:
            //lcd.setCursor( 0,1); lcd.print(" 75 min, 3 hops     ");
            //lcd.setCursor( 0,2); lcd.print(" 60'  20'   0'      ");  
            BoilProfile.TotalTime    =   75;
            BoilProfile.NumberOfHops =    3;
            BoilProfile.TimeOfHop[0] =   60;
            BoilProfile.TimeOfHop[1] =   20;
            BoilProfile.TimeOfHop[2] =    0;
           break;
        case 12 :
            //lcd.setCursor( 0,1); lcd.print(" 90 min, 2 hops     ");
            //lcd.setCursor( 0,2); lcd.print(" 60'  15'           ");  
            BoilProfile.TotalTime    =   90;
            BoilProfile.NumberOfHops =    2;
            BoilProfile.TimeOfHop[0] =   60;
            BoilProfile.TimeOfHop[1] =   15;
           break;
        case 13 :
            //lcd.setCursor( 0,1); lcd.print(" 90 min, 2 hops     ");
            //lcd.setCursor( 0,2); lcd.print(" 60'  20'           ");  
            BoilProfile.TotalTime    =   90;
            BoilProfile.NumberOfHops =    2;
            BoilProfile.TimeOfHop[0] =   60;
            BoilProfile.TimeOfHop[1] =   20;
           break;
        case 14 :
            //lcd.setCursor( 0,1); lcd.print(" 90 min, 3 hops     ");
            //lcd.setCursor( 0,2); lcd.print(" 60'  20'   0'      ");  
            BoilProfile.TotalTime    =   90;
            BoilProfile.NumberOfHops =    3;
            BoilProfile.TimeOfHop[0] =   60;
            BoilProfile.TimeOfHop[1] =   20;
            BoilProfile.TimeOfHop[2] =    0;
           break;
        case 15 :
            //lcd.setCursor( 0,1); lcd.print("120 min, 2 hops     ");
            //lcd.setCursor( 0,2); lcd.print(" 60'  15'           ");  
            BoilProfile.TotalTime    =  120;
            BoilProfile.NumberOfHops =    2;
            BoilProfile.TimeOfHop[0] =   60;
            BoilProfile.TimeOfHop[1] =   15;
           break;
        case 16 :
            //lcd.setCursor( 0,1); lcd.print("120 min, 2 hops     ");
            //lcd.setCursor( 0,2); lcd.print(" 60'  20'           ");  
            BoilProfile.TotalTime    =  120;
            BoilProfile.NumberOfHops =    2;
            BoilProfile.TimeOfHop[0] =   60;
            BoilProfile.TimeOfHop[1] =   20;
           break;
        case 17 :
            //lcd.setCursor( 0,1); lcd.print("120 min, 3 hops     ");
            //lcd.setCursor( 0,2); lcd.print(" 60'  20'   0'      ");  
            BoilProfile.TotalTime    =  120;
            BoilProfile.NumberOfHops =    3;
            BoilProfile.TimeOfHop[0] =   60;
            BoilProfile.TimeOfHop[1] =   20;
            BoilProfile.TimeOfHop[2] =    0;
           break;
        case 18 :
            //lcd.setCursor( 0,1); lcd.print("120 min, 4 hops     ");
            //lcd.setCursor( 0,2); lcd.print(" 60'  20'   15'  0' ");  
#ifdef TEST   
            BoilProfile.TotalTime    =  5;
            BoilProfile.NumberOfHops =  4;
            BoilProfile.TimeOfHop[0] =  4;
            BoilProfile.TimeOfHop[1] =  3;
            BoilProfile.TimeOfHop[2] =  1;
            BoilProfile.TimeOfHop[3] =  0;
#else
            BoilProfile.TotalTime    =  120;
            BoilProfile.NumberOfHops =    4;
            BoilProfile.TimeOfHop[0] =   60;
            BoilProfile.TimeOfHop[1] =   20;
            BoilProfile.TimeOfHop[2] =   15;
            BoilProfile.TimeOfHop[3] =    0;
#endif         
           break;      
    }  
}

void ProcessChoose(){
   ProcessChooseSteps();
   delay(200);
   ProcessChooseBoil();
   ActualStage    = Prepare;
   PreviousStage  = Choose ;   
#ifdef TEST
   //ActualStage       = Boil;
#endif   
}

void ProcessPrepare(){
   if (ActualStage != PreviousStage) {
      CleanDisplay();
      PreviousStage = ActualStage;
      MessageStage(Prepare);
      TimeStageStarted = millis()/1000;
      TurnHeat(On);
    
      Message( 5,true); //"Heat the wather"
      WaitForOkButton    = false;
      OkButtonWasPressed = false;
   }
   if ((ActualTemperature >= StepProfile.InitialTemperature) && (WaitForOkButton == false)){
      TurnHeat (Off);
      TurnMotor(On );
      Message( 6, true); //"Add grains  (ok)"
      WaitForOkButton = true;
   }
   if (WaitForOkButton && OkButtonWasPressed)
     ActualStage++; //ok, let's go to next stage
}

void ProcessMash(){
  if (ActualStage != PreviousStage) {
    CleanDisplay();
    PreviousStage = ActualStage;
    TimeStageStarted = millis()/1000;
    ActualMashStep = 1;
    TimeMashStepStarted = millis()/1000;
    WaitForOkButton    = false;
    OkButtonWasPressed = false;
    MessageStage(Mash);
  }

  // actual mash step time is over, tests once; add 1 more step, the mash out
  // same stage, next step
  if (     (WaitForOkButton == false)
       && (((millis()/1000) - TimeMashStepStarted) > StepProfile.Time[ActualMashStep-1]*60)
       && (ActualMashStep <= StepProfile.NumberOfSteps+2))
    if (ActualMashStep == StepProfile.NumberOfSteps+1) {
       TurnMotor(Off);
       TurnHeat (Off);
       ActualMashStep++;
       Message( 7, true); //"Mash complete (Ok)"
       WaitForOkButton = true;
       //lcd.setCursor(13, 2); lcd.print("#"); // McGaiverism, to clean a zero that I don't know where comes from.
     
    } else {
       TurnMotor(On);
       TurnHeat (On);
       Message( 8, true); //"Warm up til ramp (Ok)"
       WaitForOkButton = true;
    }
 
   // same step, but temperature too low.
   if (WaitForOkButton == false) {
      if (ActualTemperature < StepProfile.Temperature[ActualMashStep-1]) {
         if (!HeatIsOn) {
            TurnMotor(On);
            TurnHeat (On);
         }
      } else { if (HeatIsOn) {
            TurnMotor(Off);
            TurnHeat (Off);
         }
      }
   } else { // go to next mash step, and it's temperature has been reached - no need to wait for OK press
      if ((ActualMashStep < StepProfile.NumberOfSteps+1) && (ActualTemperature >= StepProfile.Temperature[ActualMashStep])){
          TurnMotor(Off);
          TurnHeat (Off);
          ActualMashStep++;
          lcd.setCursor(15,0);
          if (ActualMashStep  <= StepProfile.NumberOfSteps)
               lcd.print(ActualMashStep);
          else lcd.print("M-out");
          WaitForOkButton     = false;
          OkButtonWasPressed  = false;
          TimeMashStepStarted = millis()/1000;
          lcd.setCursor(0,1);  lcd.print("                   "); // best place in the code to clean line in display.
      }
   }
   if (WaitForOkButton && OkButtonWasPressed) {
      if (ActualMashStep > StepProfile.NumberOfSteps+1) {
         ActualStage++; //ok, let's go to next stage
         TurnMotor(Off);
         TurnHeat (Off);
      } else { // same stage, next step.
         ActualMashStep++;
         lcd.setCursor(15,0);
         if (ActualMashStep  <= StepProfile.NumberOfSteps)
              lcd.print(ActualMashStep);
         else lcd.print("M-out");
         WaitForOkButton     = false;   
         OkButtonWasPressed  = false;
         TurnMotor(On);
         TurnHeat (On);
         Message( 0, false);  // cleans message
         lcd.setCursor(0,1);  lcd.print("                   "); // best place in the code to clean line in display.
         TimeMashStepStarted = millis()/1000;
         if (ActualTemperature < StepProfile.Temperature[ActualMashStep-1]) {
            TurnMotor(On);
            TurnHeat (On);
         }
      }
   }
}

void ProcessLauther(){
   if (ActualStage != PreviousStage) {
      CleanDisplay();
      PreviousStage = ActualStage;
      MessageStage(Lauther);
      TimeStageStarted   = millis()/1000;
      WaitForOkButton    = false ;
      OkButtonWasPressed = false;
      TurnPump(Recirculate, On);
      Message( 9, true); //"Recirculate"
      lcd.setCursor(11,1); lcd.print("/"); lcd.print(10); lcd.print("\'");
   }
#ifdef TEST
   if ((WaitForOkButton == false) &&  (((millis()/1000) - TimeStageStarted) > 1*60)) {
#else
   if ((WaitForOkButton == false) &&  (((millis()/1000) - TimeStageStarted) > ClarificationTime*60)) {
#endif
      TurnPump(Recirculate, Off);
      TurnPump(Sparge     , On );
      delay(100);
      lcd.setCursor(11,1); lcd.print("    ");
      delay (100);
      WaitForOkButton = true;
   } 
   if (WaitForOkButton && OkButtonWasPressed) {
      TurnPump(Sparge     , Off );
      ActualStage++; //ok, let's go to next stage
   }
 
}

void ProcessBoil(){
  if (ActualStage != PreviousStage) {
     CleanDisplay();
     PreviousStage = ActualStage;
     MessageStage(Boil);
     TimeStageStarted = millis()/1000;  // saves the time when boil up started
     ActualBoilStepStarted = TimeStageStarted;
     WaitForOkButton    = true;
     OkButtonWasPressed = false;
     RedAlert           = false ;
     YellowAlert        = false;
     ActualBoilStep     = BoilUp;
     TurnHeat(On);
     HopNumber = 0;
     TurnHeat(On);
     Message(10, true); //"Heat to boiling!(ok)"

     //Serial.println("Fervura");
     //Serial.print  ("TotaTime "); //Serial.println(BoilProfile.TotalTime);
     //Serial.print  ("N hops   "); //Serial.println(BoilProfile.NumberOfHops);
     //Serial.print  ("Time Hop0 "); //Serial.println(BoilProfile.TimeOfHop[0]);
     //Serial.print  ("Time Hop1 "); //Serial.println(BoilProfile.TimeOfHop[1]);
     //Serial.print  ("Time Hop2 "); //Serial.println(BoilProfile.TimeOfHop[2]);
     //Serial.print  ("Time Hop3 "); //Serial.println(BoilProfile.TimeOfHop[3]);
  /*
 * int TotalTime;
   int NumberOfHops;
   int TimeOfHop[4];  // max of 4 hoppings during boil
      */
  // lcd.setCursor(11,1); lcd.print("/"); lcd.print(BoilProfile.TotalTime); lcd.print("\'");
  
  // proccess whirflock time!!!!!
     lcd.setCursor(13,2); lcd.print(" ");
  }
  switch (ActualBoilStep) {
      case BoilUp :     
         if (WaitForOkButton && OkButtonWasPressed) {
            ActualBoilStep++;
            ActualBoilStepStarted = millis()/1000;  // saves the time when boil on started
            WaitForOkButton = false;
            Message( 0,false);  // cleans message
            //lcd.setCursor(13,2); lcd.print("      ");
         }
         break;
    
      case BoilOn :
/* #ifdef TEST
*         if ((!WaitForOkButton) && (((millis()/1000) - ActualBoilStepStarted) > 63)) {  // for development purposes
* #else
*
 */
         if (!WaitForOkButton && (((millis()/1000) - ActualBoilStepStarted) > (BoilProfile.TotalTime * 60))) {
// #endif
            ActualBoilStep++; //ok, let's go to next step
            ActualBoilStepStarted = millis()/1000;  // saves the time when boil ok
            WaitForOkButton    = true;
            OkButtonWasPressed = false;
            TurnHeat(Off);
            Message(11, true); //"Boil completed (Ok)"          
         }
         // Proccess Hops:
         //Serial.println((millis()/1000) - ActualBoilStepStarted);
         //Serial.print("wfok? ");Serial.print(WaitForOkButton); 
         //Serial.print("  hopnumber: ");Serial.print(HopNumber);
         //Serial.print("  calc: "); Serial.print(((millis()/1000) - ActualBoilStepStarted)/60 + (BoilProfile.TimeOfHop[HopNumber] ));
         //Serial.print("  Boil.totaltime: "); Serial.print(BoilProfile.TotalTime);
         //Serial.print("  TimeofHop de hopnumber: "); Serial.println(BoilProfile.TimeOfHop[HopNumber]);
         if (!WaitForOkButton && (HopNumber <= BoilProfile.NumberOfHops) &&
            (((millis()/1000) - ActualBoilStepStarted)/60 + (BoilProfile.TimeOfHop[HopNumber] ) >= BoilProfile.TotalTime  )) {         
//            (((BoilProfile.TotalTime - (millis()/1000) - ActualBoilStepStarted)) <= (BoilProfile.TimeOfHop[HopNumber] * 60))) {
            /* If this is the first time for this hop (not WaitForOk)
             *    and There's still hops to add (HopNumber <= NumberOfHops)
             *    and it's time to add a hop (BoilTotalTime - TimeElapsed) <= time to add Hop
             */
            Message(12, true); //"Add hop  (Ok)"
            lcd.print(HopNumber+1); lcd.print("/"); lcd.print(BoilProfile.TimeOfHop[HopNumber]); lcd.print("\'(OK)");
            //HopNumber++;
            WaitForOkButton    = true;
            OkButtonWasPressed = false;         
         }      
         // Whirflock 15 minutes before boil ends
         if (!WaitForOkButton &&
            (((BoilProfile.TotalTime - (millis()/1000) - ActualBoilStepStarted)) /60 <= 15)) {
            Message(13, true); //"Add whirflock (Ok)"
            WaitForOkButton    = true;
            OkButtonWasPressed = false;         
         }      
         if (WaitForOkButton && OkButtonWasPressed) {
            Message( 0,false);  // cleans message
            HopNumber++;
            WaitForOkButton    = false;
            OkButtonWasPressed = false;          
         }
         break;
    
      case BoilOk :
//         TurnHeat (Off);
//         Message(11, true); //"Boil completed (Ok)"
         if (WaitForOkButton && OkButtonWasPressed) {             
             Message(14,true); //"Rest for 10 min."
             ActualBoilStep++; //ok, let's go to next step
             ActualBoilStepStarted = millis()/1000;  // saves the time when boil ok
             WaitForOkButton    = false;
             OkButtonWasPressed = false;
         }
         //delay(100);
         break;   
     
       case BoilRest:
#ifdef TEST
          if (((millis()/1000) - ActualBoilStepStarted) > 67) {  // for development purposes
#else
          if (((millis()/1000) - ActualBoilStepStarted) > ClarificationTime * 60) {  // rest for ten minutes
#endif
             ActualStage++; //ok, let's go to next stage
          }
          break;
  }
}
 
void ProcessCool(){
  if (ActualStage != PreviousStage) {
     CleanDisplay();
     PreviousStage = ActualStage;
     MessageStage(Cool);
     TimeStageStarted   = millis()/1000;
     WaitForOkButton    = false ;
     OkButtonWasPressed = false;
     RedAlert           = false;
     YellowAlert        = false;
     Message(15, true); //"Cool the wort "
  }
  if ((ActualTemperature <= CoolToTemperature) && !WaitForOkButton){
     WaitForOkButton = true;
     Message (16, false); //"Remove chiller (OK)"
  }
  if (WaitForOkButton && OkButtonWasPressed) {
     ActualStage++; //ok, let's go to next stage
  }
} 
 
void ProcessWhirlpool(){
  if (ActualStage != PreviousStage) {
    CleanDisplay();
    PreviousStage = ActualStage;
    MessageStage(Whirlpool);
    TimeStageStarted   = millis()/1000;
    WaitForOkButton    = false ;
    OkButtonWasPressed = false;
    //Message ("", false);
    lcd.setCursor(11,1);  lcd.print("/"); lcd.print(5); lcd.print("\'");
    lcd.setCursor(12,2);  lcd.print(" ");
    TurnMotor(On);
    TurnPump(Whirlpool, On);
  }
#ifdef TEST
    if (((millis()/1000) - TimeStageStarted) > 60) {  // for development purposes
#else
    if (((millis()/1000) - TimeStageStarted) >= WhirlpoolTime * 60) {  
#endif    
       TurnMotor(Off);
       TurnPump (Whirlpool, Off);
       ActualStage++;
   }

} 

void ProcessEnd(){
  if (ActualStage != PreviousStage) {
     CleanDisplay();
     PreviousStage = ActualStage;
     TimeStageStarted = millis()/1000;
     lcd.setCursor(0,0); lcd.print (F("Sotile Brewing v001a"));
#ifdef ENGLISH
     lcd.setCursor(0,1); lcd.print (F("It took" ));
     lcd.setCursor(0,2); lcd.print (F("to complete brewing" ));
#else        
     lcd.setCursor(0,1); lcd.print (F("Foram" ));
     lcd.setCursor(0,2); lcd.print (F("em todo o processo." ));
#endif 
     Minutes =  (int) ((TimeStageStarted)        / 60) ;
     Seconds =  (int) ((TimeStageStarted)-(Minutes*60));
     DisplayATime(8, Minutes, Seconds);
     lcd.setCursor(15,3); lcd.print ("(ESC)" );
     WaitForOkButton = false ;
  }
}

/*ProcessCancelButton();
 * 
 * This function should allow user to:
 * - Restart whole proccess
 * - Restart current Stage
 * - Restart current step within a stage
 * 
 * This can be done changing values of status variables like 
 *      ActualStage,      PreviousStage,
        ActualMashStep,   ActualBoilStep,     
 */
 
void setup(){
   /* Parallel display:
    * lcd.begin(20, 4);   // start the library 
    */
   //i2c serial display:
   lcd.init();
   lcd.backlight();
   CleanDisplay();
   // starting message
   lcd.setCursor(0,0); lcd.print (F(" Sotile Brewing 1.0"));
#ifdef ENGLISH
   lcd.setCursor(0,1); lcd.print (F("    2017 January   "));
   lcd.setCursor(0,2); lcd.print (F(" by: Ioquir Sotile "));
   lcd.setCursor(0,3); lcd.print (F("setting up..."));  // nothing! just to keep name and version on the display
#else
   lcd.setCursor(0,1); lcd.print (F("  Janeiro de 2017  "));
   lcd.setCursor(0,2); lcd.print (F(" de: Ioquir Sotile "));
   lcd.setCursor(0,3); lcd.print (F("configurando..."));
#endif
   delay(2500); 
   Message(17, true); //" Press (OK) to start "
   do 
      Aux = digitalRead (OkButton);
   while (Aux); // OK button pressed goes to false/low/zero
   CleanDisplay();
   
   lcd.createChar(0, Cedilha);  // creates "ç" char
   lcd.createChar(1, ATil   );  // creates "ã" char
   lcd.createChar(2, AAgudo );  // creates "á" char

   // Start up the DallasTemperature library used for temperature sensor
   //sensors.begin();
 
#ifdef TEST
   Serial.begin(9600);      // open the serial port at 9600 bps:  
#endif

   pinMode(Buzzer,        OUTPUT);
   pinMode(OkButton,      INPUT );
   pinMode(Encoder0PinA,  INPUT );
   pinMode(Encoder0PinB,  INPUT );
   //pinMode(CancelButton,  INPUT );
   
   pinMode(RGBLedBluePin, OUTPUT);
   pinMode(RGBLedGreenPin,OUTPUT);
   pinMode(RGBLedRedPin,  OUTPUT);

   ActualStage       = Choose;
   PreviousStage     = ActualStage;
   WaitForOkButton   = false; 
   OkButtonWasPressed= false;
   Now               = millis();
   HalfSecondAgo     = Now;
   TimeStageStarted  = Now;
   RedAlert          = false;
   YellowAlert       = false;
   
// one wire temperature sensor setup:
if ( !ONE_WIRE_BUS.search(addr)) {
    //Serial.println("No sensor found - No more addresses.");
    ONE_WIRE_BUS.reset_search();    
    delay(250);
    return;
  }
if (OneWire::crc8(addr, 7) != addr[7]) {
      //Serial.println("CRC is not valid!");
      return;
  }
  //Serial.println();
// the first ROM byte indicates which chip
  switch (addr[0]) {
    case 0x10:
      //Serial.println("  Chip = DS18S20");  // or old DS1820
      type_s = 1;
      break;
    case 0x28:
      //Serial.println("  Chip = DS18B20");
      type_s = 0;
      break;
    case 0x22:
      //Serial.println("  Chip = DS1822");
      type_s = 0;
      break;
    default:
      //Serial.println("Device is not a DS18x20 family device.");
      return;
  } 
   
}
 
void loop(){
    
   // buzzer funcions
   //Serial.println("loop");
      
   if (WaitForOkButton){
       Aux = digitalRead(OkButton);
       //Serial.print("Aux: "); Serial.print(Aux); Serial.print("-- Ok pressed: "); Serial.println(OkButtonWasPressed); 
       if (Aux == LOW) {
          Serial.println ("entrou");
          OkButtonWasPressed = 1;
       } 
   }
   // Cancel Button  // I still need to write the Cancel Routine
   /*
   Aux = digitalRead(CancelButton);
   if (Aux == HIGH){
       ProcessCancelButton();
   }
   */
   switch (ActualStage) {
      case Choose    : { ProcessChoose   ();   break; }
      case Prepare   : { ProcessPrepare  ();   break; }
      case Mash      : { ProcessMash     ();   break; }
      case Lauther   : { ProcessLauther  ();   break; }
      case Boil      : { ProcessBoil     ();   break; }
      case Cool      : { ProcessCool     ();   break; }
      case Whirlpool : { ProcessWhirlpool();   break; }
      default        : { ProcessEnd      ();   break; }
   }
   
   ShowAlert ();
  
   if ((ActualStage <= Whirlpool) && (PreviousStage != Choose)) {
      DisplayTimes();
      DisplayTemperature();
   }
}

