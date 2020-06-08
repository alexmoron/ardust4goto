//#include <AccelStepper.h>
//#include <AFMotor.h>
//AF_Stepper motor1(200, 1);
#define MOTORLATCH 12
#define MOTORCLK 4
#define MOTORENABLE 7
#define MOTORDATA 8

// 8-bit bus after the 74HC595 shift register 
// (not Arduino pins)
// These are used to set the direction of the bridge driver.
#define MOTOR1_A 2
#define MOTOR1_B 3
#define MOTOR2_A 1
#define MOTOR2_B 4
#define MOTOR3_A 5 //TERAZ ENABLE A4988
#define MOTOR3_B 7 //MS
#define MOTOR4_A 0 //MS
#define MOTOR4_B 6 //MS

// Arduino pins for the PWM signals.
#define MOTOR1_PWM 11
#define MOTOR2_PWM 3
#define MOTOR3_PWM 5
#define MOTOR4_PWM 6


#define RAMOTOR_PWM Powerl
#define RAMOTOR_OCR Powerl
#define DECMOTOR_PWM 255
#define DECMOTOR_OCR 255

// trackingrate //26712//24932.0//13150//24932//13150//56000// teoretyczna na fullkrok 598.36 sek//RAFAST 178700 KROKÓW NA OROT KOLA// 3225600 400*56*144

//#define RA_MINUSSPEED 6575//28000 //5.15
///#define RA_PLUSSPEED 26300
//#define RA_SPEEDFAST 950///1315 26.2694634146 RATE (OK 3280 SEK NA 24H)
//#define DEC_SPEED 65750//280000 //143750 //287500
//#define DEC_SPEEDFAST 130000 //65750

//#define RA_MINUSSPEED raminus//28000 //5.15
//#define RA_PLUSSPEED raplus
#define RA_SPEEDFAST rafast///1315
#define DEC_SPEED decguide//280000 //143750 //287500
#define DEC_SPEEDFAST decfast //65750 //
#define DEC_TRACKSPEED 552333.0// 120ZEBY OBROT 718SEK//460278.0//28767.0 //TEORETYCZNA 144ZEBYDEC DLA 1/16 MICROSTEP I 6.5 OBR SILNIA NA OBORT OSI(O.46027777 DLA FULLSTEP) 
int fakar = 1;
int fakar2 = 1;
unsigned long trackingrate =26580.0; //SOFTPOM 598200SEK NA 44800STEP
//unsigned long rarate =1;
unsigned long decrate =100;
//unsigned long rarate =23.45;
//unsigned long decrate =100.0;
double guiderate =10;
unsigned long microstepp =16.0;//ILE MICROKROKOW NA FULLKROK
unsigned long ilosckrokowdec;
String krokowiec;
int Powerl =160;
float RAstep=2.0;
int RAPINSTATE = 0;
int FAPINSTATE = 0;
unsigned long StepsFA = 0;

unsigned long ilosckrokowRA;
String krokowiecRA;
unsigned long rafast=870; //26 

long timeFA;
unsigned long last_timeFA;
long timeFA2;
unsigned long last_timeFA2;
unsigned long currentMillisFA ;
unsigned long currentMillisFA2 ;

int latch_copy;
int decrest=1;
int rarest;
//int cv;
int zmiennadec = 1;
int zmiennara = 1;
int sumator;

int Steps = 0;
int StepsHALF = 0;
unsigned long Stepsdec = 0;
unsigned long StepsRA = 0;

int Direction = 0;
int Directiondec = 0;
unsigned long last_time;
unsigned long timeRA;
unsigned long currentMillis ;
long time;
int steps_left=4095;
long timedec;
unsigned long last_timedec;
long timedec2;
unsigned long last_timedec2;
unsigned long currentMillisdec ;
unsigned long currentMillisdec2 ;
long timedecfast;
unsigned long last_timedecfast;
unsigned long currentMillisdecfast ;
int steps_leftdec=4095;
unsigned long czaskrok;
int semafor = 0;
int timerosiem ;
 int    tymczas ;
 unsigned long decfast = (DEC_TRACKSPEED)/(decrate*microstepp);

 unsigned long raminus = trackingrate/(1+guiderate/10);
 unsigned long raplus = trackingrate/(1-(guiderate/10)+0.0001);
 unsigned long decguide = DEC_TRACKSPEED/((guiderate/10)*microstepp);
unsigned long mojczas;
void setup()
{
  
 // pinMode(2, OUTPUT);
  // pinMode(10, OUTPUT);
  //  pinMode(9, OUTPUT);
   //  pinMode(13, OUTPUT);
    //  digitalWrite(2, LOW);
    //  digitalWrite(9, HIGH);
    //  digitalWrite(10, HIGH);
  //    digitalWrite(13, HIGH);
  // Set pins for shift register to output

    pinMode(MOTORLATCH, OUTPUT);
    pinMode(MOTORENABLE, OUTPUT);
    pinMode(MOTORDATA, OUTPUT);
    pinMode(MOTORCLK, OUTPUT);
    // Set pins for shift register to default value (low);
    digitalWrite(MOTORDATA, LOW);
    digitalWrite(MOTORLATCH, LOW);
    digitalWrite(MOTORCLK, LOW);
    // Enable the shift register, set Enable pin Low.
    digitalWrite(MOTORENABLE, LOW);
    // start with all outputs (of the shift register) low
    latch_copy = 0;
    ///PWM
    pinMode(MOTOR1_PWM, OUTPUT);
//digitalWrite (MOTOR1_PWM, HIGH);
pinMode(MOTOR2_PWM, OUTPUT);
//digitalWrite (MOTOR1_PWM, HIGH);
//pinMode(3, OUTPUT);
//  pinMode(11, OUTPUT);
 // TCCR2A |= _BV(COM2A1) | _BV(WGM20) | _BV(WGM21); // fast PWM, turn on oc2b
   // TCCR2B =  _BV(CS22);
//TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20); //fast PWM
   TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM20); //phse correct PWM/2
TCCR2B = _BV(CS20); //64 khz /21 =8 /20 = 1
OCR2B = RAMOTOR_OCR;
OCR2A = RAMOTOR_OCR;

pinMode(MOTOR3_PWM, OUTPUT);
digitalWrite (MOTOR3_PWM, HIGH);
//analogWrite (MOTOR3_PWM, 255);
//
//  TCCR0A |= _BV(COM0A1) | _BV(WGM00) | _BV(WGM01); // fast PWM, turn on OC0A
//TCCR0B = _BV(CS22); // & 0x7;
//OCR0A = 255;
pinMode(MOTOR4_PWM, OUTPUT);
digitalWrite (MOTOR4_PWM, HIGH);
//analogWrite (MOTOR4_PWM, 255);
//TCCR0A |= _BV(COM0B1) | _BV(WGM00) | _BV(WGM01);
// TCCR0B = _BV(CS22);
//OCR0B = 255;

 mic16(microstepp);

  //57.6k, 8 data bits, no parity, one stop bit.
  Serial.begin(57600, SERIAL_8N1);
  //Wait for serial port to connect. Needed for Leonardo only
  while (!Serial);
  Serial.println("INITIALIZED#");
}

void loop ()

{
  
  
  
  
  if (Serial.available() > 0) {
    //Received something
    String opcode = Serial.readStringUntil('#');
    boolean validOpcode=true;
    //Parse opcode
    if(opcode=="CONNECT"){


     
     
  //    resetPins();
    }
    else if (opcode=="DISCONNECT"){
  
//     resetPins();
    }
    else if(opcode=="RA0"){//blad w sterowniku dec zamiast ra
  //    rightAscension.reset();
    
    zmiennara = 1;
    }
    else if(opcode=="RA+"){//
 //     rightAscension.plus();
zmiennara = 3;

    }
    else if(opcode=="RA-"){//
 //     rightAscension.minus();
 zmiennara = 2;   
  
    }
    else if(opcode=="DEC0"){//
  //    declination.reset();
zmiennadec = 1;

    }
    else if(opcode=="DEC+"){//
 //     declination.plus(); 
 digitalWrite (MOTOR3_PWM, LOW);
zmiennadec = 2;



    }
    else if(opcode=="DEC-"){//
 //     declination.minus();
 digitalWrite (MOTOR3_PWM, HIGH);
zmiennadec = 3;}
      
       else if(opcode=="WS"){//
 //     rightAscension.plus();
zmiennara =7;
zmiennadec=4;

    }
    else if(opcode=="ZA"){//
 //     rightAscension.minus();
 zmiennara = 6; 
 zmiennadec=4;  
  
    

    }
    else if(opcode=="PN"){//
 //     declination.plus();
 digitalWrite (MOTOR3_PWM, LOW); 
zmiennadec = 6;
//Stepsdec=0;


    }
    else if(opcode=="PD"){//
 //     declination.minus();
 digitalWrite (MOTOR3_PWM, HIGH);
zmiennadec = 7; 
//Stepsdec=0;
     }
         
    else if(opcode=="100X"){ //46S
rafast = 2400; 
     }
             
    else if(opcode=="50X"){
rafast = 4800; 
//rafast = 900;
     }
             
    else if(opcode=="209X"){ //2.17 21.82 1200 29.8 1130 26.7 1160 28.6
rafast = 1160; 
     }
             
    else if(opcode=="272X"){//27.1818
rafast = 800; 
     }
             
    else if(opcode=="260X"){ //26.06
rafast = 870; 
     }
         else if(opcode=="A"){
rafast = rafast-10; 
     }
    else if(opcode=="10XD"){
decrate = 10.0; 
decfast = (DEC_TRACKSPEED)/(decrate*microstepp);
     }
             
    else if(opcode=="5XD"){
decrate = 5.0; 
decfast = (DEC_TRACKSPEED)/(decrate*microstepp);
     }
             
    else if(opcode=="20XD"){
decrate = 20.0; 
decfast = (DEC_TRACKSPEED)/(decrate*microstepp);
     }
             
    else if(opcode=="50XD"){
decrate = 50.0;
decfast = (DEC_TRACKSPEED)/(decrate*microstepp); 
     }
             
    else if(opcode=="58XD"){
decrate = 100.0; 
decfast = (DEC_TRACKSPEED)/(decrate*microstepp);
     }
         else if(opcode=="116XD"){
//decrate = 200.0; 

mic16(8);
     }
         else if(opcode=="232XD"){
//decrate = 400.0; 
mic16(4);
     }
              else if(opcode=="600XD"){
//decrate = 600.0; 
mic16(2);
     }
                  else if(opcode=="1M"){
microstepp = 1; 
 mic16(microstepp);
 decfast = (DEC_TRACKSPEED)/(decrate*microstepp);
 decguide = DEC_TRACKSPEED/((guiderate/10)*microstepp);
     }
                  else if(opcode=="2M"){
microstepp = 2; 
 mic16(microstepp);
 decfast = (DEC_TRACKSPEED)/(decrate*microstepp);
 decguide = DEC_TRACKSPEED/((guiderate/10)*microstepp);
     }
                  else if(opcode=="4M"){
microstepp = 4;
 mic16(microstepp); 
 decfast = (DEC_TRACKSPEED)/(decrate*microstepp);
 decguide = DEC_TRACKSPEED/((guiderate/10)*microstepp);
     }
                  else if(opcode=="8M"){
microstepp = 8;
 mic16(microstepp); 
 decfast = (DEC_TRACKSPEED)/(decrate*microstepp);
 decguide = DEC_TRACKSPEED/((guiderate/10)*microstepp);
     }
                      else if(opcode=="16M"){
microstepp = 16;
 mic16(microstepp);
 decfast = (DEC_TRACKSPEED)/(decrate*microstepp);
 decguide = DEC_TRACKSPEED/((guiderate/10)*microstepp); 
     }
                           else if(opcode=="1MR"){
RAstep = 1.0; 
     }
                           else if(opcode=="2MR"){
RAstep = 2.0; 
     }
  else if(opcode=="PL"){
Powerl = 200; 
rarest=0;
     }
       else if(opcode=="PM"){
Powerl = 230;
rarest=0; 
     }
       else if(opcode=="PH"){
Powerl = 255; 
rarest=0;
     }
          else if(opcode=="130Z"){
//trackingrate = 13000; 
Powerl = 160;
rarest=0; 
     }
               else if(opcode=="144Z"){
//trackingrate = 26712.0;
Powerl = 165;
rarest=0; 
     }
               else if(opcode=="150Z"){
//trackingrate = 15000;
Powerl = 170;
rarest=0;  
     }
               else if(opcode=="156Z"){
//trackingrate = 156; 
Powerl = 185;
rarest=0; 
     }
     
    else if(opcode=="STOP"){
zmiennara = 8; 
zmiennadec = 1;

     }

     else if (opcode.startsWith("NORTH")){
        String krokowiec = opcode.substring(5);
       ilosckrokowdec = krokowiec.toInt();
      validOpcode=true;
 //     Serial.println(krokowiec);
 //     Serial.println(ilosckrokowdec);
      Stepsdec=0;
      digitalWrite (MOTOR3_PWM, HIGH);
      zmiennadec=4;
    }
      else if(opcode.startsWith("SOUTH")){
        String krokowiec = opcode.substring(5);
       ilosckrokowdec = krokowiec.toInt();
      validOpcode=true;
 //     Serial.println(krokowiec);
 //     Serial.println(ilosckrokowdec);
      digitalWrite (MOTOR3_PWM, LOW);
      Stepsdec=0;
      zmiennadec=4;
    }

      else if(opcode.startsWith("WEST")){
        String krokowiecRA = opcode.substring(4);
       ilosckrokowRA = krokowiecRA.toInt();
      validOpcode=true;
  //    Serial.println(krokowiecRA);
  //    Serial.println(ilosckrokowRA);
      StepsRA=0;
      zmiennara=4;
      zmiennadec=4;
    }

          else if(opcode.startsWith("EAST")){
        String krokowiecRA = opcode.substring(4);
       ilosckrokowRA = krokowiecRA.toInt();
      validOpcode=true;
//      Serial.println(krokowiecRA);
//      Serial.println(ilosckrokowRA);
      StepsRA=0;
      mojczas = millis();
      zmiennara=5;
      zmiennadec=4;
    }

          else if(opcode.startsWith("GR")){
        String grstring = opcode.substring(2);
       guiderate = grstring.toInt();
      validOpcode=true;
  //    Serial.println(guiderate);
    
          raminus = trackingrate/(1+guiderate/10);
          raplus = trackingrate/(1-(guiderate/10)+0.0001);
          decguide = DEC_TRACKSPEED/((guiderate/10)*microstepp);
//          Serial.println(1+guiderate/10);
//          Serial.println(1-(guiderate/10)+0.00001);
//          Serial.println((guiderate/10)*microstepp);
//          
//     Serial.println(raplus);
//     Serial.println(raminus);
//     Serial.println(decguide);

    }



else {
    if(validOpcode){
      //Acknowledge valid command
      Serial.println("OK#");
      
   }
  }}
  //100xd 16M 58 N
//200XD 4M DAJE RATE 90 (89.75)? 
//400XD 4M DAJE 240



//unsigned long dectrack = 3125;
//Serial.println(decfast);
 
switch (zmiennara)
{
  case 1:
  Direction=1; 
  mojstepperHALF(trackingrate);
  break;

    case 2:
  Direction=1; 
  mojstepperHALF(raminus);
  break;

    case 3:
  Direction=1; 
  mojstepperHALF(raplus);
  break;
      case 4:
fastraminus();

  break;
      case 5:

fastraplus();

  break;
      case 6:
  fastraminusSI();

  break;
      case 7:
fastraplusSI();
  break;
      case 8:
resetrapins();


  break;
}

switch (zmiennadec)
{
  case 1:
decstepper(decguide);
  break;

    case 2:
//Directiondec = 0;
decstepperFake2(decguide);
  break;

    case 3:
//Directiondec = 1;
decstepperFake(decguide);
  break;
      case 4:
//Directiondec = 0;   
decstepperFAST(decfast);
  break;
      case 5:
//Directiondec = 1;   
decstepperFAST(decfast);
  break;
      case 6:
//Directiondec = 0;   
decstepperFASTSI(decfast);
  break;
      case 7:
//Directiondec = 1;   
decstepperFASTSI(decfast);
  break;
      case 8:
//resetdecpins();
  break;
}


//switch (sumator)
//{  
//  case 9: 
//  
// 
//  Direction=1;
//  
//   mojstepperHALF(trackingrate);
// //  Directiondec = 1;
//decstepperFake(decguide);
///// mojstepperHALF(RA_SPEEDFAST);
//   //shiftWrite();
//
//  //    resetrapins();
// //  resetdecpins();
// //  shiftWrite();
//
//   break;
//   
//   case 10: 
//  //resetdecpins(); //w
//   Direction=1;
//   mojstepperHALF(raminus);
// //  decstepperFake(decguide);
// //  shiftWrite();
// 
//   
//   
//   case 11:  //e
// // resetdecpins();
//   Direction=1;
//   mojstepperHALF(raplus);
// //  decstepperFake(decguide);
////  shiftWrite();
// 
//   
//   break;
//   
//   
//     
//   case 13: //north
//
//  Direction=1;
//   mojstepperHALF(trackingrate);
//
// //  Directiondec = 1;
//   decstepper(decguide);
////   shiftWrite();
//  
//   
//   break;
//   case 17:  //South
//   Direction=1;
//   mojstepperHALF(trackingrate);
// 
////   Directiondec = 0;   
//   decstepper(decguide);
////   shiftWrite();
//  
//   break;
//   
//   
//   
//   case 14: //north w
//
//  Direction=1;
//   mojstepperHALF(raminus);
// //  Directiondec = 1;
//
//   decstepper(decguide);
////   shiftWrite();
// 
//   break;
//   
//   case 19:  //South e
//   Direction=1;
//   mojstepperHALF(raplus);
// 
////   Directiondec = 0;   
//   decstepper(decguide);
// //  shiftWrite();
//   
//   break;  
//   
//   case 15: //north e
//
//  Direction=1;
//   mojstepperHALF(raplus);
//
// //  Directiondec = 1;
//   decstepper(decguide);
////   shiftWrite();
//   break;
//   
//   case 18:  //South w
//   Direction=1;
//   mojstepperHALF(raminus);
//
// //  Directiondec = 0;   
//   decstepper(decguide);
////   shiftWrite();
//   
//   break;
//   
//   case 51:  //WW
//   fastraplus();
////   Direction=0;
//  // mojstepperHALF(RA_SPEEDFAST);
////  Directiondec = 1;
//decstepperFAST(decfast);
//  // Directiondec = 0;   
//  // resetdecpins();
//  // shiftWrite();
//   
//   case 59:  //ZA
//fastraminus();
////Directiondec = 1;
//decstepperFAST(decfast);
//  // Direction=1;
//  // mojstepper2(RA_SPEEDFAST);
//
//  // Directiondec = 0;   
//   //resetdecpins();
//   //shiftWrite();
//   
//   break;
//   
//   case 62:  //pn
//   Direction=1;
//  mojstepperHALF(trackingrate);
//
//   Directiondec = 1;   
////mic16();
//   decstepperFAST(decfast);
// //  shiftWrite();
//   break;
//   case 72:  //pd
//   Direction=1;
//   mojstepperHALF(trackingrate);
//
//   Directiondec = 0;   
//   decstepperFAST(decfast);
////   shiftWrite();
//   break;
//   
//  case 112: //PN ZA
//fastraminus();
//Directiondec = 1;   
//   decstepperFAST(decfast);
// 
// //  shiftWrite();
//   break;
//   
//   case 122:  //PD ZA
//fastraminus();
//   Directiondec = 0;   
//   decstepperFAST(decfast);
////   shiftWrite();;
//   
//   break;  
//   
//   case 104: //PN WS
// fastraplus();
//      Directiondec = 1;   
//   decstepperFAST(decfast);
////   shiftWrite();
//   break;
//   
//   case 114:  //PD WS
// fastraplus();
//      Directiondec = 1;   
//   decstepperFAST(decfast);
// //  shiftWrite();
//   
//   break; 
//   
//      case 261:  //Pn
//
//      Directiondec = 0;   
//   decstepperFAST(decfast);
//   shiftWrite();
//   
//   break;
//
//      case 271:  //Pd
//
//      Directiondec = 1;   
//   decstepperFAST(decfast);
//   shiftWrite();
//   
//   break;
//
//         case 10201:  //PN
//
//      Directiondec = 0;   
//   decstepperFASTSI(decfast);
// //  shiftWrite();
//   
//   break;
//            case 10002:  //PN
//
//      Directiondec = 0;   
//   decstepperFASTSI(decfast);
// //  shiftWrite();
//   
//   break;
//               case 11002:  //PN WS
//
//      Directiondec = 0;   
//   decstepperFASTSI(decfast);
//  // shiftWrite();
//  Direction=0;
//   fastraplusSI();
//   
//   break;
//                  case 11003:  //PN za
//
//      Directiondec = 0;   
//   decstepperFASTSI(decfast);
// //  shiftWrite();
// Direction=1;
//   fastraminusSI();
//   
//   break;
//            case 10003:  //PD
//
//      Directiondec = 1;   
//   decstepperFASTSI(decfast);
// //  shiftWrite();
//   
//   break;
//
//         case 10202:  //Pd
//
//      Directiondec = 1;   
//   decstepperFASTSI(decfast);
// //  shiftWrite();
//   
//   break;
//
//      case 1009:  //WW
//      Direction=0;
//   fastraplusSI();
////   Direction=0;
//  // mojstepperHALF(RA_SPEEDFAST);
//
//  // Directiondec = 0;   
//  // resetdecpins();
//  // shiftWrite();
//   
//   case 1010:  //ZA
//   Direction=1;
//fastraminusSI();
//  // Direction=1;
//  // mojstepper2(RA_SPEEDFAST);
//
//  // Directiondec = 0;   
//   //resetdecpins();
//   //shiftWrite();
//   
//   break;
//   
//   default:
//   
//   resetrapins();
//   resetdecpins();
// //  shiftWrite();
//   
//   break;   
//}
shiftWrite();
}

void fastraminusSI(){
   if (rarest==0){ 
   digitalWrite(MOTOR1_PWM,Powerl);
     digitalWrite(MOTOR2_PWM,Powerl);
  rarest=1;}
       currentMillis = micros();
  if(currentMillis-last_time>=rafast){
 //   motor1.onestep(BACKWARD, DOUBLE);
 Direction=1;
   stepper(1);
  timeRA=timeRA+micros()-last_time;
  last_time=micros();}
}

void fastraminus(){
   if (rarest==0){ 
   digitalWrite(MOTOR1_PWM,Powerl);
     digitalWrite(MOTOR2_PWM,Powerl);
  rarest=1;}
  if (StepsRA<ilosckrokowRA){
    
  //Serial.println("LAST ");
//  Serial.println(last_time);
       currentMillis = micros();
      // Serial.println("CURRR ");
     //  Serial.println(currentMillis);
   //    Serial.println(currentMillis-last_time); 
  if(currentMillis-last_time>rafast){ ///RA RAT 23.45
  //  motor1.onestep(BACKWARD, DOUBLE); 
  Direction=1;
  stepper(1); 
 // Serial.println(currentMillis-last_time);  
 // timeRA=timeRA+micros()-last_time;
  last_time=micros();
     
 StepsRA=StepsRA+1;}
//  if (semafor == 0){
// faker();}
 }
 else {zmiennara=1;}
}

void fastraplusSI(){
   if (rarest==0){ 
   digitalWrite(MOTOR1_PWM,Powerl);
     digitalWrite(MOTOR2_PWM,Powerl);
  rarest=1;}
       currentMillis = micros();
  if(currentMillis-last_time>=rafast){
  //  motor1.onestep(FORWARD, DOUBLE);
  Direction=0; 
 stepper(1);
 //shiftWrite();
 
  timeRA=timeRA+micros()-last_time;
  last_time=micros();}
}

//void fastraplus(){
//   if (rarest==0){ 
//   digitalWrite(MOTOR1_PWM,Powerl);
//     digitalWrite(MOTOR2_PWM,Powerl);
//  rarest=1;}
//  
//  if (StepsRA<ilosckrokowRA){
//
//       currentMillis = micros();
//  if(currentMillis-last_time>=raminus/2){
// //   motor1.onestep(FORWARD, DOUBLE); 
////   Direction=0;
//Direction=1;
//  stepperHALF(1); 
// //  Serial.println(currentMillis-last_time); 
//  timeRA=timeRA+micros()-last_time;
//  last_time=micros();
//  //   Serial.println(StepsRA);
// StepsRA=StepsRA+1;}
//  //if (semafor == 0){
//// faker();}
//}
//else {zmiennara=1;
//Serial.println(millis()-mojczas); 
//}
//}

void fastraplus(){
   if (rarest==0){ 
   digitalWrite(MOTOR1_PWM,Powerl);
     digitalWrite(MOTOR2_PWM,Powerl);
  rarest=1;}
  if (StepsRA<ilosckrokowRA){

       currentMillis = micros();
  if(currentMillis-last_time>=rafast){
 //   motor1.onestep(FORWARD, DOUBLE); 
   Direction=0;
  stepper(1); 
 //  Serial.println(currentMillis-last_time); 
  timeRA=timeRA+micros()-last_time;
  last_time=micros();
  //   Serial.println(StepsRA);
 StepsRA=StepsRA+1;}
  //if (semafor == 0){
// faker();}
}
else {zmiennara=1;}
}





void stepper(int xw){
 
  for (int x=0;x<xw;x++){
switch(Steps){

case 0:
     
     motor_output(MOTOR1_A, HIGH, RAMOTOR_PWM);  
     motor_output(MOTOR1_B, LOW, RAMOTOR_PWM);
     motor_output(MOTOR2_A, LOW, RAMOTOR_PWM);
     motor_output(MOTOR2_B, HIGH, RAMOTOR_PWM);
      
  

   break; 

   case 1:
     
    
     motor_output(MOTOR1_A, HIGH, RAMOTOR_PWM);
     motor_output(MOTOR1_B, LOW, RAMOTOR_PWM);
      motor_output(MOTOR2_A, HIGH, RAMOTOR_PWM);
      motor_output(MOTOR2_B, LOW, RAMOTOR_PWM);
   
   break; 

   case 2:
     motor_output(MOTOR1_A, LOW, RAMOTOR_PWM);   
     motor_output(MOTOR1_B, HIGH, RAMOTOR_PWM);     
     motor_output(MOTOR2_A, HIGH, RAMOTOR_PWM);
     motor_output(MOTOR2_B, LOW, RAMOTOR_PWM); 

 
   break; 

   case 3:
  
     motor_output(MOTOR1_A, LOW, RAMOTOR_PWM);
     motor_output(MOTOR1_B, HIGH, RAMOTOR_PWM);      
     motor_output(MOTOR2_A, LOW, RAMOTOR_PWM);
     motor_output(MOTOR2_B, HIGH, RAMOTOR_PWM);

   break; 
   default:
      motor_output(MOTOR2_B, LOW, 0); 
     motor_output(MOTOR2_A, LOW, 0);
     motor_output(MOTOR1_A, LOW, 0);
     motor_output(MOTOR1_B, LOW, 0);

   
   break; 
  
}
SetDirection();
}
} 

void stepperHALF(int xw){
 
  for (int x=0;x<xw;x++){
switch(StepsHALF){

case 0:
     
     motor_output(MOTOR1_A, HIGH, RAMOTOR_PWM);  
     motor_output(MOTOR1_B, LOW, RAMOTOR_PWM);
     motor_output(MOTOR2_A, LOW, RAMOTOR_PWM);
     motor_output(MOTOR2_B, HIGH, RAMOTOR_PWM);
      
  

   break; 
      case 1:
     
    
     motor_output(MOTOR1_A, HIGH, RAMOTOR_PWM);
     motor_output(MOTOR1_B, LOW, RAMOTOR_PWM);
      motor_output(MOTOR2_A, LOW, RAMOTOR_PWM);
      motor_output(MOTOR2_B, LOW, RAMOTOR_PWM);
   
   break; 

   case 2:
     
    
     motor_output(MOTOR1_A, HIGH, RAMOTOR_PWM);
     motor_output(MOTOR1_B, LOW, RAMOTOR_PWM);
      motor_output(MOTOR2_A, HIGH, RAMOTOR_PWM);
      motor_output(MOTOR2_B, LOW, RAMOTOR_PWM);
   
   break; 
      case 3:
     
    
     motor_output(MOTOR1_A, LOW, RAMOTOR_PWM);
     motor_output(MOTOR1_B, LOW, RAMOTOR_PWM);
      motor_output(MOTOR2_A, HIGH, RAMOTOR_PWM);
      motor_output(MOTOR2_B, LOW, RAMOTOR_PWM);
   
   break; 

   case 4:
     motor_output(MOTOR1_A, LOW, RAMOTOR_PWM);   
     motor_output(MOTOR1_B, HIGH, RAMOTOR_PWM);     
     motor_output(MOTOR2_A, HIGH, RAMOTOR_PWM);
     motor_output(MOTOR2_B, LOW, RAMOTOR_PWM); 

 
   break; 

      case 5:
  
     motor_output(MOTOR1_A, LOW, RAMOTOR_PWM);
     motor_output(MOTOR1_B, HIGH, RAMOTOR_PWM);      
     motor_output(MOTOR2_A, LOW, RAMOTOR_PWM);
     motor_output(MOTOR2_B, LOW, RAMOTOR_PWM);

   break; 

   case 6:
  
     motor_output(MOTOR1_A, LOW, RAMOTOR_PWM);
     motor_output(MOTOR1_B, HIGH, RAMOTOR_PWM);      
     motor_output(MOTOR2_A, LOW, RAMOTOR_PWM);
     motor_output(MOTOR2_B, HIGH, RAMOTOR_PWM);

   break; 
     case 7:
  
     motor_output(MOTOR1_A, LOW, RAMOTOR_PWM);
     motor_output(MOTOR1_B, LOW, RAMOTOR_PWM);      
     motor_output(MOTOR2_A, LOW, RAMOTOR_PWM);
     motor_output(MOTOR2_B, HIGH, RAMOTOR_PWM);

   break; 
   default:
      motor_output(MOTOR2_B, LOW, 0); 
     motor_output(MOTOR2_A, LOW, 0);
     motor_output(MOTOR1_A, LOW, 0);
     motor_output(MOTOR1_B, LOW, 0);

   
   break; 
  
}
SetDirectionHALF();
}
} 

void SetDirection(){
if(Direction==1){ Steps--;}
if(Direction==0){ Steps++; }
if(Steps>3){Steps=0;}
if(Steps<0){Steps=3;}
}
void SetDirectionHALF(){
if(Direction==1){ StepsHALF--;}
if(Direction==0){ StepsHALF++; }
if(StepsHALF>7){StepsHALF=0;}
if(StepsHALF<0){StepsHALF=7;}
}



void mojstepperHALF(unsigned long czasok)
{
 if (rarest==0){ 
 //  digitalWrite(MOTOR1_PWM,Powerl);
  //   digitalWrite(MOTOR2_PWM,Powerl);
       analogWrite(MOTOR1_PWM,Powerl);
     analogWrite(MOTOR2_PWM,Powerl);
  rarest=1;}
  currentMillis = micros();
  if(currentMillis-last_time>=czasok/RAstep){
    if (RAstep==2.0){
    //  motor1.onestep(BACKWARD, DOUBLE);   
      //} else {motor1.onestep(BACKWARD, INTERLEAVE);}
   stepperHALF(1);} else {stepper(1);} 
 //  Serial.println(currentMillis-last_time);
  //time=time+micros()-last_time;
  last_time=micros();
  }
  //   Serial.println(Stepsdec);
// Stepsdec=Stepsdec+1;}
// else{resetrapins();}
}


   


void stepperdec(int xwdec){
 
  for (int xdec=0;xdec<xwdec;xdec++){
switch(Stepsdec){

   case 0:
     motor_output(MOTOR3_A, HIGH, DECMOTOR_PWM);
      motor_output(MOTOR3_B, LOW, DECMOTOR_PWM);
        motor_output(MOTOR4_A, LOW, DECMOTOR_PWM);
     motor_output(MOTOR4_B, HIGH, DECMOTOR_PWM); 
  

   break; 

   case 1:
    
     motor_output(MOTOR3_A, HIGH, DECMOTOR_PWM);
     motor_output(MOTOR3_B, LOW, DECMOTOR_PWM);
     motor_output(MOTOR4_A, HIGH, DECMOTOR_PWM);
     motor_output(MOTOR4_B, LOW, DECMOTOR_PWM); 
  
   break; 

   case 2:
    
     motor_output(MOTOR3_A, LOW, DECMOTOR_PWM);
     motor_output(MOTOR3_B, HIGH, DECMOTOR_PWM);
     motor_output(MOTOR4_A, HIGH, DECMOTOR_PWM);
      motor_output(MOTOR4_B, LOW, DECMOTOR_PWM); 
  //   shiftWrite();
     

   break; 

   case 3:
  motor_output(MOTOR3_A, LOW, DECMOTOR_PWM); 
     motor_output(MOTOR3_B, HIGH, DECMOTOR_PWM);
     motor_output(MOTOR4_A, LOW, DECMOTOR_PWM);
     motor_output(MOTOR4_B, HIGH, DECMOTOR_PWM);


    
   break; 
   default:
      motor_output(MOTOR4_B, LOW, 0); 
     motor_output(MOTOR4_A, LOW, 0);
     motor_output(MOTOR3_A, LOW, 0);
     motor_output(MOTOR3_B, LOW, 0);
 
     

         
   break; 

  
}
SetDirectiondec();
}
}

void SetDirectiondec(){
if(Directiondec==1){ Stepsdec++;}
if(Directiondec==0){ Stepsdec--; }
if(Stepsdec>3){Stepsdec=0;}
if(Stepsdec<0){Stepsdec=3; }
}

void decstepperFake2(unsigned long czasok)
{ 
enabledecpins();

//if (Stepsdec<624000){ 
//  if(Directiondec==1){ digitalWrite (MOTOR3_PWM, LOW);}
//if(Directiondec==0){ digitalWrite (MOTOR3_PWM, HIGH); }
  currentMillisdec = micros();
  if((currentMillisdec-last_timedec>=czasok)){
    motor_output(MOTOR4_B, HIGH, 0);
 //  shiftWrite();
  // digitalWrite(MOTOR4_PWM,HIGH); 
   RAPINSTATE = 1;
//  Serial.println("hi");
//   delay(260);
//   digitalWrite(stepPin,LOW); 
 // decPINSTATE = 0;
  // delay(10);
   //digitalWrite(stepPin,LOW);
 // timedec=timedec+micros()-last_timedec;
  last_timedec=micros();}
 // last_timedec2=micros();}
  
  currentMillisdec2 = micros();
 if((currentMillisdec2-last_timedec>=50)&&(RAPINSTATE == 1)){
      motor_output(MOTOR4_B, LOW, 0);
 //  shiftWrite();
//   digitalWrite(MOTOR4_PWM,LOW);
   RAPINSTATE = 0; 
 //  Serial.println("lo");
//   Serial.println(Stepsdec);
 //Stepsdec=Stepsdec+1;
 // timedec=timedec+micros()-last_timedec;
 // last_timedec2=micros();
//// }

  }
  
  }
void decstepper(unsigned long czasok)
{ 
enabledecpins();

//if (Stepsdec<624000){ 
//  if(Directiondec==1){ digitalWrite (MOTOR3_PWM, LOW);}
//if(Directiondec==0){ digitalWrite (MOTOR3_PWM, HIGH); }
  currentMillisdec = micros();
  if((currentMillisdec-last_timedec>=czasok)){
    motor_output(MOTOR4_B, LOW, 0);
 //  shiftWrite();
  // digitalWrite(MOTOR4_PWM,HIGH); 
   RAPINSTATE = 1;
//  Serial.println("hi");
//   delay(260);
//   digitalWrite(stepPin,LOW); 
 // decPINSTATE = 0;
  // delay(10);
   //digitalWrite(stepPin,LOW);
 // timedec=timedec+micros()-last_timedec;
  last_timedec=micros();}
 // last_timedec2=micros();}
  
  currentMillisdec2 = micros();
 if((currentMillisdec2-last_timedec>=50)&&(RAPINSTATE == 1)){
      motor_output(MOTOR4_B, LOW, 0);
 //  shiftWrite();
//   digitalWrite(MOTOR4_PWM,LOW);
   RAPINSTATE = 0; 
 //  Serial.println("lo");
//   Serial.println(Stepsdec);
 //Stepsdec=Stepsdec+1;
 // timedec=timedec+micros()-last_timedec;
 // last_timedec2=micros();
//// }

  }
  
  }


void decstepperFake(unsigned long czasok)
{ 
enabledecpins();

//if (Stepsdec<624000){ 
//  if(Directiondec==1){ digitalWrite (MOTOR3_PWM, LOW);}
//if(Directiondec==0){ digitalWrite (MOTOR3_PWM, HIGH); }
  currentMillisdec = micros();
  if((currentMillisdec-last_timedec>=czasok)){
    motor_output(MOTOR4_B, HIGH, 0);
 //  shiftWrite();
  // digitalWrite(MOTOR4_PWM,HIGH); 
   RAPINSTATE = 1;
//  Serial.println("hi");
//   delay(260);
//   digitalWrite(stepPin,LOW); 
 // decPINSTATE = 0;
  // delay(10);
   //digitalWrite(stepPin,LOW);
 // timedec=timedec+micros()-last_timedec;
  last_timedec=micros();}
 // last_timedec2=micros();}
  
  currentMillisdec2 = micros();
 if((currentMillisdec2-last_timedec>=50)&&(RAPINSTATE == 1)){
      motor_output(MOTOR4_B, LOW, 0);
 //  shiftWrite();
//   digitalWrite(MOTOR4_PWM,LOW);
   RAPINSTATE = 0; 
 //  Serial.println("lo");
//   Serial.println(Stepsdec);
 //Stepsdec=Stepsdec+1;
 // timedec=timedec+micros()-last_timedec;
 // last_timedec2=micros();
//// }

  }
  
  }



void decstepperFAST(unsigned long czasok)
{ 

if (Stepsdec<ilosckrokowdec*microstepp){ 
  
enabledecpins();
 //if(Directiondec==1){ digitalWrite (MOTOR3_PWM, LOW);}
//if(Directiondec==0){ digitalWrite (MOTOR3_PWM, HIGH); }
  currentMillisdec = micros();
  if((currentMillisdec-last_timedec>=czasok)){
    motor_output(MOTOR4_B, HIGH, 0);
 //  shiftWrite();
  // digitalWrite(MOTOR4_PWM,HIGH); 
   RAPINSTATE = 1;
   
//   Serial.println("hi");
//   delay(260);
//   digitalWrite(stepPin,LOW); 
 // decPINSTATE = 0;
  // delay(10);
   //digitalWrite(stepPin,LOW);
 // timedec=timedec+micros()-last_timedec;
  last_timedec=micros();}
 // last_timedec2=micros();}
  
  currentMillisdec2 = micros();
 if((currentMillisdec2-last_timedec>=50)&&(RAPINSTATE == 1)){
      motor_output(MOTOR4_B, LOW, 0);
 //  shiftWrite();
//   digitalWrite(MOTOR4_PWM,LOW);
   RAPINSTATE = 0; 
 //  Serial.println("lo");
 //  Serial.println(Stepsdec);
 Stepsdec=Stepsdec+1;
 // timedec=timedec+micros()-last_timedec;
 // last_timedec2=micros();
}

  }
  else { 
  
enabledecpins();
 // if(Directiondec==1){ digitalWrite (MOTOR3_PWM, LOW);}
//if(Directiondec==0){ digitalWrite (MOTOR3_PWM, HIGH); }
  currentMillisdec = micros();
  if((currentMillisdec-last_timedec>=czasok)){
    motor_output(MOTOR4_B, LOW, 0);
 //  shiftWrite();
  // digitalWrite(MOTOR4_PWM,HIGH); 
   RAPINSTATE = 1;
   fakar=fakar+2;
  // fakar2=fakar2+2;
//   Serial.println("hi");
//   delay(260);
//   digitalWrite(stepPin,LOW); 
 // decPINSTATE = 0;
  // delay(10);
   //digitalWrite(stepPin,LOW);
 // timedec=timedec+micros()-last_timedec;
  last_timedec=micros();}
 // last_timedec2=micros();}
  
  currentMillisdec2 = micros();
 if((currentMillisdec2-last_timedec>=50)&&(RAPINSTATE == 1)){
      motor_output(MOTOR4_B, LOW, 0);
 //  shiftWrite();
//   digitalWrite(MOTOR4_PWM,LOW);
   RAPINSTATE = 0;
   fakar=fakar*23; 
   fakar2= fakar2+2;
 //  Serial.println("lo");
 //  Serial.println(Stepsdec);
 Stepsdec=Stepsdec+1;
 // timedec=timedec+micros()-last_timedec;
 // last_timedec2=micros();
}

  }
  
  }











  

void decstepperFASTSI(unsigned long czasok)
{ 


enabledecpins();
//  if(Directiondec==1){ digitalWrite (MOTOR3_PWM, LOW);}
//if(Directiondec==0){ digitalWrite (MOTOR3_PWM, HIGH); }
  currentMillisdec = micros();
  if((currentMillisdec-last_timedec>=czasok)){
    motor_output(MOTOR4_B, HIGH, 0);
//   shiftWrite();
  // digitalWrite(MOTOR4_PWM,HIGH); 
   RAPINSTATE = 1;
//   Serial.println("hi");
//   delay(260);
//   digitalWrite(stepPin,LOW); 
 // decPINSTATE = 0;
  // delay(10);
   //digitalWrite(stepPin,LOW);
 // timedec=timedec+micros()-last_timedec;
  last_timedec=micros();}
 // last_timedec2=micros();}
  
  currentMillisdec2 = micros();
 if((currentMillisdec2-last_timedec>=50)&&(RAPINSTATE == 1)){
      motor_output(MOTOR4_B, LOW, 0);
 //  shiftWrite();
//   digitalWrite(MOTOR4_PWM,LOW);
   RAPINSTATE = 0; 
 //  Serial.println("lo");
  
 // timedec=timedec+micros()-last_timedec;
 // last_timedec2=micros();
}

  
  
  }  
  
  

void resetdecpins() {
  if (decrest==1) { 
 //  motor_output(MOTOR4_B, HIGH, 0); 
 //    motor_output(MOTOR4_A, HIGH, 0);
     motor_output(MOTOR3_A, HIGH, 0);
 //    motor_output(MOTOR3_B, HIGH, 0);
 
   shiftWrite();
     decrest = 0;
     
     }}
     void enabledecpins() {
  if (decrest==0) { 
    // motor_output(MOTOR4_B, LOW, 0); 
    // motor_output(MOTOR4_A, LOW, 0);
     motor_output(MOTOR3_A, LOW, 0);
   //  motor_output(MOTOR3_B, LOW, 0);
 
  // shiftWrite();
     decrest = 1;
     
     }}


     
     
 void mic16(int mstepping) {
            switch(mstepping){
              
            case 1:
  digitalWrite(MOTOR4_PWM,LOW);
 //    motor_output(MOTOR4_B, LOW, 0); 
     motor_output(MOTOR4_A, LOW, 0);
   //  motor_output(MOTOR3_A, LOW, 0);
     motor_output(MOTOR3_B, LOW, 0);
 
 //  shiftWrite();
   
     break;
    case 8:
  digitalWrite(MOTOR4_PWM,LOW);
 //    motor_output(MOTOR4_B, LOW, 0); 
     motor_output(MOTOR4_A, HIGH, 0);
   //  motor_output(MOTOR3_A, LOW, 0);
     motor_output(MOTOR3_B, HIGH, 0);
 
 //  shiftWrite();
   
     
     break;
    case 4:
  digitalWrite(MOTOR4_PWM,LOW);
 //    motor_output(MOTOR4_B, LOW, 0); 
     motor_output(MOTOR4_A, HIGH, 0);
   //  motor_output(MOTOR3_A, LOW, 0);
     motor_output(MOTOR3_B, LOW, 0);
 
 //  shiftWrite();
   
     
     break;
    case 2:
  digitalWrite(MOTOR4_PWM,LOW);
 //    motor_output(MOTOR4_B, LOW, 0); 
     motor_output(MOTOR4_A, LOW, 0);
   //  motor_output(MOTOR3_A, LOW, 0);
     motor_output(MOTOR3_B, HIGH, 0);
 
 //  shiftWrite();
   
     
     break;
    case 16:
  digitalWrite(MOTOR4_PWM,HIGH);
//    motor_output(MOTOR4_B, HIGH, 0); 
  motor_output(MOTOR4_A, HIGH, 0);
   //  motor_output(MOTOR3_A, HIGH, 0);
     motor_output(MOTOR3_B, HIGH, 0);
 
//   shiftWrite();
   break;}
     
     }
void resetrapins() {
  if (rarest==1) { 
 motor_output(MOTOR1_A, LOW, 0); 
     motor_output(MOTOR1_B, LOW, 0);
     motor_output(MOTOR1_A, LOW, 0);
     motor_output(MOTOR1_B, LOW, 0);
     digitalWrite(MOTOR1_PWM,LOW);
     digitalWrite(MOTOR2_PWM,LOW);
     rarest = 0;
     
     }
 
   
}





void motor_output (int output, int high_low, int speed)
{
 
  int motorPWM;

  switch (output)
  {
  case MOTOR1_A:
  case MOTOR1_B: 
    motorPWM = MOTOR1_PWM;
        break;
  case MOTOR2_A:
  case MOTOR2_B:
    motorPWM = MOTOR2_PWM;
    break;
  case MOTOR3_A:
 case MOTOR3_B:

  if (speed == 0)
  {
//  digitalWrite (MOTOR3_PWM, LOW);

  }
  else 
  {
//  digitalWrite (MOTOR3_PWM, HIGH);
 
  }
    break;
  case MOTOR4_A: 
  case MOTOR4_B:

  if (speed == 0)
  {
 // digitalWrite (MOTOR4_PWM, LOW);

  }
  else 
  {
 // digitalWrite (MOTOR4_PWM, HIGH);
  
  }
  
  
    break;
  default:
    // Use speed as error flag, -3333 = invalid output.
    speed = -3333;
    break;
  }

 
    bitWrite(latch_copy, output, high_low);
    


}



void shiftWrite()
{

  shiftOut(MOTORDATA, MOTORCLK, MSBFIRST, latch_copy);
//  delayMicroseconds(5);    // For safety, not really needed.
  digitalWrite(MOTORLATCH, HIGH);
// delayMicroseconds(5);    // For safety, not really needed.
  digitalWrite(MOTORLATCH, LOW);
  
  
}




  
