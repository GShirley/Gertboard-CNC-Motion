/* Closed Loop DC motor motion with polynomial step shape advance and Digital Lead Lag Controller*/
/* Typical application NC control, Indexing machine drive */
/* Implemented with Raspberrypi and the original GertBoard */
/* polynomial motion stored in an array and data read in real time*/
/* Encoder used Farnell Part 100 5754 GRAY HILL 62P22L6S */
/* Uses Gertboard fitted with ATmega328 for closed loop postion control */
/* and Gertboard PWM Bi Directional motor control for ROHM BD6222HFP motor controller */
/* Motor MFA/Como Drills Pt 719-RE280  http://www.maplin.co.uk/multi-ratio-motorgearboxes-25246 */
/* Encoder connected to the motor via 4:1 reduction gear set - This provides enough torque to drive the pot detent */
/* Encoder read modified from http://www.circuitsathome.com/mcu/reading-rotary-encoder-on-arduino */
/* If the motor jumps on startup and the Gertboard H bridge gets hot quickly you have positive feedback 
/* - swap the motor cable connections over to reverse directions and attain correct operation */
/* Gertboard R2 manual http://www.farnell.com/datasheets/1683444.pdf  */
/* Copyright Graham Shirley 14th June 2015  MIT License */

#include <EEPROM.h>

// Encoder connection
// Connect Encoder A to BUF1, this has 10 KOhm pull up already.
// Place jumper B1 In and connect B1 to PC2
// Connect Encoder B to BUF2, this has 10 KOhm pull up already.
// Place jumper B2 In and connect B2 to PC3
// Works with Farnell Digital Encoder 100 5754 GRAYHILL 62P22L6S
// Connect Encoder V in via 150 Ohm resistor in the supply to 3.3V see encoder data sheet.
// Use Minicom to display the results. See the Gertboard Manual page 48 for how to do this.
#define ENC_A 3 // Gertboard PC2 to encoder A signal
#define ENC_B 4 // Gertboard PC3 to encoder B signal
/* Possible encoder connections options on Gertboard ATMEGA328/Arduino Uno
   PINB(digital pin 8 to 13)
   PINC(analog input pins)
   PIND(digital pins 0 to 7)
   Modified by Graham Shirley 10th August 2013
*/
#define ENC_PORT PINC
// ATmega 328 on Gertboard
#define PWMA 3  // sets pin number as output for Speed
#define DIRA 12 // sets pin number as output for Direction
#define LOW 0
#define HIGH 1

//  Read Encoder Position set up
    int8_t tmpdata;
    
  long int DemandP = 0;  // Demanded position from motion generation.
  long int DemandPInitial = 0; // DemandP initial condition motion in array
  long int DemandPInitial2 = 0; // DemandP initial condition speed
  long int ActualP =0;
  int error=0;  // Tracking error
  int vd_last=0; // Last output to motor
  int error_last=0; // Last tracking error
  const int A=200;  // MAX A=255 - LEAD LAG FILTER ZERO  eg 200
  const int B=5;    // MAX B=255 - LEAD LAG FILTER POLE  eg 5
  const int K=128;    // LEAD LAG FILTER FORWARD PATH GAIN eg 64
 // NOTE on setting up the Digital Filter controller gains :-
// Set gains for your motor. Change the figures above and compile upload the program.
// test and read just until a stiff no oscillatory selection is found. 
// Start with figures guess figures of A,B and K. Ajust each individually
// When the best condition for is found for this adjust a different parameter
// if the motor runs away or the H bridge gets hot very quickly you may need 
// to reverse the motor wires to change motor feedback direction.
  // setup I/O
  signed int  v;
  signed int vd;
// Calculate motor position for cycle position N on 3-4-5-6-7-R Polynominal Law Ref Book "Cams for Industry" by John Reeve page 278
// Other motion profiles - Mod Trap, Mod Sine, Cycloidal..etc can also programmed see the book
const unsigned int NP=512 ; // The max number of points in the motion profile. Want more use flash.h
const int Amplitude=320; // Amplitude of the step forwards 20 encoder revs
const int Period=3000;  // Time in mS to complete one cycle of the motion
unsigned int i;  // Look up indicator in the table
unsigned int i_old=0; // To detect wrap around 
unsigned long int StartTime;  // ATmega Counter time at start of loop


float Dist;    // Motor position demand at N in Cycle of NP
int DistInt;  // copy of Dist but as an integer
byte DistInt1; // First byte of Dist Int for EEPROM
byte DistInt2; // Second byte of Dist Int for EEPROM
float Cyclepnt;  // Point in the Cycle of NP
unsigned int N=0;  // Cycle counter
int fwd=1;           // Define motor forwards output

void setup()
{
  Serial.begin (9600);
// Make connections in Uploading Sketches using SPI Bus on Page 43 of the manual  
// To see this use Minicom- See Gertboard Manual Page 48
// In Linux window use on Pi - sudo minicom ama0
  Serial.println("Start motion array calc");
/* Setup encoder pins as inputs uncomment is using Digital inputs*/
//  4 lines Not required for Analog -already inputs but needed for digital !
//  pinMode(ENC_A, INPUT);
//  digitalWrite(ENC_A, HIGH);
//  pinMode(ENC_B, INPUT);
//  digitalWrite(ENC_B, HIGH);
// Set up for Ardunio Motor Shield
  pinMode(PWMA, OUTPUT);   // sets the pin as output for Speed
  pinMode(DIRA,OUTPUT);    // sets the pin as output for Direction
  analogWrite(PWMA,0);   // sets motor to zero speed
  int fwd=1;           // Define motor forwards output  

// PWM Adjustment  http://playground.arduino.cc//Main/TimerPWMCheatsheet
// Timer 2 for PWMA and PWMB Adjust the PWM frequency Default is 366 Hz 
// TCCR2B = TCCR2B & 0b11111000 |0x04;  // (Default setting 366 Hz for 12 MHz clock)
// TCCR2B = TCCR2B & 0b11111000 | 0x02;  //(Default setting 2929 Hz for 12 MHz Clock) 
TCCR2B = TCCR2B & 0b11111000 | 0x01;  //(Default setting 23437 Hz for 12 MHz Clock)

// Generate Motion look up table
// Generate Cam law curve data set up a table in EEPROM.

for (N=0;N<256;N++)
  {
    Cyclepnt=(float)N/(float)NP;
    Dist= Amplitude*(27*pow(Cyclepnt,3)-113*pow(Cyclepnt,4)+224.4*pow(Cyclepnt,5)-228.8*pow(Cyclepnt,6)+96*pow(Cyclepnt,7));
    // Convert Dist to an integer for storage
    DistInt=Dist;
    // Divide DistInt into two parts for storage
    DistInt1=DistInt;
    DistInt2=(DistInt>>8);
    EEPROM.write(N,DistInt1);
    EEPROM.write(N+512,DistInt2);
    // Motion is symetrical about the mid point
    DistInt=Amplitude-DistInt;
    DistInt1=DistInt;
    DistInt2=(DistInt>>8);
    EEPROM.write(NP-1-N,DistInt1);
    EEPROM.write((NP-1-N+512),DistInt2);
//    Serial.print(N,DEC);
//    Serial.print("\t");
//  Serial.print(DistInt1,DEC);
//  Serial.print("\t");
//    Serial.print(DistInt2,DEC);
//  Serial.print("\t");
//    Serial.print(DistInt,DEC);
//    Serial.println("");
  }
  
 for (N=0;N<NP;N++)
  {
    DistInt1=EEPROM.read(N);
    DistInt2=EEPROM.read(N+512);
    DistInt=(DistInt2<<8)|DistInt1;    // Combine DistInt1 and DistInt2 to form original DistInt
    Serial.print(N,DEC);
    Serial.print("\t");
//    Serial.print(DistInt1,DEC);
//    Serial.print("\t");
 //     Serial.print(DistInt2,DEC);
//    Serial.print("\t");
   Serial.print(DistInt,DEC);
   Serial.println("");
  }  
  N=0;  //Reset counter
  Serial.println("Starting now control ..");
  DemandPInitial=DemandP;   // Start at current position Motion Profile
  DemandPInitial2=read_encoder();   // Start at current position speed control
  StartTime=millis();  // Get counter time at start
}
 
void loop()
{
//  Check current time and update position
    unsigned long currentTimemS =millis();
    i = ((currentTimemS-StartTime)%Period)*512 / Period; //Determine positon in loop up table 512 period of the table
    // Detect table wrap around and increment forward one cycle.
    if (i>512) i=512;
    if (i<i_old)
        { DemandPInitial = DemandPInitial+Amplitude;
        }
    i_old=i;
//  Read Encoder Position and maintain 32 bit counter.
    tmpdata = read_encoder();
//    if( tmpdata ) 
//       {
//        Serial.print("Counter value: ");
//        Serial.println(ActualP, DEC);
//         }
    ActualP += tmpdata; // Change direction += or -=
    
// Look up Demand Position
      // Read Demand position
      DistInt1=EEPROM.read(i);
      DistInt2=EEPROM.read(i+512);
      DistInt=(DistInt2<<8)|DistInt1;    // Combine DistInt1 and DistInt2 to form original DistInt   
      DemandP=DistInt+DemandPInitial;  // Follows table in motion
 //   Alternative motion demands comment out lines as required.     
 //     DemandP=DemandPInitial2;  // Hold a stationary position useful for setting gains and checking feedback direction
 //   DemandP=DemandPInitial2+(currentTimemS-StartTime)/62;  // Runs at constant speed
  //     DemandP=DemandPInitial2; // Runs at constant speed 1 step per 50 loop cycle ( include 7 followings lines)
  //     if(N<=50)
  //     ++N;
 //      else
  //     {
 //        N=0;
 //        ++DemandPInitial2;
 //      }
 //       Serial.println(i, DEC);
 //        delay(100);
      Controller(ActualP,DemandP);
}

void Controller(long int ActualP, long int DemandP)
{
// Calculate error and apply feedback controller filter correction
     error=int(ActualP-DemandP);
     // calculate LEAD LAG DIGITAL FILTER d(z)=K(z-A/256)/4(z+B/256)
     vd = (K*error-K*A*error_last/256-B*vd_last/64)/4;
     // Limit v - output to motor ATMEGA 328 output limits.
     v=vd;
     if (vd> 511) v=511;
     if (vd<-511) v=-511;
     vd_last=vd;
     error_last=error;
//     Serial.println(error,DEC);
//     delay (500);
     DriveMotor(v);
}

// H Bridge driver for Gertboard PWM Bi Directional motor control for ROHM BD6222HFP motor controller. 
// See page 27 of Gertboard manual
// These are the connections:
// J5 MOTA TO  PD3 (Arduino pin 3) carries PWM
// J5 MOTB TO PB4 (Arduino pin 12) carries direction
// + of external power source ---  MOT+ on screw terminal
// ground of external power source  --- GND on screw terminal
// one wire for your motor in MOTA motor screw terminal
// the other wire for your motor  in -MOTB motor screw terminal
  
void DriveMotor(int v)
{
//    Serial.println(v,DEC);
//    delay(1000);
if (v <= 0)
      { 
      // map -511 to 0 to going "backwards" -- --511  means
      // go backwards max fast (v sent to PWM is near 255), as we increase 
      // towards -1, motor speed slows, and at 0 the  motor is stopped
      // (v sent to PWM is near 0)
      v = (-v>>1);
      // we want v near 0 to mean motor slow/stopped and v near -511 to
      // mean motor going "backwards" fast
      if (fwd)
          { // going in the wrong direction
            // reverse polarity
            digitalWrite(DIRA,HIGH);
            // On ROHM BD6222HFP  high v gives us a low speed in this direction. 
            analogWrite(PWMA,511-v); 
            fwd = 0;
          }
      else
        analogWrite(PWMA,511-v);
     }
    else
         {   
           // map A/D value of 0 to 511 to going "forwards" -- at 0
           // motor is stopped (v sent to PWM is near 0), as we increase A/D value
           // motor speed increases (in the "forwards" direction), and when A/D
           // value is at 511 (Max forwards) , we send PWM a
           // value near 255 so it goes very fast "forwards".
           v = (v>>1);
           if (!fwd)
              { // going in the wrong direction
                // reverse polarity
                digitalWrite(DIRA,LOW);
                // Now normal polarity works for us: 
                // With a low v sent to PWM we get a low duty cycle, power
                // is off most of the time, and since motor b input is low this
                // means a slow motor; when v goes to near 255 we get a high duty
                // cycle which means power on most of the time which results in
                // motor going quickly
                analogWrite(PWMA,v);
                fwd = 1;
             }
      else
        analogWrite(PWMA,v);
         }
}

/* returns change in encoder state (-1,0,1) */
int8_t read_encoder()
{
  static int8_t enc_states[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
  static uint8_t old_AB = 0;
  /**/
  old_AB <<= 2;                   //remember previous state
  // old_AB |= ( ENC_PORT & 0x03 );  //add current state from pins D8,D9 or A0.A1
  old_AB |= ( ENC_PORT>>2 & 0x03 );  // add current state from pins A2,A3
  return ( enc_states[( old_AB & 0x0f )]);
}

 
