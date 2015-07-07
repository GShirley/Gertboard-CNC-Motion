// DC Motor controlled by a Potentiometer with wiper connected to Analog pin Gertboard PC3 with zero speed mid position
// and forwards and backwards at the extremes.
/* Implemented with Raspberrypi and the original GertBoard */
/* Uses Gertboard fitted with ATmega328 */
/* and Gertboard PWM Bi Directional motor control for ROHM BD6222HFP motor controller */
/* Motor MFA/Como Drills Pt 719-RE280  http://www.maplin.co.uk/multi-ratio-motorgearboxes-25246 */
// Potentiometer also connected to 0V and 3.3 V for gertboard
/* Gertboard R2 manual http://www.farnell.com/datasheets/1683444.pdf  */
// Created by Graham Shirley 9th August 2013
/* Copyright Graham Shirley 14th June 2015  MIT License */


#define PWMA 3      // Gertboard connection make PD3 to J5 MotA PWM modulation signal with jump lead 
#define DIRA 12     // Gertboard connection make PB4 to J5 MotB Direction modulation signal with jump lead
#define analogPin 3   // potentiometer wiper connected to analog pin PC3 other connections GND and 3V3
int v = 0;         // variable to store the read value
byte fwd=1;         // motor direction -forwards or backwards
#define LOW 0
#define HIGH 1

void setup()
{
  pinMode(PWMA, OUTPUT);   // sets the pin as output for Speed
  pinMode(DIRA,OUTPUT);    // sets the pin as output for Direction
  
// PWM Adjustment  http://playground.arduino.cc//Main/TimerPWMCheatsheet
// Timer 2 for PWMA and PWMB Adjust the PWM frequency Default is 366 Hz 
// TCCR2B = TCCR2B & 0b11111000 |0x04;  // (Default setting 366 Hz for 12 MHz clock)
// TCCR2B = TCCR2B & 0b11111000 | 0x02;  //(Default setting 2929 Hz for 12 MHz Clock) 
// TCCR2B = TCCR2B & 0b11111000 | 0x01;  //(Default setting 23437 Hz for 12 MHz Clock)
}

void loop()
{
  v = analogRead(analogPin);   // read the input pin
  v=v-512;  // Correct so Zero speed is mid position on the pot
// analogRead values go from 0 to 1023, analogWrite values from 0 to 255
 if (v <= 0)
      { 
      // map -511 to 0 to going "backwards" -- --511  means
      // go backwards max fast (v sent to PWM is near 255), as we increase 
      // towards -1, motor speed slows, and at 0 the  motor is stopped
      // (v sent to PWM is near 0)
      v = (-v>>1);
      // we want v near 0 to mean motor slow/stopped and v near -512 to
      // mean motor going "backwards" fast
      if (fwd)
          { // going in the wrong direction
            // reverse polarity
            digitalWrite(DIRA,HIGH);
            // On LM629  high v gives us a high speed.
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
