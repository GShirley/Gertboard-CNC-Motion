/* Rotary Grey Code encoder read example */
/* https://www.sparkfun.com/products/9117  */
/* http://www.circuitsathome.com/mcu/reading-rotary-encoder-on-arduino */
// Connect Encoder A to BUF1, this has 10 KOhm pull up already.
// Place jumper B1 In and connect B1 to PC2
// Connect Encoder B to BUF2, this has 10 KOhm pull up already.
// Place jumper B2 In and connect B2 to PC3
// Works with Farnell Digital Encoder 100 5754 GRAYHILL 62P22L6S
// Connect Encoder V in via 150 Ohm resistor in the supply to 3.3V see encoder data sheet.
// Use Minicom to display the results. See the Gertboard Manual page 48 for how to do this.
#define ENC_A 3  // Gertboard PC2 to encoder A signal
#define ENC_B 4  // Gertboard PC3 to encoder B signal
/* PINB(digital pin 8 to 13)
   PINC(analog input pins)
   PIND(digital pins 0 to 7)
   Modified by Graham Shirley 9th Aug 2013
   Copyright Graham Shirley 14th June 2015  MIT License
*/
#define ENC_PORT PINC
 
void setup()
{
  /* Setup encoder pins as inputs */
//  pinMode(ENC_A, INPUT);
//  digitalWrite(ENC_A, HIGH);
//  pinMode(ENC_B, INPUT);
//  digitalWrite(ENC_B, HIGH);
  Serial.begin (9600);
  Serial.println("Start");
}
 
void loop()
{
 static long counter = 0;      //this variable will be changed by encoder input
 int8_t tmpdata;
 /**/
  tmpdata = read_encoder();
  if( tmpdata ) {
    Serial.print("Counter value: ");
    Serial.println(counter, DEC);
    counter += tmpdata;
  }
}
 
/* returns change in encoder state (-1,0,1) */
int8_t read_encoder()
{
  static int8_t enc_states[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
  static uint8_t old_AB = 0;
  /**/
  old_AB <<= 2;                   //remember previous state
  old_AB |= ( ENC_PORT>>2 & 0x03 );  //add current state
  return ( enc_states[( old_AB & 0x0f )]);
}
