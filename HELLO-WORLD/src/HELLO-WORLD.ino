/*
 * Project HELLO-WORLD
 * Description: Introduction to Particle Argon
 */

#include <Particle.h>

/* 
  THESE ARE PARTICLE SPECIFIC PARAMETERS APPLIED AT CODE RUNTIME
  RUN ALL PARTICLE CLOUD COMMUNICATION IN SEPARATE THREAD 
*/
SYSTEM_THREAD(ENABLED);
/* HOW TO CONNECT TO WiFi & INTERNET: AUTOMATIC, SEMI_AUTOMATIC, MANUAL */
SYSTEM_MODE(SEMI_AUTOMATIC);

#define DEBUG_LED D7 // SMALL BLUE LED NEXT USB CONNECTOR (RIGHT OF USB)
#define WHITE_LED D2 // GOOD OL' LED
#define R_LED     D6 // RED LED
#define G_LED     D5 // GREEN LED
#define B_LED     D4 // BLUE LED
#define LDR       A0 // ANALOG_IN = ( 0 ... 4095 ) => 2^12 bits.
#define B_TN      D0 // MOMENTARY BUTTON

/* SOME VARIABLES */
uint8_t b_tn = -1; // GLOBAL :(

/* 
  FORWARD DECLARATIONS OF FUNCTIONS
  void determine_code_location();
*/

void setup() {
  Serial.begin(57600);
  while( !Serial.isConnected() ) // wait for Host to open serial port
  //waitFor(Serial.isConnected, 15000); 

  /* BLUE DEBUG LED */
  pinMode(DEBUG_LED, OUTPUT);
  /* WHITE LED ? DIGITAL : PWM */
  pinMode(WHITE_LED, OUTPUT);
  /* RGB LED */
  pinMode(R_LED, OUTPUT);
  pinMode(G_LED, OUTPUT);
  pinMode(B_LED, OUTPUT);

  /* INPUT: PUSH BUTTON */
  pinMode(B_TN, INPUT); // REQUIRE PULLUP OR PULLDOWN RESISTOR
  /* pinMode(B_TN, INPUT_PULLUP); // UTILISE BUILTIN PULLUP RESISTOR - EXPECT LOW */
  /* pinMode(B_TN, INPUT_PULLDOWN); // UTILISE BUILTIN PULLDOWN RESISTOR - EXPECT HIGH */

  delay(5); // Force Serial.println in void setup()
  Serial.println("Completed void setup");
  Serial.println(sizeof(b_tn));
  
}

void determine_code_location() {
  digitalWrite(DEBUG_LED, HIGH);
  delay(125);
  digitalWrite(DEBUG_LED, LOW);
  delay(125);
}

void loop() {

  /* EXTERNAL 10K OHM RESISTOR */
  if(digitalRead( B_TN ) == HIGH) {
    determine_code_location();
    b_tn++;
    delay(125);
    Serial.println(b_tn);
    Serial.println("B_TN ACTIVE");
  }
  
  switch(b_tn % 5) {
    
    case 0: 
    analogWrite(G_LED, 0, 100); 
    analogWrite(B_LED, 0, 100); 
    analogWrite(WHITE_LED, 0, 100);
    analogWrite(R_LED, map(analogRead( LDR ), 0, 4095, 0, analogRead( LDR )/16 ), 100); 
    break;
    case 1: 
    analogWrite(R_LED, 0, 100); 
    analogWrite(B_LED, 0, 100); 
    analogWrite(WHITE_LED, 0, 100);
    analogWrite(G_LED, map(analogRead( LDR ), 0, 4095, 0, analogRead( LDR )/16 ), 100); 
    break;
    case 2: 
    analogWrite(G_LED, 0, 100); 
    analogWrite(R_LED, 0, 100);
    analogWrite(WHITE_LED, 0, 100); 
    analogWrite(B_LED, map(analogRead( LDR ), 0, 4095, 0, analogRead( LDR )/16 ), 100); 
    break;
    case 3: 
    analogWrite(WHITE_LED, 0, 100);
    analogWrite(R_LED, map(analogRead( LDR ), 0, 4095, 0, analogRead( LDR )/16 ), 100); 
    analogWrite(G_LED, map(analogRead( LDR ), 0, 4095, 0, analogRead( LDR )/16 ), 100); 
    analogWrite(B_LED, map(analogRead( LDR ), 0, 4095, 0, analogRead( LDR )/16 ), 100); 
    break;
    default: 
    analogWrite(G_LED, 0, 100); 
    analogWrite(R_LED, 0, 100); 
    analogWrite(B_LED, 0, 100); 
    analogWrite(WHITE_LED, analogRead( LDR )/16, 100);
  }

  if(b_tn > 255) b_tn = -1;
}