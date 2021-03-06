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
//SYSTEM_MODE(SEMI_AUTOMATIC);

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
  void determine_code_location(void);
*/

void setup() {
  Serial.begin(57600);
  //while( !Serial.isConnected() ) // wait for Host to open serial port
  waitFor(Serial.isConnected, 15000); 

  /* BLUE DEBUG LED */
  pinMode(DEBUG_LED, OUTPUT);
  /* WHITE LED ? DIGITAL : PWM */
  pinMode(WHITE_LED, OUTPUT);
  /* RGB LED */
  pinMode(R_LED, OUTPUT);
  pinMode(G_LED, OUTPUT);
  pinMode(B_LED, OUTPUT);

  /* INPUT: PUSH BUTTON */
  //pinMode(B_TN, INPUT); // REQUIRE PULLUP OR PULLDOWN RESISTOR
  pinMode(B_TN, INPUT_PULLUP); // UTILISE BUILTIN PULLUP RESISTOR - EXPECT LOW */
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

  if(Serial.available()) {
    String _t;

    while(Serial.available()) {
    _t += (char)Serial.read();
    }

    Serial.print("Received: ASCII");
    Serial.println(_t.c_str());
  }
  

  /* EXTERNAL 10K OHM RESISTOR */
  if(digitalRead( B_TN ) == LOW) {
    determine_code_location();
    b_tn++;
    delay(125);
    //Serial.println(b_tn);
    Serial.println("B_TN ACTIVE");

    analogWrite(WHITE_LED, 255);
  }

digitalWrite(WHITE_LED, 0);
}