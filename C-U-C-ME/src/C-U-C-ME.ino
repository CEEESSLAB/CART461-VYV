/*
 * Project C-U-C-ME
 * Description: Fully automated Particle Cloud 
 * Author:
 * Date:
 */
 
#include <Particle.h>

/* THESE ARE PARTICLE SPECIFIC PARAMETERS APPLIED AT CODE RUNTIME */
/* ALWAYS RUN PARTICLE CLOUD COMMUNICATION IN SEPARATE THREAD */ 
SYSTEM_THREAD(ENABLED);

/* HOW TO CONNECT TO WiFi & INTERNET: AUTOMATIC, SEMI_AUTOMATIC, MANUAL */
//SYSTEM_MODE(AUTOMATIC); // DONE BY DEFAULT
//SYSTEM_MODE(SEMI_AUTOMATIC); // CONTROL WHEN & HOW TO CONNECT TO PARTICLE CLOUD (WIFI RADIO ON)
//SYSTEM_MODE(MANUAL); // CONTROL WHEN & HOW TO CONNECT TO WIFI NETWORK

#define DEBUG_LED D7 // SMALL BLUE LED NEXT USB CONNECTOR (RIGHT OF USB)
#define WHITE_LED D2 // GOOD OL' LED
#define R_LED     D6 // RED LED
#define G_LED     D5 // GREEN LED
#define B_LED     D4 // BLUE LED
#define LDR       A0 // ANALOG_IN = ( 0 ... 4095 ) => 2^12 bits.
#define B_TN      D0 // MOMENTARY BUTTON

/* PARTICLE CLOUD VARIABLES */
int whiteled = 1;

void setup() {

  /* PARTICLE CLOUD INTENTIONS ALWAYS DECLARED IN VOID SETUP */
  /* 
    PARTICLE FUNCTIONS 'n' VARIABLES ARE SPECIAL CLOUD API OPERATIONS - 
    MORE ABOUT FUNCTIONS AND VARIABLES IN A FUTURE WORKSHOP (#3). 
  */
  //Particle.function("whiteled", whiteled);
  Particle.variable("controlWhiteLED", whiteled);
  Particle.variable("attributedTo", "ME @ N.D.G", STRING);

  /* PARTICLE PUBLISH 'n' SUBSCRIBE ARE MULTICAST OPERATIONS - FULLY CONNECTED 
    SIMPLY SUBSCRIBE TO A KNOWN DATA SOURCE OR PUBLISH DATA AS SOURCE
    THE ONLY CAVEAT - ONE (1) MSG PER SECOND OVER THE CLOUD 
  */
  Particle.subscribe("others-ldr", others_ldr);

  Serial.begin(57600);
  while( !Serial.isConnected() ) // wait for Host to open serial port

  /* BLUE DEBUG LED */
  pinMode(DEBUG_LED, OUTPUT);
  /* WHITE LED ? DIGITAL : PWM */
  pinMode(WHITE_LED, OUTPUT);

  /* RGB LED */
  pinMode(R_LED, OUTPUT);
  pinMode(G_LED, OUTPUT);
  pinMode(B_LED, OUTPUT);

  /* INPUT: PUSH BUTTON */
  pinMode(B_TN, INPUT_PULLUP); // REQUIRE PULLUP OR PULLDOWN RESISTOR

  delay(5); // Force Serial.println in void setup()
  Serial.println("Completed void setup");
}

void loop() {
  Serial.println(whiteled++);
  delay(1250);
}


/* PARTICLE CLOUD FUNCTION CALLBACK */
// int whiteled(const char* arg) {
//   return white_led = (atoi(arg) >=0 && atoi(arg) <= 255) ? atoi(arg) : 0;
// }

/* PARTICLE CLOUD SUBSCRIBE CALLBACK */
void others_ldr(const char *event, const char *data) {
  Log.info("event=%s data=%s", event, (data ? data : "NULL"));
}