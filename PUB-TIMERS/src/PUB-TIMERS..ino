/*
 * Project PUB-SLEEP
 * Description:
 * Software Timers provide a way to have timed actions in your program. FreeRTOS 
 * provides the ability to have up to 10 Software Timers at a time with a minimum 
 * resolution of 1 millisecond. It is common to use millis() based "timers" though 
 * exact timing is not always possible (due to other program delays). 
 * 
 * Timers may be started, stopped, reset within a user program or an ISR. 
 * They may also be "disposed", removing them from the (max. 10) active timer list.
 */

#include <Particle.h>

/* PARTICLE CLOUD COMMUNICATION IN SEPARATE THREAD */
SYSTEM_THREAD(ENABLED);
/* CONTROL WiFi & INTERNET: AUTOMATIC, SEMI_AUTOMATIC, MANUAL */
SYSTEM_MODE(MANUAL);

/* TIMER DEF AND TIMER CALLBACK */
void heregoesone();

/* TIMERS - SYSTEM - DECLARE UPTO 10 TIMERS */
Timer timer_one(100, heregoesone);

/* ONBOARD LED = DEBUG LED */
#define DEBUG_LED D7 // SMALL BLUE LED NEXT USB CONNECTOR (RIGHT OF USB)
#define R_LED     D6 // RED LED
#define G_LED     D5 // GREEN LED
#define B_LED     D4 // BLUE LED
#define B_TN      D2 // MOMENTARY BUTTON

void connectToLAN() {
  /* IF ARGON ALREADY CONFIGURED FOR SSID/ROUTER - THEN THIS */
  /* TRY CONNECT TO SSID - ROUTER */
  WiFi.connect();
  /* WAIT UNTIL DHCP SERVICE ASSIGNS ARGON IPADDRESS */
  while(!WiFi.ready());

  delay(5);
 /* GET HOST (ARGON) ASSIGNED IP */
  Serial.print("ARGON IP (DHCP): ");
  Serial.println(WiFi.localIP());
}

void setup() {
  /* NO PARTICLE CLOUD VARIABLE OR FUNCTIONS */
  Serial.begin(115200);
  waitFor(Serial.isConnected, 5000); 

  /* BLUE DEBUG LED */
  pinMode(DEBUG_LED, OUTPUT);

  /* RGB LED */
  pinMode(R_LED, OUTPUT);
  pinMode(G_LED, OUTPUT);
  pinMode(B_LED, OUTPUT);

  /* INPUT: PUSH BUTTON */
  pinMode(B_TN, INPUT_PULLUP); // NO RESISTOR

  connectToLAN();
  timer_one.start();
  
  delay(5); // Force Serial.println in void setup()
  Serial.println("Completed void setup");
}

void loop() { 
  if(digitalRead(B_TN) == LOW)
    timer_one.changePeriod(random(15,1000));
}

void heregoesone() {
  uint8_t _l = random(4,7);
  digitalWrite(_l, HIGH);
  delay(50);
  digitalWrite(_l, LOW);
  delay(50);
}