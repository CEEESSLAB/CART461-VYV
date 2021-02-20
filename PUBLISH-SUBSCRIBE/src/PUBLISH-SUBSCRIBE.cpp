/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#line 1 "/Users/eo/Documents/CART461-2021/PUBLISH-SUBSCRIBE/src/PUBLISH-SUBSCRIBE.ino"
/********************************************************************************
 * Project PUBLISH AND SUBSCRIBE 
 * Description: This example will illustrate how to fully exploit the Particle 
 * Cloud: PUBLISH, SUBSCRIBE, FUNCTIONS and VARIABLES. Additionally, how to 
 * appropriately handle subscription data received via a Particle.subscription()
 * handler. In order to illustrate the approach, this example program will make
 * user of a LDR (Photocell), 9DoF (IMU - AHRS) and a RGB LED.
 * Author: slab
 * Date: 18022021
 ********************************************************************************/

#include <Particle.h>
#include <Wire.h>

/* IMU IMPLEMENTATION */
#include "Razor.h"
String accelerometer();
String magnetometer();
String gyrometer();
String yawPitchRollMagnitude();
void setup();
void loop();
void someothersLDR(const char *event, const char *data);
void someothersBTN(const char *event, const char *data);
#line 17 "/Users/eo/Documents/CART461-2021/PUBLISH-SUBSCRIBE/src/PUBLISH-SUBSCRIBE.ino"
IMU imu;

/* ALWAYS RUN PARTICLE CLOUD COMMUNICATION IN SEPARATE THREAD */ 
SYSTEM_THREAD(ENABLED);

/* HOW TO CONNECT TO WiFi & INTERNET: AUTOMATIC, SEMI_AUTOMATIC, MANUAL */
SYSTEM_MODE(AUTOMATIC); // DONE BY DEFAULT
//SYSTEM_MODE(SEMI_AUTOMATIC); // CONTROL WHEN & HOW TO CONNECT TO PARTICLE CLOUD (WIFI RADIO ON)
//SYSTEM_MODE(MANUAL); // CONTROL WHEN & HOW TO CONNECT TO WIFI NETWORK

#define DEBUG_LED D7 // SMALL BLUE LED NEXT USB CONNECTOR (RIGHT OF USB)
#define R_LED     D6 // RED LED
#define G_LED     D5 // GREEN LED
#define B_LED     D4 // BLUE LED
#define LDR       A5 // ANALOG_IN = ( 0 ... 4095 ) => 2^12 bits.
#define B_TN      D2 // MOMENTARY BUTTON

/* TIMER */
	unsigned long timed = 0;
	unsigned long timepassed;

/* IMU ACCELERO, MAGNETO, GYRO */
	float accel[3];  
	// Magnetometer
	float magnetom[3];
	// Gyrometer
	float gyro[3];
	// Euler angles
	float yaw;
	float pitch;
	float roll;
	float heading;

char buffer[256];

String accelerometer() {
  snprintf(buffer, sizeof(buffer), "%.02f,%.02f,%.02f", accel[0], accel[1], accel[2]);
  return buffer;
}

String magnetometer() {
  snprintf(buffer, sizeof(buffer), "%.02f,%.02f,%.02f", magnetom[0],magnetom[1],magnetom[2]);
  return buffer;
}

String gyrometer() {
  snprintf(buffer, sizeof(buffer), "%.02f,%.02f,%.02f", gyro[0],gyro[1],gyro[2]);
  return buffer;
}

String yawPitchRollMagnitude() {
  snprintf(buffer, sizeof(buffer), "%.02f,%.02f,%.02f,%.02f", yaw, pitch, roll, heading);
  return buffer;
}

void setup() {

  /* PARTICLE CLOUD INTENTIONS ALWAYS DECLARED IN VOID SETUP */
  Particle.variable("location", "ME @ MTL", STRING);
  Particle.variable("accelerometer", accelerometer);
  Particle.variable("magnetometer", magnetometer);
  Particle.variable("gyrometer", gyrometer);
  Particle.variable("yawPitchRollMagnitude", yawPitchRollMagnitude);

  /* PARTICLE PUBLISH 'n' SUBSCRIBE ARE MULTICAST OPERATIONS - FULLY CONNECTED 
    SIMPLY SUBSCRIBE TO A KNOWN DATA SOURCE OR PUBLISH DATA AS SOURCE
    THE ONLY CAVEAT - ONE (1) MSG PER SECOND OVER THE CLOUD 
  */
  //Particle.subscribe("someothersLDR", someothersLDR);
  Particle.subscribe("a-ble-centralBTN", someothersBTN);

  Serial.begin(57600);
  //while( !Serial.isConnected() ) // wait for Host to open serial port
  waitFor(Serial.isConnected, 15000); 
  timepassed = timed;

  /* BLUE DEBUG LED */
  pinMode(DEBUG_LED, OUTPUT);

  /* RGB LED */
  pinMode(R_LED, OUTPUT);
  pinMode(G_LED, OUTPUT);
  pinMode(B_LED, OUTPUT);

  /* INPUT: PUSH BUTTON */
  pinMode(B_TN, INPUT_PULLUP); // NO RESISTOR

  delay(5); // Force Serial.println in void setup()
  Serial.println("Completed void setup");
}

void loop() { 
  timed = millis();
    if (timed > (timepassed + 1000) ) {
    /* UPDATE IMU READINGS */
    imu.loop();

    }


    if( digitalRead(B_TN) == LOW ) {
        /* EVENTS CAN ONLY BE PUBLISHED ONCE PER SECOND */
        digitalWrite(DEBUG_LED, HIGH); 

        //Particle.publish("a-ble-centralBTN", "GUDRUN", PRIVATE); //ONLY PRIVATE OR DEVICE ID
        //bool success = Particle.publish("a-ble-centralBTN", "1", PRIVATE);
        if( Particle.publish("a-ble-centralBTN", "45.4786288,-73.617024,45.4953688,-73.57799640000002") ) Serial.println ("published");
        delay(1001); // PUBLISH ONCE PER SECOND

        digitalWrite(DEBUG_LED, LOW);
    }

    
}

/* PARTICLE CLOUD SUBSCRIBE CALLBACK */
void someothersLDR(const char *event, const char *data) { }

#include <vector>
using namespace std;
void someothersBTN(const char *event, const char *data) {
/* 
  Particle.subscribe handlers are void functions. They take two variables the name of your event, 
  and any data (UTF-8) that goes along with your event.
  In our case, the event will be "someothersBTN" and the data will be "1" or "0". We're going to 
  strcmp(), which compares two chars. If they are the same, strcmp will return 0.
  */

 /* EXAMPLE TOKENISER */
    //char *datas = "45.4786288,-73.617024,45.4953688,-73.57799640000002";
     vector<string> result;
      //get original length
    char copyData[strlen(data)];
    strcpy(copyData, data);

    char* token; 

    //get a token 
    token = strtok(copyData, ",");

    while (token != NULL) { 
    //add to the result array
      result.push_back(token);
      // strtok() contains a static pointer to the previous passed string
      token = strtok(NULL, ","); 
    } 

    for(int i=0; i<result.size();i++){
        Serial.println( result.at(i).c_str() );
    }
    Serial.println("called");

   if (strcmp(data, "1") == 0) {
  }
//   else if (strcmp(data,"low") == 0) {
//     digitalWrite(EVENTLED, HIGH);
//   }
//   else {
//     // if the data is something else, don't do anything.
//     // Really the data shouldn't be anything but those two listed above.
//   }
}