/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#line 1 "/Users/eo/Documents/CART461-2021/PUB-UDP-ARGON-ARGON/src/PUB-UDP-ARGON-ARGON.ino"
/*
 * Project ARGON <-> ARGON VIA UDP (USER DATAGRAMS PROTOCOL)
 * Description: Communication between ARGONs over UDP.
 */

#include <Particle.h>
#include <Wire.h>

/* PARTICLE CLOUD COMMUNICATION IN SEPARATE THREAD */
void connectToParticleCloud();
void disconnectFromParticleCloud();
void connectToLAN();
void setup();
void loop();
void uuudeepee();
void dof();
void locationCallback(float lat, float lon, float accu);
#line 10 "/Users/eo/Documents/CART461-2021/PUB-UDP-ARGON-ARGON/src/PUB-UDP-ARGON-ARGON.ino"
SYSTEM_THREAD(ENABLED);
/* CONTROL WiFi & INTERNET: AUTOMATIC, SEMI_AUTOMATIC, MANUAL */
SYSTEM_MODE(SEMI_AUTOMATIC);

/* PARTICLE CLOUD GOOGLE MAPS INTEGRATION */
#include "google-maps-device-locator.h"
GoogleMapsDeviceLocator locator;

/* IMU IMPLEMENTATION */
#include "Razor.h"
IMU imu;

/* IF USING MAX-MSP - THEN THIS */
#include "simple-OSC.h"
#include "math.h"

/* SIMPLE-OSC USES UDP AS TRANSPORT LAYER */
UDP Udp;

IPAddress myArgonIP;
/* EXPLICIT REMOTE ADDRESS DECLARATION IF IS KNOWN - 
 * REMOTE ADDRESS CAN ALSO BE RETRIEVED FROM RECEIVED 
 * udp.remoteIP() in received PACKET  */
static uint8_t argons[4][4] = { 
  { 10, 0, 1, 5 },
  { 10, 0, 1, 6 },
  { 10, 0, 1, 7 },
  { 10, 0, 1, 8 }
};

/* PORTS FOR INCOMING & OUTGOING DATA */
unsigned int outPort = 8000;
unsigned int inPort = 8001;

/* ONBOARD LED = DEBUG LED */
#define DEBUG_LED D7 // SMALL BLUE LED NEXT USB CONNECTOR (RIGHT OF USB)
#define R_LED     D6 // RED LED
#define G_LED     D5 // GREEN LED
#define B_LED     D4 // BLUE LED
#define B_TN      D2 // MOMENTARY BUTTON

/* VARIABLES USED FOR BOTH PARTICLE CLOUD VARIABLES AND PARTICLE SUBSCRIPTION
  - IMU ACCELERO, MAGNETO, GYRO */
float * accel;  
float * magnetom;
float * gyro;
float yaw, pitch, roll, magnitude;

unsigned long int cTime = -1;

/* PARTICLE CLOUD SERVICE - ONLY IF SHARING DATA WITH OTHER ARGONS */
void connectToParticleCloud() {
    if( !Particle.connected() ) {
        Particle.connect();
    }
    waitUntil( Particle.connected );
}

/* PARTICLE CLOUD SERVICE - ONLY IF SHARING DATA WITH OTHER ARGONS */
void disconnectFromParticleCloud() {
    if( Particle.connected() ) {
        Particle.disconnect();
    }
    waitUntil( Particle.disconnected );
}

void connectToLAN() {
  /* IF ARGON ALREADY CONFIGURED FOR SSID/ROUTER - THEN THIS */
  /* TRY CONNECT TO SSID - ROUTER */
  WiFi.connect();
  /* WAIT UNTIL DHCP SERVICE ASSIGNS ARGON IPADDRESS */
  while(!WiFi.ready());

  delay(5);
 /* GET HOST (ARGON) ASSIGNED IP */
  Serial.print("ARGON IP (DHCP): ");
  //argonIP = WiFi.localIP();
  //sprintf(argonIPAddress, "%d.%d.%d.%d", argonIP[0], argonIP[1], argonIP[2], argonIP[3]);
  Serial.println(WiFi.localIP());
}

void setup() {
  /* NO PARTICLE CLOUD VARIABLE OR FUNCTIONS */
  Serial.begin(115200);
  waitFor(Serial.isConnected, 5000); 

  /* GEOLATION: LAT, LONG AND ACCRUACY - GOOGLE MAPS */
  locator.withEventName("cslablocation");
  locator.withSubscribe(locationCallback).withLocateOnce();

  /* BLUE DEBUG LED */
  pinMode(DEBUG_LED, OUTPUT);

  /* RGB LED */
  pinMode(R_LED, OUTPUT);
  pinMode(G_LED, OUTPUT);
  pinMode(B_LED, OUTPUT);

  /* INPUT: PUSH BUTTON */
  pinMode(B_TN, INPUT_PULLUP); // NO RESISTOR

  connectToLAN();
  /* START UDP SERVICE - USED BY SIMPLE-OSC */
  Udp.begin(inPort);

  delay(5); // Force Serial.println in void setup()
  Serial.println("Completed void setup");
}

void loop() {
  /* CHECK FOR UDP DATA */
  uuudeepee();

  /* IMU RUN @ 50Hz */
  imu.loop();

  /* TRANSMIT UDP PACKET @ 50Hz */
  if( millis() - cTime > 20) {
    dof();
    cTime = millis();
  }

  /* GOOGLE GEOLOCATION LOOP - OPTIONAL (CAN BE REMOVED) */
  locator.loop(); // 

  /* EVENT WHICH INDUCES A GOOGLE GEOLOCATION REQUEST & PARTICLE PUBLISH ROUTINE */
  if( digitalRead(B_TN) == LOW ) {
    /* WEBHOOK REQUEST - CONNECT TO PARTICLE CLOUD */
    connectToParticleCloud();
    /* REQUEST LOCATION FROM GOOGLE */
    locator.publishLocation();

    digitalWrite(R_LED, HIGH);
    delay(1000);
  }
}

/* UDP */
void uuudeepee() {

  int size = 0;
  OSCMessage inMessage;
    if ( ( size = Udp.parsePacket() ) > 0)
    {
        while (size--){
            inMessage.fill(Udp.read());
        }
    }
}

void dof() {

  /* UPDATE IMU READINGS: ACCELERO[3], MAGENTO[3], GYRO[3], YAW, PITCH, ROLL, HEADING */
  /* ACCESS UPDATED IMU STATE */
  accel = imu.getAccelerometer();  
  magnetom = imu.getMagnetometer();
  gyro = imu.getGyrometer();
  yaw = imu.getYaw();
  pitch = imu.getPitch();
  roll = imu.getRoll();
  magnitude = imu.getMagnitude();

  OSCMessage outMessage("/9DOF");
  //outMessage.addFloat(-3.14);
  outMessage.addFloat(accel[0]);
  outMessage.addFloat(accel[1]);
  outMessage.addFloat(accel[2]);
  outMessage.addFloat(magnetom[0]);
  outMessage.addFloat(magnetom[1]);
  outMessage.addFloat(magnetom[2]);
  outMessage.addFloat(gyro[0]);
  outMessage.addFloat(gyro[1]);
  outMessage.addFloat(gyro[2]);
  outMessage.addFloat(yaw);
  outMessage.addFloat(pitch);
  outMessage.addFloat(roll);
  outMessage.addFloat(magnitude);
  outMessage.send( Udp, argons[0], outPort );
}

/* PARTICLE CLOUD GOOGLE GEOLATION: LAT, LONG AND ACCURACY - CALLBACK */
void locationCallback(float lat, float lon, float accu) {

  disconnectFromParticleCloud();
  char _buffer[45];
  sprintf(_buffer, "%.7f.%.7f.%.7f", lat, lon, accu);
  Serial.println(_buffer);

  digitalWrite(R_LED, LOW);
  digitalWrite(B_LED, HIGH);
  delay(1000);
  
  digitalWrite(B_LED, LOW);
  digitalWrite(G_LED, HIGH);
  delay(1002);
  digitalWrite(G_LED, LOW);
}
