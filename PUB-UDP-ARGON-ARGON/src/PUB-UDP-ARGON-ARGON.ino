/*
 * Project ARGON <-> ARGON VIA UDP (USER DATAGRAMS PROTOCOL)
 * Description: UDP does not guarantee that messages are always delivered, 
 * or that they are delivered in the order supplied. In cases where your 
 * application requires a reliable connection, TCPClient is the alternative.
 * 
 * Buffered UDP Communication.
 */

#include <Particle.h>
#include <Wire.h>

/* PARTICLE CLOUD COMMUNICATION IN SEPARATE THREAD */
SYSTEM_THREAD(ENABLED);
/* CONTROL WiFi & INTERNET: AUTOMATIC, SEMI_AUTOMATIC, MANUAL */
SYSTEM_MODE(MANUAL);

/* PARTICLE CLOUD GOOGLE MAPS INTEGRATION */
#include "google-maps-device-locator.h"
GoogleMapsDeviceLocator locator;

/* IMU IMPLEMENTATION */
#include "Razor.h"
IMU imu;

/* BUFFERED UDP AS TRANSPORT LAYER */
UDP udp;

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
  Serial.println(myArgonIP = WiFi.localIP());
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
  /* START UDP SERVICE */
  if (WiFi.ready()) {
    udp.begin(inPort);
  }
  
  delay(5); // Force Serial.println in void setup()
  Serial.println("Completed void setup");
}

void loop() {
  /* CHECK FOR UDP DATA */
  uuudeepee();

  /* IMU RUN @ 50Hz */
  imu.loop();

  /* TRANSMIT UDP PACKET @ 1Hz */
  if( millis() - cTime > 1000) {
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

/* UDP - BUFFERED UNMARSHALLING ALWAYS FOLLOWS THREE STEPS:
   a) udp.parsePacket() (UDP DATA PACKET LENGTH) > 0
   b) udp.read() (OPTIONAL)
   c) udp.available() (ACCESS ENTIRE PACKET)
*/
void uuudeepee() {
  int sized = udp.parsePacket();
  char _rBuffer[sized] = { 0 };

  if ( sized > 0) {
    Serial.println(sized); 
    udp.read(_rBuffer, sized);
    //just for debug
    for(int z = 0; z < sized; z++) Serial.print(_rBuffer[z]);
    Serial.println();
    // Echo back data to sender
    udp.beginPacket(udp.remoteIP(), udp.remotePort()); //outPort
    udp.write("TEST: ");
    udp.write(udp.remoteIP().toString().c_str());
    udp.write(" ");
    udp.write( String(udp.remotePort()).c_str() );
    udp.endPacket();
  }
  for(int z = 0; z < sized; z++) _rBuffer[z] = NULL;
}

void dof() {

  /* UPDATE IMU READINGS: ACCELERO[3], MAGENTO[3], GYRO[3], 
     YAW, PITCH, ROLL, HEADING */
  accel = imu.getAccelerometer();  
  magnetom = imu.getMagnetometer();
  gyro = imu.getGyrometer();
  yaw = imu.getYaw();
  pitch = imu.getPitch();
  roll = imu.getRoll();
  magnitude = imu.getMagnitude();

  char _buffer[195];
  sprintf(_buffer, "%.7f,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f",  \
          accel[0], accel[1], accel[2], \
          magnetom[0], magnetom[1], magnetom[2], \
          gyro[0], gyro[1], gyro[2], \
          yaw, pitch, roll, magnitude \
  );
  //Serial.println(_buffer);

  // udp.beginPacket(argons[2], outPort);
  //   udp.write( _buffer );
  // udp.endPacket();


}

/* PARTICLE CLOUD GOOGLE GEOLATION: LAT, LONG AND ACCURACY - CALLBACK */
void locationCallback(float lat, float lon, float accu) {

  disconnectFromParticleCloud();

  char _buffer[45];
  sprintf(_buffer, "%.7f,%.7f,%.7f", lat, lon, accu);
  Serial.println(_buffer);

  digitalWrite(R_LED, LOW);
  digitalWrite(B_LED, HIGH);
  delay(1000);
  
  digitalWrite(B_LED, LOW);
  digitalWrite(G_LED, HIGH);
  delay(1002);
  digitalWrite(G_LED, LOW);
}

    // // Echo back data to sender
    // Udp.beginPacket(ipAddress, port);
    // Udp.write("TEST: ");
    // Udp.write(Udp.remoteIP().toString().c_str());
    // Udp.write(" ");
    // Udp.write( String(Udp.remotePort()).c_str() );
    // Udp.endPacket();