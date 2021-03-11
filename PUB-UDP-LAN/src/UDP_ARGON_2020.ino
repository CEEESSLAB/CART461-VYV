/*
 * Project UDP_ARGON_2020
 * Description:
 * Author:
 * Date:
 */

/* THESE ARE PARTICLE SPECIFIC PARAMETERS APPLIED AT CODE RUNTIME */
/* RUN ALL PARTICLE CLOUD COMMUNICATION IN SEPARATE THREAD */
SYSTEM_THREAD(ENABLED);
/* HOW TO CONNECT TO WiFi & INTERNET: AUTOMATIC, SEMI_AUTOMATIC, MANUAL */
SYSTEM_MODE(SEMI_AUTOMATIC);

/* IF USING MAX-MSP - THEN THIS */
#include "simple-OSC.h"
#include "math.h"

/* FLASHING CODE TO PARTICLE - 3 OPTIONS 
  a) PARTICLE CONNECTED TO CLOUD
    particle flash YOUR_DEVICE_NAME target/blinkled.bin
  b) PARTICLE DFU MODE (blinking yellow)
    particle flash --usb target/blinkled.bin/Users/eo/Documents/Particle/UDP_ARGON_2020/lib/simple-OSC/src/simple-OSC.h
  c) PARTICLE LISTENING MODE (blinking dark blue):
    particle flash --serial target/blinkled.bin
*/

/* FINDING PARTICLE DEVICE INFORMATION - PARTICLE LISTENING MODE
  particle identify 
*/

/* PHOTON WiFi MEMORY 
  Photon: remembers the 5 most recently set credentials
  WiFi.setCredentials(ssid, password);
  WiFi.setCredentials("My_Router", "mypasswordishuge");
*/

// UDP Port used for two way communication
unsigned int localPort = 8888;
//IPAddress multicastAddress(224,0,0,0);

// A UDP instance to let us send and receive packets over UDP
UDP Udp;
IPAddress argonIP;
IPAddress remoteIP(10, 0, 1, 8);
char argonIPAddress[24];

//IPAddress outIP(192, 168, 0, 14);
IPAddress senderAddress;
unsigned int outPort = 8000;
unsigned int inPort = 8001;


/* ONBOARD LED = DEBUG LED */
#define DEEBUG 7

/* PARTICLE CLOUD SERVICE */
boolean cloudConnected = false;

/* OSC CALLBACK SIGNATURES */
void startup(OSCMessage &inMessage);
void led(OSCMessage &inMessage);
void motor(OSCMessage &inMessage);

/* PARTICLE CLOUD SERVICE - ONLY IF SHARING DATA WITH OTHER ARGONS */
void connectToParticleCloud() {
    if(!cloudConnected && Particle.connected() == false) {
        Particle.connect();
    }
    if(Particle.connected()) cloudConnected = true;
}

void setup() {
  Serial.begin(57600);
  while(!Serial);

  pinMode(DEEBUG, OUTPUT);

  /* IF NOT CONFIGURED FOR ROUTER - THEN THIS */
  //WiFi.setCredentials("My_Router", "mypasswordishuge");

  /* IF ALREADY CONFIGURED FOR SPECFIC ROUTER - THEN THIS */
  WiFi.connect();
  while(!WiFi.ready());
/* SET STATIC IP ADDRESS - IF DHCP NOT ENABLED ON ROUTER 
   SET STATIC ARGON IP VIA ROUTER
     IPAddress myAddress(10,0,1,30);
     IPAddress netmask(255,255,255,0);
     IPAddress gateway(10,0,1,1);
     IPAddress dns(10,0,1,1);
     WiFi.setStaticIP(myAddress, netmask, gateway, dns);
*/
  Udp.begin(inPort);

  /* SET HOSTNAME - NO LONGER WORKS - 9TH NOVEMBER 2020 */
  /* WiFi.setHostname("lymphoid");
     Serial.print("ARGON: ");
     Serial.println(WiFi.hostname());
  */

 /* GET HOST (ARGON) IP */
  Serial.print("ARGON IP (DHCP): ");
  argonIP = WiFi.localIP();
  sprintf(argonIPAddress, "%d.%d.%d.%d", argonIP[0], argonIP[1], argonIP[2], argonIP[3]);
  Serial.println(argonIPAddress);
  delay(5000);
}


void loop() {
  /* CHECK IF THERE IS SOME DATA IN UDP FORMAT */
  oooesscee();
  Serial.println(ME);
}

void startup(OSCMessage &inMessage) {
    Serial.println("START-UP");
    Serial.println(inMessage.getInt(0));
    Serial.println(inMessage.getFloat(1));

    /* SENT DATA BACK TO SENDER */
    accelerometer();
}

void led(OSCMessage &inMessage) {
    Serial.println("LED");
    Serial.println(inMessage.getInt(0));
    Serial.println(inMessage.getInt(1));
  
    /* SENT DATA BACK TO SENDER */
    ldr();
}

void motor(OSCMessage &inMessage) {
    Serial.println("MOTOR");
    Serial.println(inMessage.getInt(0));
    Serial.println(inMessage.getInt(1));

    /* SENT DATA BACK TO SENDER */
    dof();
}

/* USES BUFFERED UDP CLIENT */
void oooesscee() {

  int size = 0;
  OSCMessage inMessage;
    if ( ( size = Udp.parsePacket() ) > 0)
    {
        while (size--)
        {
            inMessage.fill(Udp.read());
            //Serial.print(Udp.read());
            //Serial.print(' ');
        }
        
        if( inMessage.parse() ) {
            inMessage.route("/startup", startup);
            inMessage.route("/led", led);
            inMessage.route("/motor", motor);
        }
    }
    

  // if (Udp.parsePacket() > 0) {

  //   // Read first char of data received
  //   int c = Udp.read();

  //   digitalWrite(DEEBUG, HIGH);
  //   delay(5);
  //   digitalWrite(DEEBUG, LOW);
  //   delay(5);

  //   /* BRUTE FORCE DISPOSAL OF ALL UDP DATA */
  //   // while(Udp.available())
  //   //   Udp.read();

  //   Serial.println(c);
  //   /* CAPTURE SENDERS IP ADDRESS */
  //   // IPAddress ipAddress = Udp.remoteIP();
  //   senderAddress = Udp.remoteIP();
  //   int port = Udp.remotePort();

  //   Serial.println(senderAddress);
  //   Serial.println(port);

  //   //Udp.sendPacket(msg, sizeof(msg), ipAddress, 8889);
  //   osc_send(senderAddress);
  //   //Udp.beginPacket(ipAddress, 8889);
  //   //Udp.write(msg);
  //   //Udp.endPacket();
  // }
}

void accelerometer() {

  OSCMessage outMessage("/accelerometer");
  //outMessage.addFloat(-3.14);
  outMessage.addInt(random(0, 4096));
  outMessage.addInt(random(0, 4096));
  outMessage.addInt(random(0, 4096));
  outMessage.send( Udp, remoteIP, 8000 );
}

void ldr() {

  OSCMessage outMessage("/photocell");
  //outMessage.addFloat(-3.14);
  outMessage.addInt(random(0, 4096));
  outMessage.addInt(random(0, 4096));
  outMessage.addInt(random(0, 4096));
  outMessage.send( Udp, remoteIP, 8000 );
}

void dof() {

  OSCMessage outMessage("/9DOF");
  //outMessage.addFloat(-3.14);
  outMessage.addInt(random(0, 4096));
  outMessage.addInt(random(0, 4096));
  outMessage.addInt(random(0, 4096));
  outMessage.addInt(random(0, 4096));
  outMessage.addInt(random(0, 4096));
  outMessage.addInt(random(0, 4096));
  outMessage.addInt(random(0, 4096));
  outMessage.addInt(random(0, 4096));
  outMessage.addInt(random(0, 4096));
  outMessage.send( Udp, remoteIP, 8000 );
}

void osc_send(IPAddress remoteAdd) {
  int c = 1023;
  //SEND
  OSCMessage outMessage("/" + System.deviceID());
  // ACCERLOMETER
  outMessage.addInt( c >> 8 );
  outMessage.addInt( c );

  outMessage.addInt( c >> 8 );
  outMessage.addInt( c );

  outMessage.addInt( c >> 8);
  outMessage.addInt( c );

  outMessage.addInt( c );
  outMessage.addInt( c );
  outMessage.addInt( c );
  outMessage.addInt( c );

  outMessage.send( Udp, remoteAdd, 8889 );
  Serial.println("message sent");
}

