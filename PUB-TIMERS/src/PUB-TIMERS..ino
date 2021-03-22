/*
 * Project PUB-SLEEP
 * Description: Options for controlling Argons during periods of inactivity.
 * System.sleep() can be used to dramatically improve the battery life of a 
 * Particle-powered project. The SystemSleepConfiguration class configures 
 * all of the sleep parameters: STOP, ULTRA_LOW_POWER and HIBERNATE. 
 * 
 * HIBERNATE = MOST EXTREME LOW POWER MODE (MAJORITY OF SERVICES ARE OFF)
 * STOP & ULTRA_LOW_POWER = SELECTED SERVICES ARE PRIORITSED. IN THIS DEMO, 
 * WE WILL LOOK AT CONFIG OF STOP and ULTRA_LOW_POWER. 
 *  
 * https://docs.particle.io/reference/device-os/firmware/argon/#sleep-sleep-
 * STOP MODE = ULTRA_LOW_POWER = LEAST RESTRICTIVE
 */

#include <Particle.h>
#include <Wire.h>
#include <vector>
using namespace std;

/* PARTICLE CLOUD COMMUNICATION IN SEPARATE THREAD */
SYSTEM_THREAD(ENABLED);
/* CONTROL WiFi & INTERNET: AUTOMATIC, SEMI_AUTOMATIC, MANUAL */
SYSTEM_MODE(MANUAL);

/* IMU IMPLEMENTATION */
#include "Razor.h"
IMU imu;

IPAddress myArgonIP;

/* SLEEP ARGON SLEEP CONFIG */
SystemSleepConfiguration config;

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
uint8_t sleepcounter = 0;

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

  /* BLUE DEBUG LED */
  pinMode(DEBUG_LED, OUTPUT);

  /* RGB LED */
  pinMode(R_LED, OUTPUT);
  pinMode(G_LED, OUTPUT);
  pinMode(B_LED, OUTPUT);

  /* INPUT: PUSH BUTTON */
  pinMode(B_TN, INPUT_PULLUP); // NO RESISTOR

  connectToLAN();
  
  delay(5); // Force Serial.println in void setup()
  Serial.println("Completed void setup");
}

void loop() {
  /* IMU RUN @ 50Hz */
  imu.loop();

  if(sleepcounter > 100) {
    Serial.println(sleepcounter); sleepcounter = 0; 
    config.mode(SystemSleepMode::STOP)
    //config.mode(SystemSleepMode::ULTRA_LOW_POWER)
    //.network(NETWORK_INTERFACE_ALL) // WAKE ON ANY NETWORK ACTIVITY (DISRUPTS SLEEP)
    .network(NETWORK_INTERFACE_WIFI_STA, \
    SystemSleepNetworkFlag::INACTIVE_STANDBY ) // SLEEP NETWORK READY UPON WAKE
    //.usart(Serial1); NOT USB PORT -> HARDWARE TX/RX 
    //.analog(A0, 412, AnalogInterruptMode::BELOW); //412milliVolts (3.3Volt)
    //.analog(A1, 1650, AnalogInterruptMode::CROSS);
    //.analog(A2, 2392, AnalogInterruptMode::ABOVE); //2392milliVolts (3.3Volt)
    // WKP Pin: Argon, Boron, and Xenon is pin D8.
    //.gpio(D8, RISING)
    //.gpio(D9, FALLING)
    .gpio(D2, CHANGE)
    .duration(1min); // <chrono> 1min, 1s, 1ms

    System.sleep(config);
  }

  if( ( millis() - cTime > 100 ) ) {
    dof();
    cTime = millis();
    sleepcounter++;
  } 
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

  char _buffer[205];
  sprintf(_buffer, "~DOF,F,13,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f",  \
          accel[0], accel[1], accel[2], \
          magnetom[0], magnetom[1], magnetom[2], \
          gyro[0], gyro[1], gyro[2], \
          yaw, pitch, roll, magnitude \
  );
  Serial.println(_buffer);
}