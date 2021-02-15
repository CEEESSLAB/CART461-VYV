// ********************************************************************** /
//
// To run this example prepare a micro SD card with a file hierarchy like
// SD:
// └───01
//        001.mp3
//        002.mp3
// optionally up to ...  
//        998.mp3
//        999.mp3
//
// ********************************************************************** /

SYSTEM_THREAD(ENABLED)
#include "SdFat.h"
#include "Adafruit_VS1053.h"

SerialLogHandler traceLog(LOG_LEVEL_WARN, { { "app", LOG_LEVEL_INFO } });

SdFat SD;

// These are the pins used for the music maker FeatherWing
const int  MP3_RESET        = -1;                 // VS1053 reset pin (unused!)
const int  SD_CS            = D2;                 // SD Card chip select pin
const int  MP3_CS           = D3;                 // VS1053 chip select pin (output)
const int  DREQ             = D4;                 // VS1053 Data request, ideally an Interrupt pin
const int  MP3_DCS          = D5;                 // VS1053 Data/command select pin (output)
const char *fileNamePattern = "%03d.mp3";         // file name pattern to insert track number
Adafruit_VS1053_FilePlayer musicPlayer(MP3_RESET, MP3_CS, MP3_DCS, DREQ, SD_CS); 

int        trackNumber      = 1;
bool       needStart        = true;



void setup() {
  pinMode(D7, OUTPUT);
  // Particle.function("playSine", playSine);
  // Particle.function("playTrack", playTrack);
  // Particle.function("setVolume", setVolume);
  // Particle.variable("temo", trackNumber);
  Particle.variable("attributed", "ME @ N.D.G", STRING);
  
  
  if (!SD.begin(SD_CS)) {
    Log.error("SD failed, or not present");
    while(1) yield();                             // don't do anything more
  }
  Serial.println("SD OK!");

  Log.info("Adafruit VS1053 Library Test");
  // initialise the music player
  if (!musicPlayer.begin()) {                     // initialise the music player
     Log.error("Couldn't find VS1053, do you have the right pins defined?");
     while(1) yield();
  }
  Log.info("VS1053 found");

  // Make a tone to indicate VS1053 is working
  musicPlayer.sineTest(0x44, 200);

  // set current working directory
  SD.chdir("/01", true);
  // list files
  SD.ls(&Serial, LS_R);

  // Set volume for left, right channels. lower numbers == louder volume!
  musicPlayer.setVolume(0, 0);

  // ***** Two interrupt options! ***** 
  // This option uses timer0, this means timer1 & t2 are not required
  // (so you can use 'em for Servos, etc) BUT millis() can lose time
  // since we're hitchhiking on top of the millis() tracker
  musicPlayer.useInterrupt(VS1053_FILEPLAYER_TIMER0_INT);

  // Alternatively just play an entire file at once
  // This doesn't happen in the background, instead, the entire
  // file is played and the program will continue when it's done!
  //musicPlayer.playFullFile("001.mp3");
  //playTrack("001");
}

void loop() {
  //Serial.println("playing - I hope");
  static uint32_t msPlayStarted = 0;
  static uint32_t msLastAction = 0;

  if (needStart && trackNumber) {
    char fileName[32];
    char msg[128];
    uint32_t us = micros();

    // Start playing a file, then we can do stuff while waiting for it to finish
    snprintf(fileName, sizeof(fileName), fileNamePattern, trackNumber);
    Log.trace("Starting: %lu", micros() - us); us = micros();
    if (musicPlayer.startPlayingFile(fileName)) {
      Log.trace("Started: %lu", micros() - us); us = micros();
      snprintf(msg, sizeof(msg), "Started playing '%s'",fileName);
      msPlayStarted = millis();
      
    }
    else {
      Log.trace("Not started: %lu", micros() - us); us = micros();
      snprintf(msg, sizeof(msg), "Could not open file '%s'",fileName);
    }
    Log.info(msg);
    needStart = false;
  }

  if (millis() - msLastAction >= 1000) {
    uint32_t sec = (millis() - msPlayStarted) / 1000.0;
    // file is now playing in the 'background' so now's a good time
    // to do something else like handling LEDs or buttons :)
    msLastAction = millis();
    Serial.printf("\r%02lu:%02lu %s  ", sec / 60, sec % 60, musicPlayer.playingMusic ? "playing" : "stopped");
  }
  
}

int playTrack(const char* arg) {
  int n = atoi(arg);

  if (n > 0) {
    trackNumber = n;
    if (musicPlayer.playingMusic) {
      musicPlayer.stopPlaying();
    }
    needStart = true;
  }
  return trackNumber;
}

int setVolume(const char* arg) {
  int vol = atoi(arg);
  musicPlayer.setVolume(vol, vol);
  return vol;
}

int playSine(const char* arg) {
  int freq = atoi(arg);
  musicPlayer.sineTest(freq, 1000);            
  musicPlayer.reset();    
  return freq;
}

