/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#line 1 "/Users/eo/Documents/CART461-2021/SOUND-SYNTHESIS/MOZZI/src/MOZZI.ino"
#include <Particle.h>

#include <MozziGuts.h>
#include <Oscil.h> // oscillator template
#include <tables/sin2048_int8.h> // sine table for oscillator

// use: Oscil <table_size, update_rate> oscilName (wavetable), look in .h file of table #included above
void setup();
void updateControl();
int updateAudio();
void loop();
#line 8 "/Users/eo/Documents/CART461-2021/SOUND-SYNTHESIS/MOZZI/src/MOZZI.ino"
Oscil <SIN2048_NUM_CELLS, AUDIO_RATE> aSin(SIN2048_DATA);

// control variable, use the smallest data size you can for anything used in audio
byte gain = 255;


void setup(){
  startMozzi(); // start with default control rate of 64
  aSin.setFreq(3320); // set the frequency
}


void updateControl(){
  // as byte, this will automatically roll around to 255 when it passes 0
  gain = gain - 3 ;
}


int updateAudio(){
  return (aSin.next()* gain)>>8; // shift back to STANDARD audio range, like /256 but faster
}


void loop(){
  audioHook(); // required here
}