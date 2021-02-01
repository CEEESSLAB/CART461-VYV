/*
 * Project QWIIC-RGB
 * Description:
 * Author:
 * Date:
 */
/* THESE ARE PARTICLE SPECIFIC PARAMETERS APPLIED AT CODE RUNTIME */
/* RUN ALL PARTICLE CLOUD COMMUNICATION IN SEPARATE THREAD */
SYSTEM_THREAD(ENABLED);
/* HOW TO CONNECT TO WiFi & INTERNET: AUTOMATIC, SEMI_AUTOMATIC, MANUAL */
SYSTEM_MODE(SEMI_AUTOMATIC);

#include <SparkFun_BH1749NUC_Arduino_Library.h>
BH1749NUC rgb;

// setup() runs once, when the device is first turned on.
void setup() {
  Serial.begin(115200);
  while(!Serial);

  if (rgb.begin() != BH1749NUC_SUCCESS)
  {
    Serial.println("Error initializing the rgb sensor.");
    while (1) ;
  }
  
  // IR and RGB gain can be set to either BH1749NUC_GAIN_X1 or
  // BH1749NUC_GAIN_X32
  rgb.setIRGain(BH1749NUC_GAIN_X1);
  rgb.setRGBGain(BH1749NUC_GAIN_X1);
  // Measurement mode can be set to either BH1749NUC_MEASUREMENT_MODE_35_MS,
  // BH1749NUC_MEASUREMENT_MODE_120_MS, or BH1749NUC_MEASUREMENT_MODE_240_MS
  // (35ms, 120ms, 240ms between measurements).
  rgb.setMeasurementMode(BH1749NUC_MEASUREMENT_MODE_240_MS);
}

// loop() runs over and over again, as quickly as it can execute.
void loop() {
  Serial.println("Red: " + String(rgb.red()));
  Serial.println("Green: " + String(rgb.green()));
  Serial.println("Blue: " + String(rgb.blue()));
  Serial.println("IR: " + String(rgb.ir()));
  Serial.println("Green2: " + String(rgb.green2()));
  Serial.println();
  delay(1000);
}