
#include "config.h"
#include <SPI.h>
#include <Wire.h>
#include <Common.h>
#include <PID_v1.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

enum Mode {Iddle, Flying, RXConfig, MotorsConfig, PIDConfig};
Mode mode = Iddle;
int config_value = 0, config_max = 0; // These are global config values used when configuring PID and Motors

float ypr[3]; // Values of the MPU will be stored here, for global access


#define RX_ROLL 0
#define RX_PITCH 1
#define RX_THROTTLE 2
#define RX_YAW 3
#define RX_AUX1 4
#define RX_AUX2 5
PROGMEM const byte rxPins[6]={RX_PIN_ROLL, RX_PIN_PITCH, RX_PIN_THROTTLE, RX_PIN_YAW, RX_PIN_AUX1,RX_PIN_AUX2}; // So    ROLL PITCH THROTTLE YAW AUX1 AUX2
float rx_vals[6]; // On a scale from -100 to 100, all values are scaled internally **** NOT DONE

int motor_offsets[4]; // Flight controller will set these, motors will decide how they're assigned

double longs[4]; // Debug longs that will be  sent over RF24 to the debug station


#if DEBUG_SAMPLE_COUNT > 0
unsigned long d_time;
#endif

void setup() {
  #ifdef DEBUG_
  Serial.begin(115200);
  #endif

  pinMode(SCREAM_PIN, OUTPUT);
  
  // SETUP ******************************
  flight_init();
  
  mpu_init();
  rf24_init();
  
  rx_init();

  
#if DEBUG_SAMPLE_COUNT > 0
  d_time = micros();
#endif
}
#if DEBUG_SAMPLE_COUNT > 0
uint8_t d_sc = 0;
uint16_t d_samples[DEBUG_SAMPLE_COUNT];
#endif
bool mpu_updated = false;
void loop() {
  


  // LOOP*******************************
  
  scream_update(); // About 0 msc
  mpu_update(); // About 1000 msc
  
  rf24_update(); // About 15 msc
  rx_update(); // About 300 msc

#if DEBUG_SAMPLE_COUNT > 0
  d_time = micros();
#endif

  
  
  
  
  #if DEBUG_SAMPLE_COUNT > 0
    d_samples[d_sc++] = micros() - d_time;
    if (d_sc == DEBUG_SAMPLE_COUNT)d_sc = 0;
    
    uint16_t ave = 0;
    
    for(int i = 0;i<DEBUG_SAMPLE_COUNT;i++)
      ave += d_samples[i] / DEBUG_SAMPLE_COUNT;
#endif
#if DEBUG_SAMPLE_COUNT > 0 and defined(DEBUG_)
    Serial.println(ave);
#endif
  
  
  if (mpu_updated){
    mpu_updated = false;
    
    flight_update();
    
    if (mode == Iddle)
      for (int i = 0; i<3; i++)
        longs[i] = ypr[i];


    
  }
}
