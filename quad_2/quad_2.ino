
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



int throttle=MOTOR_ZERO_LEVEL;
byte rateAngleSwitch = 0;

#if DEBUG_SAMPLE_COUNT > 0
unsigned long d_time;
#endif

void setup() {
  #ifdef DEBUG_
  Serial.begin(115200);
  #endif

  pinMode(SCREAM_PIN, OUTPUT);
  
  // SETUP ******************************
  mpu_init();
  rf24_init();
  flight_init();
  rx_init();

  
  
}
#if DEBUG_SAMPLE_COUNT > 0
uint8_t d_sc = 0;
uint16_t d_samples[DEBUG_SAMPLE_COUNT];
#endif
bool mpu_updated = false;
void loop() {
  
#if DEBUG_SAMPLE_COUNT > 0
  d_time = micros();
#endif

  // LOOP*******************************
  scream_update();
  mpu_update();
  rf24_update();
  rx_update();
  flight_update();

  if (mpu_updated){
    mpu_updated = false;
    

#if DEBUG_SAMPLE_COUNT > 0
    d_samples[d_sc++] = micros() - d_time;
    if (d_sc == DEBUG_SAMPLE_COUNT)d_sc = 0;
    
    longs[2] = 0;
    
    for(int i = 0;i<DEBUG_SAMPLE_COUNT;i++)
      longs[2] += d_samples[i] / DEBUG_SAMPLE_COUNT;
#endif
#if DEBUG_SAMPLE_COUNT > 0 and defined(DEBUG_)
    Serial.println(longs[2]);
#endif
    
  }
}
