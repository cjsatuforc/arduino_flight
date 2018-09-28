
#include <RF24.h>

bool rf24_ready = false;
RF24 radio(RF24Pin_0, RF24Pin_1);

byte rf24_addresses[][6] = {"0quad", "hquad"};

unsigned long rft;
void rf24_init(){

  if(!radio.begin()){
    
    #ifdef DEBUG_
    Serial.println("RF24 begin BAD");
    #endif
    
    rf24_ready = false;
    
    #ifdef BLOCK_ON_FAIL
    while(1)
      scream_blocking(50, 1950);
    #endif
    
    return;
  }else {
    #ifdef DEBUG_
    Serial.println(F("RF24 begin OK!"));
    #endif
    rf24_ready = true;
  }
  
  radio.setRetries(0,2); // Wait of p500 with 2 retries
  radio.setPALevel(RF24_PA_MAX);

  radio.openWritingPipe(rf24_addresses[1]);
//  radio.openReadingPipe(1,rf24_addresses[0]);

  rft = millis();
//  radio.startListening();
}

void rf24_update(){
  if (!rf24_ready)return;
  
  auto now = millis();
  if (now - rft >= 250) { // 1/4 second
    
//    radio.stopListening();
//    Serial.println(longs[2]);
    bool s = radio.write(&longs, sizeof(double) * 4);
    
    #ifdef DEBUG_
//    if (!s)Serial.println("Send fail");
    #endif
//    radio.startListening();
    
    rft = now;
  }
}
