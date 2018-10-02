
Timer scream_t;
bool scream_on = false,scream_continuous = false;
int scream_on_time, scream_off_time, scream_counter;

void scream_update(){
  if(scream_continuous || scream_counter > 0){
    if((scream_on && scream_t.completed(scream_on_time)) || (!scream_on && scream_t.completed(scream_off_time))){
      scream_on = !scream_on;
      if(!scream_on)scream_counter--;
      digitalWrite(SCREAM_PIN, scream_on);
    }
  }
}

void scream_forever(int on_time, int off_time){
  scream_on_time = on_time;
  scream_off_time = off_time;
  scream_continuous = true;
  
  scream_on = true;
  digitalWrite(SCREAM_PIN, scream_on);

  scream_t.disable();
}
void scream_mult(int on_time, int off_time, int counter){
  if (counter < 1)return;
  scream_counter = counter;
  scream_on_time = on_time;
  scream_off_time = off_time;
  scream_continuous = false;
  
  scream_on = true;
  digitalWrite(SCREAM_PIN, scream_on);

  scream_t.disable();
}
void scream_once(int on_time){
  scream_mult(on_time, 0, 1);
}

void scream_blocking(int d0, bool wait_after){
  digitalWrite(SCREAM_PIN, true);
  delay(d0);
  digitalWrite(SCREAM_PIN, false);
  if (wait_after)
    delay(d0);
}
void scream_blocking(int on, int off){
  digitalWrite(SCREAM_PIN, true);
  delay(on);
  digitalWrite(SCREAM_PIN, false);
  delay(off);
}
