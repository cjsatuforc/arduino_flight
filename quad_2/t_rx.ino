
//#define RX_AVERAGE_COUNT 3
int positives[] = {RX_THROTTLE, RX_AUX1, RX_AUX2};
volatile int rx_time[6] = {0, 0, 0, 0, 0, 0}; // Values for the receiver, they will be assigned for global access
//int rx_time_last[6][RX_AVERAGE_COUNT];
//int rx_time_av_c = 0;
//int rx_time_av[6];

int rx_limits[6][2];


bool rx_ready = false; // Turns true when the rx is configured
void rx_reset_limits(){
  for(int i=0;i<6;i++)
    for(byte e=0;e<2;e++)
      rx_limits[i][e] = 1500; 
}
void rx_init(){
  // Set the limits
  rx_reset_limits();
  
  for(byte i=0;i<6;i++)
    pinMode(pgm_read_byte(&rxPins[i]),INPUT);
  
  // Set Interrupts
  PCICR |= (1 << PCIE2) | (1 << PCIE1);
  PCMSK2 |= (1 << PCINT18)|(1 << PCINT20);
  PCMSK1 |= (1 << PCINT8)|(1 << PCINT9)|(1 << PCINT10)|(1 << PCINT11);
  sei();

  if(!eeprom_rx(rx_limits, EEPROM_LOAD))set_mode(RXConfig);
}

// ******** TIMERS **********************

Timer t0;

//************************************

#define RX_MENU_V 75
#define RX_MENU_T 1000
#define RX_FLYING_TO_IDDLE_T 7000
//int rx_stable = 10; // Value used by RXConfig to ditch first 10 values of RX
//volatile bool rx_receiving = false;
void rx_update(){// Will set the rx_vals based on the rx_limits and rx_time and control the mode since RX is main/only input

  // Set last times for average
//  if(rx_receiving){
//    for(int i = 0;i<6;i++){
//      rx_time_last[i][rx_time_av_c] = rx_time[i];
//      rx_time_av[i] = 0;
//      for(int e = 0;e<RX_AVERAGE_COUNT;e++)
//        rx_time_av[i] += rx_time_last[i][e] / RX_AVERAGE_COUNT;
//    }
//    rx_time_av_c++;if(rx_time_av_c==RX_AVERAGE_COUNT)rx_time_av_c = 0; // Upping counter
//  }
  
  // Map the values
  for(int i=0;i<6;i++)
//    rx_vals[i] = clamp((int)(((double) rx_time[i] - (double) rx_limits[i][0]) * (100.0 - (-100.0)) / ((double) rx_limits[i][1] - (double) rx_limits[i][0]) + (-100.0)), -100, 100);
    rx_vals[i] = clamp(map(rx_time[i], rx_limits[i][0], rx_limits[i][1], -100, 100), -100, 100); 
  
  for(int i=0;i<3;i++)
    rx_vals[positives[i]] = (rx_vals[positives[i]] + 100)/2; // Map values for 0 - 100 only

  #ifdef DEBUG_RX and defined(DEBUG_)
  for(int i = 0; i<6; i++)
    {Serial.print(rx_vals[i]);Serial.print(" ");}
  Serial.println();
  #endif
  
  
  switch(mode){
    case RXConfig:
      
      // Set the debug longs
      for(int i = 0;i<4;i++){
//        longs[i] = rx_limits[0][i];
        longs[i] = rx_vals[i];
      }
      
      // Configure limits
//      if(!rx_stable){
        for(int i = 0;i<6;i++){
          if(rx_time[i] < rx_limits[i][0] && rx_time[i] > 900)rx_limits[i][0] = rx_time[i];
          if(rx_time[i] > rx_limits[i][1] && rx_time[i] < 2000)rx_limits[i][1] = rx_time[i];
        } 
//      }else if(rx_receiving)rx_stable--;
      
      // Exit function
      if(rx_limits[RX_ROLL][1] - rx_limits[RX_ROLL][0] > 200 && 
        rx_limits[RX_PITCH][1] - rx_limits[RX_PITCH][0] > 200 && 
        rx_vals[RX_ROLL] < -RX_MENU_V && rx_vals[RX_PITCH] < -RX_MENU_V){ // Making sure we at least have some values in, and the stick is in the right place for > 2 sec
        if(t0.completed(RX_MENU_T))
          set_mode(Iddle);
      }else t0.disable();
      break;
    case Iddle:
      // Menus 
      
      // MOTORS CONFIG
      if(t0.available(0))
        if(rx_vals[RX_ROLL] < -RX_MENU_V && rx_vals[RX_PITCH] > RX_MENU_V){ // top left, motors config
          if(t0.completed(RX_MENU_T, 0))
            set_mode(MotorsConfig);
        }else t0.disable();
      // PID CONFIG
      if(t0.available(1))
        if(rx_vals[RX_ROLL] > RX_MENU_V && rx_vals[RX_PITCH] > RX_MENU_V){ // top right, PID config
          if(t0.completed(RX_MENU_T, 1))
            set_mode(PIDConfig);
        }else t0.disable();
      // ARM
      if(t0.available(2))
        if(rx_vals[RX_ROLL] > RX_MENU_V && rx_vals[RX_PITCH] < -RX_MENU_V){ // bottom right, arm, go flying
          if(t0.completed(RX_MENU_T, 2))
            set_mode(Flying);
        }else t0.disable();

      // RECONFIGURE RX
      if(t0.available(3))
        if(rx_vals[RX_YAW] > RX_MENU_V && rx_vals[RX_THROTTLE] < 10){ // bottom right (left stick), reconfigure rx
          if(t0.completed(RX_MENU_T, 3))
            set_mode(RXConfig);
        }else t0.disable();
      
      break;
      
    case Flying:
      if(rx_vals[RX_THROTTLE] < 10){
        if(t0.completed(RX_FLYING_TO_IDDLE_T))
          set_mode(Iddle);
      }else t0.disable();
      break;
  }
  // Config switch
  switch(mode){
    case MotorsConfig:
    case PIDConfig:
      
      if(t0.available(0))
        if( rx_vals[RX_ROLL] < -RX_MENU_V && rx_vals[RX_PITCH] > RX_MENU_V){ // top left, config_val --
          if(t0.completed(RX_MENU_T, 0)){
            config_value = wrap(config_value - 1, 0, config_max);
            scream_mult(100,200,config_value + 1);
          }
        }else t0.disable();
      
      if(t0.available(1))
        if(rx_vals[RX_ROLL] > RX_MENU_V && rx_vals[RX_PITCH] > RX_MENU_V){ // top right, config_val ++
          if(t0.completed(RX_MENU_T, 1)){
            config_value = wrap(config_value + 1, 0, config_max);
            scream_mult(100,200,config_value + 1);
          }
        }else t0.disable();

      if(t0.available(2))
        if(rx_vals[RX_ROLL] < -RX_MENU_V && rx_vals[RX_PITCH] < -RX_MENU_V){ // bottom left, back to Iddle
          if(t0.completed(RX_MENU_T, 2))
            set_mode(Iddle);
        }else t0.disable();
        
      break;
  }
  
//  rx_receiving = false;
}


void set_mode(Mode n_mode){
  if (mode == n_mode)return;
  switch(mode){ // What happens when exiting
    case RXConfig:
      scream_mult(50, 100, 3);
      eeprom_rx(rx_limits, EEPROM_SAVE);
      break;
    case MotorsConfig:
      zero_motors();
      eeprom_motors(motors_start, EEPROM_SAVE);
      break;
    case PIDConfig:
      save_pid();
      break;
    case Flying:
      zero_motors();
      break;
  }
  switch(n_mode){ // What happens when entering
    case RXConfig:
      scream_forever(50, 9000);
      rx_reset_limits();
      break;
    case Flying:
      scream_once(700);
      break;
    case PIDConfig:
      config_value = 0;
      config_max = 5;
      scream_once(100);
      break;
    case MotorsConfig:
      config_value = 0;
      config_max = 3;
      scream_once(100);
      break;
    case Iddle:
      scream_forever(30, 9000);
      break;
  }
  mode = n_mode;
}




volatile byte rx_state[6]={0,0,0,0,0,0};
volatile unsigned long rx_prev[6]={0,0,0,0,0,0};

ISR(PCINT1_vect) 
{
//  rx_receiving = true;
  for(byte i=0;i<6;i++){
    byte rx_temp=digitalRead(pgm_read_byte(&rxPins[i]));
    if((rx_state[i] == 0) && (rx_temp==1)){
      rx_prev[i]=micros();
      rx_state[i]=1;
    }
    else if((rx_state[i] == 1) && (rx_temp==0)){
      rx_time[i]=micros()-rx_prev[i];
      rx_state[i]=0;
    }
  }
}

ISR(PCINT2_vect, ISR_ALIASOF(PCINT1_vect));
