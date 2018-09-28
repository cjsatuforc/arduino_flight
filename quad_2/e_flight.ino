
#include <Servo.h>

// MOTORS
const int motor_pins[4] = {MOTOR0, MOTOR1, MOTOR2, MOTOR3};
int motors_start[4] = {1000, 1000, 1000, 1000};
Servo motors[4];

// PID Controllers, EVERYTHING is    YAW     PITCH    ROLL
double yaw_set = 0; // This is the setpoint for the yaw value
double in[3], out[3], set[3];
PID pids[3] = {
  PID(&in[0], &out[0], &set[0], YAW_PID_KP, YAW_PID_KI, 0, DIRECT),
  PID(&in[1], &out[1], &set[1], PITCH_PID_KP, PITCH_PID_KI, PITCH_PID_KD, DIRECT),
  PID(&in[2], &out[2], &set[2], PITCH_PID_KP, PITCH_PID_KI, PITCH_PID_KD, DIRECT)
  };


// PID LOADING/SAVING ******************************
void save_pid(){
  double pid_save[3][3];
  for(int i = 0;i<3;i++){
    pid_save[i][0] = pids[i].GetKp();
    pid_save[i][1] = pids[i].GetKi();
    pid_save[i][2] = pids[i].GetKd();
  }
  //Saving PID
  eeprom_pid(pid_save, EEPROM_SAVE);
}

void load_pid(){
  double pid_load[3][3];
  if (eeprom_pid(pid_load, EEPROM_LOAD)){
    for(int i = 0;i<3;i++){
      pids[i].SetTunings(pid_load[i][0],pid_load[i][1],pid_load[i][2]);
    }
  }
}
void reset_pid(){
  eeprom_clear_offset(EEPROM_PID);
  pids[0].SetTunings(YAW_PID_KP, YAW_PID_KI, YAW_PID_KD);
  pids[1].SetTunings(PITCH_PID_KP, PITCH_PID_KI, PITCH_PID_KD);
  pids[2].SetTunings(PITCH_PID_KP, PITCH_PID_KI, PITCH_PID_KD);
}
// ******************************************************


void flight_init(){
  for(int i=0;i<3;i++)
    pids[i].SetSampleTime(PID_SAMPLE_TIME);

  // Load values from EEPROM
  load_pid();
  eeprom_motors(motors_start, EEPROM_LOAD);
  
  pids[0].SetOutputLimits(YAW_PID_MIN, YAW_PID_MAX);
  pids[1].SetOutputLimits(PITCH_PID_MIN, PITCH_PID_MAX);
  pids[2].SetOutputLimits(PITCH_PID_MIN, PITCH_PID_MAX);
  
  for(int i =0;i<4;i++){
    pinMode(motor_pins[i], OUTPUT);
    motors[i].attach(motor_pins[i]);
    motors[i].writeMicroseconds(MOTOR_ZERO_LEVEL);
  }

  for(int i=0;i<3;i++)
    pids[i].SetMode(AUTOMATIC);
}

void zero_motors(){
  for (int i = 0; i<4; i++)
    motors[i].writeMicroseconds(MOTOR_ZERO_LEVEL);
}

// You will work after RX and MPU, calculate error, then set the motor output 
void flight_update(){
  
  // MOTORS AND PID CONFIG ****************************************
  if(mode == MotorsConfig){
    int write_vals[4] = {MOTOR_ZERO_LEVEL, MOTOR_ZERO_LEVEL, MOTOR_ZERO_LEVEL, MOTOR_ZERO_LEVEL};
    motors_start[config_value] = MOTOR_ZERO_LEVEL + (int)(rx_vals[RX_THROTTLE] / 100.0 * 1000); // Stablish special value
    write_vals[config_value] = motors_start[config_value]; // Copy value over to the setting values
    for(int i = 0; i<4; i++){
      motors[i].writeMicroseconds(write_vals[i]);
      longs[i] = motors_start[i];
    }
  }else if (mode == PIDConfig){
    // Pull PIDs
    int pid_pull = 0;
    double curr_pids[3];
    if(config_value >= 3)pid_pull = 1;
    curr_pids[0] = pids[pid_pull].GetKp();
    curr_pids[1] = pids[pid_pull].GetKi();
    curr_pids[2] = pids[pid_pull].GetKd();

    // Get pid index required, by wrapping the config_value to be less than 3
    int pid_index = config_value;
    while(pid_index > 2)pid_index -= 3;
    curr_pids[pid_index] = clamp(curr_pids[pid_index] + (double)rx_vals[RX_AUX1] / 100.0 * (double)rx_vals[RX_YAW] / 100.0 * 0.01, 0.0, 20.0);
    
    // Change PIDS in debug long
    for(int i=0;i<3;i++)
      longs[i] = curr_pids[i];
    
    // Set the pids back
    pids[pid_pull].SetTunings(curr_pids[0], curr_pids[1], curr_pids[2]);
  }
  
  // **************************************
  
  // FLIGHT -----------------------------------------
  if (!(mode == Flying))return;
  
  // PID UPDATE **************************
  // To understand this better, I inverted the yaw value because there's wrapping when the quad reaches 180 degrees, 
  // it goes to -180, so I accomodate for that by setting the input and keeping the setpoint at 0
  // Set the inputs
  yaw_set = wrap(yaw_set + (rx_vals[RX_YAW] / 100) * RX_MAX_ANGLE, -180.0, 180.0); // Update the yaw set
  in[0] = wrap(ypr[0] - yaw_set, -180.0, 180.0);
  for(int i = 1; i<3; i++)
    in[i] = ypr[i];
  
  // Setpoints are a little more complicated
  set[0] = 0; // YAW
  set[1] = (rx_vals[RX_PITCH] / 100) * RX_MAX_ANGLE;
  set[2] = (rx_vals[RX_ROLL] / 100) * RX_MAX_ANGLE;


  for(int i=0;i<3;i++)
    pids[i].Compute();
  //**********************************************


  #ifdef DEBUG_
//  Serial.print(" I ");
////  Serial.println(pids[0].inAuto);
//  for(int i = 0; i<3; i++)
//    {Serial.print(in[i]);Serial.print(" ");}
////  Serial.println();
//
//  Serial.print(" S ");
//  for(int i = 0; i<3; i++)
//    {Serial.print(set[i]);Serial.print(" ");}
////  Serial.println();
//
//  Serial.print(" O ");
//  for(int i = 0; i<3; i++)
//    {Serial.print(out[i]);Serial.print(" ");}
//  Serial.println();
  #endif
  
  // Motor UPDATE **************************
//  int throttle = ( / 100) * 600 + MOTOR_ZERO_LEVEL;
  int m_vals[] = { // Configured as positive PID means clockwise, forward, right
    map(rx_vals[RX_THROTTLE],10,100, motors_start[0], MOTOR_MAX_LEVEL - 150) - out[0] - out[1] + out[2],
    map(rx_vals[RX_THROTTLE],10,100, motors_start[1], MOTOR_MAX_LEVEL - 150) + out[0] - out[1] - out[2],
    map(rx_vals[RX_THROTTLE],10,100, motors_start[2], MOTOR_MAX_LEVEL - 150) - out[0] + out[1] - out[2],
    map(rx_vals[RX_THROTTLE],10,100, motors_start[3], MOTOR_MAX_LEVEL - 150) + out[0] + out[1] + out[2]
  };
  
  for (int i = 0; i<4; i++){
    m_vals[i] = clamp(m_vals[i], MOTOR_ZERO_LEVEL, MOTOR_MAX_LEVEL);
    
    longs[i] = m_vals[i];
    motors[i].writeMicroseconds(m_vals[i]);
  }
}
