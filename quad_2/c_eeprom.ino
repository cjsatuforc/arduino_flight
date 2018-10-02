#include <EEPROM.h>
#include <RF24Boot.h>

#define SPECIAL_VALUE 0x35
#define SPECIAL_VALUE_ESC 0x37
#define E0    EEPROM_OFFSET
#define E0_S  1 + sizeof(double)*3*3 // 9 doubles
#define E1    E0 + E0_S
#define E1_S  1 + sizeof(int)*6*2 // 12 ints, 6 channels, max and min
#define E2    E1 + E1_S
#define E2_S  1 + sizeof(int)*4 // the 4 motor's start speed
#define E3    E2 + E2_S
#define E3_S  1       // Are the motor's esc's programmed
#define EEPROM_PID 0
#define EEPROM_RX 1
#define EEPROM_MOTORS 2
#define EEPROM_ESC 3
#define EEPROM_LOAD true
#define EEPROM_SAVE false
const unsigned int eeprom_offsets[] = {E0, E1, E2, E3}; // The start writing offset for each block

// Clears the first byte of the offset, effectively disabling it
void eeprom_clear_offset(int offset){
  EEPROM.write(eeprom_offsets[offset], 0);
}

//************** CONJOINTED SETTING/LOADING *******************

void special_set(int addr, uint8_t special){ EEPROM.write(addr, special);  }
bool special_is_set(int addr, uint8_t special){ byte v = EEPROM.read(addr);return v == special;}

bool get_addr(int &addr,bool load,int offset, uint8_t special){
  addr = eeprom_offsets[offset];
  if(load){
    if (!special_is_set(addr++, special))return false;
  }else{
    special_set(addr++, special);
  }
  return true;
}
bool eeprom_pid(double val[][3], bool load){
  int addr;if (!get_addr(addr,load,EEPROM_PID, SPECIAL_VALUE))return false;
  for(int i = 0;i<3;i++)
    for(int e = 0;e<3;e++)
      {
        if(load)EEPROM.get(addr, val[i][e]);else EEPROM.put(addr, val[i][e]);
        addr+=sizeof(double);
      }
  return true;
}
bool eeprom_rx(int val[][2], bool load){
  int addr;if (!get_addr(addr,load,EEPROM_RX, SPECIAL_VALUE))return false;
  for(int i = 0;i<6;i++)
    for(int e = 0;e<2;e++)
      {
        if(load)EEPROM.get(addr, val[i][e]);else EEPROM.put(addr, val[i][e]);
        addr+=sizeof(int);
      }
  return true;
}
bool eeprom_motors(int val[], bool load){
  int addr;if (!get_addr(addr,load,EEPROM_MOTORS, SPECIAL_VALUE))return false;
    for(int i = 0;i<4;i++)
      {
        if(load)EEPROM.get(addr, val[i]);else EEPROM.put(addr, val[i]);
        addr+=sizeof(int);
      }
  return true;
}
bool eeprom_esc(bool load){
  int addr;
  return get_addr(addr,load,EEPROM_ESC, SPECIAL_VALUE_ESC);
}


//*********************************************
