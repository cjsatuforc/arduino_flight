
#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include "Common.h"


// TIMER
//Timer::Timer(){}
void Timer::disable(){wait_started = false;} // Disables the bit on the shift
bool Timer::available(int shift){
	if(wait_started)return BIT_EXTRACT(wait_started, shift);
	else return true;
}
bool Timer::completed(int delay){
	return completed(delay, 0);
}
bool Timer::completed(int delay, int shift){ // In milliseconds
  if(wait_started){
  	auto done = millis() - wait_start > delay;
  	if (done) disable();
  	return done;
  }else {
  	BIT_SET(wait_started, shift);
    wait_start = millis();
    return false;
  }
}


// OTHERS
float wrap(float val, float min, float max){
	if (val < min)
		val += max - min;
	else if (val > max)
		val -= max - min;
	return val;
}
double wrap(double val, double min, double max){
	if (val < min)
		val += max - min;
	else if (val > max)
		val -= max - min;
	return val;
}

int wrap(int val, int min, int max){
	if (val < min)
		val += max - min + 1;
	else if (val > max)
		val -= max - min + 1;
	return val;
}


int clamp(int val, int min, int max){
	if (val < min)
		val = min;
	else if (val > max)
		val = max;
	return val;
}

double clamp(double val, double min, double max){
	if (val < min)
		val = min;
	else if (val > max)
		val = max;
	return val;
}


