#ifndef COMMON_FUNCS
#define COMMON_FUNCS

#define BIT_SET(x,y)     x |= (1 << y)
#define BIT_CLEAR(x,y)   x &= ~(1 << y)
#define BIT_EXTRACT(x,y) x & (1 << y)

// TIMER
class Timer{
	private:
	uint8_t wait_started = 0;
	unsigned long wait_start = 0;
	
	
	public:
	void disable();
	bool available(int shift);
	bool completed(int delay, int shift);
	bool completed(int delay);
	
};


// OTHERS
/*
 * Exclusive wrap, max is not included in the allowed range
*/
float wrap(float val, float min, float max);
double wrap(double val, double min, double max);
/*
 * Inclusive wrap, min and max are within the allowed range
*/
int wrap(int val, int min, int max);
int clamp(int val, int min, int max);
double clamp(double val, double min, double max);

#endif
