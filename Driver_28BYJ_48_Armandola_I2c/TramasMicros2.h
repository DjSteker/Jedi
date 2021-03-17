/*
* TramasMicros2.h
*
* Created: 15/09/2017 14:45:33
*  Author: ivanmv
*/


#ifndef TRAMASMICROS2_H_
#define TRAMASMICROS2_H_


#if defined(ARDUINO) && ARDUINO >= 100
#include <Arduino.h>
#else
#include <WProgram.h>
#endif


#define NO_PREDELAY 0

class TramaTiempo{
	
	public:
	TramaTiempo(unsigned long interval,void(*function)());
	TramaTiempo(unsigned long prev,unsigned long interval,void(*function)());
	
	void reset();
	void disable();
	void enable();
	void check();
	
	void setInterval( unsigned long interval );

	private:
	boolean active;
	unsigned long previous;
	unsigned long interval;
	void(*execute)();
	
};

#endif /* TRAMASMICROS2_H_ */
