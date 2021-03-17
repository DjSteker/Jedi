/*
* TramasMicros2.cpp
*
* Created: 15/09/2017 14:45:16
*  Author: ivanmv
*/


#include "TramasMicros2.h"

TramaTiempo::TramaTiempo(unsigned long intervl,void(*function)()){
	active = true;
	previous = 0;
	interval = intervl;
	execute = function;
}

TramaTiempo::TramaTiempo(unsigned long prev,unsigned long intervl,void(*function)()){
	active = true;
	previous = prev;
	interval = intervl;
	execute = function;
}

void TramaTiempo::reset(){
	previous = micros();
}

void TramaTiempo::disable(){
	active = false;
}

void TramaTiempo::enable(){
	active = true;
}

void TramaTiempo::check(){
	//unsigned long 4,294,967,295
	if ( active && (micros()-previous >= interval) ) {
		previous = micros();
		execute();
	}
	else if ( active && micros()<previous) {
		unsigned long TMr = (4294967295-previous);
		if (TMr < interval) { previous = interval -TMr; }
		else { previous = micros(); execute(); }
		//previous =	interval - (4294967295- previous) ;
	}
}

void TramaTiempo::setInterval( unsigned long intervl){
	interval = intervl;
}
