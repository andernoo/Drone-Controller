/*
  AnooRC.h
  A library for handling RC Controllers
  Created by Andrew Downey on 2/5/15
*/
#ifndef RC_h
#define RC_h
#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

class RCController {
	public:
	bool init(byte newChannelPins[], byte newNumChannels);
	void getChannelData();
	void setSmoothing(uint8_t smoothingFactor);
	bool getAux(int aux);
	int throttle;
	int roll;
	int pitch;
	int yaw;
	int aux1;
	int aux2;
	
	private:
	uint8_t channelPins[6]={99,99,99,99,99,99};
	unsigned int channelData[6]={0,0,0,0,0,0};
	byte numChannels=0;
	uint8_t smoothing=0;
};

#endif
