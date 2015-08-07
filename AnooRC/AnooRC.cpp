/*
  AnooRC.cpp
  A library for handling RC Controllers
  Created by Andrew Downey on 2/5/15
*/
#include "AnooRC.h"
#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define GUI

#define PITCH_FLAG 1
#define ROLL_FLAG 2
#define THROTTLE_FLAG 4
#define YAW_FLAG 8
#define AUX1_FLAG 16
#define AUX2_FLAG 32

volatile uint8_t prev; // remembers state of input bits from previous interrupt
volatile uint32_t risingEdge[6]; // time of last rising edge for each channel
volatile uint32_t uSec[6]; // the latest measured pulse width for each channel

ISR(PCINT2_vect)
{
	uint32_t now = micros();
	uint8_t curr = PIND; // current state of the 6 input bits
	uint8_t changed = curr ^ prev;
	int channel = 0;
	for (uint8_t mask = 0x04; mask; mask <<= 1) {
		if (changed & mask) { // this pin has changed state
			if (curr & mask) { // +ve edge so remember time
				risingEdge[channel] = now;
			}
			else { // -ve edge so store pulse width
				uSec[channel] = now - risingEdge[channel];
			}
		}
		channel++;
	}
	prev = curr;
}

bool RCController::getAux(int aux)
{
	if(3+aux>numChannels)
		return 0;
	return channelData[3+aux];
}

bool RCController::init(byte newChannelPins[], byte newNumChannels)
{ 
	Serial.println("Starting RCController...");
	numChannels=newNumChannels;
	if(numChannels>6 || numChannels<4)
		return false;
	for (int i = 0; i < numChannels; i++) {
		channelPins[i]=newChannelPins[i];
		pinMode(channelPins[i], INPUT);
		Serial.print("Setting up channelPin ");
		Serial.println(channelPins[i]);
	} 
	PCMSK2 |= 0xFC; // set the mask to allow those 6 pins to generate interrupts
	PCICR |= 0x04;  // enable interupt for port D
	Serial.println("Controller initialised");
	return true;
}

void RCController::setSmoothing(uint8_t smoothingFactor)
{
	if(smoothingFactor>=0 and smoothingFactor<1)
	{
		smoothing=smoothingFactor;
	}else{
		Serial.print(F("Smoothing factor must be between 0 and .99"));
		smoothing=0;
	}
}

void RCController::getChannelData() 
{
	uint32_t now = micros();
	for(int i=0; i<numChannels; i++){	
		if(smoothing>0)
		{
			channelData[i]=channelData[i]*smoothing+uSec[i]*(1-smoothing);
		}else{
			channelData[i]=uSec[i];
		}
	}

	#ifdef GUI
	Serial.print("c");
	for(int i=0; i<numChannels; i++){	
		Serial.print(',');
		Serial.print(channelData[i]);
	}
	#endif

	pitch=channelData[1];
	roll=channelData[0];
	throttle=channelData[2];
	yaw=channelData[3];
	aux1=channelData[4];
	aux2=channelData[5];
}