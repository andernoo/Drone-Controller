/*
  Drone.h - A library for controlling a
  quadcopter using sensor data and RxTx.
  Created by Andrew Downey on 31/3/15
*/
#ifndef Drone_h
#define Drone_h
#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <Servo.h>
#include <Kalman.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_9DOF.h>
#include <AnooRC.h>

#ifndef SMA_LENGTH
#define SMA_LENGTH 5
#endif

typedef struct {
	union {
		struct {
			float roll;
			float pitch;
			float heading;
		};
		float v[3];
	};
	long height;
} drone_vec_t;

class Drone {
	public:
	drone_vec_t   orientation;
	int throttle[4]={0,0,0,0};
	float throttleTarget[3];
	double kalman[3]={0,0,0};
	bool armed=false;

	bool init(byte motorPins[4]);
	void setPID(int axis, int p, int i, int d);
	void setPingPins(int trig, int echo);
	void getAccelOffset();
	void updateOrientation();
	long getHeightCM();
	void kill();
	void manualControl(RCController *controller, uint8_t limit);
	void stabilise(RCController *controller);
	void updateMotors();

	private:
	unsigned int collective;
	float lastError[3]={0,0,0};
	Servo motor[4];
	double lastUpdate=0;
	double lastStab=0;

	Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
	Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
	Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);

	sensors_event_t accel_event;
	sensors_event_t mag_event;
	sensors_event_t gyro_event;
	float magOffset[3];
	float PIDS[3][3]={
		{5,0,1},
		{5,0,1},
		{5,0,1},
	};
	int pingPins[2]={
		99,
		99
	};
	Kalman kalmanX, kalmanY, kalmanZ;
};

#endif
