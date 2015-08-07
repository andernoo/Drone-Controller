/*
  Drone.cpp - A library for controlling a
  quadcopter using sensor data and RxTx.
  Created by Andrew Downey on 31/3/15
*/
#include "AnooDrone.h"
#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

void write8(byte address, byte reg, byte value)
{
	Wire.beginTransmission(address);
	#if ARDUINO >= 100
	Wire.write((uint8_t)reg);
	Wire.write((uint8_t)value);
	#else
	Wire.send(reg);
	Wire.send(value);
	#endif
	Wire.endTransmission();
}

bool Drone::init(byte motorPins[4])
{
	if(!Serial)
	{
		Serial.begin(115200);
	}
	if (!gyro.begin() || !accel.begin() || !mag.begin())
	{
		Serial.println("Error starting sensors");
		return false;
	}
	mag.enableAutoRange(true);
	gyro.enableAutoRange(true);
	
	//fix for the accel being too sensitive
	//write8(LSM303_ADDRESS_ACCEL, LSM303_REGISTER_ACCEL_CTRL_REG1_A, 0b10000111);
	
	//fix for the gyro being too sensitive
	//write8(L3GD20_ADDRESS, GYRO_REGISTER_CTRL_REG1, 0b0000 << 4);
	
	Serial.println("Sensors Okay");

	#define MagMinX -47.73
	#define MagMaxX 59.36
	magOffset[0]=(MagMaxX + MagMinX) / 2;
	#define MagMinY -46.27
	#define MagMaxY 54.73
	magOffset[1]=(MagMaxY + MagMinY) / 2;
	#define MagMinZ -61.94
	#define MagMaxZ 51.02
	magOffset[2]=(MagMaxZ + MagMinZ) / 2;

	for (int i = 0; i < 4; i++) {
		motor[i].attach(motorPins[i]);
		motor[i].writeMicroseconds(0);
	}
	delay(200);
	getAccelOffset();
	updateOrientation();
	kalmanX.setAngle(orientation.roll);
	kalmanY.setAngle(orientation.pitch);
	kalmanZ.setAngle(orientation.heading);
	Serial.println("Drone initialised");
	return true;
}

#define ROLL 0
#define PITCH 1
#define YAW 2

void Drone::setPID(int axis, int p, int i, int d)
{
	PIDS[axis][0]=p;
	PIDS[axis][1]=i;
	PIDS[axis][2]=d;
}

void Drone::getAccelOffset()
{
	unsigned int xOffset;
	unsigned int yOffset;
	unsigned int zOffset;

	#define ITERATIONS 60
	for (int i = 1; i <= ITERATIONS; i++) {
		accel.getEvent(&accel_event);
		xOffset += accel_event.acceleration.x;
		yOffset += accel_event.acceleration.y;
		zOffset += accel_event.acceleration.z;
	}
	xOffset /= ITERATIONS;
	yOffset /= ITERATIONS;
	zOffset /= ITERATIONS;
	Serial.print("xOffset: ");
	Serial.print(xOffset);
	Serial.print("   yOffset: ");
	Serial.print(yOffset);
	Serial.print("   zOffset: ");
	Serial.println(zOffset);
}

void Drone::setPingPins(int trig, int echo)
{
	pingPins[0]=trig;
	pingPins[1]=echo;
	pinMode(pingPins[0], OUTPUT);
	pinMode(pingPins[1], INPUT);
}

long Drone::getHeightCM()
{
	if(pingPins[0]==99)
		return 0;
	// The sensor is triggered by a HIGH pulse of 10 or more microseconds.
	// Give a short LOW pulse beforehand to ensure a clean HIGH pulse:

	digitalWrite(pingPins[0], LOW);
	delayMicroseconds(2);
	digitalWrite(pingPins[0], HIGH);
	delayMicroseconds(10);
	digitalWrite(pingPins[0], LOW);

	// Read the signal from the sensor: a HIGH pulse whose
	// duration is the time (in microseconds) from the sending
	// of the ping to the reception of its echo off of an object.
	long duration = pulseIn(pingPins[1], HIGH);
	// The speed of sound is 340 m/s or 29 microseconds per centimeter.
	// The ping travels out and back, so to find the distance of the
	// object we take half of the distance travelled.
	orientation.height = duration / 29. / 2.;
}

void Drone::updateOrientation() 
{
	accel.getEvent(&accel_event);
	mag.getEvent(&mag_event);
	gyro.getEvent(&gyro_event);

	double rollRate = gyro_event.gyro.x * RAD_TO_DEG;
	double pitchRate = gyro_event.gyro.y * RAD_TO_DEG;
	double yawRate = gyro_event.gyro.z * RAD_TO_DEG;

	float const PI_F = 3.14159265F;

	//mag_event.magnetic.x -= magOffset[0];
	//mag_event.magnetic.y -= magOffset[1];
	//mag_event.magnetic.z -= magOffset[2];

	orientation.roll = (float)atan2(accel_event.acceleration.y, accel_event.acceleration.z);

	if (accel_event.acceleration.y * sin(orientation.roll) + accel_event.acceleration.z * cos(orientation.roll) == 0)
	{
		orientation.pitch = accel_event.acceleration.x > 0 ? (PI_F / 2) : (-PI_F / 2);
	}else{
		orientation.pitch = (float)atan(-accel_event.acceleration.x / (accel_event.acceleration.y * sin(orientation.roll) + accel_event.acceleration.z * cos(orientation.roll)));
	}

	orientation.heading = (float)atan2(mag_event.magnetic.z * sin(orientation.roll) - mag_event.magnetic.y * cos(orientation.roll), mag_event.magnetic.x * cos(orientation.pitch) + mag_event.magnetic.y * sin(orientation.pitch) * sin(orientation.roll) + mag_event.magnetic.z * sin(orientation.pitch) * cos(orientation.roll));

	orientation.roll = orientation.roll * RAD_TO_DEG;
	orientation.pitch = orientation.pitch * RAD_TO_DEG;
	orientation.heading = orientation.heading * RAD_TO_DEG;
	
	double dt=(double)(micros()-lastUpdate);
	lastUpdate=micros();

	kalman[1] = kalmanY.getAngle(orientation.pitch, pitchRate, dt);
	//fix for inverse pitch
	kalman[1]*=-1;

	if (abs(orientation.pitch) > 90)
	{
		rollRate *=-1;
	}
	kalman[0] = kalmanX.getAngle(orientation.roll, rollRate, dt);

	kalman[2] = kalmanZ.getAngle(orientation.heading, yawRate, dt);

	Drone::getHeightCM();	
}

void Drone::manualControl(RCController *controller, uint8_t limit)
{
	throttleTarget[0] = map(controller->roll, 1000, 2000, -limit, limit);
	throttleTarget[1] = map(controller->pitch, 1000, 2000, -limit, limit);
	throttleTarget[2] = map(controller->yaw, 1000, 2000, -limit, limit);
	collective=controller->throttle;
}

void Drone::stabilise(RCController *controller)
{
	float error[3]={0,0,0};
	float desired[3]={0,0,0};

	//channelData 0=roll 1=pitch 3=yaw
	desired[1] = map(controller->pitch, 1000, 2000, -15, 15);
	desired[0] = map(controller->roll, 1000, 2000, -15, 15);
	desired[2] = map(controller->yaw, 1000, 2000, -100, 100);
	collective=controller->throttle;

	double dt=(double)(micros()-lastStab);
	lastStab=micros();
	
	for(int axis=0; axis<2; axis++)
	{
		error[axis] = desired[axis]-kalman[axis];
		float derivative = error[axis]-lastError[axis] / dt;

		throttleTarget[axis] = PIDS[axis][0] * error[axis] - PIDS[axis][2] * derivative;
		lastError[axis]=error[axis];
	}
}

void Drone::kill()
{
	throttleTarget[0] = 0;
	throttleTarget[1] = 0;
	throttleTarget[2] = 0;
	collective=0;
	updateMotors();
}

void Drone::updateMotors()
{
	//throttleTarget 0=Roll 1=Pitch 2=Yaw
	throttle[0] =   collective + throttleTarget[0] + throttleTarget[1] + throttleTarget[2];
	throttle[1] =   collective - throttleTarget[0] + throttleTarget[1] - throttleTarget[2];
	throttle[2] =   collective + throttleTarget[0] - throttleTarget[1] - throttleTarget[2];
	throttle[3] =   collective - throttleTarget[0] - throttleTarget[1] + throttleTarget[2];
	for (int i = 0; i < 4; i++) {
		motor[i].writeMicroseconds(throttle[i]);
	}
}