//---------------------------------------------------------------------------------
// ToDo:
// - check serial monitor for when the garbage is being sent
//---------------------------------------------------------------------------------

#include <AnooDrone.h>
#include <AnooRC.h>
#include <Kalman.h>
#include <Wire.h>
#include <math.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_9DOF.h>
#include <Adafruit_NeoPixel.h>
#include <Servo.h>

#define GUI

#define BEEP_PIN      22
#define PIN           33
#define NUMPIXELS     16
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

double timer;
double dt;
Drone drone;
RCController controller;

void setAll(int r, int g, int b)
{
  for (int i = 0; i < NUMPIXELS; i++) {
    pixels.setPixelColor(i, pixels.Color(r, g, b));
  }
  pixels.show();
}

// duration in mSecs, frequency in hertz
void playTone(long duration, int freq) {
  duration *= 1000;
  int period = (1.0 / freq) * 1000000;
  long elapsed_time = 0;
  while (elapsed_time < duration) {
    digitalWrite(BEEP_PIN , HIGH);
    delayMicroseconds(period / 2);
    digitalWrite(BEEP_PIN , LOW);
    delayMicroseconds(period / 2);
    elapsed_time += (period);
  }
}

void setup()
{
  //  pinMode(BEEP_PIN, OUTPUT);
  //  playTone(100, 500);
  //  delay(100);
  //  playTone(100, 500);
  //  delay(100);
  //  playTone(100, 500);
  //  delay(100);
  Serial.begin(38400);
  Serial.println("Starting up...");
  //pixels.begin(); // This initializes the NeoPixel library.
  byte motorPins[] = {8, 9, 10, 11};
  byte channelPins[] = {2, 3, 4, 5, 6, 7};
  // pitch, roll, throttle, yaw, aux1, aux2
  if (!drone.init(motorPins) || !controller.init(channelPins, 6))
  {
    //playTone(500, 200);
    Serial.println(F("Error initialising drone or controller"));
    while (1);
  } else {
    //playTone(500, 1000);
  }
  //drone.setPingPins(32, 33);
  timer = micros();
}

void loop()
{
  dt = (double)(micros() - timer) / 1000000;
  timer = micros();

  controller.getChannelData();
  drone.updateOrientation();
  
  static float elapsed = 0;
  static bool timing;
  static bool toggled;
  bool leftStickBottomRight = controller.pitch > 1800 && controller.roll < 1200;
  bool rightStickBottomLeft = controller.throttle < 1200 && controller.yaw > 1800;
  if (leftStickBottomRight && rightStickBottomLeft)
  {
    timing = true;
    elapsed += dt;
    if (elapsed > 1 && !toggled)
    {
      drone.armed = !drone.armed;
      toggled = true;
      Serial.println();
      Serial.print("ARMED: ");
      Serial.println(drone.armed);
      Serial.println();
    }
  } else {
    timing = false;
    elapsed = 0;
    toggled = false;
  }

  if (controller.aux2 > 1500) //aux switch 2 is on
  {
    drone.armed = false;
  }

  if (drone.armed)
  {
    setAll(50, 0, 0);

    if (controller.aux1 > 1500) { //aux switch 1 is on
      drone.manualControl(&controller, 100);
    } else { //aux switch 1 is off
      if (controller.throttle > 1100) {
        drone.stabilise(&controller);
      } else { // the throttle is at the bottom so we'll just turn all of the motors off
        drone.kill();
      }
    }
  } else {
    drone.kill();
    setAll(0, 50, 0);
  }

  drone.updateMotors();

#ifdef GUI

  Serial.println();
  Serial.print("t");
  for (int i = 0; i < 4; i++) {
    Serial.print(',');
    Serial.print(drone.throttle[i]);
  }
  Serial.println();

  Serial.print("v");
  for (int i = 0; i < 3; i++) {
    Serial.print(',');
    Serial.print(drone.throttleTarget[i]);
  }
  Serial.println();

  Serial.print("a,");
  Serial.println(drone.armed);

  Serial.print("z,");
  Serial.print(drone.kalman[0]);
  Serial.print(",");
  Serial.print(drone.kalman[1]);
  Serial.print(",");
  Serial.println(drone.kalman[2]);

  //  Serial.println();
  //  Serial.print("z,");
  //  Serial.print(drone.orientation.pitch);
  //  Serial.print(",");
  //  Serial.print(drone.orientation.roll);
  //  Serial.print(",");
  //  Serial.println(drone.orientation.heading);

  Serial.print("h,");
  Serial.print(drone.orientation.height);
  Serial.print(",");
  Serial.println(dt * 1000);
#endif
}
