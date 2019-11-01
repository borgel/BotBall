#include "rotary.h"

#include <ComponentObject.h>
#include <RangeSensor.h>
#include <SparkFun_VL53L1X.h>
#include <vl53l1x_class.h>
#include <vl53l1_error_codes.h>

#include <Wire.h>

// TODO abstract these
const int distanceNShutdown = 2;
const int distanceInt = 1;
const int distanceSDA = 23;
const int distanceSCL = 22;

SFEVL53L1X distanceSensor(Wire1, distanceNShutdown, distanceInt);

void rotary_Begin(void) {
  /*
  pinMode(distanceInt, INPUT);
  pinMode(distanceSDA, OUTPUT);
  pinMode(distanceSCL, OUTPUT);
  */

  Wire1.begin();
  Wire1.setSDA(distanceSDA);
  Wire1.setSCL(distanceSCL);
  
  if(distanceSensor.begin() == 0) {
    Serial.println("sensor OK");
  }
  else {
    Serial.println("Failed to init distance sensor");
  }

  // TODO config window, etc
}

void rotary_Home(void) {
  
  
  distanceSensor.startRanging(); //Write configuration bytes to initiate measurement
  int distance = distanceSensor.getDistance(); //Get the result of the measurement from the sensor
  distanceSensor.stopRanging();
  Serial.print("Distance(mm): ");
  Serial.println(distance);
}
