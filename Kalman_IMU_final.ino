#include <ADXL345.h>
//#include <bma180.h>
#include <HMC58X3.h>
#include <ITG3200.h>
//#include <MS561101BA.h>
#include <I2Cdev.h>
//#include <MPU60X0.h>
#include <EEPROM.h>

//#define DEBUG
#include "DebugUtils.h"
#include "FreeIMU.h"
#include <Wire.h>
#include <SPI.h>
#include <Kalman.h>

#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf
#define DEBUG_RAW 0 //Turn on/off debug logging
#define DEBUG_YPR 1
#define DEBUG_KAL 1

//Three instances for roll, pitch, yaw
Kalman kalmanP; // Create the Kalman instances
Kalman kalmanR;
Kalman kalmanY;

//Angle calculated using Kalman Filter
double kal_roll, kal_pitch, kal_yaw;
double roll, pitch, yaw;

float val[11];
uint32_t timer;

FreeIMU razor_imu = FreeIMU();

void setup()
{
  Serial.begin(115200);
  Wire.begin();
  
  delay(500);
  //parameter enables/disables fast mode
  razor_imu.init(true);
  delay(500);
  razor_imu.getValues(val);
  
  //Pitch, roll, yaw calculations
  roll = atan(val[1]/sqrt(val[1]*val[1] + val[2]*val[2])) * RAD_TO_DEG;
  pitch = atan2(-val[0], val[2]) * RAD_TO_DEG;
  yaw = atan2((-val[6]*cos(roll) + val[8]*sin(roll)), 
              (val[6]*cos(pitch) + val[7]*sin(pitch)*sin(roll)+ val[8]*sin(pitch)*cos(roll)));

  //Set starting angle for filters
  kalmanR.setAngle(roll*RAD_TO_DEG);
  kalmanP.setAngle(pitch*RAD_TO_DEG);
  kalmanY.setAngle(yaw*RAD_TO_DEG);
  
  timer = micros();
}

void loop()
{
  razor_imu.getValues(val);
  double dt = (double)(micros() - timer)/1000000;
  timer = micros();
  
  //Calculate pitch, roll, yaw
  roll = atan(val[1]/sqrt(val[1]*val[1] + val[2]*val[2]));
  pitch = atan2(-val[0], val[2]);  
  yaw = atan2((-val[6]*cos(roll) + val[8]*sin(roll)), 
              (val[6]*cos(pitch) + val[7]*sin(pitch)*sin(roll)+ val[8]*sin(pitch)*cos(roll)));
  
  double roll_deg = roll * RAD_TO_DEG;
  double pitch_deg = pitch * RAD_TO_DEG;
  double yaw_deg = yaw * RAD_TO_DEG;  
  double roll_rate = val[3] / 131.0;
  double pitch_rate = val[4] / 131.0;
  double yaw_rate = val[5] / 131.0;
  
  //Feed values into the kalman object
  //And get the filtered angle back
  kalmanR.setAngle(roll_deg);
  kal_roll = kalmanR.getAngle(roll_deg, roll_rate, dt);
  
  kalmanP.setAngle(pitch*RAD_TO_DEG);
  kal_pitch = kalmanP.getAngle(pitch_deg, pitch_rate, dt);
  
  kalmanY.setAngle(yaw*RAD_TO_DEG);
  kal_yaw = kalmanY.getAngle(yaw_deg, yaw_rate, dt);
  
  
  //Printing raw values (For debugging)
  if(DEBUG_RAW){
    for(int i=0; i<11; i++)
    {
      Serial.print(val[i]);
      Serial.print("\t");
    }
  }
  
  if(DEBUG_YPR){
    //Serial.print("YPR: ");
    Serial.print(yaw_deg);
    Serial.print(",");
    Serial.print(pitch_deg);
    Serial.print(",");
    Serial.print(roll_deg);
  }
  Serial.print(",");
  if(DEBUG_KAL){
    //Serial.print("YPR: ");
    Serial.print(kal_yaw);
    Serial.print(",");
    Serial.print(kal_pitch);
    Serial.print(",");
    Serial.print(kal_roll);
  }
  Serial.print(millis());
  Serial.print("\n");
  
}
  

