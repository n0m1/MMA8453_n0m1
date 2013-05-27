/************************************************************************************
 * 	
 * 	Name    : MMA8453_n0m1 Library Example: MotionMode                       
 * 	Author  : Noah Shibley, NoMi Design Ltd. http://n0m1.com                       
 *		    : Michael Grant, Krazatchu Design Systems. http://krazatchu.ca/
 * 	Date    : May 5th 2013                                    
 * 	Version : 0.2                                              
 * 	Notes   : Arduino Library for use with the Freescale MMA8453Q via Arduino native WIRE with repeated start (was i2c of DSS circuits). 
 *
 ***********************************************************************************/

#include <Wire.h>
#include <MMA8453_n0m1.h>

MMA8453_n0m1 accel;

void setup()
{
  Serial.begin(9600);
  accel.setI2CAddr(0x1D); //change your device address if necessary, default is 0x1C
  /*
  threshold [0-127] formula: 0.5g/ 0.063g = 7.9 counts, round to 8 counts ,
  enable X, 
  enable Y,
  enable Z, 
  enable MMA8453Q INT pin 2 (false= pin 1), 
  arduino INT pin number
  */
  accel.motionMode(8,true,true,true,false,2);
  
  Serial.println("MMA8453_n0m1 library");
  Serial.println("Motion Example");
  Serial.println("n0m1.com & krazatchu.ca");
}

void loop()
{
  accel.update();
  
  if(accel.motion())
  {
   Serial.println("motion!");
    
  }
  
  
  
}


