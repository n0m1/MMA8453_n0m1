/************************************************************************************
 * 	
 * 	Name    : MMA8453_n0m1 Library Example: MotionMode                       
 * 	Author  : Noah Shibley, Michael Grant, NoMi Design Ltd. http://n0m1.com                       
 * 	Date    : Feb 10th 2012                                    
 * 	Version : 0.1                                              
 * 	Notes   : Arduino Library for use with the Freescale MMA8453Q via i2c. 
 *
 ***********************************************************************************/

#include <I2C.h>
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
  Serial.println("n0m1.com");
}

void loop()
{
  accel.update();
  
  if(accel.motion())
  {
   Serial.println("motion!");
    
  }
  
  
  
}


