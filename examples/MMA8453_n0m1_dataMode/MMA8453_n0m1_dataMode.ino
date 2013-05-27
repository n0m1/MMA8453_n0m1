/************************************************************************************
 * 	
 * 	Name    : MMA8453_n0m1 Library Example: DataMode                       
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
  accel.dataMode(true, 2); //enable highRes 10bit, 2g range [2g,4g,8g]
  Serial.println("MMA8453_n0m1 library");
  Serial.println("XYZ Data Example");
  Serial.println("n0m1.com & krazatchu.ca");
}

void loop()
{
  accel.update();
  
  Serial.print("x: ");
  Serial.print(accel.x());
  Serial.print(" y: ");
  Serial.print(accel.y());
  Serial.print(" z: ");
  Serial.println(accel.z());

}



