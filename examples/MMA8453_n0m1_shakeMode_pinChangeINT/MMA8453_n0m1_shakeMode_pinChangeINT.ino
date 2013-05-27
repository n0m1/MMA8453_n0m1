/************************************************************************************
 * 	
 * 	Name    : MMA8453_n0m1 Library Example: ShakeMode PinChange INT                         
 * 	Author  : Noah Shibley, NoMi Design Ltd. http://n0m1.com                       
 *		    : Michael Grant, Krazatchu Design Systems. http://krazatchu.ca/
 * 	Date    : May 5th 2013                                    
 * 	Version : 0.2                                                                         
 * 	Notes   : Arduino Library for use with the Freescale MMA8453Q via Arduino native WIRE with repeated start (was i2c of DSS circuits). 
 *   Instruction:
 *                uncomment #define PINCHANGE_INT in MMA8453_n0m1.h to use INT pins other 
 *                then arduino pins 2 & 3 include the library 
 *                from: http://code.google.com/p/arduino-pinchangeint/ 
 ***********************************************************************************/

#include <PinChangeInt.h>
#include <PinChangeIntConfig.h>
#include <Wire.h>
#include <MMA8453_n0m1.h>

MMA8453_n0m1 accel;

void setup()
{
  Serial.begin(9600);
  accel.setI2CAddr(0x1D); //change your device address if necessary, default is 0x1C
  /*
  threshold [0-127] formula: 6g/ 0.063g = 95.2 counts, round to 96 counts ,
   enable X, 
   enable Y,
   enable Z, 
   enable MMA8453Q INT pin 2 (false= pin 1), 
   arduino INT pin number
   */
  accel.shakeMode(32,true,true,true,false,4);
  Serial.println("MMA8453_n0m1 library");
  Serial.println("PinChange INT Shake Example");
  Serial.println("n0m1.com & krazatchu.ca");
}

void loop()
{
  accel.update();

  if(accel.shake())
  {

    if(accel.shakeAxisX()) //X
    {
      Serial.println("Shake X");
    }

    if(accel.shakeAxisY()) //Y
    {
      Serial.println("Shake Y");
    }

    if(accel.shakeAxisZ()) //Z
    {
      Serial.println("Shake Z");
    }

  }



}



