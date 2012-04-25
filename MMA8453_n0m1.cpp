/************************************************************************************
 * 	
 * 	Name    : MMA8453_n0m1 Library                         
 * 	Author  : Noah Shibley, Michael Grant, NoMi Design Ltd. http://n0m1.com                       
 * 	Date    : Feb 10th 2012                                    
 * 	Version : 0.1                                              
 * 	Notes   : Arduino Library for use with the Freescale MMA8453Q via i2c. 
              Some of the lib source from Kerry D. Wong
			  http://www.kerrywong.com/2012/01/09/interfacing-mma8453q-with-arduino/
 * 
 * 
 * 	This file is part of MMA8453_n0m1.
 * 
 * 		    MMA8453_n0m1 is free software: you can redistribute it and/or modify
 * 		    it under the terms of the GNU General Public License as published by
 * 		    the Free Software Foundation, either version 3 of the License, or
 * 		    (at your option) any later version.
 * 
 * 		    AtTouch is distributed in the hope that it will be useful,
 * 		    but WITHOUT ANY WARRANTY; without even the implied warranty of
 * 		    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * 		    GNU General Public License for more details.
 * 
 * 		    You should have received a copy of the GNU General Public License
 * 		    along with MMA8453_n0m1.  If not, see <http://www.gnu.org/licenses/>.
 * 
 ***********************************************************************************/

#include "MMA8453_n0m1.h"

MMA8453_n0m1* MMA8453_n0m1::pMMA8453_n0m1 = 0; 

MMA8453_n0m1::MMA8453_n0m1()
{
	pMMA8453_n0m1 = this;
	dataMode_ = false;
	shakeMode_ = false; 
	ISRFlag = false;
	shake_ = false;
	shakeAxisX_ = false;
	shakeAxisY_ = false;
	shakeAxisZ_ = false;
	I2CAddr = 0x1c; //The i2C address of the MMA8453 chip. 0x1D is another common value.
	gScaleRange_ = 2;  //default 2g

}

/***********************************************************
 * 
 * setI2CAddr
 *
 * 
 *   
 ***********************************************************/
void MMA8453_n0m1::setI2CAddr(int address)
{
	I2CAddr = address; I2CAddr; 
}

/***********************************************************
 * 
 * update
 *
 * 
 *   
 ***********************************************************/
void MMA8453_n0m1::update()
{
	if(dataMode_)
	{
		xyz(x_,y_,z_);
	}
	
	if(shakeMode_ == true || motionMode_ == true)
	{
		clearInterrupt();
	}
	
}

/***********************************************************
 * 
 * clearInterrupt
 *
 * 
 *   
 ***********************************************************/
void MMA8453_n0m1::clearInterrupt()
{
	
  if(ISRFlag)
  {	
	
	 byte sourceSystem;
	 I2c.read(I2CAddr,REG_INT_SOURCE,byte(1),&sourceSystem); //check which system fired the interrupt
     	
	if((sourceSystem&0x20) == 0x20) //Transient
	{	
		  //Perform an Action since Transient Flag has been set
	      //Read the Transient to clear system interrupt and Transient
	      byte srcTrans;
		  shake_ = true;
	      I2c.read(I2CAddr,REG_TRANSIENT_SRC ,byte(1),&srcTrans);
		
		  if(srcTrans&0x02 == 0x02)
		  {
			shakeAxisX_ = true;
		  }
		  if(srcTrans&0x08 == 0x08)
		  {
			shakeAxisY_ = true;
		  }
		  if(srcTrans&0x20 == 0x20)
		  {
			shakeAxisZ_ = true;
		  }
		
	}
	
	if((sourceSystem&0x04) == 0x04) //FreeFall Motion
	{
	      byte srcFF;
	      I2c.read(I2CAddr,REG_FF_MT_SRC ,byte(1),&srcFF);
		  motion_ = true;
	
	}

	ISRFlag = false; 

  }

}



/*************************************************************
* 
* xyz
* 
* Get accelerometer readings (x, y, z)
* by default, standard 10 bits mode is used.
* 
* This function also convers 2's complement number to
* signed integer result.
* 
* If accelerometer is initialized to use low res mode,
* isHighRes must be passed in as false.
*
*************************************************************/
void MMA8453_n0m1::xyz(int& x, int& y, int& z)
{

  byte buf[6];

  if (highRes_) 
  {
    I2c.read(I2CAddr, REG_OUT_X_MSB, 6, buf);
    x = buf[0] << 2 | buf[1] >> 6 & 0x3;
    y = buf[2] << 2 | buf[3] >> 6 & 0x3;
    z = buf[4] << 2 | buf[5] >> 6 & 0x3;
  }
  else 
  {
    I2c.read(I2CAddr, REG_OUT_X_MSB, 3, buf);
    x = buf[0] << 2;
    y = buf[1] << 2;
    z = buf[2] << 2;
  }

  if (x > 511) x = x - 1024;
  if (y > 511) y = y - 1024 ;
  if (z > 511) z = z - 1024;
  
}

/***********************************************************
 * 
 * dataMode
 *
 * 
 *   
 ***********************************************************/
void MMA8453_n0m1::dataMode(boolean highRes, int gScaleRange)
{
	highRes_ = highRes;
	gScaleRange_ = gScaleRange;
	dataMode_ = true;
	byte statusCheck;
	byte activeMask = 0x01;
	byte resModeMask = 0x02;
	
	//setup i2c
	I2c.begin();
	
	//register settings must be made in standby mode
	I2c.read(I2CAddr,REG_CTRL_REG1,byte(1),&statusCheck);
    I2c.write(I2CAddr, REG_CTRL_REG1, byte(statusCheck & ~activeMask));
	
	if( gScaleRange_ <= 3){ gScaleRange_ = FULL_SCALE_RANGE_2g; } //0-3 = 2g
	else if( gScaleRange_ <= 5){ gScaleRange_ = FULL_SCALE_RANGE_4g; } //4-5 = 4g
	else if( gScaleRange_ <= 8){ gScaleRange_ = FULL_SCALE_RANGE_8g; }// 6-8 = 8g
	else if( gScaleRange_ > 8) { gScaleRange_ = FULL_SCALE_RANGE_8g; } //boundary
	I2c.write(I2CAddr,REG_XYZ_DATA_CFG, byte(gScaleRange_));
    
    //set highres 10bit or lowres 8bit
    I2c.read(I2CAddr,REG_CTRL_REG1,byte(1),&statusCheck);	
	if(highRes){
	    I2c.write(I2CAddr, REG_CTRL_REG1, byte(statusCheck & ~resModeMask));
	}
    else { 
  		I2c.write(I2CAddr, REG_CTRL_REG1, byte(statusCheck | resModeMask));	    
	}
 
    //active Mode
 	I2c.read(I2CAddr,REG_CTRL_REG1,byte(1),&statusCheck);
    I2c.write(I2CAddr, REG_CTRL_REG1, byte(statusCheck | activeMask));
}

/***********************************************************
 * 
 * shakeMode
 *
 * 
 *   
 ***********************************************************/
void MMA8453_n0m1::shakeMode(int threshold, boolean enableX, boolean enableY, boolean enableZ, boolean enableINT2,int arduinoINTPin)
{
	 if(arduinoINTPin == 2 || arduinoINTPin == 3)
	 {
		arduinoINTPin = arduinoINTPin - 2;
		attachInterrupt(arduinoINTPin,accelISR,FALLING); 
		//DataSheet pg40, When IPOL is ‘0’ (default value) any interrupt event will signaled with a logical 0
	 }
	
	 else
	 {	
		 #ifdef PINCHANGE_INT
		 	pinMode(arduinoINTPin, INPUT); digitalWrite(arduinoINTPin, HIGH);
		 	PCintPort::attachInterrupt(arduinoINTPin,accelISR,FALLING);  
		 #else
		 	Serial.println("no INT on pin, define PINCHANGE_INT");	
		 #endif
	 }

	
	 boolean error = false;
	 byte statusCheck;
	
	 //setup i2c
	 I2c.begin();
	
	 I2c.write(I2CAddr, REG_CTRL_REG1, byte(0x18)); //Set device in 100 Hz ODR, Standby
	
	 byte xyzCfg = 0x10; //latch always enabled
	 if(enableX) xyzCfg |= 0x02;
	 if(enableY) xyzCfg |= 0x04;
	 if(enableZ) xyzCfg |= 0x08;
	
	 I2c.write(I2CAddr, REG_TRANSIENT_CFG, xyzCfg);  //XYZ + latch 0x1E
	 I2c.read(I2CAddr, REG_TRANSIENT_CFG, byte(1), &statusCheck);
	 if(statusCheck != xyzCfg) error = true;

	
	 if(threshold > 127) threshold = 127; //8g is the max.
	 I2c.write(I2CAddr, REG_TRANSIENT_THS, byte(threshold));  //threshold 
	 I2c.read(I2CAddr, REG_TRANSIENT_THS, byte(1), &statusCheck);
	 if(statusCheck != byte(threshold)) error = true;

	 
	 I2c.write(I2CAddr, REG_TRANSIENT_COUNT, byte(0x05)); //Set the Debounce Counter for 50 ms
	 I2c.read(I2CAddr,REG_TRANSIENT_COUNT, byte(1), &statusCheck);
	 if(statusCheck != 0x05) error = true;

	 I2c.read(I2CAddr, REG_CTRL_REG4, byte(1), &statusCheck);
	 statusCheck |= 0x20;
	 I2c.write(I2CAddr, REG_CTRL_REG4, statusCheck);  //Enable Transient Detection Interrupt in the System
	  	
	 byte intSelect = 0x20;
	 if(enableINT2) intSelect = 0x00;
	 I2c.read(I2CAddr, REG_CTRL_REG5, byte(1), &statusCheck);
	 statusCheck |= intSelect;
	 I2c.write(I2CAddr, REG_CTRL_REG5, statusCheck); //INT2 0x0, INT1 0x20 

	 I2c.read(I2CAddr, REG_CTRL_REG1, byte(1), &statusCheck); //Read out the contents of the register
	 statusCheck |= 0x01; //Change the value in the register to Active Mode.
	 I2c.write(I2CAddr, REG_CTRL_REG1, statusCheck); //Write in the updated value to put the device in Active Mode
	
	if(error)
	{
		Serial.println("Shake mode setup error");
		Serial.println("retrying...");
		delay(100);
		shakeMode(threshold,enableX,enableY,enableZ,enableINT2,arduinoINTPin);
	}
	
	shakeMode_ = true;
	
}
/***********************************************************
 * 
 * motionMode
 *
 * 
 *   
 ***********************************************************/
void MMA8453_n0m1::motionMode(int threshold, boolean enableX, boolean enableY, boolean enableZ, boolean enableINT2,int arduinoINTPin)
{
	 if(arduinoINTPin == 2 || arduinoINTPin == 3)
	 {
		arduinoINTPin = arduinoINTPin - 2;
		attachInterrupt(arduinoINTPin,accelISR,FALLING); 
		//DataSheet pg40, When IPOL is ‘0’ (default value) any interrupt event will signaled with a logical 0
	 }
	
	 else
	 {	
		 #ifdef PINCHANGE_INT
		 	pinMode(arduinoINTPin, INPUT); digitalWrite(arduinoINTPin, HIGH);
		 	PCintPort::attachInterrupt(arduinoINTPin,accelISR,FALLING);  
		 #else
		 	Serial.println("no INT on pin, define PINCHANGE_INT");	
		 #endif
	 }

	
	 boolean error = false;
	 byte statusCheck;
	
	 //setup i2c
	 I2c.begin();
	
	 I2c.write(I2CAddr, REG_CTRL_REG1, byte(0x18)); //Set device in 100 Hz ODR, Standby
	
	 byte xyzCfg = 0x80; //latch always enabled
	 xyzCfg |= 0x40; //Motion not free fall
	 if(enableX) xyzCfg |= 0x08;
	 if(enableY) xyzCfg |= 0x10;
	 if(enableZ) xyzCfg |= 0x20;
	
	 I2c.write(I2CAddr, REG_FF_MT_CFG, xyzCfg);  //XYZ + latch + motion
	 I2c.read(I2CAddr, REG_FF_MT_CFG, byte(1), &statusCheck);
	 if(statusCheck != xyzCfg) error = true;

	 if(threshold > 127) threshold = 127; //a range of 0-127.
	 I2c.write(I2CAddr, REG_FF_MT_THS, byte(threshold));  //threshold 
	 I2c.read(I2CAddr, REG_FF_MT_THS, byte(1), &statusCheck);
	 if(statusCheck != byte(threshold)) error = true;

	 I2c.write(I2CAddr, REG_FF_MT_COUNT, byte(0x0A)); //Set the Debounce Counter for 100 ms
	 I2c.read(I2CAddr,REG_FF_MT_COUNT, byte(1), &statusCheck);
	 if(statusCheck != 0x0A) error = true;

	 I2c.read(I2CAddr, REG_CTRL_REG4, byte(1), &statusCheck);
	 statusCheck |= 0x04;
	 I2c.write(I2CAddr, REG_CTRL_REG4, statusCheck); //Enable Motion Interrupt in the System

	 byte intSelect = 0x04;
	 if(enableINT2) intSelect = 0x00;
	 I2c.read(I2CAddr, REG_CTRL_REG5, byte(1), &statusCheck);
	 statusCheck |= intSelect;
	 I2c.write(I2CAddr, REG_CTRL_REG5, statusCheck); //INT2 0x0, INT1 0x04
	 
	 I2c.read(I2CAddr, REG_CTRL_REG1, byte(1), &statusCheck); //Read out the contents of the register
	 statusCheck |= 0x01; //Change the value in the register to Active Mode.
	 I2c.write(I2CAddr, REG_CTRL_REG1, statusCheck); //Write in the updated value to put the device in Active Mode
	
	if(error)
	{
		Serial.println("Motion mode setup error");
		Serial.println("retrying...");
		delay(100);
		motionMode(threshold,enableX,enableY,enableZ,enableINT2,arduinoINTPin);
	}
	
	motionMode_ = true;
}

/***********************************************************
 * 
 * regRead
 *
 * 
 *   
 ***********************************************************/
void MMA8453_n0m1::regRead(byte reg, byte *buf, byte count)
{
   I2c.read(I2CAddr, reg, count, buf);
}
 
/***********************************************************
 * 
 * regWrite
 *
 * 
 *   
 ***********************************************************/
void MMA8453_n0m1::regWrite(byte reg, byte val)
{
  I2c.write(I2CAddr, reg, val);
}

/***********************************************************
 * 
 * accelISR
 *
 * 
 *   
 ***********************************************************/
void accelISR(void){
	MMA8453_n0m1::pMMA8453_n0m1->ISRFlag = true;
}
