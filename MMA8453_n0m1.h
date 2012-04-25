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


#ifndef MMA8453_N0M1_H
#define MMA8453_N0M1_H

//uncomment PINCHANGE_INT to use INT pins other then arduino pins 2 & 3
//include the library from: http://code.google.com/p/arduino-pinchangeint/

//#define PINCHANGE_INT

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"    
#else
#include "WProgram.h"
#endif

#ifdef PINCHANGE_INT
	#include <PinChangeInt.h>
	#include <PinChangeIntConfig.h>
#endif

#include <I2C.h>

const byte REG_STATUS = 0x00; //(R) Real time status
const byte REG_OUT_X_MSB = 0x01; //(R) [7:0] are 8 MSBs of 10-bit sample
const byte REG_OUT_X_LSB = 0x02; //(R) [7:6] are 2 LSBs of 10-bit sample
const byte REG_OUT_Y_MSB = 0x03; //(R) [7:0] are 8 MSBs of 10-bit sample
const byte REG_OUT_Y_LSB = 0x04; //(R) [7:6] are 2 LSBs of 10-bit sample
const byte REG_OUT_Z_MSB = 0x05; //(R) [7:0] are 8 MSBs of 10-bit sample
const byte REG_OUT_Z_LSB = 0x06; //(R) [7:6] are 2 LSBs of 10-bit sample
const byte REG_SYSMOD = 0x0b; //(R) Current system mode
const byte REG_INT_SOURCE = 0x0c; //(R) Interrupt status
const byte REG_WHO_AM_I = 0x0d; //(R) Device ID (0x3A)
const byte REG_XYZ_DATA_CFG = 0xe; //(R/W) Dynamic range settings
const byte REG_HP_FILTER_CUTOFF = 0x0f; //(R/W) cut-off frequency is set to 16Hz @ 800Hz
const byte REG_PL_STATUS = 0x10; //(R) Landscape/Portrait orientation status
const byte REG_PL_CFG = 0x11; //(R/W) Landscape/Portrait configuration
const byte REG_PL_COUNT = 0x12; //(R) Landscape/Portrait debounce counter
const byte REG_PL_BF_ZCOMP = 0x13; //(R) Back-Front, Z-Lock trip threshold
const byte REG_P_L_THS_REG = 0x14; //(R/W) Portrait to Landscape trip angle is 29 degree
const byte REG_FF_MT_CFG = 0x15; //(R/W) Freefall/motion functional block configuration
const byte REG_FF_MT_SRC = 0x16; //(R) Freefall/motion event source register
const byte REG_FF_MT_THS = 0x17; //(R/W) Freefall/motion threshold register
const byte REG_FF_MT_COUNT = 0x18; //(R/W) Freefall/motion debounce counter
const byte REG_TRANSIENT_CFG = 0x1d; //(R/W) Transient functional block configuration
const byte REG_TRANSIENT_SRC = 0x1e; //(R) Transient event status register
const byte REG_TRANSIENT_THS = 0x1f; //(R/W) Transient event threshold
const byte REG_TRANSIENT_COUNT = 0x20; //(R/W) Transient debounce counter
const byte REG_PULSE_CFG = 0x21; //(R/W) ELE, Double_XYZ or Single_XYZ
const byte REG_PULSE_SRC = 0x22; //(R) EA, Double_XYZ or Single_XYZ
const byte REG_PULSE_THSX = 0x23; //(R/W) X pulse threshold
const byte REG_PULSE_THSY = 0x24; //(R/W) Y pulse threshold
const byte REG_PULSE_THSZ = 0x25; //(R/W) Z pulse threshold
const byte REG_PULSE_TMLT = 0x26; //(R/W) Time limit for pulse
const byte REG_PULSE_LTCY = 0x27; //(R/W) Latency time for 2nd pulse
const byte REG_PULSE_WIND = 0x28; //(R/W) Window time for 2nd pulse
const byte REG_ASLP_COUNT = 0x29; //(R/W) Counter setting for auto-sleep
const byte REG_CTRL_REG1 = 0x2a; //(R/W) ODR = 800 Hz, STANDBY mode
const byte REG_CTRL_REG2 = 0x2b; //(R/W) Sleep enable, OS Modes, RST, ST
const byte REG_CTRL_REG3 = 0x2c; //(R/W) Wake from sleep, IPOL, PP_OD
const byte REG_CTRL_REG4 = 0x2d; //(R/W) Interrupt enable register
const byte REG_CTRL_REG5 = 0x2e; //(R/W) Interrupt pin (INT1/INT2) map
const byte REG_OFF_X = 0x2f; //(R/W) X-axis offset adjust
const byte REG_OFF_Y = 0x30; //(R/W) Y-axis offset adjust
const byte REG_OFF_Z = 0x31; //(R/W) Z-axis offset adjust

const byte FULL_SCALE_RANGE_2g = 0x0;
const byte FULL_SCALE_RANGE_4g = 0x1;
const byte FULL_SCALE_RANGE_8g = 0x2;



extern "C" void accelISR(void) __attribute__ ((signal)); 


class MMA8453_n0m1 {

public:
  friend void accelISR(void); //make friend so bttnPressISR can access private var keyhit	
 
  MMA8453_n0m1();

/***********************************************************
 * 
 * setI2CAddr
 *
 * set the i2c address of the MMA8453 to a new value, such as 0x1D
 *   
 ***********************************************************/
void setI2CAddr(int address);

/***********************************************************
 * 
 * dataMode
 *
 * set the device to return raw data values
 *   
 ***********************************************************/  
void dataMode(boolean highRes, int gScaleRange);

/***********************************************************
 * 
 * x
 *
 * returns the x axis value
 *   
 ***********************************************************/
int x(){ return x_; }

/***********************************************************
 * 
 * y
 *
 * returns the y axis value
 *   
 ***********************************************************/
int y() { return y_; }

/***********************************************************
 * 
 * z
 *
 * returns the z axis value
 *   
 ***********************************************************/
int z(){ return z_; }

/***********************************************************
 * 
 * shakeMode
 *
 *  set to transient detection mode
 *   
 ***********************************************************/
void shakeMode(int threshold, boolean enableX, boolean enableY, boolean enableZ, boolean enableINT2,int arduinoINTPin);

/***********************************************************
 * 
 * shake
 *
 * returns true if there is shaking (high pass filtered motion)
 *   
 ***********************************************************/
boolean shake() { boolean shakeOut = shake_; shake_ = false; return shakeOut; }

/***********************************************************
 * 
 * shakeAxisX
 *
 * returns true if there is shake on the x axis
 *   
 ***********************************************************/
boolean shakeAxisX() { boolean shakeAxisOut = shakeAxisX_; shakeAxisX_ = false; return shakeAxisOut; }

/***********************************************************
 * 
 * shakeAxisY
 *
 * returns true if there is shake on the y axis
 *   
 ***********************************************************/
boolean shakeAxisY() { boolean shakeAxisOut = shakeAxisY_; shakeAxisY_ = false; return shakeAxisOut; }

/***********************************************************
 * 
 * shakeAxisZ
 *
 * returns true if there is shake on the z axis
 *   
 ***********************************************************/
boolean shakeAxisZ() { boolean shakeAxisOut = shakeAxisZ_; shakeAxisZ_ = false; return shakeAxisOut; }

/***********************************************************
 * 
 * motionMode
 *
 * set to motion detection mode
 *   
 ***********************************************************/
void motionMode(int threshold, boolean enableX, boolean enableY, boolean enableZ, boolean enableINT2,int arduinoINTPin);

/***********************************************************
 * 
 * motion
 *
 * returns true if there is motion
 *   
 ***********************************************************/
boolean motion() { boolean motionOut = motion_; motion_ = false; return motionOut; }

/***********************************************************
 * 
 * update
 *
 * update data values, or clear interrupts. Use at start of loop()
 *   
 ***********************************************************/
  void update();

/***********************************************************
 * 
 * regRead
 *
 *   
 ***********************************************************/
  void regRead(byte reg, byte *buf, byte count = 1);

/***********************************************************
 * 
 * regWrite
 *
 *   
 ***********************************************************/
  void regWrite(byte reg, byte val);


//-----------------------------------------------------------
// Compatiblity functions to match the api of the ADXL345 library
// http://code.google.com/p/adxl345driver/
// allows for more easy updates of code from the ADXL345 to the MMA8453
//-----------------------------------------------------------
void setRangeSetting(int gScaleRange) { gScaleRange_ = gScaleRange; } //call this before setFullResBit()
void setFullResBit(boolean highRes) { dataMode(highRes,gScaleRange_); }
void readAccel(int *x, int *y, int *z) { xyz(*x,*y,*z); }



private:
	
	void xyz(int& x,int& y, int& z);
	void clearInterrupt();
	
	int x_,y_,z_;
	
	byte I2CAddr; 
	
	boolean highRes_;
	int gScaleRange_;
	boolean dataMode_;
	boolean shakeMode_;
	boolean motionMode_;
	
	boolean motion_;
	boolean shake_;
	boolean shakeAxisX_;
	boolean shakeAxisY_;
	boolean shakeAxisZ_;

	volatile boolean ISRFlag;
	static MMA8453_n0m1* pMMA8453_n0m1; //ptr to MMA8453_n0m1 class for the ISR
 
};

#endif

