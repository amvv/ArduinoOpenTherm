/*
   OpTh.h 
   Copyright 2009 Martijn van den Burg, martijn@[remove-me-first]palebluedot . nl
   
   This file is part of the OpTh library for reading OpenTherm (TM)
   communications with Arduino.

   OpTh is free software: you can redistribute it and/or modify it under
   the terms of the GNU General Public License as published by the Free
   Software Foundation, either version 3 of the License, or (at your option)
   any later version.

   OpTh is distributed in the hope that it will be useful, but WITHOUT ANY
   WARRANTY; without even the implied warranty of MERCHANTABILITY or
   FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
   for more details.

   You should have received a copy of the GNU General Public License
   along with OpTh.  If not, see <http://www.gnu.org/licenses/>.

*/


/* This library is written for the Arduino Duemilanova and Arduino software version 0017.
 *
 * It's purpose is to make data transmitted with the OpenTherm (TM) protocol available
 * for other applications.
 *
 * This library may work with other hardware and/or software. YMMV.
 */

 

#ifndef OpTh_h
#define OpTh_h

#include "WProgram.h"

#define VERSION 0.1               // Version of this library


#define DATA_PIN 3                // Arduino pin where we receive the OT communication. Tied to INT1.
//changed sample size to 4 so that we can do a sample in one reply...as far as I understand, it samples SAMPLE_SIZE^2 times
#define SAMPLE_SIZE 3            // Manchester period sample size [number of samples]
#define TICK_TO_USEC 4            // factor to convert clock ticks to us (prescaler dependent)


/* Protocol-specific definitions */
#define FRAME_LENGTH 34           // OpenTherm protocol frame length [bits]
#define MIN_WAIT_FOR_SLAVE 20     // minimum time between master and slave frame [ms]
#define MIN_WAIT_FOR_MASTER 100   // minimum time between slave and (next) master frame [ms]



class OpTh {
   public:
      OpTh();                           // constructor
      void init();                      // initialise timer and input pin
      void measureOtPeriod();           // measure the period of the OpenTherm signal
	  void setPeriod(int32_t per);		// set the period of the OpenTherm signal - added by amvv
      void waitFrame();                 // wait for a master or slave frame
      int32_t getPeriod();              // return the period of the OT Manchester signal [us]

      uint32_t getFrame();              // Get the frame value: all 32 bits
      int8_t readFrame();               // Read the frame from the wire
      void setFrame(uint32_t frame);    // Set the frame value manually
      
      char *errmsg();                   // Contains error message in case a function returned an error
      
      uint8_t getParity();              // Return the frame parity
      uint8_t getMsgType();             // Return the type of message
      uint8_t getDataId();              // Return the data ID
      uint16_t getDataValue();          // Return the data value
      
      uint8_t isMaster();               // Returns 1 for master frame, 0 for slave frame

   private:
      char *_errorMsg;                  // can contain an error message (text)
      uint8_t _dataPin;                 // Arduino pin where we read the signal
      int16_t _preload;                 // timer preload value for interrupt-based sampling
      int32_t _otPeriod;                // period of one bit in the frame [ms]
      uint32_t _frame;                  // a frame contains FRAME_LENGTH - 2 bits (start/stop are discarded)
};

#endif
