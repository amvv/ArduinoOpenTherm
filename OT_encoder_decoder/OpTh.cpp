/*
   OpTh.cpp 
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


#include "OpTh.h"

#define VERSION 0.1


/* Variables used in interrupts */
volatile byte firstEdge = 1;   // used in signal period determination
volatile byte dataReady = 0;   // used in signal period determination
volatile int transitionCount = 0;// used for error detection
volatile long data1;            // measured signal time [clock ticks]

int noframe = 0;
      

/* * * * Static functions * * * */


/* Interrupt service routine for signal period measurement. */
static void _ISRsync() {
   if (firstEdge) {
      TCNT1 = 0;        // reset timer
      firstEdge = 0;
      data1 = 0;
   }
   else {
      data1 = TCNT1;    // read value of timer
      firstEdge = 1;
      dataReady = 1;
   }
}


/* Interrupt service routine for error detection. */
static void _ISRtransition() {
   transitionCount++;
}


/* Timer1 overflow interrupt vector handler: has two purposes:
 * (1) detect an unwanted overflow while sampling for the OT_period.
 * (2) set a flag to commence sampling the signal on _dataPin, i.e. the Manchester code.
 * Ad (1): When we detect that the Timer1 overflows
 * (after approx. 262 ms), we set 'dataReady' to 1 and 'data1' to 0, this will signal
 * that the time obtained for T2 is incorrect.
 * This could occur when the measurement was triggered on the last rising edge of the frame.
 */
ISR(TIMER1_OVF_vect) {
  // for (1):
   data1 = 0;
   firstEdge = 1;
   // for (1) and (2)
   dataReady = 1;
}



/* * * * End static functions * * * */


/* Class constructor.
 * _dataPin is tied to the operation of INT0 and Timer1. If you want to use
 * another pin to read the data, you'll have to hack the code.
 */
OpTh::OpTh() {
   _dataPin = DATA_PIN;
}



/* Initialise Timer1 and set the pinmode for the pin that we
 * use to listen to the communication.
 */
void OpTh::init() {
   TCCR1A = 0 << COM1A1 | 0 << COM1A0;
   TCCR1B = 0 << CS12 | 1 << CS11 | 1 << CS10;    // 16MHz clock with prescaler means TCNT1 increments every 4 us.
   TIMSK1 = 0x00;

   pinMode(_dataPin, INPUT);
   digitalWrite(_dataPin, HIGH);                  // enable internal pull-up resistor
   
   _frame = 0;
}


/* Measure the period of the bits in the data stream so that we can synchronize
 * with the protocol.
 * - uses _ISRsync
 */
void OpTh::measureOtPeriod() {
   int8_t cnt1 = 0;              // loop counter
   int8_t cnt2 = 0;              // loop counter
   uint16_t sum = 0;             // sum of T2 values
   uint16_t average = 0;         // average of T2 values
   uint16_t T2;                  // measured minimum time period

  /* enable Timer1 overflow interrupt so we can detect when it overflows
   * while we're still waiting for the next rising edge. For example: if we
   * triggered on the last rising edge of a frame.
  */
   TIMSK1 = 1 << TOIE1;
   
   /* attach ISRsync() to INT1, digital pin 3. Pin is kept high by internal
   * pull up. Triggering on the RISING edge doesn't quite work.
   */
   attachInterrupt(1, _ISRsync, FALLING);

  /* take the average of SAMPLE_SIZE samples of T2, each being the minimum T2 of
  * SAMPLE_SIZE measurements. */
   while (cnt1 < SAMPLE_SIZE) {
      cnt2 = 0;
      T2 = 0xFFFF;
    
      while (cnt2 < SAMPLE_SIZE) {
         if (dataReady) {          // interrupt handler has performed measurement
            if ( data1 != 0 ) {     // Timer1 overflow occurred
               if ( data1 < T2 ) {    // find smallest shortest time between two rising edges
                  T2 = data1;
               }
               cnt2++;                // increment loopcounter after valid data received
            }
            dataReady = 0;          // prepare for next sample
         }
      }
      sum += T2;                  // summarize all samples, for average calculation
      cnt1++;
   }
   
   detachInterrupt(1);   // don't need the interrupt anymore
   TIMSK1 = 0 << TOIE1;  // disable Timer1 overflow interrupt

   _otPeriod = sum * TICK_TO_USEC / SAMPLE_SIZE;          // calculate average [us]
   

  /* We want Timer1 to create an overflow interrupt every _otPeriod us, which we
   * then use as a trigger to go and sample the input signal.
   * One clock tick = 64/16000000 * 1E6 = TICK_TO_USEC us. Need to time `_otPeriod [us] / TICK_TO_USEC` ticks.
   * Subtract from timer overflow value (take 0xFFFF, the maximum timer value).
   * See http://www.uchobby.com/index.php/2007/11/24/arduino-interrupts/
   */
   _preload = 0xFFFF - (_otPeriod / TICK_TO_USEC);

} // measureOtPeriod

void OpTh::setPeriod(int32_t per){
	_otPeriod = per;
	_preload = 0xFFFF - (_otPeriod / TICK_TO_USEC);
}

/* Wait for the next frame. The amount of time to wait depends on whether a Master
 * or a Slave frame is expected.
 */
void OpTh::waitFrame(){
   int8_t waitState = 1;
   uint8_t msgType = getMsgType();
   int16_t waitPeriod;
   uint32_t timeLow;
   unsigned long tstart;

 //  if ( (_frame == 0) || (msgType >= 4) ){    // just started, or a Slave frame was rcvd
 //     waitPeriod = MIN_WAIT_FOR_SLAVE;
 //  }
 //  else {
 //     waitPeriod = MIN_WAIT_FOR_SLAVE;
 //  }

tstart = millis();

   while (waitState) {
      while (digitalRead(_dataPin) == HIGH) {

		 if (millis() - tstart > 500)
		 {
		    noframe = 1;
		    break;
		 }
	  }   // do nothing

      timeLow = millis();

      while (digitalRead(_dataPin) == LOW) {
         if ( millis() - timeLow >= waitPeriod ) {
            waitState = 0;
         }
		 if (millis() - tstart > 500)
		 {
		    noframe = 1;
		    break;
		 }
	  }
	  if (millis() - tstart > 500)
	  {
	    noframe = 1;
	    break;
	  }
   }
  // fall through when signal goes HIGH and waitState == 0 
} // waitFrame


/* Read one frame from the wire.
 * - we enter on a rising signal
 * - configure Timer1 to trigger an internal interrupt at every _otPeriod us
 * - do not start the timer immediately (i.e. at the rising edge), but wait for a
 *   period of _otPeriod / TICK_TO_USEC us so that we do not risk sampling right at an edge.
 * - start timer
 * - sample the signal when the timer overflows: this is the correct bitvalue
 * - bad data check: with every HI/LO and LO/HI transition, transitionCount is incremented. When a
 *   new sample is taken we check that the current transitionCount is larger than the transitionCount
 *   of the previous sample. This way, spurious signals can be detected and sampling aborted.
 * - sample FRAME_LENGTH bits
 * - check start- and end bits on the fly
 * - perform parity check on the fly
 *
 * An unsigned long (32 bits) is used to store the 32 data bits. The start and stop bits (both 1)
 * are discarded because they hold no value after the check on them has been done.
 *
 * The functions returns 0 for success and non-zero for error, setting _errorMsg (which
 * can be accessed with the errmsg() function).
 *
 */
int8_t OpTh::readFrame() {
   int8_t cnt = 0;                     // loop counter
   int8_t parityOdd = 0;              // parity check
   int8_t previousTransitionCount = 0;
   uint32_t tempFrame = 0;
   
   /* if (noframe == 1)
		{
		   return 10;
		 }
		 */   
   dataReady = 1;      // set to 1 to obtain the first sample immediately
   transitionCount = 0;    // incremented inside ISR


  /* Wait a quarter of a period to get 'into' the period and thus prevent samples on a
   * rising/falling edge.
   * Note: delayMicroseconds() disables interrupts.
   */
   delayMicroseconds(_otPeriod / 4);
   
   /* Attach _ISRtransition() to INT0 (digital pin 2), to measure signal transitions. */
   detachInterrupt(1);
   attachInterrupt(1, _ISRtransition, CHANGE);

   /* Enable Timer1 overflow interrupt */
   TIMSK1 = 1 << TOIE1;

   /* Go and sample FRAME_LENGTH bits.
   * Note: The first sample is not obtained through timer overflow (because dataReady was
   * initialized to 1), but the remainder of samples is.
   */
   while ( cnt < FRAME_LENGTH ) {
      if (dataReady) {
         TCNT1 = _preload;    // preload timer

      /* Bad data check: number of transitions must have increaseds since previous sample
      * (except when we're measuring the first sample).
      * 'transitionCount' is incremented by INT0 handler 'ISR_transition'.
      */
         if ( (cnt > 0) && (transitionCount == previousTransitionCount) ) {
            _errorMsg = "bad data";
            _frame = 0;
            return 1;
         }
         else {
            previousTransitionCount = transitionCount;
         }

         int sample = digitalRead(_dataPin);    // get the value of the signal on _dataPin

         /* parity check, toggle parityOdd each time the signal is 1. */
         if ( sample == 1 ) {
            parityOdd = !parityOdd;
         }

         /* First sample is start bit, last is stop bit. They ought to be 1 and we don't store them. */
         if ( (cnt == 0) ) {
            if (sample != 1) {
               _errorMsg = "bad start bit";
               _frame = 0;
               return 2;
            }
         }
		 else
         if ( (cnt == (FRAME_LENGTH - 1)) ) {
            if (sample != 1) {
               _errorMsg = "bad end bit";
               _frame = 0;
               return 12;
            }
         }
		 
		 
		 
         else {
        /* store this sample by shifting all bits in the unsigned long to the left,
         * then store relevant number of bits from the sample value.
         * Note that the shifting is also done (unnecessarilly) for the first data bit, but since
         * the initialization value of 'bits' is 0, that doesn't matter.
         */
            tempFrame <<= 1;
            tempFrame |= sample;
         }
         cnt++;
         dataReady = 0;
      }  // dataReady
   }  // cnt < FRAME_LENGTH

   
  // We're done with the interrupt.
   detachInterrupt(1);
  
   // Disable Timer1 overflow interrupt when done.
   TIMSK1 = 0 << TOIE1;


  /* check parity, should be even. If it isn't then something is
   * wrong and return with an error.
  */
   if (parityOdd == 1) {
      _errorMsg = "parity error";
      _frame = 0;
      return 3;
   }
   else {
      _frame = tempFrame;               // store measured frame in class' private variable.
   }

   return 0;   // success
   
} // readFrame


/* Accessors for internal data structures. */


/* Set the frame value manually. Use this for example when receiving the frame from another
 * source (e.g. radio transmitter) to be able to access its content. */
void OpTh::setFrame(uint32_t frame) {
   _frame = frame;
}

uint32_t OpTh::getFrame() {
   return _frame;
}

char *OpTh::errmsg() {
   return _errorMsg;
}

int32_t OpTh::getPeriod() {
   return _otPeriod;
}

/* Return message type (a three-bit number) as a byte. */
uint8_t OpTh::getMsgType() {
   uint32_t tempFrame = _frame;
   tempFrame >>= 28;                  // shift 28 bits into oblivion
   uint8_t msgType = tempFrame & 7;  // take only three bits (masking with B111)
   return msgType;
}

/* Return data ID as a single byte. */
uint8_t OpTh::getDataId() {
   uint32_t tempFrame = _frame;
   tempFrame >>= 16;
   return (uint8_t)tempFrame;         // type casting discards 2 bytes
}

/* Return data value as an unsigned int. */
uint16_t OpTh::getDataValue() {
   return (uint16_t)_frame;           // type casting discards 6 bytes
}

/* Return parity as a single byte. */
uint8_t OpTh::getParity() {
   uint32_t tempFrame = _frame;
   tempFrame >>= 31;                 // shift 31 bits into oblivion
   return tempFrame;
}

/* Return 1 if it's a master frame, 0 if it's a slave frame. */
uint8_t OpTh::isMaster() {
   if (getMsgType() <= 3) {
      return 1;
   }

   return 0;
}
