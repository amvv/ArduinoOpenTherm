
/* Name:    OT_listener
   Purpose: listen to OpenTherm (TM) communications between heater and thermostat and display
            results on a 4x16 LCD
   Author:  Martijn van den Burg, martijn@[remove-me-first]palebluedot . nl
   
   Copyright (C) 2009 Martijn van den Burg. All right reserved.

   This program is free software: you can redistribute it and/or modify it under
   the terms of the GNU General Public License as published by the Free
   Software Foundation, either version 3 of the License, or (at your option)
   any later version.

   This program is distributed in the hope that it will be useful, but WITHOUT ANY
   WARRANTY; without even the implied warranty of MERCHANTABILITY or
   FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
   for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/ 


#include <FiniteStateMachine.h>  // FSM library from http://www.arduino.cc/playground/Code/FiniteStateMachine
#include <OpTh.h>


#define DEBUG 1    // enable Serial and LCD _debug_ output
#define LCDRAW 0   // 'raw' data output to LCD

#define DEG (char)223  // degree character


byte error_reading_frame;    // function return value

// Initialize the library with the numbers of the interface pins


//initialize FSM states
State GetPeriod      = State(get_period);
State DrawDisplay    = State(draw_display);
State WaitFrame      = State(wait_frame);
State GetFrame       = State(read_frame);
State DisplayFrame   = State(display_frame);


FSM OT_StateMachine = FSM(GetPeriod);     //initialize state machine, start in state: GetPeriod


OpTh OT = OpTh();  // create new OpTh class instance


void setup() {
  if (DEBUG) {
    Serial.begin(19200);
  }

  Serial.println(" OpTh listener");
  Serial.println("- MvdBurg 2009 - serial amvv - ");
  
  OT.init();

}



void loop() {
  OT_StateMachine.update();         // trigger the initialization state.
  OT_StateMachine.transitionTo(DrawDisplay).update();

  while (1) {
    OT_StateMachine.transitionTo(WaitFrame).update();
    OT_StateMachine.transitionTo(GetFrame).update();
    OT_StateMachine.transitionTo(DisplayFrame).update();
  }
  
}





/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
 *
 *    subroutines
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */


/* Get the period of the bits in the data stream. */
void get_period() {

  OT.measureOtPeriod();

  if (DEBUG) {
    long p = OT.getPeriod();
    
    Serial.print("Period: ");
    Serial.println((int)p);
    
  }

}



/* Put information on LCD. */
void draw_display() {
    Serial.print("Room --.-/--.-");
    Serial.print(DEG);  
    Serial.println("C");

    Serial.print("Boiler --");
    Serial.print(DEG);  
    Serial.println("C");

    Serial.print("CH --");
    Serial.print(DEG);  
    Serial.println("C");
}


/* Wait for the start of a frame. The wait period, a period in which the signal is LOW. 
 * is determined by the kind of frame last received. 
 * The frame we encounter can either be a MASTER or a SLAVE frame.
 */
void wait_frame() {
  OT.waitFrame();
}  // wait_frame



/* Read the bits in the frame. 
 * Returns 0 for success, 1 for failure.
 */
void read_frame() {
  error_reading_frame = OT.readFrame();
  if (error_reading_frame == 1)
  {
    Serial.println("bad data");
  }
  else
  if (error_reading_frame == 2)
  {
    Serial.println("start stop bit error");
  }
  else
  if (error_reading_frame == 3)
  {
    Serial.println("parity error");
  }
  else
  if (error_reading_frame == 0)
  {
    Serial.println("no error");
  }
  else
  {
    Serial.println("other code...");
  }


}  // read_frame



/* Display information about heating settings and heater status on the LCD. 
 * Output is sent to Serial when DEBUG is 1.
 * Unprocessed/unformatted output is sent to the LCD when LCDRAW is 1.
 * Processed and formatted output is sent to the LCD when LCDRAW is 0.
 */
void display_frame() {
  byte msg_type = OT.getMsgType();
  byte data_id = OT.getDataId();
  unsigned int data_value = OT.getDataValue();

  if (error_reading_frame && LCDRAW) {
    char *msg = OT.errmsg();
    Serial.println(msg);
  }
  else {
    if (DEBUG) {
      if (OT.isMaster() ) {
        Serial.print("M");
      }
      else {
        Serial.print("S");
      }
      Serial.print(":");
      Serial.print((int)data_id);
      Serial.print(":");
      Serial.println(data_value);
    }  // DEBUG
    
      switch(data_id) {
        case 1:  // Control setpoint
          if (OT.isMaster()) {
            int t = data_value / 256;  // don't care about decimal values
            Serial.print("set burner to ");
            Serial.print(t);
            Serial.println("C");
          }
        break;
        case 16:  // room setpoint
          if (OT.isMaster()) {
            float t = (float)data_value / 256;
            Serial.print("room set to ");
            Serial.print(t);
            if (data_value % 256 == 0) {
              Serial.print("C");
            }
            Serial.println("/");
          }
        break;
        case 24:  // room actual temperature
          if (OT.isMaster()) {
            float t = (float)data_value / 256;
            Serial.print("Room temp ");
            Serial.print(t);
            if (data_value % 256 == 0) {
              Serial.print(".0");
            }
            Serial.println("C");
          }
        break;
        case 25:  // boiler temp
          if (! OT.isMaster()) {
            int t = data_value / 256;
            Serial.print(t);
            Serial.print(DEG);
            Serial.print("C");
          }
        break;
        case 0:  // status
          if (! OT.isMaster()) {
            if (data_value & 4) {  // perform bitmasking on status frame
              Serial.print("DHW");
            }
            else if (data_value & 2) {
              Serial.print("CH ");
            }
            else {
              Serial.print("   ");
            }
            
            if (data_value & 8) {
              Serial.print("FLAME");
            }
            else {
              Serial.print("     ");
            }
          }
          break;
      }  // switch
    
    
  }  // no frame error
  
}


/* End */
