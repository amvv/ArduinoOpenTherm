/*
 * OpenTherm encoder, by AMVV 
 * based on a work by: Sebastian Wallin
 * description:
 * Example on opentherm communication
 * This example will set the water temperature to 38 degrees
 *
 * This OT encoder implements a manchester encoder, which transmits a bit every 1ms,
 * so it fires every 0.5ms to make a transition.
 * it is not very clean code...
 * It communicates with the boiler, sends set points.
 * It communicates with the server to get a setpoint.
 * It does not set the boiler to the received setpoint.
 *
 * It sends a series of messages to check compatibility of the boiler with said messages
 * It makes use of an LCD plug from JeeLabs
 *
 * TODO:
 * - Set set of messages to be exchanged, and corresponding periods.
 * - expand message to server to include data read from the boiler.
 * - Control setpoint according to received message
 * - Clean up the code
 *   - handling of the message to the boiler
 * - Enter fail safe mode - independently control set point
 *   - attempt to listen to weather station messages
 * - meaningfull messages on the display
 *   - nice pretty characters
 *
 * TODO for hardware revision - bus powered
 * - lower power (unnecessary for now)
 */

#include <OpTh.h>
#include <PortsLCD.h>
#include <RF12.h> // needed to avoid a linker error :(

#define OUTPUT_PIN (6)//for use with JeeNodeUSB
#define OTPERIOD (997) //used a little trick to find mine, see below in loop

#define DEG (char)223  // degree character for lcd Output


/* Timer2 reload value, globally available */
unsigned int tcnt2;

/*my stuff amvv */
int cycles = 0;
int originalvalue = 0;
int y;
int bits_sent = 0;

int temp_set_to = 0;

byte bit_out = HIGH;
boolean done = false;
byte state = 0; //0 - start bit; 1 - message: 2 - stop bit

typedef struct {
        unsigned int house;
  unsigned int device;
	unsigned int seq;
	unsigned int temp;
} OpenThermData;

OpenThermData buf;


//not yet used...
typedef struct {
  byte message_type;
  byte data_id;
  byte data_value_h;
  byte data_value_l;
} 
OTdata;


//OTdata MM;
byte MM1=0;
byte MM2=0;
byte MM3=0;
byte MM4=0;


OpTh OT = OpTh();  // create new OpTh class instance
PortI2C myI2C (1);
LiquidCrystalI2C lcd (myI2C);

boolean rf_success = true;

/* Setup phase: configure and enable timer2 overflow interrupt */
void setup() {

  rf12_initialize(10, RF12_868MHZ, 212);
  Serial.begin(19200);

  /* Configure the test pin as output */
  pinMode(OUTPUT_PIN, OUTPUT); 

  lcd.begin(16, 2);
  lcd.backlight();

  // Print a message to the LCD.
  lcd.print("initializing JT");

  buf.house = 192;
  buf.device = 4;
  buf.temp = 30; // delete this and replace for proper message
  
  rf12_sleep(0);

  OT.init();
  OT.setPeriod(OTPERIOD);

  StopInterrupts();
  StartInterrupts();
}

void StopInterrupts()
{
  TIMSK2 &= ~(1<<TOIE2);
}

void StartInterrupts()
{   
  /* Configure timer2 in normal mode (pure counting, no PWM etc.) */
  TCCR2A &= ~((1<<WGM21) | (1<<WGM20));
  TCCR2B &= ~(1<<WGM22);

  /* Select clock source: internal I/O clock */
  ASSR &= ~(1<<AS2);

  /* Disable Compare Match A interrupt enable (only want overflow) */
  TIMSK2 &= ~(1<<OCIE2A);

  /* Now configure the prescaler to CPU clock divided by 128 */
  TCCR2B |= (1<<CS22)  | (1<<CS20); // Set bits
  TCCR2B &= ~(1<<CS21);             // Clear bit

  /* We need to calculate a proper value to load the timer counter.
   * The following loads the value 192 into the Timer 2 counter register
   * The math behind this is:
   * (CPU frequency) / (prescaler value) = 125000 Hz = 8us.
   * (desired period) / 8us = 62.5.
   * MAX(uint8) + 1 - 62 = 194;
   * amvv note: don't know why I made it 192, if the result is 194, but it seems to work...
   */
  /* Save value globally for later reload in ISR */
  
  tcnt2 = 192; 

  /* Finally load end enable the timer */
  TCNT2 = tcnt2;
  TIMSK2 |= (1<<TOIE2);


}

boolean logic_transition = false;

/*
 * Install the Interrupt Service Routine (ISR) for Timer2 overflow.
 * This is normally done by writing the address of the ISR in the
 * interrupt vector table but conveniently done by using ISR()  */
ISR(TIMER2_OVF_vect) {
  /* Reload the timer */
  TCNT2 = tcnt2;

  switch (state) {
  case 0: //start bit 
    if (logic_transition == false)
    {
      y = 1;
      logic_transition = true;
      //Serial.print(y);
    }
    else
    {
      y = !y;  
      logic_transition = false;
      state++;
    }
    break;

  case 1://message byte 1
    if (logic_transition == false)
    {
      y = ((MM1) & 128) >> 7;
      MM1 = MM1 << 1;
      logic_transition = true;
      //Serial.print(y);
    }
    else
    {
      y = !y;
      logic_transition = false;
    }
    break;
  case 2://message byte 2
    if (logic_transition == false)
    {
      y = ((MM2) & 128) >> 7;
      MM2 = MM2 << 1;
      logic_transition = true;
      //Serial.print(y);
    }
    else
    {
      y = !y;
      logic_transition = false;
    }
    break;
  case 3://message byte 3
    if (logic_transition == false)
    {
      y = ((MM3) & 128) >> 7;
      MM3 = MM3 << 1;
      logic_transition = true;
      //Serial.print(y);
    }
    else
    {
      y = !y;
      logic_transition = false;
    }
    break;
  case 4://message byte 3
    if (logic_transition == false)
    {
      y = ((MM4) & 128) >> 7;
      MM4 = MM4 << 1;
      logic_transition = true;
      //Serial.print(y);
    }
    else
    {
      y = !y;
      logic_transition = false;
    }
    break;
  case 5: //stop bit 
    if (logic_transition == false)
    {
      y = 1;
      logic_transition = true;
      //Serial.println(y);
    }
    else
    {
      y = !y;  
      logic_transition = false;
      state = 0;
    }
    break;
  }


  digitalWrite(OUTPUT_PIN, !y);
  //if (logic_transition == true)
  //Serial.print(y, DEC);

  bits_sent = bits_sent + 1;

  if (bits_sent == 18) //(sizeof(MM*3)*8*2 + 2))  MM1 sent
  {
    state++;
  }
  if (bits_sent == 34) //(sizeof(MM*3)*8*2 + 2))  MM2 sent
  {
    state++;
  }
  if (bits_sent == 50) //(sizeof(MM*3)*8*2 + 2))  MM3 sent
  {
    state++;
  }
  if (bits_sent == 66) //(sizeof(MM*3)*8*2 + 2)) MM4 sent
  {
    state++;
  }
  if (bits_sent == 68) //(sizeof(MM)*8*2 + 4))
  {
    digitalWrite(OUTPUT_PIN, HIGH);
    done = true;
    bits_sent = 0;
    StopInterrupts();
    state=0;
  }
  else
  {
    done = false;
  }
}

int error_reading_frame;

int total_cycles = 0;

/* Main loop.*/
void loop() {

  if (done == true)
  {
    total_cycles++;

    OT.waitFrame();
    error_reading_frame = OT.readFrame();

//uncomment the cycle below to get the measurement of the OT period, then adapt the #define above, and comment it again...

//    if(total_cycles == 10)
//    {    
//      lcd.setCursor(0, 1);
//      lcd.print("measuring");
//      OT.measureOtPeriod();
//      lcd.setCursor(0, 1);
//      int perper = OT.getPeriod();
//      lcd.print(perper);
//      lcd.print("us");
//    }
    
  lcd.setCursor(0, 1);
  if (error_reading_frame == 1)
  {
      lcd.print("bad data");
  }
  else
  if (error_reading_frame == 2)
  {
      lcd.print("ss bit error");
  }
  else
  if (error_reading_frame == 3)
  {
      lcd.print("parity error");
  }
  else
  if (error_reading_frame == 0)
  {
      lcd.print("OK:        ");
      display_frame();
  }
  else
  {
      lcd.print("other error");
  }
    
     // Serial.println(total_cycles);
     // query the server every 4 cycles - should be increased to about once a minute
//    if ((total_cycles%4) == 3 || rf_success == false)
//    {
//      //send radio data
      
//      buf.seq = total_cycles;
//      rf12_sleep(-1);
//      while (!rf12_canSend())	// wait until sending is allowed
//       rf12_recvDone();
       
//       rf12_sendStart(0, &buf, sizeof buf);

//      while (!rf12_canSend())	// wait until sending has been completed
//        rf12_recvDone();
      
//      delay(5);
      //lcd.print("sent!");
        
//     unsigned long ss;
//        
//      ss = millis();
        
        
//      rf_success = false;
//      char* sss="failed";
//      lcd.setCursor(0,0);
        
      //wait 300 miliseconds for a reply from the server. If none comes, go on, and retry in the next cycle
//      while (millis() - ss < 300)
//      {  
//         if (rf12_recvDone())
//         {
//            if (rf12_crc == 0)
//            {        
//              lcd.print("OK ");
//              lcd.print((int)rf12_buf[9]);
//              if (rf12_buf[5] == 4) //received a pack from the boiler controller
//              {
//                sss="success";
//                rf_success = true;
//                //break;
//              }
//            }
        
//          }
//       }//while
//       lcd.print(sss);
   
//       rf12_sleep(0);
//    }
    
//This delay should be adjusted to stay within the tolerances, even when the communicating failed    
    delay(850);

    cycles++;

    if (cycles == 1) //enable CH
    {
      MM1=0x00;//parity is 0
      MM2=0x00;//0 bit set
      MM3=0x03;//SHOULD BE 0x03 in order for the boiler to work!!!!!
      MM4=0x00;
    }

    if (cycles == 2) //set water temp to 38 degress
    {
      MM1=0x90;
      MM2=0x01;
      MM3=0x26;
      MM4=0x00;
      temp_set_to = MM3;
      //cycles = 0;
    }
    //other messages to try out
    if (cycles == 3) // ID  3 - slave config flags
    {
      MM1=0x00;
      MM2=0x03;
      MM3=0x00;
      MM4=0x00;
    }
    if (cycles == 4) // ID  5 - faults in water pressure and flame
    {
      MM1=0x00;
      MM2=0x05;
      MM3=0x00;
      MM4=0x00;
    }
    if (cycles == 5) // ID 17 - read modulation level
    {
      MM1=0x00;
      MM2=0x11;
      MM3=0x00;
      MM4=0x00;
    }
    if (cycles == 6) // ID 18 - water pressure
    {
      MM1=0x00;
      MM2=0x12;
      MM3=0x00;
      MM4=0x00;
    }
    if (cycles == 7) // ID 19 - DHW flow rate
    {
      MM1=0x80;
      MM2=0x13;
      MM3=0x00;
      MM4=0x00;
    }
    if (cycles == 8) // ID 25 - CH water temperature
    {
      MM1=0x80;
      MM2=0x19;
      MM3=0x00;
      MM4=0x00;
    }
    if (cycles == 9) // ID 28 - return water temperature
    {
      MM1=0x80;
      MM2=0x1C;
      MM3=0x00;
      MM4=0x00;
    }
    if (cycles == 10) // ID 26 - DHW temp
    {
      MM1=0x80;
      MM2=0x1A;
      MM3=0x00;
      MM4=0x00;
    }
    if (cycles == 11) // ID 33 - exhaust temperature
    {
      MM1=0x00;
      MM2=0x21;
      MM3=0x00;
      MM4=0x00;
    }
    if (cycles == 12) // ID 116 - burner starts
    {
      MM1=0x00;
      MM2=0x74;
      MM3=0x00;
      MM4=0x00;
    }
    if (cycles == 13) // ID 117 - CH pump starts
    {
      MM1=0x80;
      MM2=0x75;
      MM3=0x00;
      MM4=0x00;
    }
    if (cycles == 14) // ID 118 - DHW pump starts/valve starts
    {
      MM1=0x80;
      MM2=0x76;
      MM3=0x00;
      MM4=0x00;
    }
    if (cycles == 15) // ID 119 - DHW burner starts
    {
      MM1=0x00;
      MM2=0x77;
      MM3=0x00;
      MM4=0x00;
    }
    if (cycles == 16) // ID 120 - burner hours
    {
      MM1=0x00;
      MM2=0x78;
      MM3=0x00;
      MM4=0x00;
    }
    if (cycles == 17) // ID 121 CH pump hours
    {
      MM1=0x80;
      MM2=0x79;
      MM3=0x00;
      MM4=0x00;
    }
    if (cycles == 18) // ID 122 - DHW pump/valve hours
    {
      MM1=0x80;
      MM2=0x7A;
      MM3=0x00;
      MM4=0x00;
    }
    if (cycles == 19) // ID 123 DHW burner hours
    {
      MM1=0x00;
      MM2=0x7B;
      MM3=0x00;
      MM4=0x00;
      //in the last message reset cycles
      cycles = 0;
    }
    
    lcd.setCursor(13, 1);
    lcd.print(total_cycles);

    
    //Serial.println();
    done = false;
    //check radio
    //wait for message from boiler  
    //decide on message
    //Serial.print(millis());
    //Serial.print(" - ");
    //Serial.print(originalvalue);
    //Serial.print(" - ");
    StartInterrupts();
  }
}

void display_frame() {
  byte msg_type = OT.getMsgType();
  byte data_id = OT.getDataId();
  unsigned int data_value = OT.getDataValue();

      lcd.setCursor(0,0);

      lcd.print((int)data_id);
      lcd.print(":");
      lcd.print(data_value, HEX);
      lcd.print("  ");

      lcd.setCursor(8,0);
      lcd.print("        ");
      lcd.setCursor(0,1);

unsigned int ut;
int t;
float f;


      switch(data_id) {
          case 0:  // status
            if (data_value & 4) {  // perform bitmasking on status frame
              lcd.print("DHW");
            }
            else if (data_value & 2) {
              lcd.print("CH ");
              lcd.print(temp_set_to);
            }
            else {
              lcd.print("   ");
            }
            
            if (data_value & 8) {
              lcd.print("FLAME");
            }
            else {
              lcd.print("     ");
            }

        case 1:  // Control setpoint
          if (OT.isMaster()) {
            t = data_value / 256;  // don't care about decimal values
            lcd.print("set burner to ");
            lcd.print(t);
            lcd.print("C");
          }
        break;
        case 3:  //slave configs
            lcd.print("slave flags OK    ");
        break;
        
        case 5:  //faults water and flame
            lcd.print("faults flags OK    ");
        break;
        
        case 16:  // room setpoint
            f = (float)data_value / 256;
            lcd.print("room set to ");
            lcd.print(f);
            if (data_value % 256 == 0) {
              lcd.print("C");
            }
            lcd.print("     ");
        break;
        case 17:  //modulation level
            t = data_value / 256;  // don't care about decimal values
            lcd.print("modul: ");
            lcd.print(t);
            lcd.print("     ");
        break;
        case 18:  //water pressure
            t = data_value / 256;  // don't care about decimal values
            lcd.print("H20 press: ");
            lcd.print(t);
            lcd.print("     ");
        break;
        case 19:  //DHW flow rate
            t = data_value / 256;  // don't care about decimal values
            lcd.print("DHW rate: ");
            lcd.print(t);
            lcd.print("     ");
        break;
        case 25:  //CH temp
            t = data_value / 256;  // don't care about decimal values
            lcd.print("CH temp: ");
            lcd.print(t);
            lcd.print("     ");
        break;
        case 26:  //DHW temp
            t = data_value / 256;  // don't care about decimal values
            lcd.print("DHW temp: ");
            lcd.print(t);
            lcd.print("     ");
        break;
        case 28:  //return temp
            t = data_value / 256;  // don't care about decimal values
            lcd.print("ret temp: ");
            lcd.print(t);
            lcd.print("     ");
        break;
        case 33:  //exhaust temp
            t = data_value / 256;  // don't care about decimal values
            lcd.print("exh temp: ");
            lcd.print(t);
            lcd.print("     ");
        break;
        case 116:  //burner starts
            ut = (unsigned int)data_value;  // don't care about decimal values
            lcd.print("bur st: ");
            lcd.print(ut);
            lcd.print("     ");
        break;
        case 117:  //CH pump starts
            ut = (unsigned int)data_value;  // don't care about decimal values
            lcd.print("CH pump st: ");
            lcd.print(ut);
            lcd.print("     ");
        break;
        case 118:  //DHW starts
            ut = (unsigned int)data_value;  // don't care about decimal values
            lcd.print("DHW st: ");
            lcd.print(ut);
            lcd.print("     ");
        break;
        case 119:  //DHW burner starts
            ut = (unsigned int)data_value;  // don't care about decimal values
            lcd.print("DHW bur st: ");
            lcd.print(ut);
            lcd.print("     ");
        break;
        case 120:  //burner hours
            ut = (unsigned int)data_value;  // don't care about decimal values
            lcd.print("bur h: ");
            lcd.print(ut);
            lcd.print("     ");
        break;
        case 121:  //CH pump hours
            ut = (unsigned int)data_value;  // don't care about decimal values
            lcd.print("CH p h: ");
            lcd.print(ut);
            lcd.print("     ");
        break;
        case 122:  //DHW pump hours
            ut = (unsigned int)data_value;  // don't care about decimal values
            lcd.print("DHW p h: ");
            lcd.print(ut);
            lcd.print("     ");
        break;
        case 123:  //DHW burner hours
            ut = (unsigned int)data_value;  // don't care about decimal values
            lcd.print("DHW b h: ");
            lcd.print(ut);
            lcd.print("     ");
        break;
        
        case 24:  // room actual temperature
          if (OT.isMaster()) {
            float t = (float)data_value / 256;
            lcd.print("Room temp ");
            lcd.print(t);
            if (data_value % 256 == 0) {
              lcd.print(".0");
            }
            lcd.print("C");
          }
        break;
        default:
           lcd.print("incorrect message ID   ");
        break;

      }  // switch  
}


