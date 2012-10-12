/*
 * OpenTherm encoder, by AMVV 
 * based on a work by: Sebastian Wallin
 * description:
 * Example on opentherm communication
 *
 * This OT encoder implements a manchester encoder, which transmits a bit every 1ms,
 * so it fires every 0.5ms to make a transition.
 * it is not very clean code...
 * It communicates with the boiler, sends set points.
 * It communicates with the server to get a setpoint.
 * It sets the boiler to the received setpoint.
 *
 * TODO:
 * - Clean up the code
 *   - handling of the message to the boiler
 * - Enter fail safe mode - independently control set point
 *   - attempt to listen to weather station messages
 *
 * TODO for hardware revision - bus powered
 * - lower power (unnecessary for now)
 *
 * DONE
 * - expand message to server to include data read from the boiler.
 * - Control setpoint according to received message
 * - meaningfull messages on the display
 *   - nice pretty characters 
 * - Set set of messages to be exchanged, and corresponding periods.
 */

#include <OpTh.h>
#include <PortsLCD.h>
#include <RF12.h> // needed to avoid a linker error :(
#include <util/parity.h>

#define OUTPUT_PIN (4)//for use with JeeNodeUSB
#define OTPERIOD (997) //used a little trick to find mine, see below in loop

#define MAX_BOILER_TEMP (80)

#define REPORTING_PERIOD (60)
#define REPORTING_DELTA  (5)
#define MAX_RF12_RETRY (50) //retry 50 times and then give up

//CHARACTER DEFINITIONS

#define CHAR_CH    (0)
#define CHAR_DHW   (1)
#define CHAR_RF    (2)
#define CHAR_FLAME (3)
#define CHAR_OT    (4)
#define CHAR_OK    (5)
#define CHAR_NOK   (6)

byte CH[8] = //OK
{
  B00100,
  B01110,
  B11111,
  B11111,
  B10001,
  B10101,
  B10101,
  B11111
};
byte DHW[8] = //OK
{
  B00100,
  B00100,
  B01110,
  B11111,
  B00000,
  B10101,
  B00000,
  B10101
};
byte OK[8] =
{
  B00001,
  B00011,
  B00010,
  B00110,
  B10100,
  B11100,
  B11000,
  B01000
};
byte NOK[8] =
{
  B10001,
  B11011,
  B01110,
  B00100,
  B01110,
  B11011,
  B10001,
  B10001
};
byte RF[8] =
{

  B00011,
  B00000,
  B00110,
  B00001,
  B01100,
  B00010,
  B11000,
  B11000
};
byte FLAME[8] =
{
  B00100,
  B00100,
  B00110,
  B00110,
  B01111,
  B11011,
  B11011,
  B01110
};
byte OTH[8] =
{
  B00100,
  B11111,
  B10001,
  B10001,
  B10001,
  B10001,
  B10101,
  B11111
};

/* Timer2 reload value, globally available */
unsigned int tcnt2;

/*my stuff amvv */
unsigned int cycles, inner_cycles = 0;
unsigned int total_cycles = 55;
int originalvalue = 0;
int y;
int bits_sent = 0;

byte flame = false;

byte bit_out = HIGH;
boolean done = false;
byte state = 0; //0 - start bit; 1 - message: 2 - stop bit

int reporting_cycles = REPORTING_PERIOD;

typedef struct {
  unsigned int house;
  unsigned int device;
  unsigned int seq;
  byte temp;
  byte CHtemp;
  byte returntemp;
  byte boilerstatus;
} 
OpenThermData;

OpenThermData buf;

typedef struct {
  unsigned int house;
  unsigned int device;
  unsigned int seq;
  unsigned int burner_starts;
  unsigned int CH_pump_starts;
  unsigned int DHW_pump_starts;
  unsigned int DHW_burner_starts;
  unsigned int burner_hours;
  unsigned int CH_pump_hours;
  unsigned int DHW_pump_hours;
  unsigned int DHW_burner_hours;
} 
OpenThermExtendedData;

OpenThermExtendedData extbuf;

//OTdata MM;
byte MM1, MM2, MM3, MM4 = 0;

OpTh OT = OpTh();  // create new OpTh class instance
PortI2C myI2C (3);
LiquidCrystalI2C lcd (myI2C);

byte rf_success = 0;

/* Setup phase: configure and enable timer2 overflow interrupt */
void setup() {

  rf12_initialize(10, RF12_868MHZ, 212);
  //Serial.begin(19200);
  lcd.createChar(CHAR_CH,CH);
  lcd.createChar(CHAR_DHW,DHW);
  lcd.createChar(CHAR_RF,RF);
  lcd.createChar(CHAR_FLAME,FLAME);
  lcd.createChar(CHAR_OT,OTH);
  lcd.createChar(CHAR_OK,OK);
  lcd.createChar(CHAR_NOK,NOK);

  /* Configure the test pin as output */
  pinMode(OUTPUT_PIN, OUTPUT); 

  lcd.begin(16, 2);
  lcd.backlight();

  // Print a message to the LCD.

  lcd.write(CHAR_RF);
  lcd.write(CHAR_OT);
  lcd.write(CHAR_FLAME);
  lcd.write(CHAR_CH);
  lcd.write(CHAR_DHW);

  lcd.print(" Ti To Ts");

  buf.house = 192;
  buf.device = 4;
  buf.seq = 0;
  buf.temp = 0;
  buf.CHtemp = 0;
  buf.returntemp = 0;
  buf.boilerstatus = 0;
  
  extbuf.house = 192;
  extbuf.device = 5;
  extbuf.seq = 0;
  extbuf.burner_starts = 0;
  extbuf.CH_pump_starts = 0;
  extbuf.DHW_pump_starts = 0;
  extbuf.DHW_burner_starts = 0;
  extbuf.burner_hours = 0;
  extbuf.CH_pump_hours = 0;
  extbuf.DHW_pump_hours = 0;
  extbuf.DHW_burner_hours = 0;
 
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

    lcd.setCursor(1, 1);
    if (error_reading_frame == 1)
    {
      lcd.print("1");//bad data");
    }
    else
      if (error_reading_frame == 2)
      {
        lcd.print("2");//ss bit error");
      }
      else
        if (error_reading_frame == 3)
        {
          lcd.print("3");//parity error");
        }
        else
          if (error_reading_frame == 0)
          {
            lcd.write(CHAR_OK);//OK:        ");
            display_frame();
          }
          else
          {
            lcd.print("9");//other error");
          }

    // Serial.println(total_cycles);
    // query the server every REPORTING_PERIOD cycles for the setpoint
    if (total_cycles == reporting_cycles || rf_success != 0)
    {
      //send radio data
      total_cycles = 0;
      buf.seq = buf.seq++;
      
      if (error_reading_frame > 0) //comms error with the boiler
      {
        buf.boilerstatus = 250 + error_reading_frame;
      }
      
      rf12_sleep(-1);
      while (!rf12_canSend())  // wait until sending is allowed
          rf12_recvDone();

      rf12_sendStart(0, &buf, sizeof buf);
      while (!rf12_canSend())	// wait until sending has been completed
          rf12_recvDone();

      delay(5);
      //lcd.print("sent!");

      unsigned long ss;

      ss = millis();


      rf_success = rf_success++;
      byte sss=CHAR_NOK;

      //wait 300 miliseconds for a reply from the server. If none comes, go on, and retry in the next cycle
      while (millis() - ss < 300)
      {  
        if (rf12_recvDone())
        {
          if (rf12_crc == 0)
          {        
            //lcd.print("OK");
            //lcd.print((int)rf12_buf[9]);
            if (rf12_buf[5] == 4) //received a pack from the boiler controller
            {
              sss=CHAR_OK;
              rf_success = 0;
              buf.temp = min(rf12_buf[9], MAX_BOILER_TEMP);
              reporting_cycles = REPORTING_PERIOD + REPORTING_DELTA - rf12_buf[10];
    lcd.setCursor(14,0);
    lcd.print(reporting_cycles);

              //HERE RESET CHTEMP, RETURNTEMP, BOILERSTATUS, TEMP
              buf.CHtemp = 0;
              buf.returntemp = 0;
              buf.boilerstatus = 0;
              //break;
            }
          }
        }
      }//while
      lcd.setCursor(0,1);
      lcd.print(sss);


      rf12_sleep(0);

      if (rf_success == MAX_RF12_RETRY)
      {
        rf_success = 0;
        buf.temp = 0;
      }
      //lcd.setCursor(14,0);
      //lcd.print((int)buf.temp);
    }
    //REMOVED, this will never happen...not very interesting data yet
    //send extended data once every 60000 cycles 
    //if ((total_cycles%36000) == 100)
    //{
    //  //send radio data

    //  extbuf.seq = extbuf.seq++;
    //  rf12_sleep(-1);
    //  while (!rf12_canSend())	// wait until sending is allowed
    //      rf12_recvDone();

    //  rf12_sendStart(0, &extbuf, sizeof extbuf);
    //  while (!rf12_canSend())	// wait until sending has been completed
    //      rf12_recvDone();

    //  delay(5);
    //  //lcd.print("sent!");

    //  rf12_sleep(0);
    //}

    //This delay should be adjusted to stay within the tolerances, even when the communicating failed    
    delay(850);

    cycles++;
//    lcd.setCursor(15,1);
//    lcd.print(cycles);
    lcd.setCursor(14,1);
    lcd.print(total_cycles);

    if (cycles == 1) //enable CH
    {
      if (buf.temp == 0)
      { //no demand or comms error
        MM1=0x80;//parity is 0
        MM2=0x00;//0 bit set
        MM3=0x02;//DHW enabled, but no CH enabled!!!!!
        MM4=0x00;
      }
      else
// The "if/else" comented here is to switch off the circulation pump when it reaches the target.
// however, this leads to longer times to cool down the circulation water and retriggering the heating
// should be better arranged such that the pump can be switched off after a bit, but garanteeing that water
// will still circulate.
//
        if (buf.temp > buf.returntemp + 2 or flame == true or buf.temp < buf.returntemp) // this piece of logic should be properly debugged
        {//there is demand
          MM1=0x00;//parity is 0
          MM2=0x00;//0 bit set
          MM3=0x03;//DHW and CH enabled!!!!!
          MM4=0x00;
        }
        else
        { //no demand or comms error
          MM1=0x80;//parity is 1
          MM2=0x00;//0 bit set
          MM3=0x02;//DHW enabled, but no CH enabled!!!!!
          MM4=0x00;
        }      
    }

    if (cycles == 2) //set water temp to 38 degress ADJUST TO RECEIVED TEMPERATURE AND PARITY
    {
      MM1=0x10;//no parity bit set
      MM2=0x01;
      MM3=buf.temp;
      MM4=0x00;

      byte par = parity_even_bit(MM3);
      MM1 = MM1+8*par;
    }
    //other messages to try out
    if (cycles == 3) // ID 25 - CH water temperature
    {
      MM1=0x80;
      MM2=0x19;
      MM3=0x00;
      MM4=0x00;
    }
    if (cycles == 4) // ID 28 - return water temperature
    {
      MM1=0x80;
      MM2=0x1C;
      MM3=0x00;
      MM4=0x00;
    }

    if (cycles == 5) // less frequent messages
    {
      inner_cycles++;
      if (inner_cycles == 1) // ID  5 - faults in water pressure and flame
      {
        MM1=0x00;
        MM2=0x05;
        MM3=0x00;
        MM4=0x00;
      }
      if (inner_cycles == 2) // ID 26 - DHW temp
      {
        MM1=0x80;
        MM2=0x1A;
        MM3=0x00;
        MM4=0x00;
      }
      if (inner_cycles == 3) // ID 116 - burner starts
      {
        MM1=0x00;
        MM2=0x74;
        MM3=0x00;
        MM4=0x00;
      }
      if (inner_cycles == 4) // ID 117 - CH pump starts
      {
        MM1=0x80;
        MM2=0x75;
        MM3=0x00;
        MM4=0x00;
      }
      if (inner_cycles == 5) // ID 118 - DHW pump starts/valve starts
      {
        MM1=0x80;
        MM2=0x76;
        MM3=0x00;
        MM4=0x00;
      }
      if (inner_cycles == 6) // ID 119 - DHW burner starts
      {
        MM1=0x00;
        MM2=0x77;
        MM3=0x00;
        MM4=0x00;
      }
      if (inner_cycles == 7) // ID 120 - burner hours
      {
        MM1=0x00;
        MM2=0x78;
        MM3=0x00;
        MM4=0x00;
      }
      if (inner_cycles == 8) // ID 121 CH pump hours
      {
        MM1=0x80;
        MM2=0x79;
        MM3=0x00;
        MM4=0x00;
      }
      if (inner_cycles == 9) // ID 122 - DHW pump/valve hours
      {
        MM1=0x80;
        MM2=0x7A;
        MM3=0x00;
        MM4=0x00;
      }
      if (inner_cycles == 10) // ID 123 DHW burner hours
      {
        MM1=0x00;
        MM2=0x7B;
        MM3=0x00;
        MM4=0x00;
        //in the last message reset cycles
        inner_cycles = 0;
      }
      if (inner_cycles == 11) // ID  3 - slave config flags
      {
        MM1=0x00;
        MM2=0x03;
        MM3=0x00;
        MM4=0x00;
      }
      cycles = 0;
    }

    //    lcd.setCursor(13, 1);
    //    lcd.print(total_cycles);


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

  unsigned int ut;
  int t;
  float f;


  switch(data_id) {
  case 0:  // status
    if (data_value & 4) {  // perform bitmasking on status frame
      lcd.setCursor(3,1);
      lcd.print(" ");//DHW
      lcd.write(CHAR_OK);
    }
    else if (data_value & 2) {
      lcd.setCursor(3,1);
      lcd.write(CHAR_OK);
      lcd.print(" ");//CH
      //lcd.print(temp_set_to);
    }
    else {
      lcd.setCursor(3,1);
      lcd.print("  ");
    }

    if (data_value & 8) {
      lcd.setCursor(2,1);
      lcd.write(CHAR_OK);
      //lcd.print("!");//FLAME
      flame = true;
    }
    else {
      lcd.setCursor(2,1);
      lcd.print(" ");
      flame = false;
    }
    buf.boilerstatus = max(buf.boilerstatus, (byte)data_value);
    break;
  case 1:  // Control setpoint
    t = data_value / 256;  // don't care about decimal values
    buf.temp = (byte)t;
    lcd.setCursor(12,1);
    //lcd.print("set burner to ");
    lcd.print(t);
    lcd.print(" ");
    //lcd.print("C");
    break;
  case 3:  //slave configs
    //lcd.print("slave flags OK    ");
    break;

  case 5:  //faults water and flame
    //lcd.print("faults flags OK    ");
    break;

  case 16:  // room setpoint
    f = (float)data_value / 256;
    //lcd.print("room set to ");
    //lcd.print(f);
    if (data_value % 256 == 0) {
      //lcd.print("C");
    }
    //lcd.print("     ");
    break;
  case 17:  //modulation level
    t = data_value / 256;  // don't care about decimal values
    //lcd.print("modul: ");
    //lcd.print(t);
    //lcd.print("     ");
    break;
  case 18:  //water pressure
    t = data_value / 256;  // don't care about decimal values
    //lcd.print("H20 press: ");
    //lcd.print(t);
    //lcd.print("     ");
    break;
  case 19:  //DHW flow rate
    t = data_value / 256;  // don't care about decimal values
    //lcd.print("DHW rate: ");
    //lcd.print(t);
    //lcd.print("     ");
    break;
  case 25:  //CH temp
    t = data_value / 256;  // don't care about decimal values
    buf.CHtemp = max(buf.CHtemp, (byte)t);
    lcd.setCursor(6,1);
    //lcd.print("CH temp: ");
    if (t<10)
    {
      lcd.print(" ");
    }
    lcd.print(t);
    lcd.print(" ");
    break;
  case 26:  //DHW temp - does not get reported properly in my boiler AMVV
    t = data_value / 256;  // don't care about decimal values
    //buf.CHtemp = (byte)t;
    //lcd.setCursor(6,1);
    //lcd.print("DHW temp: ");
    //lcd.print(t);
    //lcd.print(" ");
    break;
  case 28:  //return temp
    t = data_value / 256;  // don't care about decimal values
    buf.returntemp = max(buf.returntemp, byte(t));
    lcd.setCursor(9,1);
    //lcd.print("ret temp: ");
    lcd.print(t);
    lcd.print(" ");
    break;
  case 33:  //exhaust temp
    t = data_value / 256;  // don't care about decimal values
    //lcd.print("exh temp: ");
    //lcd.print(t);
    //lcd.print("     ");
    break;
  case 116:  //burner starts
    ut = (unsigned int)data_value;  // don't care about decimal values
    extbuf.burner_starts = ut;
    //lcd.print("bur st: ");
    //lcd.print(ut);
    //lcd.print("     ");
    break;
  case 117:  //CH pump starts
    ut = (unsigned int)data_value;  // don't care about decimal values
    extbuf.CH_pump_starts = ut;
    //lcd.print("CH pump st: ");
    //lcd.print(ut);
    //lcd.print("     ");
    break;
  case 118:  //DHW starts
    ut = (unsigned int)data_value;  // don't care about decimal values
    extbuf.DHW_pump_starts = ut;
    //lcd.print("DHW st: ");
    //lcd.print(ut);
    //lcd.print("     ");
    break;
  case 119:  //DHW burner starts
    ut = (unsigned int)data_value;  // don't care about decimal values
    extbuf.DHW_burner_starts = ut;
    //lcd.print("DHW bur st: ");
    //lcd.print(ut);
    //lcd.print("     ");
    break;
  case 120:  //burner hours
    ut = (unsigned int)data_value;  // don't care about decimal values
    extbuf.burner_hours = ut;
    //lcd.print("bur h: ");
    //lcd.print(ut);
    //lcd.print("     ");
    break;
  case 121:  //CH pump hours
    ut = (unsigned int)data_value;  // don't care about decimal values
    extbuf.CH_pump_hours = ut;
    //lcd.print("CH p h: ");
    //lcd.print(ut);
    //lcd.print("     ");
    break;
  case 122:  //DHW pump hours
    ut = (unsigned int)data_value;  // don't care about decimal values
    extbuf.DHW_pump_hours = ut;
    //lcd.print("DHW p h: ");
    //lcd.print(ut);
    //lcd.print("     ");
    break;
  case 123:  //DHW burner hours
    ut = (unsigned int)data_value;  // don't care about decimal values
    extbuf.DHW_burner_hours = ut;
    //lcd.print("DHW b h: ");
    //lcd.print(ut);
    //lcd.print("     ");
    break; 
  case 24:  // room actual temperature
    if (OT.isMaster()) {
      float t = (float)data_value / 256;
      //lcd.print("Room temp ");
      //lcd.print(t);
      if (data_value % 256 == 0) {
        //lcd.print(".0");
      }
      //lcd.print("C");
    }
    break;
  default:
    //lcd.print("incorrect message ID   ");
    break;

  }  // switch  
}