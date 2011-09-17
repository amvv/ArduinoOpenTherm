/*
 * author: Sebastian Wallin
 * description:
 * Example on how to configure the periodical execution of a user
 * defined function (Interrupt service routine) using Timer2. This
 * example will run the function every 1ms.
 *
 * For detailed information on Timer2 configuration see chapter 17 in
 * ATMEGA328 datasheet.
 */
 
 #define OUTPUT_PIN (5)//for use with JeeNodeUSB
// #define OUTPUT_PIN (9)//for use with the garbage collector box

/* Timer2 reload value, globally available */
unsigned int tcnt2;

/*my stuff amvv */
int cycles = 0;
  int originalvalue = 0;
//  int value = 0;
  int y;
  int bits_sent = 0;
  
  byte bit_out = LOW;
  boolean done = true;
  byte state = 0; //0 - start bit; 1 - message: 2 - stop bit

typedef struct {
        byte message_type;
	byte data_id;
	byte data_value_h;
	byte data_value_l;
} OTdata;


//OTdata MM;
byte MM1=0;
byte MM2=0;
byte MM3=0;
byte MM4=0;

/* Toggle HIGH or LOW digital write */
int toggle = 0;

/* Setup phase: configure and enable timer2 overflow interrupt */
void setup() {

//  MM.message_type = 127;
//  MM.data_id = 256;
//  MM.data_value_h = 0;
//  MM.data_value_l = 100;

  Serial.begin(19200);
  /* Configure the test pin as output */
  pinMode(OUTPUT_PIN, OUTPUT); 

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
   * The following loads the value 131 into the Timer 2 counter register
   * The math behind this is:
   * (CPU frequency) / (prescaler value) = 125000 Hz = 8us.
   * (desired period) / 8us = 125.
   * MAX(uint8) + 1 - 125 = 131;
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
  /* Write to a digital pin so that we can confirm our timer */
  
  
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
  
  
    //digitalWrite(OUTPUT_PIN, toggle == 0 ? HIGH : LOW);  
    //toggle = ~toggle;  


    digitalWrite(OUTPUT_PIN, !y);
    Serial.print(y, DEC);

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
    digitalWrite(OUTPUT_PIN, LOW);
      done = true;
      //originalvalue = originalvalue + 1;

      bits_sent = 0;
      StopInterrupts();
//    Serial.print(" - ");
//    Serial.print(millis());
      state=0;

    }
    else
    {
      done = false;
    }
}

/* Main loop. Empty, but needed to avoid linker errors */
void loop() {
  
  if (done == true)
  {
    delay(950);
    cycles++;
    if (cycles == 2) //set water temp to 30 degress
    {
      MM1=0x90;
      MM2=0x01;
      MM3=0x20;
      MM4=0x00;
      cycles = 0;
    }
    else
    {
      MM1=0x00;//parity is 0
      MM2=0x00;//1 bit set
      MM3=0x03;//E has 4 bits set
      MM4=0x00;
    }
    Serial.println();
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
 
