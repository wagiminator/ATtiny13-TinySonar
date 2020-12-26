// tinySonar - Ultrasonic Ranging using ATtiny13 and HC-SR04
//
// The ultrasonic ranging module HC-SR04 provides 2cm-400cm non-contact
// measurement function, the ranging accuracy can reach to 3mm. The 
// module includes ultrasonic transmitter, receiver and control circuit.
// The basic principle of work:
// - start measurement by applying high signal on TRIG for at least 10us,
// - module automatically sends eight 40 kHz pulses and detects echos,
// - module applies high signal on ECHO with the same duration as the time 
//   it took from sending ultrasonic to returning.
//
// For measuring the duration of the ECHO signal the timer0 is used with 
// no prescaler, so it runs at MCU clock speed. Timer overflow interrupt is 
// utilized to expand the 8-bit timer/counter to a virtual 16-bit counter.
// To calculate the distance in mm from the counter value, the following 
// approximation formula was derived:
// -  duration[s]  = counter / F_CPU = counter / 1200000[1/s]
// -  distance[mm] = duration * speed of sound / 2 = duration[s] * 340260[mm/s] / 2
// -> distance[mm] = counter * 340260[mm/s] / 1200000[1/s] / 2
// -> distance[mm] ~ counter / 7
//
// The distance in mm is shown on a 4-digit 7-segment display which is
// controlled via a MAX7219 in BCD decode mode. The SPI protocol is
// implemented with a simple bitbanging method. The MAX7219 is fast enough
// to be driven without delays even at the fastest clock speed of the
// ATtiny13. A transmission to the MAX7219 starts with pulling the CS
// line LOW. Afterwards two bytes are transmitted, the register address
// first and the register data second. The bytes are transmitted most
// significant bit first by setting the DIN line HIGH for a bit "1" or
// LOW for a bit "0" while the CLK line is LOW. The bit is shifted out on
// the rising edge of the CLK line. By setting the CS line HIGH again the
// end of the transmission is signified, the MAX7219 latches the two
// received bytes and writes the data byte into the register. 
//
//                              +-\/-+
//            --- A0 (D5) PB5  1|    |8  Vcc
// MAX7219 CS --- A3 (D3) PB3  2|    |7  PB2 (D2) A1 --- MAX7219 CLK
//       ECHO --- A2 (D4) PB4  3|    |6  PB1 (D1) ------ MAX7219 DIN
//                        GND  4|    |5  PB0 (D0) ------ TRIG
//                              +----+    
//
// Controller: ATtiny13
// Core:       MicroCore (https://github.com/MCUdude/MicroCore)
// Clockspeed: 1.2 MHz internal
// BOD:        2.7V
// Timing:     Micros disabled (Timer0 is in use)
//
// 2020 by Stefan Wagner 
// Project Files (EasyEDA): https://easyeda.com/wagiminator
// Project Files (Github):  https://github.com/wagiminator
// License: http://creativecommons.org/licenses/by-sa/3.0/


// libraries
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// pin definitions
#define TRIG  PB0
#define ECHO  PB4
#define DIN   PB1
#define CLK   PB2
#define CS    PB3

// -----------------------------------------------------------------------------
// MAX7219 7-Segment Display Implementation
// -----------------------------------------------------------------------------

// shift out byte value to MAX7219
void SEG_byte(uint8_t value) {
  for(uint8_t i=8; i; i--, value <<= 1) {   // shift out 8 bits, MSB first
    PORTB &= ~(1<<DIN);                     // clear the bit first (saves some flash this way)
    if(value & 0x80) PORTB |= (1<<DIN);     // set bit if appropriate
    PORTB |=  (1<<CLK);                     // clock high -> shift out the bit
    PORTB &= ~(1<<CLK);                     // clock low (50ns < 1000 / 1.2)
  }
}

// send command to MAX7219
void SEG_send(uint8_t reg, uint8_t data) {
  PORTB &= ~(1<<CS);                        // set CS low  -> select device
  SEG_byte(reg);                            // send address byte
  SEG_byte(data);                           // send data byte
  PORTB |= (1<<CS);                         // set CS high -> latch the bytes
}

// print a number on display
void SEG_print(uint16_t number) {
  for(uint8_t i=1; i<5; i++, number /= 10) SEG_send(i, number % 10);
}

// init MAX7219
void SEG_init(void) {
  DDRB  |= (1<<DIN) | (1<<CLK) | (1<<CS);   // set control pins as output
  PORTB |= (1<<CS);                         // pull high CS line
  SEG_send(0x09, 0x0f);                     // set BCD decode for 4 digits
  SEG_send(0x0a, 0x0f);                     // set intensity (0x00 .. 0x0f)
  SEG_send(0x0b, 0x03);                     // set scan limit at 4 digits
  SEG_send(0x0c, 0x01);                     // shutdown mode off
  SEG_send(0x0f, 0x00);                     // display test off
}

// -----------------------------------------------------------------------------
// HC-SR04 Ultrasonic Module Implementation
// -----------------------------------------------------------------------------

// global variables
volatile uint8_t  HC_done;                  // ranging complete flag
volatile uint16_t HC_counter;               // virtual 16-bit counter

// init HC-SR04
void HC_init(void) {
  DDRB  |= (1<<TRIG);                       // set TRIG pin as output
  PCMSK |= (1<<ECHO);                       // enable interrupt on ECHO pin
  TCCR0A = 0;                               // timer/counter normal mode
  TIMSK0 = (1<<TOIE0);                      // enable overflow interrupt
  sei();                                    // enable global interrupts
}

// get distance in mm from HC-SR04
uint16_t HC_ping(void) {
  HC_done = 0;                              // reset ranging complete flag
  HC_counter = 0;                           // reset counter
  TCNT0 = 0;                                // reset timer0
  GIMSK |= (1<<PCIE);                       // enable pin change interrupts
  PORTB |= (1<<TRIG);                       // send 10us trigger pulse
  _delay_us(10);
  PORTB &= ~(1<<TRIG);                      // module starts measurement now
  while(!HC_done);                          // wait for echo
  GIMSK &= ~(1<<PCIE);                      // disable pin change interrupts
  HC_counter += TCNT0;                      // calculate total 16-bit counter value
  return(HC_counter / 7);                   // calculate and return distance
}

// pin change interrupt service routine
ISR (PCINT0_vect) {
  if (PINB & (1<<ECHO)) {                   // if rising edge on ECHO pin:
    TCCR0B = (1<<CS00);                     // start the timer
  } else {                                  // if falling edge on ECHO pin:
    TCCR0B = 0;                             // stop the timer
    HC_done = 1;                            // set ranging complete flag
  }
}

// timer0 overflow interrupt service routine
ISR (TIM0_OVF_vect) {
  HC_counter += 256;                        // increment 16-bit counter by 256 on each overflow
}

// -----------------------------------------------------------------------------
// Main Function
// -----------------------------------------------------------------------------

int main(void) {
  // setup
  SEG_init();                               // setup MAX7219
  HC_init();                                // setup HC-SR04

  // loop
  while(1) {
    SEG_print(HC_ping());                   // get distance and print it on display
    _delay_ms(200);                         // delay between pings
  }
}
