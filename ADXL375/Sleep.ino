#include <avr/sleep.h>
#include <Arduino.h> // Arduino 1.0

long sleep_count;

void goToSleep()   
{
// The ATmega328 has five different sleep states.
// See the ATmega 328 datasheet for more information.
// SLEEP_MODE_IDLE -the least power savings 
// SLEEP_MODE_ADC
// SLEEP_MODE_PWR_SAVE
// SLEEP_MODE_STANDBY
// SLEEP_MODE_PWR_DOWN -the most power savings
// I am using the deepest sleep mode from which a
// watchdog timer interrupt can wake the ATMega328

set_sleep_mode(SLEEP_MODE_PWR_DOWN); // Set sleep mode.
sleep_enable(); // Enable sleep mode.
sleep_mode(); // Enter sleep mode.
// After waking from watchdog interrupt the code continues
// to execute from this point.

sleep_disable(); // Disable sleep mode after waking.
                     
}

void disableADC() {
  ADCSRA = ADCSRA & B01111111;
}

void disableAnalogComparitor(){
  ACSR = B10000000;
}

void disableDigitalInputBuffers(){
  DIDR0 = DIDR0 | B00111111;
}

void watchdogOn() {
  
// Clear the reset flag, the WDRF bit (bit 3) of MCUSR.
MCUSR = MCUSR & B11110111;
  
// Set the WDCE bit (bit 4) and the WDE bit (bit 3) 
// of WDTCSR. The WDCE bit must be set in order to 
// change WDE or the watchdog prescalers. Setting the 
// WDCE bit will allow updtaes to the prescalers and 
// WDE for 4 clock cycles then it will be reset by 
// hardware.
WDTCSR = WDTCSR | B00011000; 

// Set the watchdog timeout prescaler value to 1024 K 
// which will yeild a time-out interval of about 8.0 s.
//WDTCSR = B00100001;
//WDTCSR  = (0<<WDP3)|(1<<WDP2) | (1<<WDP1) ; // set //prescaler to 1 second
//Watchdog Timer Prescale Select
//
//WDP3 WDP2 WDP1 WDP0         Number of WDT     Typical Time-out at
//                                              Oscillator Cycles     VCC = 5.0V
//
//   0         0      0        0             2K (2048) cycles       16 ms
//   0         0      0        1             4K (4096) cycles       32 ms
//   0         0      1        0             8K (8192) cycles       64 ms
//   0         0      1        1            16K (16384) cycles    0.125 s
//   0         1      0        0            32K (32768) cycles    0.25 s
//WDTCSR  = (0<<WDP3)|(0<<WDP2) | (1<<WDP1) | 1; // set //prescaler to .125 second
//WDTCSR  = (0<<WDP3)|(0<<WDP2) | (1<<WDP1) | 0; // set //prescaler to .64ms 


//   0         1      0        1            64K (65536) cycles    0.5 s
//WDTCSR  = (0<<WDP3)|(1<<WDP2) | (0<<WDP1) | 1; // set //prescaler to .5 second

//   0         1      1        0            128K (131072) cycles 1.0 s
//   0         1      1        1            256K (262144) cycles 2.0 s
//   1         0      0        0            512K (524288) cycles 4.0 s
//   1         0      0        1            1024K (1048576) cycles 8.0 s
// Enable the watchdog timer interupt.
WDTCSR  = (1<<WDP3)|(0<<WDP2) | (0<<WDP1) | 1; // set //prescaler to .5 second

WDTCSR = WDTCSR | B01000000;
MCUSR = MCUSR & B11110111;

}

//ISR(WDT_vect)
//{
//sleep_count ++; // keep track of how many sleep cycles
//// have been completed.
//}

ISR(WDT_vect) { Sleepy::watchdogEvent(); }


