//Add the SPI library so we can communicate with the ADXL345 sensor

#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include "Pins.h"
#include "ADXL375.H"
#include <Wire.h>

extern volatile boolean red,yellow;

extern long sleep_count;

byte buff[6] ;    //6 bytes buffer for saving data read from the device
char str[64];                      //string buffer to transform data before sending it to the serial port


void enterSleep(void)
{
  set_sleep_mode(SLEEP_MODE_PWR_SAVE);   /* EDIT: could also use SLEEP_MODE_PWR_DOWN for lowest power consumption. */

  // set_sleep_mode(SLEEP_MODE_PWR_DOWN);   /* EDIT: could also use SLEEP_MODE_PWR_DOWN for lowest power consumption. */
  sleep_enable();

  /* Now enter sleep mode. */
  sleep_mode();

  /* The program will continue from here after the WDT timeout*/
  sleep_disable(); /* First thing to do is disable sleep. */

  /* Re-enable the peripherals. */
  power_all_enable();
}


void setup(){ 
  pinMode(GREEN,OUTPUT);
  pinMode(RED,OUTPUT);
  pinMode(YELLOW,OUTPUT);
  digitalWrite(GREEN,HIGH);

  watchdogOn(); 

  pinMode(interrupt_pin, INPUT);
  // digitalWrite(interrupt_pin, HIGH);    // pullup
  attachInterrupt(0, biff, RISING);
  Wire.begin();        // join i2c bus (address optional for master)
  setupFIFO();
  readFrom(DEVICE,0X30,1,buff);
  delay(60);
}
//int16_t ax, ay, az;
volatile  bool bam=false;
//
void biff(){
  bam=true;
}

void loop(){
  int maxShock;

  enterSleep();

  if(bam){
    bam=false;
    readFrom(DEVICE,0X30,1,buff);
    maxShock=maxFIFO();
    if(maxShock>250){
      if(maxShock<400 ) yellow=true;
      if(maxShock>450) red=true;
    } 
  }
  digitalWrite(GREEN,LOW);
  digitalWrite(YELLOW,LOW);
  digitalWrite(RED,LOW);

  if (sleep_count % 32==0) {
    sleep_count = 0;
    if(!(red || yellow)) digitalWrite(GREEN,HIGH);
    if(red) digitalWrite(RED,HIGH);
    else
      if(yellow) digitalWrite(YELLOW,HIGH);
  }   

}
















