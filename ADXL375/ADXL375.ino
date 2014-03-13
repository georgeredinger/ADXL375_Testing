//Add the SPI library so we can communicate with the ADXL345 sensor

//#include <avr/sleep.h>
//#include <avr/power.h>
//#include <avr/wdt.h>
#include "Pins.h"
#include <Wire.h>

#define DEVICE (0x53)    //ADXL345 device address
#define TO_READ (6)        //num of bytes we are going to read each time (two bytes for each axis)


byte buff[6] ;    //6 bytes buffer for saving data read from the device
char str[64];                      //string buffer to transform data before sending it to the serial port




void setup(){ 
  pinMode(led,OUTPUT);
  pinMode(RED,OUTPUT);
  pinMode(YELLOW,OUTPUT);
  // writeTo(DEVICE, 0x2D, 0);      
  //  writeTo(DEVICE, 0x2D, 16);

  //  pinMode(interrupt_pin, INPUT);
  // digitalWrite(interrupt_pin, HIGH);    // pullup
  //  attachInterrupt(0, biff, RISING);
  Wire.begin();        // join i2c bus (address optional for master)
  Serial.begin(115200);  // start serial for output
  Serial.println("Begin");
 
  enableADXL();
  setsamplerate();
  delay(60);

}
//int16_t ax, ay, az;
volatile  bool bam=false;
//
void biff(){
  bam=true;
  digitalWrite(led,!digitalRead(led));

  // if((unsigned)buff[0] & 0x40) {
  //   bams++;
  //   bam=true;
  //  Serial.println("Oow");
  // }

}

long timeout = millis();
bool lighton = false;
//int bam;
void loop(){
  int x,y,z;
  int wack;
  readXYZ(&x,&y,&z);
  x=abs(x);
  y=abs(y);
  z=abs(z);
  
     wack=y+x+z;

 
  
  
  if(wack > 100){
     sprintf(str, "%d ,%d ,%d, %d", x, y, z,wack);  
      Serial.print(str);
      Serial.write(10);
  }


  if(wack >= 500 ){
    digitalWrite(RED,HIGH);
    timeout = millis();
    lighton = true;

  } 
  else {
    if (wack > 256){
      digitalWrite(YELLOW,HIGH);
      timeout = millis();
      lighton=true;
    }
  }

  if((lighton) && ((millis() - timeout) > 1000)){
    digitalWrite(RED,LOW);
    digitalWrite(YELLOW,LOW);
    lighton=false;
  }
}











