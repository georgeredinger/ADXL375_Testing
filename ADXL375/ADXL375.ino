//Add the SPI library so we can communicate with the ADXL345 sensor

//#include <avr/sleep.h>
//#include <avr/power.h>
//#include <avr/wdt.h>
#include "Pins.h"
#include <Wire.h>

#define DEVICE (0x53)    //ADXL345 device address
#define TO_READ (6)        //num of bytes we are going to read each time (two bytes for each axis)


byte buff[6] ;    //6 bytes buffer for saving data read from the device



void writeTo(int device, byte address, byte val) {
   Wire.beginTransmission(device); //start transmission to device 
   Wire.write(address);        // send register address
   Wire.write(val);        // send value to write
   Wire.endTransmission(); //end transmission
}

void readFrom(int device, byte address, int num, byte buff[]) {
  Wire.beginTransmission(device); //start transmission to device 
  Wire.write(address);        //sends address to read from
  Wire.endTransmission(); //end transmission
  
  Wire.beginTransmission(device); //start transmission to device (initiate again)
  Wire.requestFrom(device, num);    // request 6 bytes from device
  
  int i = 0;
  while(Wire.available())    //device may send less than requested (abnormal)
  { 
    buff[i] = Wire.read(); // receive a byte
    i++;
  }
  Wire.endTransmission(); //end transmission
}

void setup(){ 
  pinMode(led,OUTPUT);
  pinMode(interrupt_pin, INPUT);
 // digitalWrite(interrupt_pin, HIGH);    // pullup
  attachInterrupt(0, biff, RISING);
 
 

  Wire.begin();        // join i2c bus (address optional for master)
  Serial.begin(9600);  // start serial for output
 // Serial.println("Begin");
  //Turning on the ADXL345
  //delay(60);
  writeTo(DEVICE, 0x2D, 0);      
  writeTo(DEVICE, 0x2D, 16);
  
  //1. Write 0x28 to Register 0x1D; set shock threshold to 31.2 g.
  writeTo(DEVICE,0x1D,0x28); // 2,5 Gs for an ADXL345,(OX28/0xFF)*FSR
  
//2. Write 0x50 to Register 0x21; set shock duration to 50 ms.
 writeTo(DEVICE,0x21,0x50);
//3. Write 0x20 to Register 0x22; set latency to 40 ms.
//
 writeTo(DEVICE,0x22,0x20);
//4. Write 0xF0 to Register 0x23; set shock window to 300 ms.
 writeTo(DEVICE,0x23,0xF0);
//5. Write 0x07 to Register 0x2A to enable X-, Y-, and Z-axes
// participation in shock detection.
 writeTo(DEVICE,0x2A,0x07);
//6. Write 0x0F to Register 0x2C to set output data rate to
//  3200 Hz.
 writeTo(DEVICE,0x2c,0X0F);
//7. Write 0x40/0x20 to Register 0x2E to enable single shock or
//  double shock, respectively.
 writeTo(DEVICE,0x2E,0x40);
//8. Write 0x40/0x20 to Register 0x2F to assign single shock or
//  double shock interrupt, respectively, to the INT2 pin.
 writeTo(DEVICE,0x2F,0X40);
//9.  Write 0xEA to Register 0x38 to enable triggered mode
//FIFO. If an interrupt is detected on the INT2 pin, the FIFO
//records the trigger event acceleration with 10 samples
//retained from before the trigger event.
 writeTo(DEVICE,0x38,0XEA);
//10. Write 0x08 to Register 0x2D to start the measurement.
//  It is recommended that the POWER_CTL register be
//  configured last.
 writeTo(DEVICE,0x2D,0x08);
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


void loop(){
//  enterSleep();
//      accel.getAcceleration(&ax, &ay, &az);

if(bam) {
    bam=false;
     readFrom(DEVICE,0X30,1,buff);

  }
}





