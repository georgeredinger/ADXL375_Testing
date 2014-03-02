//Add the SPI library so we can communicate with the ADXL345 sensor

#include <SPI.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>

volatile int toggle = 0;
const int interrupt_pin = 2;
const int led_pin = 9;
#define LED_PIN (9)
volatile int wakeups=0;

//Assign the Chip Select signal to pin 10.
int CS=10;

//This is a list of some of the registers available on the ADXL345.
//To learn more about these and the rest of the registers on the ADXL345, read the datasheet!
#define DEVID 0
char POWER_CTL = 0x2D;	//Power Control Register
char DATA_FORMAT = 0x31;
char DATAX0 = 0x32;	//X-Axis Data 0
char DATAX1 = 0x33;	//X-Axis Data 1
char DATAY0 = 0x34;	//Y-Axis Data 0
char DATAY1 = 0x35;	//Y-Axis Data 1
char DATAZ0 = 0x36;	//Z-Axis Data 0
char DATAZ1 = 0x37;	//Z-Axis Data 1


//ADXL345 Register Addresses
#define	DEVID		0x00	//Device ID Register
#define THRESH_TAP	0x1D	//Tap Threshold
#define	OFSX		0x1E	//X-axis offset
#define	OFSY		0x1F	//Y-axis offset
#define	OFSZ		0x20	//Z-axis offset
#define	DURATION	0x21	//Tap Duration
#define	LATENT		0x22	//Tap latency
#define	WINDOW		0x23	//Tap window
#define	THRESH_ACT	0x24	//Activity Threshold
#define	THRESH_INACT	0x25	//Inactivity Threshold
#define	TIME_INACT	0x26	//Inactivity Time
#define	ACT_INACT_CTL	0x27	//Axis enable control for activity and inactivity detection
#define	THRESH_FF	0x28	//free-fall threshold
#define	TIME_FF		0x29	//Free-Fall Time
#define	TAP_AXES	0x2A	//Axis control for tap/double tap
#define ACT_TAP_STATUS	0x2B	//Source of tap/double tap
#define	BW_RATE		0x2C	//Data rate and power mode control
#define POWER_CTL	0x2D	//Power Control Register
#define	INT_ENABLE	0x2E	//Interrupt Enable Control
#define	INT_MAP		0x2F	//Interrupt Mapping Control
#define	INT_SOURCE	0x30	//Source of interrupts
#define	DATA_FORMAT	0x31	//Data format control
#define DATAX0		0x32	//X-Axis Data 0
#define DATAX1		0x33	//X-Axis Data 1
#define DATAY0		0x34	//Y-Axis Data 0
#define DATAY1		0x35	//Y-Axis Data 1
#define DATAZ0		0x36	//Z-Axis Data 0
#define DATAZ1		0x37	//Z-Axis Data 1
#define	FIFO_CTL	0x38	//FIFO control
#define	FIFO_STATUS	0x39	//FIFO status

//This buffer will hold values read from the ADXL345 registers.
char values[10];
char output[20];
//These variables will be used to hold the x,y and z axis accelerometer values.
int x,y,z;
double xg, yg, zg;
char tapType=0;
















/***************************************************
 *  Name:        ISR(WDT_vect)
 *
 *  Returns:     Nothing.
 *
 *  Parameters:  None.
 *
 *  Description: Watchdog Interrupt Service. This
 *               is executed when watchdog timed out.
 *
 ***************************************************/
ISR(WDT_vect)
{
   if(wakeups % 4 == 0)
      digitalWrite(LED_PIN, HIGH);
     else
      digitalWrite(LED_PIN,LOW);
   
 
}

void pin2Interrupt(void)
{
  /* This will bring us back from sleep. */
  toggle = 1;
  
  /* Delay in here as a very crude debouncing mechanism. */ 
    detachInterrupt(0);
    Serial.println("Ouch");
  delay(100);

}
/***************************************************
 *  Name:        enterSleep
 *
 *  Returns:     Nothing.
 *
 *  Parameters:  None.
 *
 *  Description: Enters the arduino into sleep mode.
 *
 ***************************************************/
void enterSleep(void)
{
  
  /* Setup pin2 as an interrupt and attach handler. */
  attachInterrupt(0, pin2Interrupt, LOW);
  delay(100);
  
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
 //  set_sleep_mode(SLEEP_MODE_PWR_SAVE);   /* EDIT: could also use SLEEP_MODE_PWR_DOWN for lowest power consumption. */
  sleep_enable();
  
  sleep_mode();
  
  /* The program will continue from here. */
  
  /* First thing to do is disable sleep. */
  sleep_disable(); 
  sleep_disable(); /* First thing to do is disable sleep. */
  
  /* Re-enable the peripherals. */
  power_all_enable();
}

void setup(){ 
   pinMode(led_pin,OUTPUT);
  pinMode(interrupt_pin, INPUT);
  attachInterrupt(0, pin2Interrupt, RISING);
  
  Serial.println("Initialisation complete.");
      digitalWrite(led_pin, HIGH);

  //Initiate an SPI communication instance.
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV32);
  //Configure the SPI connection for the ADXL345.
  SPI.setDataMode(SPI_MODE3);
  //Create a serial connection to display the data on the terminal.
  Serial.begin(9600);
  
  //Set up the Chip Select pin to be an output from the Arduino.
  pinMode(CS, OUTPUT);
  //Before communication starts, the Chip Select pin needs to be set high.
  digitalWrite(CS, HIGH);
  
//  writeRegister(DATA_FORMAT, 0); //full range?
  //Put the ADXL345 into Measurement Mode by writing 0x08 to the POWER_CTL register.
//  writeRegister(POWER_CTL, 0x08);  //Measurement mode  
  delay(100);
   /* Clear the reset flag. */
  MCUSR &= ~(1<<WDRF);
  
  /* In order to change WDE or the prescaler, we need to
   * set WDCE (This will allow updates for 4 clock cycles).
   */
  WDTCSR |= (1<<WDCE) | (1<<WDE);

  /* set new watchdog timeout prescaler value */
  WDTCSR = 0<<WDP3 | 1<< WDP2 | 0<<WDP1| 0<<WDP0  ; /* .25 seconds */
  
  /* Enable the WD interrupt (note no reset). */
  WDTCSR |= _BV(WDIE);
  
  Serial.println("Initialisation complete.");
  delay(100); //Allow for serial print to complete.
    attachInterrupt(0, tap, FALLING);
  //Put the ADXL345 into +/- 4G range by writing the value 0x01 to the DATA_FORMAT register.
  writeRegister(DATA_FORMAT, 0x01);

  //Send the Tap and Double Tap Interrupts to INT1 pin
  writeRegister(INT_MAP, 0x9F);
  //Look for taps on the Z axis only.
  writeRegister(TAP_AXES, 0x01);
  //Set the Tap Threshold to 3g
  writeRegister(THRESH_TAP, 0x38);
  //Set the Tap Duration that must be reached
  writeRegister(DURATION, 0x10);
  
  //100ms Latency before the second tap can occur.
  writeRegister(LATENT, 0x50);
  writeRegister(WINDOW, 0xFF);
  
  //Enable the Single and Double Taps.
  writeRegister(INT_ENABLE, 0xE0);  
  
  //Put the ADXL345 into Measurement Mode by writing 0x08 to the POWER_CTL register.
  writeRegister(POWER_CTL, 0x08);  //Measurement mode
  readRegister(INT_SOURCE, 1, values); //Clear the interrupts from the INT_SOURCE register.
}
void tap(void){
  //Clear the interrupts on the ADXL345
  readRegister(INT_SOURCE, 1, values); 
  if(values[0] & (1<<5))tapType=2;
  else tapType=1;;
}

void loop(){
   char devid[1];
     delay(1000); 
  wakeups++;

  //Reading 6 bytes of data starting at register DATAX0 will retrieve the x,y and z acceleration values from the ADXL345.
  //The results of the read operation will get stored to the values[] buffer.
 //readRegister(DATAX0, 6, values);
  readRegister(DEVID,1,devid);
  Serial.println((int)devid[0]& 0xFF,HEX);
  //The ADXL345 gives 10-bit acceleration values, but they are stored as bytes (8-bits). To get the full value, two bytes must be combined for each axis.
  //The X value is stored in values[0] and values[1].
//  x = ((int)values[1]<<8)|(int)values[0];
//  //The Y value is stored in values[2] and values[3].
//  y = ((int)values[3]<<8)|(int)values[2];
//  //The Z value is stored in values[4] and values[5].
//  z = ((int)values[5]<<8)|(int)values[4];
  
  //Print the results to the terminal.
//  Serial.print(x, DEC);
//  Serial.print(',');
//  Serial.print(y, DEC);
//  Serial.print(',');
//  Serial.println(z, DEC);    
// 
   
    delay(200);
    enterSleep();
   if(tapType > 0)
  {
    if(tapType == 1){
      Serial.println("SINGLE");
      Serial.print(x);
      Serial.print(',');
      Serial.print(y);
      Serial.print(',');
      Serial.println(z);
    }
    else{
      Serial.println("DOUBLE");
      Serial.print((float)xg,2);
      Serial.print("g,");
      Serial.print((float)yg,2);
      Serial.print("g,");
      Serial.print((float)zg,2);
      Serial.println("g");
    }
    detachInterrupt(0);
    delay(500);
    attachInterrupt(0, tap, FALLING);
    tapType=0;    
  }
  delay(10); 
}

//This function will write a value to a register on the ADXL345.
//Parameters:
//  char registerAddress - The register to write a value to
//  char value - The value to be written to the specified register.
void writeRegister(char registerAddress, char value){
  //Set Chip Select pin low to signal the beginning of an SPI packet.
  digitalWrite(CS, LOW);
  //Transfer the register address over SPI.
  SPI.transfer(registerAddress);
  //Transfer the desired register value over SPI.
  SPI.transfer(value);
  //Set the Chip Select pin high to signal the end of an SPI packet.
  digitalWrite(CS, HIGH);
}

//This function will read a certain number of registers starting from a specified address and store their values in a buffer.
//Parameters:
//  char registerAddress - The register addresse to start the read sequence from.
//  int numBytes - The number of registers that should be read.
//  char * values - A pointer to a buffer where the results of the operation should be stored.
void readRegister(char registerAddress, int numBytes, char * values){
  //Since we're performing a read operation, the most significant bit of the register address should be set.
  char address = 0x80 | registerAddress;
  //If we're doing a multi-byte read, bit 6 needs to be set as well.
  if(numBytes > 1) address = address | 0x40;
  
  //Set the Chip select pin low to start an SPI packet.
  digitalWrite(CS, LOW);
  //Transfer the starting register address that needs to be read.
  SPI.transfer(address);
  //Continue to read registers until we've read the number specified, storing the results to the input buffer.
  for(int i=0; i<numBytes; i++){
    values[i] = SPI.transfer(0x00);
  }
  //Set the Chips Select pin high to end the SPI packet.
  digitalWrite(CS, HIGH);
}
