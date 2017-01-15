#include <SimpleModbusSlave.h>
#include <SoftwareSerial.h>

#define  alarmPin  13 // alarm digital input  
const int tempPin = A0; // temperature sensor input 

int analogTemp;// analog temperature value 
float voltageTemp; // analog temperature value scaled to voltage
float tempF; // temperature in fahrenheit 
int tempState; // current temperature in Celsius

SoftwareSerial modbusSerial(2,3);

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
  
  NOTES

  The modbus_update() method updates the holdingRegs register array and checks
  communication.
 
  The Arduino serial ring buffer is 128 bytes or 64 registers.
 
  In a function 3 request the master will attempt to read from your
  slave and since 5 bytes is already used for ID, FUNCTION, NO OF BYTES
  and two BYTES CRC the master can only request 122 bytes or 61 registers.

  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */


// Using the enum instruction allows for an easy method for adding and 
// removing registers. Doing it this way saves you #defining the size 
// of your slaves register array each time you want to add more registers
// and at a glimpse informs you of your slaves register layout.

// registers of Modbus slave
enum 
{     
  // just add or remove registers and your good to go
  // The first register starts at address 0
  ALARM_STATE,
  TEMP_STATE,
  TOTAL_ERRORS,
  TOTAL_REGS_SIZE 
};

unsigned int holdingRegs[TOTAL_REGS_SIZE]; // function 3 register array

// This is only ran once
void setup()
{
  /* parameters(long baudrate, 
                unsigned char ID, 
                unsigned char transmit enable pin, 
                unsigned int holding registers size,
                unsigned char low latency)
                
     The transmit enable pin is used in half duplex communication to activate 
     a MAX485 or similar to deactivate this mode use any value < 2 because 0 &
     1 is reserved for Rx & Tx. Low latency delays makes the implementation 
     non-standard but practically it works with all major modbus master
     implementations.
  */
  
  // RS-485 shield will use D2 as the transmit enable pin, slave ID will 
  // start at 1 
  modbus_configure(115200, 1, 1, TOTAL_REGS_SIZE, 0);
  pinMode(alarmPin, INPUT);
  pinMode(tempPin, INPUT);
}

void loop()
{
  // modbus_update() is the only method used in loop(). It returns the total 
  // error count since the slave started. Optional, but it's useful
  // for fault finding by the modbus master.
  holdingRegs[TOTAL_ERRORS] = modbus_update(holdingRegs);
  
  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
  
    ALARM INPUT

    Reads alarm state from digital input pin and updates holding register.

  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

  // read alarm pin (HIGH or LOW)
  byte alarmState = digitalRead(alarmPin); 
  
  // assign the alarm state value to the holding register
  holdingRegs[ALARM_STATE] = alarmState; 

  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
  
    TEMP INPUT

    Reads analog temperature input from analog input pin, converts to 
    temperature in fahrenheit, and updates holding register.

  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

  // read analog temperature value 
  analogTemp = analogRead(tempPin);
  
  //convert ADC reading to voltage 
  voltageTemp = (analogTemp/1024.0)*5.0;

  //convert voltage value to temperature(F)
  //TMP36 is 25C at 750 mV, +/- 10mV for every 1C
  //-0.5 offscale results from conversion of V to C for TMP 36
  tempF = ((9.0/5.0)*(voltageTemp - 0.5)*100) + 32.0;
  tempState = (int) tempF;

  // assign temperature value to holding register
  holdingRegs[TEMP_STATE] = tempState;

}
