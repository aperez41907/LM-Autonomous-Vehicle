#include <Wire.h>
////////////////////use below pins for input and out put/////////////////////////
/////////*************On the Arduino, SDA is on analog input pin 4, and SCL on analog input pin 5.****************///////////////////////


// I2C SRF10 or SRF08 Devantech Ultrasonic Ranger Finder 
// by Nicholas Zambetti 
// and James Tichenor

void setup() 
{ 
Wire.begin(); // join i2c bus (address optional for master) 
Serial.begin(9600);
// start serial communication at 9600bps 
} 

int reading = 0; 

void loop()


////////////////sensitivity adjust/////////////////////////////////////////////
{ Wire.beginTransmission(0x70); // transmit to device #112 (0x70) 
// the address specified in the datasheet is 224 (0xE0) 
// but i2c adressing uses the high 7 bits so it's 112 
Wire.write(0x01); // sets register pointer to the command register (0x01) reg 1
Wire.write(0x21); // command sensor to analog gain (sensitivity of sensor) 
// use 0x51 for centimeters 
// use 0x52 for ping microseconds 
Wire.endTransmission(); // stop transmitting 
//////////////////////range select////////////////////////////
Wire.beginTransmission(0x70); // transmit to device #112 (0x70) 
// the address specified in the datasheet is 224 (0xE0) 
// but i2c adressing uses the high 7 bits so it's 112 
Wire.write(0x01); // sets register pointer to the command register (0x10) reg 2 (CHANGE THIS)
Wire.write(0x00); // command sensor to analog range (distance) 
// use 0x51 for centimeters 
// use 0x52 for ping microseconds 
Wire.endTransmission(); // stop transmitting 

//////////////////////////// step 1: instruct sensor to read echoes ///////////////////////////////////////
Wire.beginTransmission(0x70); // transmit to device #112 (0x70) 
// the address specified in the datasheet is 224 (0xE0) 
// but i2c adressing uses the high 7 bits so it's 112 
Wire.write(0x00); // sets register pointer to the command register (0x00) write 
Wire.write(0x50); // command sensor to measure in "inches" (0x50) 
// use 0x51 for centimeters 
// use 0x52 for ping microseconds 
Wire.endTransmission(); // stop transmitting 

//////////////// step 2: wait for readings to happen ///////////////////////////////////////////////
delay(70); // datasheet suggests at least 65 milliseconds 

////////////// step 3: instruct sensor to return a particular echo reading ////////////////////////////
Wire.beginTransmission(0x70); // transmit to device #112 
Wire.write(0x02); // sets register pointer to echo #1 register (0x02) 
Wire.endTransmission(); // stop transmitting 

//////////////// step 4: request reading from sensor /////////////////////////
Wire.requestFrom(0x70, 2); // request 2 bytes from slave device #112 

/////////////// step 5: receive reading from sensor ////////////////////////////
if(2 <= Wire.available()) // if two bytes were received 
{ 
reading = Wire.read(); // receive high byte (overwrites previous reading) 
reading = reading << 8; // shift high byte to be high 8 bits 
reading |= Wire.read(); // receive low byte as lower 8 bits 
Serial.println(reading); // print the reading 
} 

delay(250); // wait a bit since people have to read the output
} 


/* 

// The following code changes the address of a Devantech Ultrasonic Range Finder (SRF10 or SRF08) 
// usage: changeAddress(0x70, 0xE6); 

void changeAddress(byte oldAddress, byte newAddress) 
{ 
Wire.beginTransmission(oldAddress); 
Wire.write(0x00); 
Wire.write(0xA0); 
Wire.endTransmission(); 

Wire.beginTransmission(oldAddress); 
Wire.write(0x00); 
Wire.write(0xAA); 
Wire.endTransmission(); 

Wire.beginTransmission(oldAddress); 
Wire.write(0x00); 
Wire.write(0xA5); 
Wire.endTransmission(); 

Wire.beginTransmission(oldAddress); 
Wire.write(0x00); 
Wire.write(newAddress); 
Wire.endTransmission(); 
} 

*/ 
