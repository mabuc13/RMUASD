// Wire Slave Receiver
// by Nicholas Zambetti <http://www.zambetti.com>

// Demonstrates use of the Wire library
// Receives data as an I2C/TWI slave device
// Refer to the "Wire Master Writer" example for use with this

// Created 29 March 2006

// This example code is in the public domain.


#include <Wire.h>

#define MY_ADDRESS 13

#define COORDINATE_LEN 12

volatile float coordinate_0[3] = {0.32, -1.5743, 3.127};
volatile float coordinate_1[3] = {5.12, -142.9453, -3.44114411};

unsigned char coordinate_bytes_0[COORDINATE_LEN];
unsigned char coordinate_bytes_1[COORDINATE_LEN];

void setup() {
  Wire.begin(MY_ADDRESS);                // join i2c bus with address #8
  Wire.onReceive(receiveEvent); // register event
  Wire.onRequest(requestEvent);
  Serial.begin(9600);           // start serial for output

  convert_to_bytes(&coordinate_0, &coordinate_bytes_0, COORDINATE_LEN);
  convert_to_bytes(&coordinate_1, &coordinate_bytes_1, COORDINATE_LEN);
}
void loop() {
  delay(100);
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany) {
//  while (1 < Wire.available()) { // loop through all but the last
//    char c = Wire.read(); // receive byte as a character
//    Serial.print(c);         // print the character
//  }
//  int x = Wire.read();    // receive byte as an integer
//  Serial.println(x);         // print the integer
}

// function that executes when data is requested from the master
void requestEvent(int howMany) {    
  int idx = Wire.read();        // receive byte as an integer
//  Serial.println(idx);          // print the integer
  
  void* coordinate_bytes;
  
  switch(idx) {
    case 0:
      coordinate_bytes = &coordinate_bytes_0;
      Wire.write((const char*)coordinate_bytes, COORDINATE_LEN);
      break;
    
    case 1:
      coordinate_bytes = &coordinate_bytes_1;
      Wire.write((const char*)coordinate_bytes, COORDINATE_LEN);
      break;

    default:
      break;
  }
}

// converts the data stored in a variable to raw binary goodness
void convert_to_bytes(void* input_var, void* output_array, unsigned int array_size)
{
  // disable interrupts so the data won't be corrupted by an I2C interrupt changing the variable
  noInterrupts();
  for(int i=0; i<array_size; i++)
  {
    // get the address of the variable 
    // increment it by i
    // cast it to an unsigned char pointer 
    // dereference it to get the byte value
    unsigned char single_byte = *((unsigned char*)input_var + i);
    unsigned char* output_array_char = (unsigned char*)output_array;
    *(output_array_char + i) = single_byte;
//    Serial.println(single_byte);
  }  
  interrupts();
}
