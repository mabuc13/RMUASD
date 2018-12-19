/**
  Sketch used for Decawave Ranging

  THIS SCRIPT IS USED FOR ANCHORS AND TAGS

  FIND THE ACCORDINGLY COMMENTED AREA WHERE THINGS NEED TO BE CHANGED WHEN SWITCHING BETWEEN ACHORS AND TAGS
  
*/
#include <SPI.h>
#include <math.h>
#include <Wire.h>
#include "DW1000Ranging.h"

// connection pins
const uint8_t PIN_RST = 9; // reset pin
const uint8_t PIN_IRQ = 2; // irq pin
const uint8_t PIN_SS = SS; // spi select pin

struct DWAnchor {
  uint16_t address;
  float range;
};

#define NUM_OF_ANCHORS 4
DWAnchor anchors[NUM_OF_ANCHORS];// = {{0,0},{0,0},{0,0},{0,0}};
byte numOfActiveAnchors = 0;

volatile float xyz_position[3] = {0.0, 0.0, 0.0};
#define DISTANCE_BETWEEN_ANCHORS 3.5//0.33//3.52
const float third_anchor_x = DISTANCE_BETWEEN_ANCHORS / 2.0;
const float third_anchor_y = DISTANCE_BETWEEN_ANCHORS * sqrt(3) / 2.0;

int counter = 0;

// Statisics Counter and measurements holder
int currentMeasurement = 0;
#define MEASUREMENTS_PER_TURN 60
float xMeasurements[MEASUREMENTS_PER_TURN];
float yMeasurements[MEASUREMENTS_PER_TURN];
float zMeasurements[MEASUREMENTS_PER_TURN];

bool sendNAN = false;


bool isDebug = true; // if this is true it will print serial data



       


// I2C stuff
#define MY_ADDRESS 13

#define COORDINATE_LEN 12

float notEnoughAnchors[3] = {NAN, NAN, NAN};
unsigned char coordinate_bytes_0[COORDINATE_LEN];
unsigned char notEnoughAnchors_bytes_0[COORDINATE_LEN];


void setup() {
  Serial.begin(9600);
  delay(100);
  //init the configuration
  DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ); //Reset, CS, IRQ pin
  //define the sketch as anchor. It will be great to dynamically change the type of module
  DW1000Ranging.attachNewRange(newRange);
  DW1000Ranging.attachNewDevice(newDevice);
  DW1000Ranging.attachInactiveDevice(inactiveDevice);
  //Enable the filter to smooth the distance
  DW1000Ranging.useRangeFilter(false);

  /*
      TAG/ANCHOR SWITCHING AREA START:

        - make sure to either DW1000Ranging.startAsTag or DW1000Ranging.startAsAnchor
        - the address for 
            Anchor 1 --> 00:01:22:EA:82:60:3B:9C
            Anchor 2 --> 00:02:22:EA:82:60:3B:9C 
            Anchor 3 --> 00:03:22:EA:82:60:3B:9C
            (Anchor 4) --> 00:04:22:EA:82:60:3B:9C

            Tag 1 (currently used on the woodstick)        --> ff:fe:22:EA:82:60:3B:9C
            Tag 2 (currently used on the drone 27.11.2018) --> ff:fd:22:EA:82:60:3B:9C

      
        - switch the bool variable according to your device
        - also make sure to switch the _antennaDelay number in the DW1000.cpp script!
          - Anchor: 0
          - Tag 1: 32850
          - Tag 2: 32925
   */

  //DW1000Ranging.startAsAnchor("00:03:22:EA:82:60:3B:9C", DW1000.MODE_LONGDATA_RANGE_ACCURACY, false);
  DW1000Ranging.startAsTag("ff:fe:22:EA:82:60:3B:9C", DW1000.MODE_LONGDATA_RANGE_ACCURACY, false);
  //DW1000Ranging.startAsAnchor("00:02:22:EA:82:60:3B:9C", DW1000.MODE_LONGDATA_RANGE_ACCURACY, false);
  
  bool isTag = true; 
  
  /*
   * TAG/ANCHOR SWITCHING AREA END:
   * 
   */

  

  if(isTag){
    Wire.begin(MY_ADDRESS);                // join i2c bus with address #8
    Wire.onReceive(receiveEvent); // register event
    Wire.onRequest(requestEvent);
  }
  /// convert_to_bytes(&notEnoughAnchors, &notEnoughAnchors_bytes_0, COORDINATE_LEN);
   

}


void loop() {
  DW1000Ranging.loop();

  //computePosition();
  //printData();
  
  counter++;
  if (counter > 2000) {
    //updateStatistics();
    //Serial.print("Position: x("); Serial.print(xyz_position[0]);
    //Serial.print("), \t y("); Serial.print(xyz_position[1]);
    //Serial.print("), \t z("); Serial.print(xyz_position[2]), Serial.println(")");

/*
    for(int i = 0; i < NUM_OF_ANCHORS; i++){
      Serial.print(anchors[i].address);
      Serial.print(" ");
      Serial.println();
    }
*/
    
    // Write out data for plotting 
   /*
    if(numOfActiveAnchors > 2){
      Serial.print(anchors[0].range); Serial.print(",");
      Serial.print(anchors[1].range); Serial.print(",");
      Serial.print(anchors[2].range); Serial.print(",");
      
      Serial.print(xyz_position[0]); Serial.print(",");
      Serial.print(xyz_position[1]); Serial.print(",");
      Serial.print(xyz_position[2]); Serial.println();

    }
    */
    counter = 0;
  }

}

void updateRange(uint16_t address, float range, bool newAnchor = false) {
  if (newAnchor) {
    DWAnchor a = {address, range};
    anchors[(address/256)-1] = a;
    numOfActiveAnchors++;
    // Serial.print("Successfully added: ");
    // Serial.println(address);
  }
  else {
    for (int i = 0 ; i < NUM_OF_ANCHORS ; i++) {
      if (address == anchors[i].address) {
        anchors[i].range = range;
        //Serial.print("Range update: "); Serial.println(anchors[i].range);
        break;
      }
    }
  }
}

void computePosition() {
  if (numOfActiveAnchors > 2) {
    float x = (sq(anchors[0].range) - sq(anchors[1].range) + sq(DISTANCE_BETWEEN_ANCHORS)) / (2 * DISTANCE_BETWEEN_ANCHORS);
    float y = (sq(anchors[0].range) - sq(anchors[2].range) + sq(third_anchor_x) + sq(third_anchor_y) ) / (2 * third_anchor_y) - (third_anchor_x / third_anchor_y) * x;
    float z = sqrt(sq(anchors[0].range) - sq(x) - sq(y));
    //Serial.print("If that is negative, spheares don't intersect: ");
    //Serial.println(sq(anchors[0].range) - sq(x) - sq(y));
    xyz_position[0] = x;
    xyz_position[1] = y;
    xyz_position[2] = z;


    int highValueFilterBoundary = 50;
    // If we should get values that are out of any logical reasoning, make sure to don't send them #filtering them out by sending NAN
    if(abs(x) > highValueFilterBoundary || abs(y) > highValueFilterBoundary || abs(z) > highValueFilterBoundary){
      sendNAN = true;
    }
    else{
      sendNAN = false;
    }

    //convert_to_bytes(&xyz_position, &coordinate_bytes_0, COORDINATE_LEN);
  }
  else {
    //Serial.println("Not enough active anchors to compute position...");
  }
}

void newRange() {
  if(isDebug){
    Serial.print(DW1000Ranging.getDistantDevice()->getShortAddress(), HEX);
    Serial.print(","); Serial.print(DW1000Ranging.getDistantDevice()->getRange());
    Serial.print(","); Serial.println(DW1000Ranging.getDistantDevice()->getRXPower());
    //Serial.print("from: "); Serial.print(DW1000Ranging.getDistantDevice()->getShortAddress(), HEX);
    //Serial.print("\t Range: "); Serial.print(DW1000Ranging.getDistantDevice()->getRange()); Serial.print(" m");
    //Serial.print("\t RX power: "); Serial.print(DW1000Ranging.getDistantDevice()->getRXPower()); Serial.println(" dBm");
  }
  uint16_t a = DW1000Ranging.getDistantDevice()->getShortAddress();
  float r = DW1000Ranging.getDistantDevice()->getRange();
  updateRange(a, r, false);
}


void newDevice(DW1000Device* device) {
  //Serial.print("ranging init; 1 device added ! -> short address: ");
  //Serial.println(device->getShortAddress());
  uint16_t a = device->getShortAddress();

  //byte* bsa = *device->getByteShortAddress();

  //byte* bsa_char = bsa+8;
  //Serial.print("bsa: ");
  //Serial.println(bsa_char);
  
  float r = device->getRange();
  //Serial.print("Trying to add address: "); Serial.print(a); Serial.print("\t range: "); Serial.println(r);
  updateRange(a, r, true);
}

void inactiveDevice(DW1000Device* device) {
  //Serial.print("Delete inactive device: ");
  //Serial.println(device->getShortAddress());
  for (int i = 0 ; i < NUM_OF_ANCHORS ; i++) {
    if (device->getShortAddress() == anchors[i].address) {
      anchors[i].address = 0;
      anchors[i].range = 0.0;
      //Serial.print("Successfully deleted ");
      //Serial.println(device->getShortAddress());
      numOfActiveAnchors--;
    }
  }
}


// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany) {
  // nothing here
  Serial.println("receiveEvent");
}


// function that executes when data is requested from the master
void requestEvent(int howMany) {    
  int idx = Wire.read();        // receive byte as an integer
//  Serial.println(idx);          // print the integer
  
  void* coordinate_bytes;
  
  switch(idx) {
    case 0:
      // Send the xyz coordinates if they are valid and freshly updated

      if(numOfActiveAnchors == 3 && !sendNAN){
        coordinate_bytes = &coordinate_bytes_0;
        Wire.write((const char*)coordinate_bytes, COORDINATE_LEN);
      }
      else{
        coordinate_bytes = &notEnoughAnchors_bytes_0;
        Wire.write((const char*)coordinate_bytes, COORDINATE_LEN);
      }
      break;
    
    case 1:
      // Not implemented
      
      //coordinate_bytes = &coordinate_bytes_1;
      //Wire.write((const char*)coordinate_bytes, COORDINATE_LEN);
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




void printData(){

    Serial.print("pos: ");
    Serial.print(xyz_position[0]);
    Serial.print(","); Serial.print(xyz_position[1]);
    Serial.print(","); Serial.println(xyz_position[2]);
  
}
