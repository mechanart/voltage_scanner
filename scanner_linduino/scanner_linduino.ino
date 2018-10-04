/*
 Voltage Scanner Control Firmware

 This program provides a basic serial interface 

 This is written for an unmodified Linduino from Linear Technology. 
 The Linduino provides an isolated USB interface.

   The scanner:
 * Shall not connect multiple cells to a single output
 * Shall readback state of relays before and after switching 
 * Shall wait between multiple switch comamands

 Created 30 Oct. 2017
 by Logan Cummings
 modified XX November 2017
 by Logan Cummings

 This example code is in the public domain.

 http://www.mechanart.com/scanner

 */

#include <Arduino.h>
#include <stdint.h>
#include <SPI.h>
#include "Linduino.h"
#include "LT_SPI.h"

#include "scanner_board.h"

byte rx = 0;

// globals to store state of the relays
byte POS_A = 0;
byte COM_A = 0;
byte POS_B = 0;
byte COM_B = 0;
word switch_state = 0; // words are 16-bits in arduino land (hopefully)


/* STUFF TO CHANGE GOES HERE
 *  
 */
#define num_brd 1


void setup() {
  
  // configure pin modes
  pinMode(AUX, OUTPUT);
  pinMode(SS, OUTPUT);
  pinMode(IO_SEL, OUTPUT); 
  
  digitalWrite(OEn, HIGH);
  digitalWrite(AUX, HIGH);
  
  // set default mux state to talk to '165 and do nothing
  mux_165();

  // initialize SPI interface
  quikeval_SPI_init();
  quikeval_SPI_connect();
  // start serial port at 115200
  Serial.begin(115200);

  
  
  //print_welcome(); // print introduction
  Serial.println("Welcome to Voltage Scanner Firmware Debug!");
  //read_state();
  //drive_relay(
  //initialize_relays(); // routine to ensure no relays are "on" 
  //all_off();
  read_state();
  delay(100);
  //Serial.print("Driving "); Serial.println(i);
/*
  for(int i = 0; i < 4; i ++){
    Serial.print("Connecting Cell "); Serial.print(i + 1); Serial.println(" A output.");
    drive_relay(i * 2);
    delay(100);
    read_state();
  }

  for(int i = 0; i < 4; i ++){
    Serial.print("Connecting Cell "); Serial.print(i+1); Serial.println(" B output.");
    drive_relay(8 + (i * 2));
    delay(100);
    read_state();
  }

   for(int i = 0; i < 4; i ++){
    Serial.print("Disconnecting Cell "); Serial.print(i+1); Serial.println(" A output.");
    drive_relay(i*2 + 1);
    delay(100);
    read_state();
  }
  
  for(int i = 0; i < 4; i ++){
    Serial.print("Disconnecting Cell "); Serial.print(i+1); Serial.println(" B output.");
    drive_relay(8 + (i*2 + 1));
    delay(100);
    read_state();
  }


  Serial.println("Attempting to connect cell 3 to A");
  connect_A(3);
  read_state();
  Serial.println("Attempting to connect cell 2 also to A");
  connect_A(2);
  read_state();
  Serial.println("Disconnecting all from A");
  disconnect_A();
  read_state();
*/    
}

void printByte(byte myByte){
	for(byte mask = 0x80; mask; mask >>= 1) {
		if(mask & myByte) {
			Serial.print("1");
		}
		else {
			Serial.print("0");
		}
	}
}

void printWord(word myByte){
	for(word mask = 0x8000; mask; mask >>= 1) {
		if(mask & myByte) {
			Serial.print("1");
		}
		else {
			Serial.print("0");
		}
	}
}


byte initialize_relays() {
  word  curr_state = read_state();
  
  // disconnect any connected cells
	if(curr_state != 0){
		int i = 0;
		int relay_code = 0;
		for (word mask = 0x8000; mask; mask >>= 1) {
			if(mask & curr_state) {
				//Serial.print("Switch "); Serial.print(switch_name[i]); Serial.print(" was closed. Opening/resetting...\n");
				// since both POS and COM for a given cell/output combo are on a single relay, let's drive reset for that  one
				// example: COM_B_C3 is set (11th bit in switch_state - U4 "F" , i == 10 in this loop) 
				// so we need to drive the RESET coil on the B_CELL3 = OUT6 of U2, first byte of drive word bit  clocked to MICREL
				relay_code = (i >> 1) << 1;
				Serial.print("Sending request to fire relay coil on effective OUT"); Serial.println(relay_code); 
				//drive_relay(relay_code);
			}
			i++;
		}
	}
  // read state again to verify nothing is "on"
//TODO
  // return 0 if successfully initialized, anything else if not (perhaps cell stuck on?)
  //TODO
}

word read_state(){
	// read state of relays via 74HC165D
  word switch_state = 0;
  mux_165();
  digitalWrite(AUX, HIGH);
  delay(10);
  digitalWrite(AUX, LOW);
  delay(10); // delay to allow propogation of parallel load pulse to 165
  digitalWrite(AUX, HIGH);
  
  // parallel data is latched - let's shift it out 
  // first byte is from U4 - MSB is input H 
  byte rdbk_byte_1 = SPI.transfer(0x00);
  byte rdbk_byte_0 = SPI.transfer(0x00);
  switch_state = (rdbk_byte_1 << 8) ^ rdbk_byte_0;
  Serial.print("Readback of U4 (B): "); printByte(rdbk_byte_1); Serial.println();
  
  Serial.print("Readback of U3 (A): "); printByte(rdbk_byte_0); Serial.println();
  
  Serial.print("Concatenated switch state B_A: "); printWord(switch_state); Serial.println();
  
  return(switch_state);
}

void drive_relay(byte code){
	// send the appropriate bytes to tell MICREL driver to turn ON output at #output 
	// likewise, OUT2 of U1 is in the high byte, 
	// uses the concatenated 0-indexed bit field value for OUT, i.e. OUT6 of U2 is in the low byte, at 1 << 2
  mux_MIC();
  digitalWrite(AUX, LOW); // strobe
  digitalWrite(OEn, HIGH); // hopefully output are already off/disabled
  if (code > 7){
    SPI.transfer(1 << (code - 8));
    SPI.transfer(0x00);
  }
  else {
    SPI.transfer(0x00);
    SPI.transfer(1 << code);
  }
  
  digitalWrite(AUX, HIGH);
  delay(2);
  digitalWrite(AUX, LOW); // serial to parallel latch to output buffers
  delay(2);
  
  digitalWrite(OEn, LOW);  // enable the output buffers - let's kick it off!
  delay(15);
  digitalWrite(OEn, HIGH); // disable output buffers. Relay should have switched by now... 
}

void all_off(){
  mux_MIC();
  digitalWrite(AUX, LOW);
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  digitalWrite(AUX, HIGH); // pulse strobe to latch data
  delay(2);
  digitalWrite(AUX, LOW);
  digitalWrite(OEn, HIGH); // disable output drivers anyway
}

byte connect_A(byte cell_num) {
  word curr_state = 0x0000;
  byte A_state = 0xFF;
  word expected_state = 0x00;
  // will attempt to connect the argument cell to output A
  // first checks and disconnects any previously connected cell from output A and COM A
  curr_state = read_state();
  A_state = curr_state & 0xFF;
  Serial.print("A_state: ");printByte(A_state);Serial.println();
  Serial.print("~A_state: "); printByte(~A_state);Serial.println();
  if((~A_state & 0xFF) != 0x00){
    Serial.println("Uh-oh, a cell is already connected to output A");
    disconnect_A();
    curr_state = read_state();
    A_state = curr_state & 0xFF;
    if((~A_state & 0xFF) != 0){
      Serial.println("Failed to clear A - exiting.");
      return -1;
    }
    else {
      //clearing the state worked, let's go ahead and connect the cell
      drive_relay((cell_num - 1)*2);
    }
  }
  else {
    drive_relay((cell_num - 1) * 2);
  }
  curr_state = read_state();
  // return 0 if successful, anything else is a fail (perhaps we return the cell stuck connected?)
  A_state = curr_state & 0xFF;
  expected_state = (0x03 << ((cell_num-1)*2)); 
  Serial.print("Expected state: "); printByte(expected_state); Serial.println();
  if((~A_state & 0xFF) != expected_state){
    Serial.println("Failed to connect cell");
    return -1;
  }
  else {
    Serial.println("Success!");
    return 0;
  }
}

byte disconnect_A(){
  for(int i = 0; i < 4; i ++){
    Serial.print("Disconnecting Cell "); Serial.print(i+1); Serial.println(" A output.");
    drive_relay(i*2 + 1);
    delay(100);
    read_state();
  }
}

byte connect_B(byte cell_num) {
  word curr_state = 0x0000;
  byte B_state = 0xFF;
  word expected_state = 0x00;
  // will attempt to connect the argument cell to output B
  // first checks and disconnects any previously connected cell from output B and COM B
  curr_state = read_state();
  B_state = (curr_state >> 8) & 0xFF;
  Serial.print("B_state: ");printByte(B_state);Serial.println();
  Serial.print("~B_state: "); printByte(~B_state);Serial.println();
  if((~B_state & 0xFF) != 0x00){
    Serial.println("Uh-oh, a cell is already connected to output B");
    disconnect_B();
    curr_state = read_state();
    B_state = (curr_state >> 8) & 0xFF;
    if((~B_state & 0xFF) != 0){
      Serial.println("Failed to clear B - exiting.");
      return -1;
    }
    else {
      //clearing the state worked, let's go ahead and connect the cell
      drive_relay(8 + (cell_num - 1)*2);
    }
  }
  else {
    drive_relay(8 + (cell_num - 1) * 2);
  }
  curr_state = read_state();
  // return 0 if successful, anything else is a fail (perhaps we return the cell stuck connected?)
  B_state = (curr_state >> 8) & 0xFF;
  expected_state = (0x03 << ((cell_num-1)*2)); 
  Serial.print("Expected state: "); printByte(expected_state); Serial.println();
  if((~B_state & 0xFF) != expected_state){
    Serial.println("Failed to connect cell");
    return -1;
  }
  else {
    Serial.println("Success!");
    return 0;
  }
}

byte disconnect_B(){
  for(int i = 0; i < 4; i ++){
    Serial.print("Disconnecting Cell "); Serial.print(i+1); Serial.println(" B output.");
    drive_relay(i*2 + 9);
    delay(100);
    read_state();
  }
}

void loop() {
  char rx_buf[5];
  memset(rx_buf, '\0', sizeof(rx_buf));
  byte index = 0;
  // put your main code here, to run repeatedly:

  // check if command is available on serial port
  // error check command - reply to serial port if needed
  // run command
  // read from serial port
  while(Serial.available()){
      if (Serial.available() > 0) {
        delay(3);
        char rx = Serial.read(); 
        //debug
        Serial.println(rx);       
        if(rx == '\n'){
          Serial.println("Got a command");
           break;
        }
        else{
          rx_buf[index] = rx;
          Serial.println(rx_buf);
        }
        //Serial.println(index);
        index++;
      }
  }
  
  if (rx_buf[0] == 'C'){
    Serial.println("Got a connect command");
    Serial.println(rx_buf);
    // we're asked to connect a cell
    if (rx_buf[3] == 'A'){
      // to output A
      // get cell num
      char cell_num_str[3];
      memset(cell_num_str, '\0', sizeof(cell_num_str));
      strncpy(cell_num_str, rx_buf+1, 2);
      byte cell_num = atoi(cell_num_str);
      if((cell_num < 1) || (cell_num > (4 * num_brd))){
        Serial.println("Error bad cell number in connect request.");
      }
      else{
        Serial.print("Attempting to connect cell "); Serial.print(cell_num); Serial.println(" to output A.");
        connect_A(cell_num);
      }
    }
    if (rx_buf[3] == 'B'){
      // to output B
      // get cell num
      char cell_num_str[3];
      memset(cell_num_str, '\0', sizeof(cell_num_str));
      strncpy(cell_num_str, rx_buf+1, 2);
      byte cell_num = atoi(cell_num_str);
      if((cell_num < 1) || (cell_num > (8 * num_brd))){
        Serial.println("Error bad cell number in connect request.");
      }
      else{
        Serial.print("Attempting to connect cell "); Serial.print(cell_num); Serial.println(" to output B.");
        connect_B(cell_num);
      }
    }
  }  
  if (rx_buf[0] == 'D'){
    Serial.println("Got a disconnect command");
    Serial.println(rx_buf);
    // we're asked to discconnect all cells
    if (rx_buf[1] == 'A'){
      // from output A
      disconnect_A();
    }
    if (rx_buf[1] == 'B'){
      // from output B
      disconnect_B();
    }
  }  
  // clear rx_buf
  memset(rx_buf, '\0', sizeof(rx_buf));
}
