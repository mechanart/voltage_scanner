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

/* STUFF TO CHANGE GOES HERE
 *  
 */
#define num_brd 2


byte rx = 0;

// globals to store state of the relays
byte POS_A = 0;
byte COM_A = 0;
byte POS_B = 0;
byte COM_B = 0;
word switch_state = 0; // words are 16-bits in arduino land (hopefully)
byte global_state[2*num_brd];
byte A_state[num_brd];
byte B_state[num_brd];

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

  Serial.println("Welcome to Voltage Scanner Firmware Debug!");
  
  initialize_relays(); // routine to ensure no relays are "on" 
}

byte disconnect_A(){
  for(int i = 0; i < 4 * num_brd; i ++){
    Serial.print("Disconnecting Cell "); Serial.print(i+1); Serial.println(" A output.");
    byte offset = 16 * (i/4); // 0 for cells 1-4 (i = 0->3), 16 for cells 5-8, etc.
    drive_relay(offset + ((i%4)*2 + 1)); // disconnect cell 1 from A we send 1, disconnect cell 6 from A we send 19
    
    delay(100);
    read_state(global_state);
  }
}


byte disconnect_cell(byte cell_num, char output){
    byte i = cell_num - 1;
    Serial.print("Disconnecting Cell "); Serial.print(i+1); Serial.print(" from output "); Serial.println(output);
    if(output == 'A'){
      byte offset = 16 * (i/4); // 0 for cells 1-4 (i = 0->3), 16 for cells 5-8, etc.
      drive_relay(offset + ((i%4)*2 + 1)); // disconnect cell 1 from A we send 1, disconnect cell 6 from A we send 19
    }
    if(output == 'B'){
       byte offset = 8 * (2*(i/4) + 1); // 8 if on board 1, 24 if on board 2, etc.
      drive_relay(offset + ((i%4)*2 + 1)); // disconnect cell 1 from B we send 9, disconnect cell 6(i=5) from B we send 24 + 2 + 1 = 27
    }
    
    delay(100);
    read_state(global_state);
}

byte disconnect_B(){
  for(int i = 0; i < 4 * num_brd; i ++){
    Serial.print("Disconnecting Cell "); Serial.print(i+1); Serial.println(" from B output.");
    byte offset = 8 * (2*(i/4) + 1); // 8 if on board 1, 24 if on board 2, etc.
    drive_relay(offset + ((i%4)*2 + 1)); // disconnect cell 1 from B we send 9, disconnect cell 6(i=5) from B we send 24 + 2 + 1 = 27
    
    delay(100);
    read_state(global_state);
  }
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
 disconnect_A();
 disconnect_B();
}

byte read_state(byte *curr_state){
	// read state of relays via 74HC165D
  //word switch_state = 0;
  //byte rdbk_byte[2*num_brd];
  
  mux_165();
  digitalWrite(AUX, HIGH);
  delay(10);
  digitalWrite(AUX, LOW);
  delay(10); // delay to allow propogation of parallel load pulse to 165
  digitalWrite(AUX, HIGH);
  
  // parallel data is latched - let's shift it out 
  // first byte is from U4 - MSB is input H 
  //byte rdbk_byte_1 = SPI.transfer(0x00);
  //byte rdbk_byte_0 = SPI.transfer(0x00);
  for(int i = 0; i < 2*num_brd; i++){
    //rdbk_byte[i] = SPI.transfer(0x00);
    curr_state[i] = SPI.transfer(0x00);
    if(i%2 == 0){ // even meaning B state since U4 is tied to MISO
      Serial.print("Readback of B state from board "); Serial.print(i/2 + 1); Serial.print(": "); printByte(curr_state[i]); Serial.println();  
    }
    else{
      Serial.print("Readback of A state from board "); Serial.print(i/2 + 1); Serial.print(": "); printByte(curr_state[i]); Serial.println();
    }
  }
 
  return(0);
}

byte read_state(byte *curr_state, byte * A_state, byte * B_state){
  // read state of relays via 74HC165D  
  mux_165();
  digitalWrite(AUX, HIGH);
  delay(10);
  digitalWrite(AUX, LOW);
  delay(10); // delay to allow propogation of parallel load pulse to 165
  digitalWrite(AUX, HIGH);
  
  // parallel data is latched - let's shift it out 
  // first byte is from U4 aka Board 1 Output B - MSB is input H 
  for(int i = 0; i < 2*num_brd; i++){
    //rdbk_byte[i] = SPI.transfer(0x00);
    curr_state[i] = SPI.transfer(0x00);
    if(i%2 == 0){ // even meaning B state since U4 is tied to MISO
      Serial.print("Readback of B state from board "); Serial.print(i/2 + 1); Serial.print(": "); printByte(curr_state[i]); Serial.println();  
      // set B state appropriately given first byte is from 
      B_state[num_brd - i] = curr_state[i]; // 
    }
    else{
      Serial.print("Readback of A state from board "); Serial.print(i/2 + 1); Serial.print(": "); printByte(curr_state[i]); Serial.println();
    }
  }
 
  return(0);
}

void drive_relay(byte code){
	// send the appropriate bytes to tell MICREL driver to turn ON output at #output 
	// likewise, OUT2 of U1 is in the high byte, 
	// uses the concatenated 0-indexed bit field value for OUT, i.e. OUT6 of U2 is in the low byte, at 1 << 2
  Serial.print("Received instruction to drive relay #"); Serial.println(code); 
  mux_MIC();
  digitalWrite(AUX, LOW); // strobe
  digitalWrite(OEn, HIGH); // hopefully output are already off/disabled

  // create the output code array - 2 bytes per board
  byte output_code[2*num_brd];
  memset(output_code, 0x0, sizeof(output_code)); // all zeroes
  output_code[code/8] ^= 1 << code%8; 
  for(int i = sizeof(output_code); i > 0; i--){
    Serial.print(output_code[i-1], BIN);
    SPI.transfer(output_code[i-1]);
  }
  Serial.println();
  
  digitalWrite(AUX, HIGH);
  delay(5);
  digitalWrite(AUX, LOW); // serial to parallel latch to output buffers
  delay(5);
  
  digitalWrite(OEn, LOW);  // enable the output buffers - let's kick it off!
  delay(7); // how much delay do we need here?
  // 3 was too short, sometimes fail
  // 10 worked so far
  // 7 worked so far - keeping 7 for now, will try to test for EMF soon
  digitalWrite(OEn, HIGH); // disable output buffers. Relay should have switched by now... 
}

bool already_connected(char output){
  if(check_connected(output) > 0){
    return(true); 
  }
  else {
    return(false);
  }
}

byte check_connected(char output){
  bool already_connected = false;
  byte curr_state[2*num_brd];
  byte expected_state[2*num_brd];
  memset(curr_state, 0x00, sizeof(curr_state)); 
  memset(expected_state, 0x00, sizeof(expected_state));
  byte output_state[num_brd];
  memset(output_state, 0xFF, sizeof(output_state));
  
  // will attempt to connect the argument cell to output A
  // first checks and disconnects any previously connected cell from output A and COM A
  read_state(curr_state); // read_state now takes a pointer to an array

  if(output == 'A'){
    Serial.print("Output A ");
    for(int i = 0; i < sizeof(curr_state); i++){
      if(i%2 != 0){
        output_state[i/2] = curr_state[i];
      }
    }
  }
  else{
    Serial.print("Output B ");
    for(int i = 0; i < sizeof(curr_state); i++){
      if(i%2 == 0){
        output_state[i/2] = curr_state[i];
      }
    }
  }
  
  Serial.print("output_state: ");
  for(int i = 0; i < sizeof(output_state); i++){
    printByte(output_state[i]);
  }
  Serial.print("    ");

  // Initialize connected cell to zero as default showing not connected
  byte connected_cell = 0; 

  Serial.print("~output_state: ");
  for(int i = 0; i < sizeof(output_state); i++){
    printByte(~output_state[i]);
    //Serial.println();
    if(((~output_state[i]) & 0xFF) != 0x00){
      already_connected = true;
      //Serial.println(i);
    }
    if(already_connected){
      for(int j = 0; j < 4; j++){
        if(~output_state[i] & (0x03 << (j * 2))){
          Serial.print(i); Serial.print(" ");
          Serial.println(j);
          connected_cell = j + 1 + i * 4; // 4 cells per board
        } 
      }
      if(connected_cell <= num_brd * 4){
        Serial.println(); Serial.print("Cell "); Serial.print(connected_cell); Serial.print(" connected to output "); Serial.println(output);
      }
    }
   }

  Serial.println();

  return(connected_cell);
}

byte connect_cell(byte cell_num, char output){
  if(already_connected(output)){
    Serial.print("Uh-oh, a cell is already connected to output "); Serial.println(output);
    disconnect_cell(check_connected(output), output);
    
    if(already_connected(output)){
      Serial.print("Exiting - failed to clear output "); Serial.println(output);
      return -1;
    }
  }
  // either nothing was connected or clearing worked so let's go ahead and connect the new cell
  byte i = cell_num - 1;
  byte offset = 0;
  byte code = 0;
  Serial.print("Connecting Cell "); Serial.print(i+1); Serial.print(" to output "); Serial.println(output);
  // only this section is different for A/B output

  if(output == 'A'){
    offset = 16 * (i/4); // 0 for cells 1-4 (i = 0->3), 16 for cells 5-8, etc.
    code = offset + (i%4)*2;
  }
  if(output == 'B'){
    offset = 8 * (2*(i/4) + 1); // 8 if on board 1, 24 if on board 2, etc.
    code = offset + (i%4)*2;
  }

  // common code
  Serial.print("Sending code: "); Serial.print(code); Serial.println(" to drive_relay");
  drive_relay(code); // connect cell 1 to B we send 8, connect cell 6(i=5) to B we send 24 + 2 = 26 
  
  if(check_connected(output) == cell_num){
    Serial.println("Success!");
    return 0;
  }
  else {
    Serial.println("Failed to connect cell...");
    return -1;
  }
  //should never get here
  return -1;
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
    char output = rx_buf[3];
    // we're asked to connect a cell
    char cell_num_str[3];
    memset(cell_num_str, '\0', sizeof(cell_num_str));
    strncpy(cell_num_str, rx_buf+1, 2);
    byte cell_num = atoi(cell_num_str);
    if((cell_num < 1) || (cell_num > (4 * num_brd))){
      Serial.println("Error bad cell number in connect request.");
    }
    else{
      Serial.print("Attempting to connect cell "); Serial.print(cell_num); Serial.print(" to output "); Serial.println(output);
      connect_cell(cell_num, rx_buf[3]);
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
    // we're asked to disconnect a cell
    else{
      char output = rx_buf[3];
      // we're asked to connect a cell
      char cell_num_str[3];
      memset(cell_num_str, '\0', sizeof(cell_num_str));
      strncpy(cell_num_str, rx_buf+1, 2);
      byte cell_num = atoi(cell_num_str);
      if((cell_num < 1) || (cell_num > (4 * num_brd))){
        Serial.println("Error bad cell number in disconnect request.");
      }
      else{
        Serial.print("Attempting to disconnect cell "); Serial.print(cell_num); Serial.print(" to output "); Serial.println(output);
        disconnect_cell(cell_num, rx_buf[3]);
      }
    }  
  }  

  if (rx_buf[0] == 'R'){
    Serial.println("Got a readback command"); // "R" for readback - get it? Right now just runs check_connected('OUTPUT')
    Serial.println(rx_buf);
    check_connected(rx_buf[1]);
  }
  // clear rx_buf

}
