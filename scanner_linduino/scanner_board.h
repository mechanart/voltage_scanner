/*
Some stuff describing this file.

Defines for use with scanner_linduino.ino or any other scanner_board firmware.

@author Logan Cummings
@company Mechanart

http://www.mechanart.com/scanner 

31 Oct. 2017

*/

#ifndef SCANNER_BOARD_H
#define SCANNER_BOARD_H

#define IO_SEL A4
#define OEn SS
#define AUX 9

#define mux_165() digitalWrite(IO_SEL, HIGH)
#define mux_MIC() digitalWrite(IO_SEL, LOW)


// names as we shift out data from '165 - see schematic
// '165 shifts H first, to A, then next part. In this case 
/* we shift from U4 first, so we get the B's then the A's
const char *switch_name[16]; // change me to be a multiple of num_brd
switch_name[0] = "CELL 4 COM B"; // U4 H 
switch_name[1] = "CELL 4 POS B"; // U4 G
switch_name[2] = "CELL 3 COM B"; // U4 F 
switch_name[3] = "CELL 3 POS B"; // U4 E
switch_name[4] = "CELL 2 COM B"; // U4 D 
switch_name[5] = "CELL 2 POS B"; // U4 C 
switch_name[6] = "CELL 1 COM B"; // U4 B
switch_name[7] = "CELL 1 POS B"; // U4 A 
switch_name[8] = "CELL 4 COM A"; // U3 H 
switch_name[9] = "CELL 4 POS A"; // U3 G
switch_name[10] = "CELL 3 COM A"; // U3 F 
switch_name[11] = "CELL 3 POS A"; // U3 E
switch_name[12] = "CELL 2 COM A"; // U3 D 
switch_name[13] = "CELL 2 POS A"; // U3 C 
switch_name[14] = "CELL 1 COM A"; // U3 B
switch_name[15] = "CELL 1 POS A"; // U3 A 
*/

#endif
