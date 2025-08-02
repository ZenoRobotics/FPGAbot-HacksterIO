#include <stdint.h>

void setup() {
  // Initialize the default Serial port for communication with the computer (Serial Monitor)
  Serial.begin(115200); 
  Serial.println("Serial Monitor Ready!");

  // Initialize Serial1 for communication with an external device
  // Replace 9600 with the desired baud rate for your device
  Serial1.begin(57600); //115200); 
  Serial.println("Serial1 Ready!");
}

byte incomingByte;
byte headerByte;
uint16_t encdr_val;
int byteCnt = 0;

void loop() {
  if (Serial1.available() > 0) { // Check if data is available
    incomingByte = Serial1.read(); // Read the incoming byte
    if ((incomingByte == 0xFD) || (incomingByte == 0xFE)) {
       headerByte = incomingByte;
       if (incomingByte == 0xFD)
          Serial.print("L: "); 
       else
          Serial.print("  R: "); 
       
       while (Serial1.available() == 0) {};
       incomingByte = Serial1.read(); // Read the incoming byte
       encdr_val = incomingByte << 8;
       byteCnt = 1;
       if ((headerByte == 0xFD) && (incomingByte == 0xFD)) {
         Serial.print("    ");
         return;
       }
       else if ((headerByte == 0xFE) && (incomingByte == 0xFE)) {
         Serial.println("");
         return;
       }
         
         
       //Serial.print(incomingByte,HEX); // Print the received byte
       
       while (Serial1.available() == 0) {};
       incomingByte = Serial1.read(); // Read the incoming byte
       encdr_val = encdr_val | incomingByte;
       byteCnt = 2;
       //Serial.print(incomingByte,HEX); // Print the received byte
       //Serial.print("   ");
       //Serial.print(encdr_val,HEX); // Print the received byte

       if (byteCnt == 2) {
         Serial.print(encdr_val,HEX); // Print the received byte
         if (headerByte == 0xFE)
           Serial.println(""); 
       }
       encdr_val = 0;
       incomingByte = 0;
       byteCnt = 0;
          
    }
  }
}
