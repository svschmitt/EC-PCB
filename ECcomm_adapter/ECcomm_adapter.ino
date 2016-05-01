/* ECcomm_adapter - Microcontroller interface for General Motors
            Entertainment & Comfort serial bus communication.

    Copyright (C) 2016 Stuart V. Schmitt

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    
    
    
    This code assumes a 16 MHz AVR with the E&C bus connected through
    inverting transistors to digital pins 3 and 5. Direct port registers
    are used because the overhead associated with Arduino functions
    digitalRead and digitalWrite result in too many lost or damaged
    E&C packets. For more information about the bus or the electrical
    interface, please look at information at
        http://stuartschmitt.com/e_and_c_bus/                      */
        

#include "packet.h"

const byte reg_in = 4;      // Pin 3 register
const byte reg_out = 16;    // Pin 5 register
byte mode = 0;              // 0=ASCII, 1=binary; add 2 if an E&C packet is
                            // being sent from the computer (ESC + 4 bytes)
unsigned long timer = 0;    // Timestamp of last E&C packet received/sent


void setup() {
  Serial.begin(9600);       // No need for faster serial; this is 9.6x E&C
  DDRD |= reg_out;
  DDRD &= ~reg_in;
  PORTD &= ~reg_in;
}


void loop() {
  if (PIND & reg_in) {      // If the bus is active, start a read.
    Packet msg(reg_in, reg_out);
    msg.Read();
    if (msg.Sent()) {       // Here, Sent() filters out some false triggers
      if (mode % 2)
        WriteBinary(msg.Unique(), millis() - timer, msg.Incomplete());
      else
        WriteText(msg.Unique(), millis() - timer, msg.Incomplete());
      timer = millis();
    }
  }
  if (Serial.available() > 0) {
    if (mode > 1) {         // An E&C packet is being sent from the computer
      if (Serial.available() >= 4) { // Don't read until it's fully in buffer
        byte header = Serial.read();
        byte byte1 = Serial.read();
        byte byte2 = Serial.read();
        byte byte3 = Serial.read();
        Packet out(reg_in, reg_out);
        out.Send(header & 3, header >> 2, byte1, byte2, byte3);
        mode -= 2;          // Done reading packet from computer
        if (mode)
          WriteBinary(out.Unique(), millis() - timer, 0);
        else
          WriteText(out.Unique(), millis() - timer, 0);
        timer = millis();
      }
    }
    else {                  // Not in packet read mode
      byte input = Serial.read();
      if (input == 32) {    // Set ASCII display mode
        mode = 0;
        Serial.println("Interface in ASCII mode.");
      }
      else if (input == 0 || input == 2) { // Set binary display mode
        mode = 1;
        Serial.println("Interface in binary mode.");
      }
      else if (input == 27) // Enable packet read mode
        mode += 2;
      // Ignore any other characters!
    }
  }
}


void WriteBinary(unsigned long packet, unsigned long time_gap, byte repaired) {
  // Send an encoded 12-byte sequence to the computer that contains the time
  // since the last E&C packet in milliseconds, the flag for incomplete E&C
  // reception, and the E&C packet contents. The start byte is an ESC, and the
  // subsequent bytes are limited to ASCII 32-95 in order to keep the tty sane
  // regardless of the terminal program. The bytes are packed in a method
  // based on UUencode, which allows six usable bits per byte.
  //   bits 1-32:   time gap between E&C packets
  //   bit 33:      incomplete E&C reception flag
  //   bit 34:      zero
  //   bits 35-66:  the E&C packet
  Serial.write((byte) 27);
  Serial.write(32 + (time_gap & 0x3F));
  Serial.write(32 + ((time_gap & 0xFC0) >> 6));
  Serial.write(32 + ((time_gap & 0x3F000) >> 12));
  Serial.write(32 + ((time_gap & 0xFC0000) >> 18));
  Serial.write(32 + ((time_gap & 0x3F000000) >> 24));
  Serial.write(32 + (((time_gap & 0xC0000000) >> 30) + ((repaired & 3) << 2) + ((packet & 3) << 4)));
  Serial.write(32 + ((packet & 0xFC) >> 2));
  Serial.write(32 + ((packet & 0x3F00) >> 8));
  Serial.write(32 + ((packet & 0xFC000) >> 14));
  Serial.write(32 + ((packet & 0x3F00000) >> 20));
  Serial.write(32 + ((packet & 0xFC000000) >> 26));
}


void WriteText(unsigned long packet, unsigned long time_gap, byte repaired) {
  // Simply print the E&C packet to the serial port. This is convenient when
  // using a simple terminal program to connect to the interface. First, print
  // the time since the last E&C packet in milliseconds. Then print a ":"
  // unless the packet was received incomplete; then print a "#". Then print
  // the E&C packet in PRIORITY-ADDRESS-BYTE1-[BYTE2]-[BYTE3] format.
  Serial.print(time_gap);
  if (repaired)
    Serial.print('#');
  else
    Serial.print(':');
  Serial.print(' ');
  Serial.print(packet & 3);
  Serial.print('-');
  Serial.print((packet & 252) >> 2);
  Serial.print('-');
  Serial.print((packet & 0xFF00) >> 8);
  if (packet > 0xFFFF) {
    Serial.print('-');
    Serial.print((packet & 0xFF0000) >> 16);
  }
  if (packet > 0xFFFFFF) {
    Serial.print('-');
    Serial.print((packet & 0xFF000000) >> 24);
  }
  Serial.println("");
}
