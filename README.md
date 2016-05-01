# EC-PCB
Arduino interface for communication with the Entertainment &amp; Comfort serial bus found in some GM vehicles.

The PCB subdirectory contains EAGLE files for an interface to be used with an Arduino Pro Mini.

The ECcomm_adapter subdirectory contains the sketch to upload to the Arduino. This code could be used with other Arduino variants.

The ECcomm_adapter Arduino sketch can communicate with a serial terminal application as well as the co-developed ECcomm program for Linux (https://github.com/svschmitt/ECcomm/ ), which provides additional functionality included message decoding and bidirectional communication.
