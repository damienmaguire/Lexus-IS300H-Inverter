# Lexus-IS300H-Inverter
Open Source Logic board to run the Lexus IS300 Hybrid Inverter for EV Conversion projects


12/03/19 : Initial release of design files for V1 board. This board is intended for design and development only.
Uses the Heubner inverter brain board available from the openinverter.org webshop : 
https://openinverter.org/shop/

Files in DesignSpark PCB 8.1 Format
https://www.rs-online.com/designspark/pcb-software

Update : 18/08/22
Added Sync serial logs and beta tool for organising toyota serial logs.

23/08/22 : Added basic code for Lexus VCU to run GS300h system.

ATTENTION : THE IS300H USES A 140 BYTE MYH PACKAGE. THE ARDUINO LIBRARY LIMITS THE RINGBUFFER SIZE TO 128 BYTES.
YOU MUST MODIFY THIS TO 150 IN ringbuffer.h TO RUN THIS CODE!!!
https://github.com/arduino/ArduinoCore-sam/blob/790ff2c852bf159787a9966bddee4d9f55352d15/cores/arduino/RingBuffer.h#L28
 
