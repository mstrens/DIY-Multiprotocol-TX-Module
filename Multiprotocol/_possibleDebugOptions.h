#pragma once
/*



// debug options
// for Milo with ESP8266 (not tested with other MCU's)
//-----------
#define DEBUG_ESP8266  // must be defined before Multiprotocol.h in order to define debugln() etc...
//#define DEBUG_ESP32

#define SIM_HANDSET_DATA  // ESP8266 has only one UART. If UART Tx it is used for debuging, then UART Rx can't be used for reading handet;
                          // so here we force milo tx to use some dummy data (channels, failsafe, uplink)

#define DEBUG_SEQUENCE  // print info about up and down link sequence each time a message is sent or received     
#define DEBUG_DOWNLINK  // print info about downlink (when this frame is received)
#define DEBUG_AVOID_MULTI_STATUS  // print "MP status" instead of sending a multi_status to handset (otherwise Serial can't be read in IDE terminal)

//#define DEBUG_FHSS
//#define DEBUG_WITH_FIXED_FHSS

//#define DEBUG_UNUSED_TX_SLOTS // when activated, it is possible to skip some consecutive channels (e.g. to test synchro TX-RX)
//#define DEBUG_SKIP_TX_FROM_CHANNEL 10 // lower index 
//#define DEBUG_SKIP_TX_UPTO_CHANNEL 20 // upper index

#define DEBUG_ON_GPIO3      // allow to generate pulses on pin 3 (normally sport pin) of ESP8266 
#define DEBUG_PACKET_COUNT  // generates 1, 2 or 3 pulses depending on the value of packet_count ; DEBUG_ON_GPIO3 must be activated

#define DEBUG_AVOID_SENDING_TO_SPORT     // avoid sending data on sport port (to use when we send pulses on this port to debug with DEBUG_ON_GPIO3)            

//#define DEBUG_WIFI


// for Milo and others
//--------------------


// for others only 
//-----------------
//#define DEBUG_PIN     // Use pin TX for AVR and SPI_CS for STM32 => DEBUG_PIN_on, DEBUG_PIN_off, DEBUG_PIN_toggle
//#define DEBUG_SERIAL  // Use STM32_BOARD, 
                              // for stm32 boad, compiled with Upload method "Serial"->usart1, "STM32duino bootloader"->USB serial
// Note: this list is not 100% completed (see e.g. pin.h, SPI.ino, Validate.h) 


*/