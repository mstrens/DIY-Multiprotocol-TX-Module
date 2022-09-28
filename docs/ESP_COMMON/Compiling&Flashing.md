## Compiling & Flashing ##

**ESP32 -target**

First select from **Multiprotocol.ino** file the board used.For example uncomment(if already commented) **BETAFPV_500** if used a hacked ExpresslRS BetaFPV 500 TX module 
or **nothing if used DIY TX module**.Make sure use only one selection.

- #ifdef ESP32_PLATFORM
- #define BETAFPV_500 //if hacked this expresslrs Tx module
- //#define HM_ES24TX //if hacked this expresslrs Tx module
- #endif

For ESP32 target using Arduino IDE version > 1.8.13, **Board manager** and install ESP32 core by Expressif Systems.
Along with ESP32 core  you nned to install 2 more libraries **AsyncElegantOTA and  ESPAsyncWebserver** like below :

For  AsyncElegantOTA  Go to Sketch > Include Library > Library Manager > Search for "AsyncElegantOTA" > Install

For ESPAsyncWebserver you nned to do it manually see below.

Click [here](https://github.com/me-no-dev/ESPAsyncWebServer/archive/refs/heads/master.zip) to download the ESPAsyncWebServer library. You should have a .zip folder in your Downloads folder.
- Unzip the .zip folder and you should get ESPAsyncWebServer-master folder
- Rename your folder from ESPAsyncWebServer-master to ESPAsyncWebServer
- Move the ESPAsyncWebServer folder to your Arduino IDE installation libraries folder.

After libraries installation restart Arduino IdE and Browse to Multiprotocol.ino file and load it in Arduino IDE.Select **Tools,ESP32 arduino, ESP32 Dev module,**.
Connect the USB cable tom the TX module if it has one if not connect your USB-FTDI serial device TX,RX,5V,GND pins to  coresponding board pins(TX ->RX and RX->TX).
Select the serial port in Tools and press(select) **Upload**.The firmware will be uploaded on your board.First flashing must be done serially using USB port or an USB -serial device(FTDI).
Subsequent flashing can be done via OTA.For that press **Sketch ,select Export compiled  Binary**.Browse to the location of the binary(in the same folder as multiprotocol).Copy and store it in an accesible location for flashing.
Navigate to the protocol menu screen in your TX select protocol number **128** (MiLo protocol number) then select **WiFi-TX**.
Check your wifi connections and find the AP opened by the TX module.

It will open an AP with SSID name **"MiLo_TX"** click connect and introduce the password **"milo_sx1280"**.

It will open a captive portal at **"10.0.0.1"** adress.Inside you will browse to the firmware(.bin file you already stored before) and upload it.After uploading is completed restart the TX with new firmware.

**ESP 8266 -target**

First select from **Multiprotocol.ino** file the board used.Use only one selection.
- #ifdef ESP8266_PLATFORM
- #define BETA_FPV_RX_NANO //or clone
- //#define MATEK_RX_R24D
- //#define DIY_RX //use RX as TX(diversity) no PA/LNA
- #endif

Same steps for uploading multiprotocol.ino file in Arduino IDE.And install ESP8266 arduino core as described for MiLo receiver here below.
https://github.com/midelic/MILO-RX/blob/main/README.md#software-installation

https://github.com/midelic/MILO-RX/blob/main/README.md#flashing
