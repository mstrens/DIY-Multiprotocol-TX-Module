/*                       Documentation about Milo
    Protocol description:
    2.4Ghz LORA modulation
    - 142 Hz frame rate(7ms)
    - Data Rate ~76kb/s(-108dBm)
    - Bw-812; SF6 ; CR -LI -4/7 .
    - Preamble length 12 symbols
    - Fixed length packet format(implicit) -16 bytes
    - Downlink telemetry rate(1:3)
    - Uplink telemetry rate(1:6)
    - Hardware CRC is ON.
    
    # Normal frame channels 1-8; frame rate 7ms.
    
    0. reserve 2 bits (bits 7..6) | next expected telemetry down link frame counter(sequence) (bits 5..4 (2 bits=4 val)) | reserve (bit 3) | Frame type(bits 2..0 (3 lsb bits))
    1. txid1 TXID on 16 bits
    2. txid2
    3. flag next frame must be dwn tlm frame (bit 7) | flag requesing starting WIFI (bit 6) | Model ID /Rx_Num(bits 5....0 = 6 bits) 
    4. channels 8 channels/frame ; 11bits/channel
    5. channels total 11 bytes of channels data in the packet frame
    6. channels
    7. channels
    8. channels
    9. channels
    10. channels
    11. channels
    12. channels
    13. channels
    14. channels
    15. Index of RF channel (0...37) ; used by Rx to (check) synchronize

    # Normal frame channels 9-16 separate; frame rate 7ms.
    0. reserve 2 bits (bits 7..6) | next expected telemetry down link frame counter(sequence) (bits 5..4 (2 bits=4 val)) | reserve (bit 3) | Frame type(bits 2..0 (3 lsb bits))
    1. txid1 TXID on 16 bits
    2. txid2
    3. flag next frame must be dwn tlm frame (bit 7) | flag requesing starting WIFI (bit 6) | Model ID /Rx_Num(bits 5....0 = 6 bits) 
    4. channels 8 channels/frame ; 11bits/channel
    5. channels total 11 bytes of channels data in the packet frame
    6. channels
    7. channels
    8. channels
    9. channels
    10. channels
    11. channels
    12. channels
    13. channels
    14. channels
    15. Index of RF channel (0...37) ; used by Rx to (check) synchronize
    
    # TX uplink telemetry frame can be sent separate ;frame rate 7ms;1:6 telemetry data rate.
    0. reserve 2 bits (bits 7..6) | next expected telemetry down link frame counter(sequence) (bits 5..4 (2 bits=4 val)) | Frame type(bits 2..0 (3 lsb bits))
    1. txid1 TXID on 16 bits
    2. txid2
    3. telemetry uplink counter sequence(2 bits 7..6) | Model ID /Rx_Num(bits 5....0 = 6 bits)
    4.Sport data byte1
    5.Sport data byte 2
    6.Sport data byte 3
    7.Sport data byte 4
    8.SPort data byte 5
    9.SPort data byte 6
    10.SPort data byte 7
    11.SPort data byte 8
    12.Reserve
    13.Reserve
    14.Reserve 
    15.Index of RF channel (0...37) ; used by Rx to (check) synchronize

    # RX downlink telemetry frame sent separate at a fixed rate of 1:3;frame rate 7ms. Can contain 2 Sport frame 
    0. - bits 7...2 : MSB of TXID1 (6 bits)
       - bits 1...0 : current downlink tlm counter (2 bits); when received TX should send this counter + 1type of link data packet(RSSI/SNR /LQI) (2 bits= 3 values currently) 
    1. - bits 7...2 : MSB of TXID2 (6 bits)
       - bits 1...0 : last upllink tlm counter received (2 bits); 
    2. - bit 7 : reserve
         bits 6..5 : recodified PRIM from sport frame1 (0X30=>0, 0X31=>1,0X32=>2, 0X10=>0)
         bits 4..0 : PHID from sport frame1 (5 bits using a mask 0X1F; 0X1F = no data; 0X1E = link quality data)
    3. - bit 7 : reserve
         bits 6..5 : recodified PRIM from sport frame2 (0X30=>0, 0X31=>1,0X32=>2, 0X10=>0)
         bits 4..0 : PHID from sport frame2 (5 bits using a mask 0X1F; 0X1F = no data; 0X1E = link quality data)
    4. field ID1 from sport frame1
    5. field ID2 from sport frame1
    6...9. Value from sport frame1 (4 bytes)
    10. field ID1 from sport frame2
    11. field ID2 from sport frame2
    12...15. Value from sport frame2 (4 bytes)
    
    
    # bind packet
    0. Frame type = BIND_PACKET = 0
    1. rx_tx_addr[3];
    2. rx_tx_addr[2];
    3. rx_tx_addr[1];
    4. rx_tx_addr[0];
    5. RX_num;
    6. chanskip;
    7. up to 15.  0xA7


    # Frame Sequence
    0- downlink telemetry
    1- RC channels 1_8_1 
    2- RC channels 9_16
    3- downlink telemetry
    4- RC channels 1_8_2
    5 -uplink telemetry                 
    6- downlink telemetry
    7- RC channels 9_16
    8- RC channels 1_8_1
    9- downlink telemetry
    10- RC channels 9_16
    11- uplink telemetry               
    12- downlink telemetry
    13- RC channels 1_8_2
    14- RC channels 9_16
    15- downlink telemetry
    
    
    0 - downlink telemetry
    1- RC channels 1_8_1         
    2- RC channels 1_8_2      
    3- downlink telemetry      
    4- RC channels 1_8_1       
    5 -uplink telemetry              
    6- downlink telemetry      
    7- RC channels 1_8_2         
    8- RC channels 1_8_1          
    9- downlink telemetry            
    10- RC channels 1_8_2       
    11- uplink telemetry           
    12- downlink telemetry
    13- RC channels 1_8_1            
    14- RC channels 1_8_2              
    15- downlink telemetry
    16- RC channels 1_8_1            
    17  uplink telemetry              
    15- downlink telemetry
    
    
Process flow:

- there is a callback() in Milo_sx1280.ino that is called by main loop at regular interval (intervals are returned by callback())
- callback() handles different cases to decide if a frame has to be sent (and which type) or received.
    it works with 3 slots of 7msec each (2 for sending 1 for receiving)
    uplink tlm frame are only sent in the second slot, if tlm data exist and only once every 6 slots at max 
    callback() manages the SX1280 (write commands) and frequency hop only when sending data.
    Note: there is no frequency hop for a slot reserved to receive a dwnlink tlm frame (otherwise it would generates an issue)
- when a frame with RcData (or failsafe) must be sent, callback() call MiLo_data_frame() and then look at the next frame type (second RCData, uplink frame or Dwownlink)
- when a frame with uplink tlm data must be sent, callback() call MiLo_Telemetry_frame() and then expect a downlink tlm frame 
- when we have to receive a downlink tlm frame, first we switch in receive mode and wait (non blocking) for some delay.
     Then if a frame is received, we read it from SX1280 to packet_in[] and we process packet[in] with frsky_process_telemetry()
In frsky_process_telemetry(), we check if it has the expected downlink tlm counter (sequence check) and if OK, we extract the data and put 
     - link quality data (if any) in specific variables
     - sport set of data (8 bytes) into pktx1[]; we can append one or 2 set of data. There is some uncompression performed for PHID and PRIM bytes
        note : pktx1[] can store more than 2 sets of 8 bytes data.
    After filling pktx1[], we call sportSend();

In main loop there is a Update_All() that is called as often as possible
Update_All() call Update_Telem() which call TelemetryUpdate()
TelemetryUpdate() :
    - when MULI_TELEMETRY is activated
        call multi_send_status() at regular interval (500 msec) to send data to handset
        call mult_send_inputsync() at regular interval (if option is activated)  to send data to handset
        call sportSendFrame() (for MILO to send data to handset) which  
            - create a frame based on 8 first bytes from pktx1[32]
            - sent it using sportSend() in 2 possible formats multiprotocol (header,1 byte + 8 payload + CRC) or sport format (START+ Stuffing + CRC)
            - handle also the link quality data (RSSI/SNR/LQI)


---------------------------


Not specific to milo but general for multiprotocol:
Uplink tlm data:
Handset sent at regular interval a packet (over serial) that contains:
- some multiprotocol parameters (e.g. range flag, power flag, failsafe flag, protocol and sub protocol, Rx_num)
- channels values
- uplink tlm packet (optional)

Main loop call Update_all()  which call update_serial_data() that manages the data coming from the handset stored in rx_ok_buffer[])
- update_serial_data() updates some flags (e.g. once per 9 sec a flag to say that the packet contains failsafe data - and no normal RC channels)
- if there are more than 27 bytes, there are uplink tlm data (8 bytes in incoming packet).
    uplink tlm data are converted (add START and stuffing) and added to a circular buffer SportData[] (and SportTail + SportHead)
    Note: for Milo, we just store the 8 incoming bytes in SportData[] without adding START and stuffing. We also use SportCount
    If the circular buffer is full, there is a special process sending a flag in a MULTI_STATUS message to stop getting new data

The data in SportData[] will be processed by  MILO_callback() which call MiLo_Telemetry_frame() when a uplink tlm slot may be sent   

*/