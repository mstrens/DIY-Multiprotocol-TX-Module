/*
documentation about MILO

- there is a callback() in Milo_sx1280.ino that is called by main loop at regular interval (returned by callback())
- callback() handles different cases to decide if a frame has to be sent (and which type) or received.
    it works with 3 slots of 7msec each (2 for sending 1 for receiving)
    it manages the SX1280 (write commands) and frequency hop (only when sending data; not for a slot reserved to receive a dwnlink tlm frame)
- when a frame with RcData (or failsafe) must be sent, we call MiLo_data_frame() and then check if have to send a second RCData, a uplink frame or wait for Dwownlink
- when a frame with uplink tlm data must be sent, we call MiLo_Telemetry_frame() and then expect a downlink tlm frame 
- when we have to receive a downlink tlm frame, first we switch in receive mode and wait (non blocking) for some delay.
     Then if a frame has been received, we read it from SX1280 to packet_in[] and we process packet[in] with frsky_process_telemetry()
in frsky_process_telemetry(), we check if it has the expected downlink tlm counter (sequence check) and if OK, we extract the data and put 
     - link quality data (if any) in specific variables
     - sport set of data (8 bytes) into pktx1[]; we can append one or 2 set of data. There is some uncompression performed for PHID and PRIM bytes
        note : pktx1[] can store more than 2 sets of 8 bytes data.
    After filling pktx1[], we call sportSend();

In main loop there is a Update_All() that is called as often as possible
Update_All() call Update_Telem() which call TelemetryUpdate()
TelemetryUpdate() :
    - when MULI_TELEMETRY is activated
        call multi_send_status() at regular interval (500 msec)
        call mult_send_inputsync() at regular interval (if option is activated)
        call sportSendFrame() (for MILO) which 
            - create a frame based on 8 first bytes from pktx1[32]
            - sent it using sportSend() in 2 possible formats multiprotocol (header,1 byte + 8 payload + CRC) or sport format (START+ Stuffing + CRC)
            - handle also the link quality data (RSSI/SNR/LQI)

*/