/*  **************************
    * By Midelic on RCGroups *
    **************************
    This project is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    
    Multiprotocol is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
    You should have received a copy of the GNU General Public License
    along with Multiprotocol.  If not, see <http://www.gnu.org/licenses/>.
*/

#if defined(MILO_SX1280_INO)
    
    #include "MiLo_FHSS.h"
    #include "iface_sx1280.h"
    #include "SX1280.h"
    #define RATE_DEFAULT 0 
    #define RATE_BINDING 0  
    #define RATE_100HZ 1 //100HZ
    #define RATE_150HZ 0 //150HZ
    #define RATE_MAX 3
    //#define MILO_USE_LBT


    #ifdef DEBUG_ON_GPIO3
        #define G3ON digitalWrite(3,HIGH)
        #define G3OFF digitalWrite(3,LOW)
        #define G3TOGGLE digitalWrite(3,!digitalRead(3))
        #define G3PULSE(usec) digitalWrite(3,HIGH);delayMicroseconds(usec); digitalWrite(3,LOW)
    #else
        #define G3ON 
        #define G3OFF 
        #define G3TOGGLE 
        #define G3PULSE(usec) 
    #endif



    uint8_t TelemetryId;
    uint8_t TelemetryExpectedId;
    
    uint16_t timeout = 0xFFFF;
    uint8_t currOpmode;
    uint8_t PayloadLength = 15;
    bool IQinverted = 0;
    uint8_t packetLengthType;
    uint8_t uplinkLQ;
    int8_t LastPacketRSSI = 0;
    int8_t LastPacketSNR = 0;
    uint32_t currFreq = 0;
    uint8_t chanskip = 0;
    bool frameReceived = false;
    uint8_t frameType = 0;
    extern bool LBTEnabled;
    bool LBTStarted = false;
    uint32_t miloSportTimer = 0;
    bool miloSportStart = false;
    uint8_t ThisPacketDropped ;
   uint8_t DropHistory[100] ;
   uint8_t DropHistoryIndex ;
   uint8_t DropHistoryPercent ;
   uint8_t DropHistorySend ;
    
    #ifdef SPORT_SEND
        uint8_t idxOK;
    #endif
    void ICACHE_RAM_ATTR dioISR(void);
    
    typedef struct 
    {
        uint8_t uplink_RSSI_1;
        uint8_t uplink_RSSI_2;
        uint8_t uplink_Link_quality;
        int8_t uplink_SNR;
        uint8_t active_antenna;
        uint8_t uplink_TX_Power;
        uint8_t downlink_RSSI;
        uint8_t downlink_Link_quality;
        int8_t downlink_SNR;
        
    } MiLo_statistics;
    MiLo_statistics MiLoStats;
    
    
    enum {
        MiLo_BIND       = 0,
        MiLo_BIND_DONE  = 1500,
        MiLo_DATA1,
        MiLo_DATA2,
        MiLo_UPLNK_TLM,
        MiLo_DWLNK_TLM1,
        MiLo_DWLNK_TLM2,
        #ifdef MILO_USE_LBT
            MiLo_USE_LBT
        #endif
    };
    
    enum{
        BIND_PACKET = 0,
        CH1_8_PACKET1,
        CH1_8_PACKET2,
        CH1_16_PACKET,
        TLM_PACKET,
        FLSF_PACKET1,
        FLSF_PACKET2
    };
    
    enum
    {
        TLM_RATIO_NO_TLM = 0,
        TLM_RATIO_1_1 = 1,
        TLM_RATIO_1_2 = 2,
        TLM_RATIO_1_3 = 3,
        TLM_RATIO_1_4 = 4,
    };
    
    
    //RF PARAMETRS
    typedef struct MiLo_mod_settings_s
    {
        uint8_t index;//
        uint8_t radio_type;//RADIO_TYPE_SX128x_LORA
        uint8_t frame_rate_type;          // 
        uint8_t bw;
        uint8_t sf;
        uint8_t cr;
        uint32_t interval;          // interval in us seconds that corresponds to that frequency 
        uint8_t TLMinterval;        // every X packets is a response TLM packet
        uint8_t PreambleLen;         //12
        uint8_t PayloadLength;      // Number of OTA bytes to be sent//15
    } MiLo_mod_settings_t;
    
    typedef struct MiLo_rf_pref_params_s
    {
        uint8_t index;
        uint8_t frame_rate;                    // 
        int32_t RXsensitivity;                // expected RF sensitivity based on
        uint32_t TOA;                         // time on air in microseconds
        uint32_t DisconnectTimeoutMs;         // Time without a packet before receiver goes to disconnected (ms)
        uint32_t RxLockTimeoutMs;             // Max time to go from tentative -> connected state on receiver (ms)
    } MiLo_rf_pref_params_t;
    
    
    MiLo_mod_settings_s *MiLo_currAirRate_Modparams;
    MiLo_rf_pref_params_s *MiLo_currAirRate_RFperfParams;
    
    MiLo_mod_settings_s MiLo_AirRateConfig[RATE_MAX] = { 
        {0, RADIO_TYPE_SX128x_LORA, RATE_LORA_150HZ,  SX1280_LORA_BW_0800,SX1280_LORA_SF6,  SX1280_LORA_CR_LI_4_7, 7000, TLM_RATIO_1_3,12, 15 },
        {1, RADIO_TYPE_SX128x_LORA, RATE_LORA_100HZ,  SX1280_LORA_BW_0800, SX1280_LORA_SF7,  SX1280_LORA_CR_LI_4_6, 9000, TLM_RATIO_1_3,12, 15}};
    
    
    
    MiLo_rf_pref_params_s MiLo_AirRateRFperf[RATE_MAX] = {
        {0, RATE_LORA_150HZ,  -108,  5060, 3500, 2500},
        {1, RATE_LORA_100HZ,  -112,  7605, 3500, 2500}};
    // index , frame rate, sensitivity, time over the air , disconnect time out, Rx loxk time out)
    
    void  MiLo_SetRFLinkRate(uint8_t index) // Set speed of RF link (hz) index values
    {
        MiLo_mod_settings_s *const ModParams = &MiLo_AirRateConfig[index];
        MiLo_rf_pref_params_s *const RFperf = &MiLo_AirRateRFperf[index];   
        bool invertIQ = 0x01;//inverted
        if ((ModParams == MiLo_currAirRate_Modparams)
            && (RFperf == MiLo_currAirRate_RFperfParams)
            && (invertIQ == IQinverted))
            return;
        //uint32_t interval = ModParams->interval;
        SX1280_Config(ModParams->bw, ModParams->sf, ModParams->cr, GetCurrFreq(),
        ModParams->PreambleLen, invertIQ, ModParams->PayloadLength);
        MiLo_currAirRate_Modparams = ModParams;
        MiLo_currAirRate_RFperfParams = RFperf;
    }
    
    
    static void ICACHE_RAM_ATTR MiLo_telem_init(void)
    {
        #ifdef SPORT_SEND
            SportHead = SportTail=0;            // empty data buffer
            idxOK = 0;  
        #endif
        #ifdef TELEMETRY
            telemetry_lost = 1;
            telemetry_link = 0;                 //Stop sending telemetry
        #endif
        TelemetryId = 0;
        TelemetryExpectedId = 0;        
    }
    
    static void ICACHE_RAM_ATTR MiLo_build_bind_packet()
    {
        packet[0] = BIND_PACKET;
        packet[1] = rx_tx_addr[3];//ModelID(seed for Fhss channels)
        packet[2] = rx_tx_addr[2];
        packet[3] = rx_tx_addr[1];
        packet[4] = rx_tx_addr[0];
        packet[5] = RX_num;
        packet[6] = chanskip;
        memset(&packet[7], 0, PayloadLength - 7);
        for(uint8_t  i = 7; i < PayloadLength; i++)//XOR packets
        packet[i] ^= 0xA7;
    }   
    //0 240 184 107 0 0 15 167 167 167 167 167 167 167 167
    
    static void ICACHE_RAM_ATTR MiLo_data_frame()
    {   
        static uint8_t lpass = 0;
        uint8_t j = 0;
        static bool pass = false;   
        if ( lpass & 1 ){
            j += 8 ;
            packet[0] = CH1_16_PACKET;
        }
        else{//lpass =0
            if(!pass)
            packet[0] = CH1_8_PACKET1;
            else 
            packet[0] = CH1_8_PACKET2 ;
            
            pass = ! pass;
        }
        packet[0] |= (telemetry_counter<< 4); // 4 MSB are the next downlink tlm counter
        if (getCurrentChannelIdx() < FHSS_SYNCHRO_CHANNELS_NUM) { // when the channel is one of the Syncro channels set flag on
            packet[0] |=  0X08; // fill synchro flag (bit 3) when channel index is lower than the number of synchro channels
            //G3ON;  // in debug on pulse mode on ES8266 set level HIGH for a synchro channel
        } else {
            //G3OFF;    // in debug, other reset to LOW
        }  
        packet[1] = rx_tx_addr[3];
        packet[2] = rx_tx_addr[2];
        packet[3] =  RX_num & 0x3F ;//max 64 values
        if ( packet_count == 2) packet[3] |=  0x80 ; //when next packet will be a downlink, then mark it
        if (sub_protocol == WIFI_RX) packet[3] | 0x40;//trigger WiFi updating firmware for RX
        uint16_t (*ch) (uint8_t) = &convert_channel_ppm;
        packet[4] = (*ch)(0+j)&0XFF ;
        packet[5] = (*ch)(0+j)>>8 | ((*ch)(1+j)&0xFF)<<3;
        packet[6] = (*ch)(1+j)>>5| (*ch)(2+j)<<6;
        packet[7] = ((*ch)(2+j)>>2)& 0x00FF;
        packet[8] = (*ch)(2+j)>>10|((*ch)(3+j)&0xFF)<<1;
        packet[9] = (*ch)(3+j)>>7| ((*ch)(4+j)&0xFF)<<4;
        packet[10] = (*ch)(4+j)>>4|((*ch)(5+j)&0xFF)<<7;
        packet[11] = ((*ch)(5+j)>>1)& 0x00FF;
        packet[12] = (*ch)(5+j)>>9|((*ch)(6+j)&0xFF)<<2;
        packet[13] = (*ch)(6+j)>>6|((*ch)(7+j)&0xFF)<<5;
        packet[14] = ((*ch)(7+j)>>3)& 0x00FF;
        
        if(sub_protocol == MCH_8 || sub_protocol == MEU_8)// in M16/CH1-8 mode send only 8ch every interval us
            lpass = 0 ;
        else
            lpass += 1 ;
    }
    
    static void __attribute__((unused)) FrSkyX_send_sport(uint8_t start, uint8_t end)
    {  // fill part of uplink tlm frame with Sport data    
        for (uint8_t i = start;i <= end;i++)
            packet[i] = 0;
        
        #ifdef SPORT_SEND
            
            if(TelemetryId == TelemetryExpectedId)     
                idxOK = SportHead;// update read pointer to last ack'ed packet
            else
                SportHead = idxOK;
            TelemetryExpectedId = (TelemetryId + 1) & 0x0F;//4 bits             
            uint8_t nbr_bytes = 0;
            for (uint8_t i = start + 1;i <= end;i++)
            {
                if(SportHead == SportTail)
                    break; //buffer empty
                packet[i] = SportData[SportHead];
                SportHead = (SportHead + 1) & (MAX_SPORT_BUFFER - 1);
                nbr_bytes++;
            }
            packet[start] = (nbr_bytes << 4)| (TelemetryId &0x0F);
            if(nbr_bytes)
            {//Check the buffer status
                uint8_t used = SportTail;
                if ( SportHead > SportTail )
                    used += MAX_SPORT_BUFFER - SportHead ;
                else
                    used -= SportHead ;
                if ( used < (MAX_SPORT_BUFFER>>1) )
                {
                    //DATA_BUFFER_LOW_off;
                    debugln("Ok buf:%d",used);
                }
            }
        #endif
    }
    
    static void ICACHE_RAM_ATTR MiLo_Telemetry_frame()
    {
        FrSkyX_send_sport(3 , PayloadLength - 1); // fill the sport data
        packet[0] = (telemetry_counter<<4) | (TLM_PACKET) ;
        if (getCurrentChannelIdx() < FHSS_SYNCHRO_CHANNELS_NUM) { // when the channel is one of the Syncro channels set flag on
            packet[0] |=  0X08; // fill synchro flag (bit 3) when channel index is lower than the number of synchro channels
            //G3ON;  // in debug on pulse mode on ES8266 set level HIGH for a synchro channel
        } else {
            //G3OFF;    // in debug, other reset to LOW
        } 
        packet[1] = rx_tx_addr[3];
        packet[2] = rx_tx_addr[2];  
    }
    
    void MILO_init()
    {
        Fhss_Init();  // prepare full list of all freq
        Fhss_generate(MProtocol_id); // create a list that depends on the ID of the TX
        currFreq = GetInitialFreq(); //set frequency first or an error will occur!!!
        currOpmode = SX1280_MODE_SLEEP;     
        bool init_success = SX1280_Begin();
        if (!init_success) {           
            debugln("Init of SX1280 failed"); delay(100); 
            while(1) {} //return ;  // return commented by mstrens and replaced by a while       
        } 
        //else { //mstrens
        {
            if(IS_BIND_IN_PROGRESS) {
                while(!chanskip)
                    chanskip = random(0xfefefefe)%68;
                chanskip &= 0x0F;
                is_in_binding = true;
                MiLo_SetRFLinkRate(RATE_BINDING);
                state = MiLo_BIND;
                debugln("State = Milo_BIND"); // mstrens to debug simu bind button - normaly we do not go here
            }
            else {
                packet_count = 0;
                is_in_binding = false;
                //if(sub_protocol == 6)
                //MiLo_SetRFLinkRate(RATE_100HZ);
                //else 
                MiLo_SetRFLinkRate(RATE_150HZ);
                #ifdef MILO_USE_LBT
                    if(sub_protocol == MEU_16 || sub_protocol == MEU_8){       
                        state = MiLo_USE_LBT;
                        LBTEnabled = true;
                    }
                    else 
                #endif
                    state = MiLo_DATA1;
                MiLo_telem_init(); // initialise variables (flags) used by telemetry depending of SPORT_SEND and TELEMETRY
            }
            //SX1280_SetTxRxMode(TXRX_OFF);
            POWER_init();  // set power on min value.
            PayloadLength = MiLo_currAirRate_Modparams->PayloadLength;
            debugln("end of milo_init; case= %d",state); // mstrens to debug simu bind button
        }   
    }

    uint16_t MILO_callback()
    {   // this function is called at regular interval by main loop and manage all time slots for sending and receiving RF on SX1280
        uint16_t intervalMiloCallback;
        intervalMiloCallback = MiLo_currAirRate_Modparams->interval;  
        static uint32_t upTLMcounter = 2;
        switch(state)
        {   
            default :
                SX1280_SetFrequencyReg(currFreq);//middle of the band          
                MiLo_build_bind_packet();
                SX1280_SetTxRxMode(TX_EN);// do first to allow PA stablise      
                SX1280_WriteBuffer(0x00, packet,PayloadLength);//
                SX1280_SetMode(SX1280_MODE_TX);
                if(IS_BIND_DONE)
                    state = MiLo_BIND_DONE;
                else
                    state++;
                break;
            case MiLo_BIND_DONE:
                MiLo_telem_init();
                packet_count = 0;
                is_in_binding = false;
                MiLo_SetRFLinkRate(RATE_150HZ);
                BIND_DONE;
                #ifdef MILO_USE_LBT
                    if(sub_protocol == MEU_16 || sub_protocol == MEU_8) {       
                        state = MiLo_USE_LBT;
                        LBTEnabled = true;
                    }
                    else
                #endif
                    state = MiLo_DATA1;
                break;
            #ifdef MILO_USE_LBT
            case MiLo_USE_LBT:
                packet_count = (packet_count + 1)%3;
                SX1280_SetOutputPower(MaxPower);
                nextChannel(1);
                SX1280_SetFrequencyReg(GetCurrFreq());
                BeginClearChannelAssessment();
                if(LBTStarted) {
                    LBTStarted = false;
                    state = MiLo_UPLNK_TLM;
                }
                else    
                    state = MiLo_DATA1;
                intervalMiloCallback = SpreadingFactorToRSSIvalidDelayUs(MiLo_currAirRate_Modparams->sf);
                break;
            #endif
            case MiLo_DATA1:
                #ifdef ESP_COMMON
                    static uint32_t Now = millis();
                    if(sub_protocol == WIFI_TX){
                        if(startWifi == false){
                            SX1280_SetOutputPower(MinPower);
                            SX1280_SetTxRxMode(TXRX_OFF);//stop PA/LNA to reduce current before starting WiFi
                            SX1280_SetMode(SX1280_MODE_SLEEP);//start sleep mode to reduce SX120 current before starting WiFi                   
                            WIFI_start();
                            Now = millis();
                            startWifi = true;
                        }
                        else{
                            WIFI_event();
                            if (millis() - Now >= 50) {
                                Now = millis();
                                #ifndef BETAFPV_500
                                    LED_toggle;
                                    //digitalWrite(LED_pin ,!digitalRead(LED_pin));
                                #endif
                            }
                        }
                        break;
                    }
                #endif
                #ifdef MILO_USE_LBT
                    if (LBTEnabled){
                        if(!ChannelIsClear())
                            SX1280_SetOutputPower(MinPower);
                        MiLo_data_frame();      
                    }
                    else
                #endif  
                    {   
                        SX1280_SetOutputPower(MaxPower);
                        packet_count = (packet_count + 1)%3;
                        nextChannel(1);
                        SX1280_SetFrequencyReg(GetCurrFreq());
                        MiLo_data_frame();
                        
                    }
                SX1280_WriteBuffer(0x00, packet,PayloadLength); //
                SX1280_SetTxRxMode(TX_EN);// do first to allow PA stablise
                SX1280_SetMode(SX1280_MODE_TX);
                //debugln("start sending packet %d", packet_count);         
                if (packet_count == 2){// next frame is RX downlink temetry
                    state = MiLo_DWLNK_TLM1;
                    intervalMiloCallback = 5400;
                    break;
                }
                else{
                    if(SportHead != SportTail && upTLMcounter == 2){//next frame in uplink telemetry
                        #ifdef MILO_USE_LBT
                            if(LBTEnabled)
                            {       
                                state = MiLo_USE_LBT;
                                LBTStarted = true;
                            }
                            else
                        #endif      
                            state = MiLo_UPLNK_TLM;
                        upTLMcounter  = 0;//reset uplink telemetry counter
                        break;
                    }       
                }
                #ifdef MILO_USE_LBT
                    if(LBTEnabled)       
                        state = MiLo_USE_LBT;
                    else
                #endif  
                        state = MiLo_DATA1; 
                break;      
            case MiLo_UPLNK_TLM:    //Uplink telemetry
                #ifdef MILO_USE_LBT
                    if (LBTEnabled) {
                        if(!ChannelIsClear())
                            SX1280_SetOutputPower(MinPower);
                        MiLo_Telemetry_frame();     
                    }
                    else
                #endif
                    {   
                        packet_count = (packet_count + 1)%3;    
                        MiLo_Telemetry_frame();
                        nextChannel(1);
                        SX1280_SetFrequencyReg(GetCurrFreq());
                    }
                SX1280_WriteBuffer(0x00, packet, PayloadLength); 
                SX1280_SetMode(SX1280_MODE_TX); 
                state = MiLo_DWLNK_TLM1;// next frame is RX downlink temetry
                intervalMiloCallback = 5400;//
                break;      
            case MiLo_DWLNK_TLM1://downlink telemetry
                nextChannel(1);
                SX1280_SetFrequencyReg(GetCurrFreq());
                SX1280_SetTxRxMode(RX_EN);// do first to enable LNA
                SX1280_SetMode(SX1280_MODE_RX);
                //debugln("start receiving"); // mstrens
                packet_count = (packet_count + 1)%3;
                if(SportHead != SportTail)
                    upTLMcounter++; //increment using downlink TLM clock
                else
                    upTLMcounter = 0;//reset counter
                state = MiLo_DWLNK_TLM2;
                intervalMiloCallback = 7600;
                break;
            case MiLo_DWLNK_TLM2:
                if(frameReceived)
                {
                    uint8_t const FIFOaddr = SX1280_GetRxBufferAddr();
                    SX1280_ReadBuffer(FIFOaddr, packet_in, PayloadLength);
                    if((packet_in[1] == rx_tx_addr[3])&&packet_in[2] == rx_tx_addr[2]){   // check it is a frame for the right handset 
                        SX1280_GetLastPacketStats();
                        frsky_process_telemetry(packet_in, PayloadLength);//check if valid telemetry packets
                        LQICalc();
                        memset(&packet_in[0], 0, PayloadLength );               
                        frameReceived = false;
                    }
                    //debugln(" a frame has been received"); // mstrens
                }
                else{
                    miloSportStart = false;
                    ThisPacketDropped = 1;
                    //debugln("no frame received within timeout");
                }
                #ifdef MILO_USE_LBT
                    if(LBTEnabled)        
                        state = MiLo_USE_LBT;
                    else
                #endif
                    state = MiLo_DATA1;
                intervalMiloCallback = 1000;        
        }       
        //debugln("   next state %d  at interval %d " , state , intervalMiloCallback );  // mstrens
        return intervalMiloCallback;        
    }
    
    void ICACHE_RAM_ATTR dioISR()
    {
        uint16_t irqStatus = SX1280_GetIrqStatus();   
        SX1280_ClearIrqStatus(SX1280_IRQ_RADIO_ALL);
        #ifdef DEBUG_ESP_COMMON
            //callMicrosSerial();
        #endif
        if (irqStatus & SX1280_IRQ_TX_DONE) {
            SX1280_SetTxRxMode(TXRX_OFF);//TX_EN off and RX_EN pins off
            currOpmode = SX1280_MODE_FS; // radio goes to FS after TX           
        }
        else
            if (irqStatus & (SX1280_IRQ_RX_DONE | SX1280_IRQ_CRC_ERROR | SX1280_IRQ_RX_TX_TIMEOUT)){
                uint8_t const fail =
                    ((irqStatus & SX1280_IRQ_CRC_ERROR) ? SX1280_RX_CRC_FAIL : SX1280_RX_OK) +
                    ((irqStatus & SX1280_IRQ_RX_TX_TIMEOUT) ? SX1280_RX_TIMEOUT : SX1280_RX_OK);
                // In continuous receive mode, the device stays in Rx mode
                if (timeout != 0xFFFF) {
                    // From table 11-28, pg 81 datasheet rev 3.2
                    // upon successsful receipt, when the timer is active or in single mode, it returns to STDBY_RC
                    // but because we have AUTO_FS enabled we automatically transition to state SX1280_MODE_FS
                    currOpmode = SX1280_MODE_FS;
                }
                if (fail == SX1280_RX_OK)
                    frameReceived = true;
            }
    }
    
    void LQICalc(){
        uint8_t oldDropBit = DropHistory[DropHistoryIndex];
        DropHistory[DropHistoryIndex] = ThisPacketDropped ;
        if ( ++DropHistoryIndex >= 100 ) 
            DropHistoryIndex = 0 ;
        DropHistoryPercent += ThisPacketDropped ;
        DropHistoryPercent -= oldDropBit ;
        ThisPacketDropped = 0 ;
        if ( ++DropHistorySend >= 30 )
        {
            if (DropHistoryPercent < 100)
                MiLoStats.downlink_Link_quality = (100 - DropHistoryPercent ) ;
            DropHistorySend = 0 ;
        }   
    }
#endif



/*
    Protocol description:
    2.4Ghz LORA modulation
    - 142 Hz frame rate(7ms)
    - Data Rate ~76kb/s(-108dBm)
    - Bw-812; SF6 ; CR -LI -4/7 .
    - Preamble length 12 symbols
    - Fixed length packet format(implicit) -15 bytes
    - Downlink telemetry rate(1:3)
    - Uplink telemetry rate(1:6)
    - Hardware CRC is ON.
    
    # Normal frame channels 1-8; frame rate 7ms.
    
    0. next expected telemetry down link frame counter(sequence) (bits 7..4 (4 bits=16 val)) | synchro channel (bit 3) | Frame type(bits 2..0 (3 lsb bits))
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
    14. channels ;15 bytes payload frame
    
    # Normal frame channels 9-16 separate; frame rate 7ms.
    0. next expected telemetry down link frame counter(sequence) (bits 7..4 (4 bits=16 val)) | synchro channel (bit 3) | Frame type(bits 2..0 (3 lsb bits))
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
    14. channels ;15 bytes payload frame
    
    # TX uplink telemetry frame can be sent separate ;frame rate 7ms;1:6 telemetry data rate.
    0. next expected telemetry down link frame counter(sequence) (bits 7..4 (4 bits=16 val)) | synchro channel (bit 3) | Frame type(bits 2..0 (3 lsb bits))
    1. txid1 TXID on 16 bits
    2. txid2
    3. no. of bytes in sport frame(on max 4bits) | telemetry uplink counter sequence(4 bits)
    4.Sport data byte1
    5.Sport data byte 2
    6.Sport data byte 3
    7.Sport data byte 4
    8.SPort data byte 5
    9.SPort data byte 6
    10.SPort data byte 7
    11.SPort data byte 8
    12.SPort data byte 9
    13.SPort data byte 10
    14.SPort data byte 11 ;15bytes payload/11 bytes sport telemetry
    
    # RX downlink telemetry frame sent separate at a fixed rate of 1:3;frame rate 7ms.  
    0. - bits 7...4 : No. of bytes in sport frame(4 bits)
       - bits 3...2 : unused (2 bits) 
       - bitss 1...0 : type of link data packet(RSSI/SNR /LQI) (2 bits= 3 values currently) 
    1.txid1
    2.txid2
    3.  - bits 7...4 : current downlink tlm counter (4 bits); when received TX should send this counter + 1
        - bits 3...0 : last upllink tlm counter received(4 bits)
    4.RSSI/LQI/SNR alternate every ~80 ms update for each data
    5.Sport data byte1
    6.Sport data byte2
    7.Sport data byte3
    8.Sport data byte4
    9.Sport data byte5
    10.Sport data byte6
    11.Sport data byte7;
    12.Sport data byte8;
    13.Sport data byte9
    14.Sport data byte10; 15 bytes payload;10 bytes sport telemetry
    
    # bind packet
    0. Frame type = BIND_PACKET = 0
    1. rx_tx_addr[3];
    2. rx_tx_addr[2];
    3. rx_tx_addr[1];
    4. rx_tx_addr[0];
    5. RX_num;
    6. chanskip;
    7. up to 14.  0xA7


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
    
    
*/
