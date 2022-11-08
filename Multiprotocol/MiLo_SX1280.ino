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

    //#define DEBUG_UNUSED_TX_SLOTS // when activated, it is possible to skip some consecutive channels (e.g. to test synchro TX-RX)
    //#define DEBUG_SKIP_TX_FROM_CHANNEL 10 // lower index 
    //#define DEBUG_SKIP_TX_UPTO_CHANNEL 20 // upper index

    #define NBR_BYTES_IN_PACKET 16 // number of bytes in a LORA packet


    uint8_t uplinkTlmId;
    uint8_t expectedUplinkTlmId;
    
    uint16_t timeout = 0xFFFF;
    uint8_t currOpmode;
    uint8_t PayloadLength = NBR_BYTES_IN_PACKET;
    bool IQinverted = 0;
    uint8_t packetLengthType;
    uint8_t uplinkLQ;
    int8_t LastPacketRSSI = 0;
    int8_t LastPacketSNR = 0;
    uint32_t currFreq = 0;
    uint8_t chanskip = 0;
    bool frameReceived = false;
    uint8_t frameType = 0;
    bool LBTEnabled = false;
    bool LBTStarted = false;
  	uint8_t LBTdelay = 0;
    uint32_t miloSportTimer = 0;
    bool miloSportStart = false;
    uint8_t ThisPacketDropped ;
   uint8_t DropHistory[100] ;
   uint8_t DropHistoryIndex ;
   uint8_t DropHistoryPercent ;
   uint8_t DropHistorySend ;
    
    #ifdef SPORT_SEND
        uint8_t SportToAck;
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
        uint8_t PayloadLength;      // Number of OTA bytes to be sent//16
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
        {0, RADIO_TYPE_SX128x_LORA, RATE_LORA_150HZ,  SX1280_LORA_BW_0800,SX1280_LORA_SF6,  SX1280_LORA_CR_LI_4_7, 7000,
         TLM_RATIO_1_3,12, NBR_BYTES_IN_PACKET },
        {1, RADIO_TYPE_SX128x_LORA, RATE_LORA_100HZ,  SX1280_LORA_BW_0800, SX1280_LORA_SF7,  SX1280_LORA_CR_LI_4_6, 9000,
         TLM_RATIO_1_3,12, NBR_BYTES_IN_PACKET}};
    
    
    
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
            SportToAck = 0;  
        #endif
        #ifdef TELEMETRY
            telemetry_lost = 1;
            telemetry_link = 0;                 //Stop sending telemetry
        #endif
        uplinkTlmId = 0;
        expectedUplinkTlmId = 0;        
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
        packet[0] |= ( (telemetry_counter<<4) & 0X30) ; // 2 bits (5..4) are the next downlink tlm counter
        #ifdef DEBUG_ON_GPIO03
            if (getCurrentChannelIdx() == 0) { // when the channel is the first one
                G3PULSE(5);
            }
        #endif      
        packet[1] = rx_tx_addr[3];
        packet[2] = rx_tx_addr[2];
        packet[3] =  RX_num & 0x3F ;//max 64 values
        if ( packet_count == 2) packet[3] |=  0x80 ; //when next packet will be a downlink, then mark it
        if (sub_protocol == WIFI_RX) packet[3] |= 0x40;//trigger WiFi updating firmware for RX
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
        packet[15] = getCurrentChannelIdx() & 0x3F ; // channel index is max 37 and so coded on 5 bits 
        if(sub_protocol == MCH_8 || sub_protocol == MEU_8)// in M16/CH1-8 mode send only 8ch every interval us
            lpass = 0 ;
        else
            lpass += 1 ;
    }

    static void ICACHE_RAM_ATTR MiLo_Telemetry_frame() 
    {   // fill an uplink tfm frame with 8 bytes from SportData only when an uplink must be send (checked just after preparing a RcData frame)
        packet[0] = ( (telemetry_counter<<4) & 0X30) | (TLM_PACKET) ;
        packet[1] = rx_tx_addr[3];
        packet[2] = rx_tx_addr[2];
        expectedUplinkTlmId = (expectedUplinkTlmId + 1) & 0x03;//2 bits ; increased each time we send an uplinlk tlm frame                      
        packet[3] = ( (expectedUplinkTlmId & 0X03) << 6)  | RX_num & 0x3F ;//max 64 values
        memcpy( &packet[4], &SportData[SportTail], 8 ) ; // copy 8 bytes 
        SportTail = (SportTail + 8) & 0X3F; // update SportTail
        // SportCount is not decreased here and SportToAck is not updated.
        // they will be updated only when a donwlink frame confirms that the uplink has been received
        // those updates occur while processing the downlink tlm frame   
        packet[12] = 0XA7; // fill reserve with dummy data
        packet[13] = packet[12];
        packet[14] = packet[12];   
        packet[15] = getCurrentChannelIdx() & 0x3F ; // channel index is max 37 and so coded on 5 bits
        if (SportCount < 6) DATA_BUFFER_LOW_off;  // This will say in Milo_status that handset can continue to send uplink data
    }
    
    void MILO_init()
    {
        Fhss_Init();  // prepare full list of all freq
        Fhss_generate(MProtocol_id); // create a list that depends on the ID of the TX
        currFreq = GetBindFreq(); //set frequency first or an error will occur!!!
        currOpmode = SX1280_MODE_SLEEP;     
        bool init_success = SX1280_Begin();
        if (!init_success) {           
            debugln("Init of SX1280 failed"); delay(100); 
            while(1) {} // This line can be commented when we want to test without a SX1280        
        } 
        {
            if(IS_BIND_IN_PROGRESS) {
                while(!chanskip)
                    chanskip = random(0xfefefefe)%68;
                chanskip &= 0x0F;
                is_in_binding = true;
                MiLo_SetRFLinkRate(RATE_BINDING);
                state = MiLo_BIND;
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
                        LBTEnabled = true;
                    } 
                #endif
                state = MiLo_DATA1;
                MiLo_telem_init(); // initialise variables (flags) used by telemetry depending of SPORT_SEND and TELEMETRY
            }
            //SX1280_SetTxRxMode(TXRX_OFF);
            POWER_init();  // set power on min value.
            PayloadLength = MiLo_currAirRate_Modparams->PayloadLength;
            debugln("end of milo_init; case= %d",state); 
        }   
    }

    uint16_t MILO_callback()
    {   // this function is called at regular interval by main loop and manage all time slots for sending and receiving RF on SX1280
        uint16_t intervalMiloCallback;
        intervalMiloCallback = MiLo_currAirRate_Modparams->interval;  
   		#ifdef MILO_USE_LBT
          	LBTdelay = SpreadingFactorToRSSIvalidDelayUs(MiLo_currAirRate_Modparams->sf);
        #endif
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
                        LBTEnabled = true;
                    }
                #endif
                state = MiLo_DATA1;
                break;
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
                packet_count = (packet_count + 1)%3;
                nextChannel(1);
                SX1280_SetFrequencyReg(GetCurrFreq());    
                SX1280_SetOutputPower(MaxPower);
                #ifdef MILO_USE_LBT
                    if (LBTEnabled)
                    { 
                        SX1280_SetTxRxMode(RX_EN);// LBTmode - BeginClearChannelAssessment
                        SX1280_SetMode(SX1280_MODE_RX);//start RX mode in order to get RSSI 				
                        delayMicroseconds(LBTdelay);		
                        if(!ChannelIsClear()) SX1280_SetOutputPower(MinPower);
                    }
                #endif  
                #if defined (DEBUG_UNUSED_TX_SLOTS) && defined (DEBUG_SKIP_TX_FROM_CHANNEL) && defined (DEBUG_SKIP_TX_UPTO_CHANNEL)
                    if ( (getCurrentChannelIdx() <  DEBUG_SKIP_TX_FROM_CHANNEL) || (getCurrentChannelIdx() >  DEBUG_SKIP_TX_UPTO_CHANNEL) ) {
                #endif
                MiLo_data_frame();
                SX1280_WriteBuffer(0x00, packet,PayloadLength); //
                SX1280_SetTxRxMode(TX_EN);// do first to allow PA stablise
                SX1280_SetMode(SX1280_MODE_TX);
                #if defined (DEBUG_UNUSED_TX_SLOTS) && defined (DEBUG_SKIP_TX_FROM_CHANNEL) && defined (DEBUG_SKIP_TX_UPTO_CHANNEL)
                    }
                #endif
                //debugln("start sending packet %d", packet_count);         
                
                // after having prepare sending the RCData frame, we have to determine the type of the next frame 
                if (packet_count == 2){// next frame is RX downlink temetry
                    state = MiLo_DWLNK_TLM1;
                    intervalMiloCallback = 5400;
                    break;
                }
                else if( (upTLMcounter == 2) &&  (SportCount > 0) )
                {// The slot can be used for uplink and there are date to send (or to resend)
                    state = MiLo_UPLNK_TLM;
                    upTLMcounter  = 0;//reset uplink telemetry counter
                    break;
                }       
                state = MiLo_DATA1; // ense continue with a RCData frame
                break;      
            case MiLo_UPLNK_TLM:    //Uplink telemetry
                packet_count = (packet_count + 1)%3;
                nextChannel(1);
                SX1280_SetFrequencyReg(GetCurrFreq());    
                SX1280_SetOutputPower(MaxPower);
                #ifdef MILO_USE_LBT
                    if (LBTEnabled) {
                        SX1280_SetTxRxMode(RX_EN);// LBTmode - BeginClearChannelAssessment
                        SX1280_SetMode(SX1280_MODE_RX);//start RX mode in order to get RSSI 				
                        delayMicroseconds(LBTdelay);		
                        if(!ChannelIsClear()) SX1280_SetOutputPower(MinPower);
                    }
                #endif
                #if defined (DEBUG_UNUSED_TX_SLOTS) && defined (DEBUG_SKIP_TX_FROM_CHANNEL) && defined (DEBUG_SKIP_TX_UPTO_CHANNEL)
                    if ( (getCurrentChannelIdx() <  DEBUG_SKIP_TX_FROM_CHANNEL) || (getCurrentChannelIdx() >  DEBUG_SKIP_TX_UPTO_CHANNEL) ) {
                #endif
                MiLo_Telemetry_frame();
                SX1280_WriteBuffer(0x00, packet, PayloadLength); 
                SX1280_SetMode(SX1280_MODE_TX); 
                #if defined (DEBUG_UNUSED_TX_SLOTS) && defined (DEBUG_SKIP_TX_FROM_CHANNEL) && defined (DEBUG_SKIP_TX_UPTO_CHANNEL)
                    }
                #endif
                state = MiLo_DWLNK_TLM1;// next frame is RX downlink temetry
                intervalMiloCallback = 5400;//
                break;      
            case MiLo_DWLNK_TLM1://downlink telemetry
                SX1280_SetTxRxMode(RX_EN);// do first to enable LNA
                SX1280_SetMode(SX1280_MODE_RX);
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
                    if( (packet_in[0] & 0xFC) == ( rx_tx_addr[3] & 0XFC) &&
                        (packet_in[1] & 0xFC) == ( rx_tx_addr[2] & 0XFC) ){   // check it is a frame for the right handset (only on 6 bits MSB) 
                        SX1280_GetLastPacketStats();     // read SX1280 to get LastPacketRSSI and LastPacketSNR (data are not in the received packed)
                        telemetry_link|=1;               // Telemetry data is available
                        frsky_process_telemetry(packet_in, PayloadLength); //check if valid telemetry packets
                        LQICalc();
                        memset(&packet_in[0], 0, PayloadLength ); // reset packet_in[]              
                        frameReceived = false;
                    }
                }
                else{
                    miloSportStart = false;
                    ThisPacketDropped = 1;
                }
                state = MiLo_DATA1;
                intervalMiloCallback = 1000;
                break;        
        } // end switch       
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

    uint32_t ICACHE_RAM_ATTR SpreadingFactorToRSSIvalidDelayUs(uint8_t SF)
    {
    // The necessary wait time from RX start to valid instant RSSI reading
    // changes with the spreading factor setting.
    // The worst case necessary wait time is TX->RX switch time + Lora symbol time
    // This assumes the radio switches from either TX, RX or FS (Freq Synth mode)
    // TX to RX switch time is 60us for sx1280
    // Lora symbol time for the relevant spreading factors is:
    // SF5: 39.4us
    // SF6: 78.8us
    // SF7: 157.6us
    // SF9: 630.5us
    // However, by measuring when the RSSI reading is stable and valid, it was found that
    // actual necessary wait times are:
    // SF5 ~100us (60us + SF5 symbol time)
    // SF6 ~141us (60us + SF6 symbol time)
    // SF7 ~218us (60us + SF7 symbol time)
    // SF9 ~218us (Odd one out, measured to same as SF7 wait time)

        switch(SF) {
            case SX1280_LORA_SF5: return 100;
            case SX1280_LORA_SF6: return 141;
            case SX1280_LORA_SF7: return 218;
            case SX1280_LORA_SF9: return 218;
            default: return 218; // Values above 100mW are not relevant, default to 100mW threshold
        }
    }	
        
    bool ICACHE_RAM_ATTR ChannelIsClear(void)
    {
    // Calculated from EN 300 328, adjusted for 800kHz BW for sx1280
    // TL = -70 dBm/MHz + 10*log10(0.8MHz) + 10 × log10 (100 mW / Pout) (Pout in mW e.i.r.p.)
    // This threshold should be offset with a #define config for each HW that corrects for antenna gain,
    // different RF frontends.
    // TODO: Maybe individual adjustment offset for differences in
    // rssi reading between bandwidth setting is also necessary when other BW than 0.8MHz are used.

    // Read rssi after waiting the minimum RSSI valid delay.
    // If this function is called long enough after RX enable,
    // this will always be ok on first try as is the case for TX.

    // TODO: Better way than busypolling this for RX?
    // this loop should only run for RX, where listen before talk RX is started right after FHSS hop
    // so there is no dead-time to run RX before telemetry TX is supposed to happen.
    // if flipping the logic, so telemetry TX happens right before FHSS hop, then TX-side ends up with polling here instead?
    // Maybe it could be skipped if there was only TX of telemetry happening when FHSShop does not happen.
    // Then RX for LBT could stay enabled from last received packet, and RSSI would be valid instantly.
    // But for now, FHSShops and telemetry rates does not divide evenly, so telemetry will some times happen
    // right after FHSS and we need wait here.

        int8_t rssiInst = SX1280_GetRssiInst(); 
        SX1280_SetMode(SX1280_MODE_FS);//SX1280_SetTxIdleMode();
        bool channelClear = rssiInst < -71;//// TL = -70 dBm/MHz + 10*log10(0.8MHz) + 10 × log10 (100 mW / Pout) (Pout in mW e.i.r.p.)
        return channelClear;
    }			

#endif

