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

    #define PREAMBLE_LENGTH 12     // length of preamble in LORA
    #define NBR_BYTES_IN_PACKET 16 // number of bytes in a LORA packet

    uint8_t upTLMcounter = 0;
    uint8_t miloFailsafePass = 0 ;    // miloFailsafePass is used when a Rcframe is generated; 
                            // 0=no failsafe to send , 1= send failsafe 1_8 , 2 = wait for availableslot ,  3 = send failsafe 9_16

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
    //volatile bool dioOccured = false ;     // true when a dio1 interrupt occurs
    bool frameReceived = false;
    uint8_t frameType = 0;
    bool LBTEnabled = true;
    //bool LBTStarted = false;
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
    void ICACHE_RAM_ATTR SX1280_SetTxRxMode(uint8_t mode);

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
        MiLo_DATA,
        MiLo_UPLNK_TLM,
        MiLo_DWLNK_TLM1,
        MiLo_DWLNK_TLM2,
    };
    
    enum{
        BIND_PACKET = 0,
        CH1_8_PACKET, //  channels 1-8
        CH9_16_PACKET, // channels 9-16
        TLM_PACKET,
        FS1_8_PACKET,  //  failsafe values for channels 1-8
        FS9_16_PACKET, //  failsafe values for channels 9-16
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
    //    uint8_t index;//
    //    uint8_t radio_type;//RADIO_TYPE_SX128x_LORA
        uint8_t frame_rate_type;          // 
        uint8_t bw;
        uint8_t sf;
        uint8_t cr;
        uint32_t interval;          // interval in us seconds that corresponds to that frequency 
        uint32_t intervalBeforeDwnlnk1;
        uint32_t intervalBeforeDwnlnk2;
        uint32_t intervalAfterDwnlnk2;
    } MiLo_mod_settings_t;
  /*
    typedef struct MiLo_rf_pref_params_s
    {
        uint8_t index;
        uint8_t frame_rate;                    // 
        int32_t RXsensitivity;                // expected RF sensitivity based on
        uint32_t TOA;                         // time on air in microseconds
        uint32_t DisconnectTimeoutMs;         // Time without a packet before receiver goes to disconnected (ms)
        uint32_t RxLockTimeoutMs;             // Max time to go from tentative -> connected state on receiver (ms)
    } MiLo_rf_pref_params_t;
*/    
    
    MiLo_mod_settings_s *MiLo_currAirRate_Modparams;
//    MiLo_rf_pref_params_s *MiLo_currAirRate_RFperfParams;
    
    MiLo_mod_settings_s MiLo_AirRateConfig[RATE_MAX] = { 
    //    {0, RADIO_TYPE_SX128x_LORA, RATE_LORA_150HZ,  SX1280_LORA_BW_0800,SX1280_LORA_SF6,  SX1280_LORA_CR_LI_4_7, 7000,
    //     TLM_RATIO_1_3,12, NBR_BYTES_IN_PACKET },
    //    {1, RADIO_TYPE_SX128x_LORA, RATE_LORA_100HZ,  SX1280_LORA_BW_0800, SX1280_LORA_SF7,  SX1280_LORA_CR_LI_4_6, 9000,
    //     TLM_RATIO_1_3,12, NBR_BYTES_IN_PACKET}};
        {RATE_LORA_150HZ,  SX1280_LORA_BW_0800,SX1280_LORA_SF6,  SX1280_LORA_CR_LI_4_7, 7000, 5400 , 7600 , 1000 }, // TOA= 5222, Rx sens= -108 db
        {RATE_LORA_100HZ,  SX1280_LORA_BW_0800, SX1280_LORA_SF7,  SX1280_LORA_CR_LI_4_6, 9000, 8500 , 8500, 1000} // TOA= 7912, Rx sens= -108 db
//        {RATE_LORA_100HZ,  SX1280_LORA_BW_0800, SX1280_LORA_SF7,  SX1280_LORA_CR_LI_4_5, 9000, 8500 , 8500, 1000} // TOA= 6075, Rx sens= -108 db
        };
    
    /*    
    MiLo_rf_pref_params_s MiLo_AirRateRFperf[RATE_MAX] = {
        {0, RATE_LORA_150HZ,  -108,  5060, 3500, 2500},
        {1, RATE_LORA_100HZ,  -112,  7605, 3500, 2500}};
    // index , frame rate, sensitivity, time over the air , disconnect time out, Rx loxk time out)
    */

    void  MiLo_SetRFLinkRate(uint8_t index) // Set speed of RF link (hz) index values
    {
        MiLo_mod_settings_s *const ModParams = &MiLo_AirRateConfig[index];
        //MiLo_rf_pref_params_s *const RFperf = &MiLo_AirRateRFperf[index];   
        bool invertIQ = 0x01;//inverted
        if ((ModParams == MiLo_currAirRate_Modparams)
        //    && (RFperf == MiLo_currAirRate_RFperfParams)
            && (invertIQ == IQinverted))
            return;
        //uint32_t interval = ModParams->interval;
        SX1280_Config(ModParams->bw, ModParams->sf, ModParams->cr, GetCurrFreq(),
            PREAMBLE_LENGTH, invertIQ, NBR_BYTES_IN_PACKET);
        MiLo_currAirRate_Modparams = ModParams;
        //MiLo_currAirRate_RFperfParams = RFperf;
    }
    
    
    static void ICACHE_RAM_ATTR3 MiLo_telem_init(void)
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
    
    static void ICACHE_RAM_ATTR3 MiLo_build_bind_packet()
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
    
    /*  not used anymore
    void convert8FailsafeValuesToPpm(uint8_t fromI, uint16_t failsafeTemp[])
    { //convert value from handset like for PPM but keep original for no pulse and hold
        uint16_t val;
        for ( uint8_t i = 0; i<8; i++){
            if(Failsafe_data[i+fromI]==FAILSAFE_CHANNEL_NOPULSES) {
                failsafeTemp[i]=FAILSAFE_CHANNEL_NOPULSES;
            } else if(Failsafe_data[i+fromI]==FAILSAFE_CHANNEL_HOLD) {
                failsafeTemp[i]=FAILSAFE_CHANNEL_HOLD;
            } else {
                val = Failsafe_data[i+fromI];
                failsafeTemp[i] = (((val<<2)+val)>>3)+860; //value range 860<->2140 -125%<->+125%
            }    
        }
    }
    */
    static void ICACHE_RAM_ATTR3 MiLo_data_frame()
    {   
        static uint8_t lpass = 0;
        uint8_t j = 0;
        if ( miloFailsafePass == 1){
            packet[0] = FS1_8_PACKET;
            miloFailsafePass = 2;   // set value on 2 means that we do not have to send failsafe values;
                                    // when slot would be an uplink but there are no data, miloFailsafePass will be set on 3(in callback())
                                    // and so second part (9-16) can be sent 
        } else if ( miloFailsafePass == 3){
            j=8;
            packet[0] = FS9_16_PACKET;
            miloFailsafePass = 0; // reset when 2 failsafe packet have been sent
            FAILSAFE_VALUES_off;  // avoid to send other failsafe values as long it is not requested by hanset
        } else {   // this frame is not use for failsafe; for CH16 or EU16 sub protocol we alternate che channels
            if ( lpass & 1 ){
                j += 8 ;
                packet[0] = CH9_16_PACKET;
            } else{//lpass =0
                packet[0] = CH1_8_PACKET;
            }
            if(sub_protocol == SP8CH150HZ || sub_protocol == SP8CH100HZ) { //SP16CH150HZ=0, SP8CH150HZ=1, SP16CH100HZ=2, SP8CH100HZ=3
                lpass = 0 ; // send always the first 8 channels
            } else {
                lpass += 1 ; // alternate the 2 groups of channels
            }
        }
        #ifdef DEBUG_SEQUENCE
            debugln("RC%d dc=%d c1=%d c9=%d",packet[0],telemetry_counter, Channel_data[0], Channel_data[8]); 
        #endif
        if( LBTEnabled) { packet[0] |= ( 1<<3); }; //set EU LBT flag
        packet[0] |= ( (telemetry_counter<<4) & 0X30) ; // 2 bits (5..4) are the next downlink tlm counter
        #ifdef DEBUG_ON_GPIO3
            if (getCurrentChannelIdx() == 0) { // when the channel is the first one
                //G3PULSE(5);
        }  
        #endif      
        packet[1] = rx_tx_addr[3];
        packet[2] = rx_tx_addr[2];
        packet[3] =  RX_num & 0x3F ;//max 64 values
        if ( packet_count == 2) packet[3] |=  0x80 ; //when next packet will be a downlink, then mark it
        if (sub_protocol == WIFI_RX) packet[3] |= 0x40;//trigger WiFi updating firmware for RX
        packet[15] = getCurrentChannelIdx() & 0x3F ; // channel index is max 37 and so coded on 5 bits 
        if ( ( ( packet[0] & 0X07) == CH1_8_PACKET ) || ( ( packet[0] & 0X07) == CH9_16_PACKET ) ) {
            packet[4] = Channel_data[0+j]&0XFF ;
            packet[5] = Channel_data[0+j]>>8 | (Channel_data[1+j]&0xFF)<<3;
            packet[6] = Channel_data[1+j]>>5| Channel_data[2+j]<<6;
            packet[7] = (Channel_data[2+j]>>2)& 0x00FF;
            packet[8] = Channel_data[2+j]>>10|(Channel_data[3+j]&0xFF)<<1;
            packet[9] = Channel_data[3+j]>>7| (Channel_data[4+j]&0xFF)<<4;
            packet[10] = Channel_data[4+j]>>4|(Channel_data[5+j]&0xFF)<<7;
            packet[11] = (Channel_data[5+j]>>1)& 0x00FF;
            packet[12] = Channel_data[5+j]>>9|(Channel_data[6+j]&0xFF)<<2;
            packet[13] = Channel_data[6+j]>>6|(Channel_data[7+j]&0xFF)<<5;
            packet[14] = (Channel_data[7+j]>>3)& 0x00FF;
        } else {
            #ifdef FAILSAFE_ENABLE
                packet[4] = Failsafe_data[0+j]&0XFF ;
                packet[5] = Failsafe_data[0+j]>>8 | (Failsafe_data[1+j]&0xFF)<<3;
                packet[6] = Failsafe_data[1+j]>>5| Failsafe_data[2+j]<<6;
                packet[7] = (Failsafe_data[2+j]>>2)& 0x00FF;
                packet[8] = Failsafe_data[2+j]>>10|(Failsafe_data[3+j]&0xFF)<<1;
                packet[9] = Failsafe_data[3+j]>>7| (Failsafe_data[4+j]&0xFF)<<4;
                packet[10] = Failsafe_data[4+j]>>4|(Failsafe_data[5+j]&0xFF)<<7;
                packet[11] = (Failsafe_data[5+j]>>1)& 0x00FF;
                packet[12] = Failsafe_data[5+j]>>9|(Failsafe_data[6+j]&0xFF)<<2;
                packet[13] = Failsafe_data[6+j]>>6|(Failsafe_data[7+j]&0xFF)<<5;
                packet[14] = (Failsafe_data[7+j]>>3)& 0x00FF;
            #endif    
        }     
    }
    
    static void ICACHE_RAM_ATTR3 MiLo_Telemetry_frame()
    {   // fill an uplink tfm frame with 8 bytes from SportData only when an uplink must be send (checked just after preparing a RcData frame)
        packet[0] = ( (telemetry_counter<<4) & 0X30) | (TLM_PACKET) ; // telemetry_counter is used to manage downlink tlm sequence
        packet[1] = rx_tx_addr[3];
        packet[2] = rx_tx_addr[2];
        packet[3] = ( (uplinkTlmId & 0X03) << 6)  | (RX_num & 0x3F) ;//max 64 values
        if( uplinkTlmId == expectedUplinkTlmId )
        { // when the RX confirms that it get the previous uplink tlm
            expectedUplinkTlmId = (uplinkTlmId + 1) & 0x03;//2 bits ; increased only when sequence match                          
        }
        // Note:  SportToAck, SportTail and SportCount are update immediately when a downlink tlm frame is processed (in frsky_process_telemetry()_)
        memcpy( &packet[4], &SportData[SportToAck], 8 ) ; // copy 8 bytes 
        SportTail = (SportToAck + 8) & 0X3F;  
        #ifdef DEBUG_SEQUENCE
            debugln("Tlm dnC=%d  upSent=%d  upExp=%d c=%d a=%d t=%d h=%d sp=%d",\
                telemetry_counter , uplinkTlmId , expectedUplinkTlmId, SportCount, SportToAck, SportTail, SportHead, packet[8]);
        #endif
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
                if ( (sub_protocol == SP16CH100HZ) || (sub_protocol == SP8CH100HZ) ) {
                    MiLo_SetRFLinkRate(RATE_100HZ);
                } else { 
                    MiLo_SetRFLinkRate(RATE_150HZ);
                }
                #ifdef MILO_USE_LBT
                    LBTEnabled = true;  // to modify when we know how handset can (de) activate LBT
                #endif
                state = MiLo_DATA;
                MiLo_telem_init(); // initialise variables (flags) used by telemetry depending of SPORT_SEND and TELEMETRY
            }
            //SX1280_SetTxRxMode(TXRX_OFF);
            POWER_init();  // set power on min value.
            PayloadLength = NBR_BYTES_IN_PACKET;
            debugln("end of milo_init; case= %d",state); 
        }   
    }

    void MILO_prepareAndSendFrame(uint16_t frameType ){
        uint32_t now;
        packet_count = (packet_count + 1)%3;
        nextChannel(1);
        SX1280_SetFrequencyReg(GetCurrFreq());
        if (LBTEnabled) {   
            SX1280_SetTxRxMode(RX_EN);// LBTmode - BeginClearChannelAssessment
            SX1280_SetMode(SX1280_MODE_RX);//start RX mode in order to get RSSI 				
        }
        now = micros();
        if (frameType == MiLo_DATA) {
            MiLo_data_frame();
            G3PULSE(5);
        } else {
            MiLo_Telemetry_frame();
            G3PULSE(10); 
        }
        SX1280_WriteBuffer(0x00, packet,PayloadLength); //    
        if (LBTEnabled) {   
            while (( micros() - now ) < LBTdelay) { NOP();}
            if(!ChannelIsClear()) SX1280_SetOutputPower(MinPower);
            else SX1280_SetOutputPower(MaxPower);    
        } else {
            SX1280_SetOutputPower(MaxPower);
        }
        #if defined (DEBUG_UNUSED_TX_SLOTS) && defined (DEBUG_SKIP_TX_FROM_CHANNEL) && defined (DEBUG_SKIP_TX_UPTO_CHANNEL)
            if ( (getCurrentChannelIdx() <  DEBUG_SKIP_TX_FROM_CHANNEL) || (getCurrentChannelIdx() >  DEBUG_SKIP_TX_UPTO_CHANNEL) ) 
        #endif
            {    
                SX1280_SetTxRxMode(TX_EN);// do first to allow PA stablise
                SX1280_SetMode(SX1280_MODE_TX);
            }            
    }

    uint16_t MILO_callback()
    {   // this function is called at regular interval by main loop and manage all time slots for sending and receiving RF on SX1280
        uint16_t intervalMiloCallback;
        intervalMiloCallback = MiLo_currAirRate_Modparams->interval;  
   		LBTdelay = SpreadingFactorToRSSIvalidDelayUs(MiLo_currAirRate_Modparams->sf);
        #ifdef DEBUG_PACKET_COUNT
            //static uint8_t prevPacketCount = 0; 
            if (packet_count == 0) {G3PULSE(1);}
            else if (packet_count == 1) {G3PULSE(1);G3PULSE(1);}
            else if (packet_count == 2) {G3PULSE(1);G3PULSE(1);G3PULSE(1);}
            else { G3PULSE(50);}
        #endif
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
                #ifdef MILO_USE_LBT  // to change when we know how the handset will communicate if LBT is ON or OFF
                    LBTEnabled = true;
                #endif
                state = MiLo_DATA;
                break;
            case MiLo_DATA:
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
                #endif // end Wifi
                MILO_prepareAndSendFrame(state);  
                //debugln("start sending packet %d", packet_count);         
                
                // after having prepared sending the RCData frame, we have to determine the type of the next frame 
                if (packet_count == 2){// next frame is RX downlink temetry
                    state = MiLo_DWLNK_TLM1;
                    intervalMiloCallback = MiLo_currAirRate_Modparams->intervalBeforeDwnlnk1;
                    break;
                }
                else if( (packet_count == 1) && (upTLMcounter == 1) ) 
                {// The next slot can be used for uplink or for failsafe data
                // packet_count==1 means that next slot is just before a downlink tlm
                // upTLMcounter is incremented %2 each time a downlink tlm is send (in order to have max one uplink every 6 slot)
                    // There are data to send if 
                    //     - SportCount >1 or
                    //     - uplinkTlmId != ExpectedUplinkTlmId
                    if(SportCount > 0) 
                    { // SportCount is decreased when a downlink tlm frame is received and sequences matche
                        //debugln("SCount=%d  upTlmId=%d  expId=%d ", SportCount, uplinkTlmId, expectedUplinkTlmId);
                        state = MiLo_UPLNK_TLM;
                        break;
                    }       
                    #ifdef FAILSAFE_ENABLE
                        if (IS_FAILSAFE_VALUES_on){
                            miloFailsafePass++; // miloFailsafePass is use when a Rcframe is generated; 
                                            // 0=no failsafe to send , 1= send failsafe 1_8 , 2 = wait for a slot, 3 = send failsafe 9_16
                        }
                    #endif        
                }       
                state = MiLo_DATA; // else continue with a RCData frame
                break;      
            case MiLo_UPLNK_TLM:    //Uplink telemetry
                MILO_prepareAndSendFrame(state);
                state = MiLo_DWLNK_TLM1;// next frame is RX downlink temetry
                intervalMiloCallback = MiLo_currAirRate_Modparams->intervalBeforeDwnlnk1;//
                break;      
            case MiLo_DWLNK_TLM1://downlink telemetry
                SX1280_SetTxRxMode(RX_EN);// do first to enable LNA
                SX1280_SetMode(SX1280_MODE_RX);
                packet_count = (packet_count + 1)%3;
                upTLMcounter = (upTLMcounter + 1) &0X01; //increment using downlink TLM clock in order to increment only once per 3 slots
                state = MiLo_DWLNK_TLM2;
                intervalMiloCallback = MiLo_currAirRate_Modparams->intervalBeforeDwnlnk2;
                break;
            case MiLo_DWLNK_TLM2:
                if(frameReceived)
                {
                    G3PULSE(20);
                    frameReceived = false;
                    uint8_t const FifoRxAddr = SX1280_GetRxBufferAddr();
                    SX1280_ReadBuffer(FifoRxAddr, packet_in, PayloadLength);
                    if( (packet_in[0] & 0xFC) == ( rx_tx_addr[3] & 0XFC) &&
                        (packet_in[1] & 0xFC) == ( rx_tx_addr[2] & 0XFC) ){   // check it is a frame for the right handset (only on 6 bits MSB) 
                        SX1280_GetLastPacketStats();     // read SX1280 to get LastPacketRSSI and LastPacketSNR (data are not in the received packed)
                        telemetry_link|=1;               // Telemetry data is available
                        G3PULSE(10);
                        frsky_process_telemetry(packet_in, PayloadLength); //check if valid telemetry packets
                        G3PULSE(10);
                        LQICalc();
                        memset(&packet_in[0], 0, PayloadLength ); // reset packet_in[]              
                    }
                }
                else{
                    G3PULSE(30);
                    miloSportStart = false;
                    ThisPacketDropped = 1;
                }
                state = MiLo_DATA;
                intervalMiloCallback =  MiLo_currAirRate_Modparams->intervalAfterDwnlnk2;
                break;        
        } // end switch       
        return intervalMiloCallback;        
    }
    
    void ICACHE_RAM_ATTR dioISR()
    {
        //dioOccured = true ;
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

    uint32_t ICACHE_RAM_ATTR3 SpreadingFactorToRSSIvalidDelayUs(uint8_t SF)
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
        
    bool ICACHE_RAM_ATTR3 ChannelIsClear(void)
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

