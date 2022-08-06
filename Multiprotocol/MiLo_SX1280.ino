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
	
	#include "FHSS.h"
	#include "iface_sx1280.h"
	#include "SX1280.h"
	#define RATE_DEFAULT 0 
	#define RATE_BINDING 0  
	#define RATE_100HZ 1 //100HZ
	#define RATE_150HZ 0 //150HZ
	#define RATE_MAX 3
	enum
	{
		PWR_10mW = 0,
		PWR_25mW = 1,
		PWR_50mW = 2,
		PWR_100mW = 3,
		PWR_250mW = 4,
		PWR_500mW = 5,
		PWR_COUNT = 6,
	} ;
	
	#if defined  HM_ES24TXH
		#define MinPower PWR_10mW
		#define MaxPower PWR_250mW
		static const int16_t powerValues[PWR_COUNT] = {-17,-13,-9,-6,-2} ;//10,25,50,100,250
		#elif defined BETAFPV_500
		#define MinPower PWR_10mW
		#define MaxPower PWR_500mW
		static const int16_t powerValues[PWR_COUNT] = {-18,-15,-13,-9,-4,3} ;//10,25,50,100,250,500
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
		MiLo_BIND		= 0,
		MiLo_BIND_DONE	= 1500,
		MiLo_DATA1,
		MiLo_DATA2,
		MiLo_UPLNK_TLM,
		MiLo_DWLNK_TLM1,
		MiLo_DWLNK_TLM2
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
	

	void  MiLo_SetRFLinkRate(uint8_t index) // Set speed of RF link (hz) index values
	{
		
		MiLo_mod_settings_s *const ModParams = &MiLo_AirRateConfig[index];
		MiLo_rf_pref_params_s *const RFperf = &MiLo_AirRateRFperf[index];
		
		bool invertIQ = 0x01;//inverted
		if ((ModParams == MiLo_currAirRate_Modparams)
			&& (RFperf == MiLo_currAirRate_RFperfParams)
		&& (invertIQ == IQinverted))
		return;
		uint32_t interval = ModParams->interval;
		
		SX1280_Config(ModParams->bw, ModParams->sf, ModParams->cr, GetCurrFreq(),
		ModParams->PreambleLen, invertIQ, ModParams->PayloadLength, ModParams->interval);
		
		MiLo_currAirRate_Modparams = ModParams;
		MiLo_currAirRate_RFperfParams = RFperf;
		
	}
	
	
	static void ICACHE_RAM_ATTR MiLo_telem_init(void)
	{
		#ifdef SPORT_SEND
			SportHead = SportTail=0;			// empty data buffer
			idxOK = 0;	
		#endif
		#ifdef TELEMETRY
			telemetry_lost = 1;
			telemetry_link = 0;					//Stop sending telemetry
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
		
		packet[0] |= (telemetry_counter<< 3);	
		packet[1] = rx_tx_addr[3];
		packet[2] = rx_tx_addr[2];
		packet[3] =  (packet_count == 2) ? RX_num | 0x80 : RX_num & 0x3F ;//max 64 values
		if(packet_count != 2){
			if (sub_protocol & 3 == WIFI_RX)
			packet[3] = RX_num & 0x7F;//trigger WiFi updating firmware for RX
		}
		
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
		
		if(sub_protocol & 1 )// in M16/CH1-8 mode send only 8ch every interval us
		lpass = 0 ;
		else
		lpass += 1 ;
		
	}
	
	static void __attribute__((unused)) FrSkyX_send_sport(uint8_t start, uint8_t end)
	{
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
			packet[start] = nbr_bytes|((TelemetryId &0x0F)<<4);
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
		FrSkyX_send_sport(3 , PayloadLength - 1);
		packet[0] = (TLM_PACKET) |(telemetry_counter<<3);
		packet[1] = rx_tx_addr[3];
		packet[2] = rx_tx_addr[2];	
	}
	
	void ICACHE_RAM_ATTR MILO_init()
	{
		Fhss_Init();
		Fhss_generate(MProtocol_id);
		currFreq = GetInitialFreq(); //set frequency first or an error will occur!!!
		currOpmode = SX1280_MODE_SLEEP;
		bool init_success = SX1280_Begin();
		if (!init_success)
		{
			
			return ;		
		}
		else
		{
			if(IS_BIND_IN_PROGRESS)
			{
				while(!chanskip)
				chanskip = random(0xfefefefe)%68;
				
				chanskip &= 0x0F;
				is_in_binding = true;
				MiLo_SetRFLinkRate(RATE_BINDING);
				state = MiLo_BIND;
				
			}
			else
			{
				packet_count = 0;
				is_in_binding = false;
				//if(sub_protocol & 3)
				// MiLo_SetRFLinkRate(RATE_100HZ);
				//else 
				MiLo_SetRFLinkRate(RATE_150HZ);
				state = MiLo_DATA1;
				MiLo_telem_init();
			}
			SX1280_SetTxRxMode(TXRX_OFF);
			POWER_init();
			PayloadLength = MiLo_currAirRate_Modparams->PayloadLength;
		}	
	}
		
	uint16_t ICACHE_RAM_ATTR MILO_callback()
	{
		static uint16_t interval = MiLo_currAirRate_Modparams->interval;
		static uint8_t TLMinterval = MiLo_currAirRate_Modparams->TLMinterval;
		static uint16_t TOA = MiLo_currAirRate_RFperfParams->TOA;
		static uint32_t upTLMcounter = 2;
		switch(state)
		{	
			default :
			SX1280_SetFrequencyReg(currFreq);//middle of the band		   
			MiLo_build_bind_packet();
			SX1280_setPower(MinPower);
			SX1280_SetTxRxMode(TX_EN);// do first to allow PA stablise		
			SX1280_WriteBuffer(0x00, packet,PayloadLength);//
			SX1280_SetMode(SX1280_MODE_TX);
			if(IS_BIND_DONE)
			state = MiLo_BIND_DONE;
			else
			state++;
			break;
			case MiLo_BIND_DONE:
			packet_count = 0;
			is_in_binding = false;
			BIND_DONE;
			state = MiLo_DATA1;				
			case MiLo_DATA1:
			//SX1280_setPower(PWR_100mW);
			packet_count = (packet_count + 1)%3;
			MiLo_data_frame();
			nextChannel(1);
			SX1280_SetFrequencyReg(GetCurrFreq());
			SX1280_WriteBuffer(0x00, packet,PayloadLength); //
			SX1280_SetTxRxMode(TX_EN);// do first to allow PA stablise
			SX1280_SetMode(SX1280_MODE_TX);
			frameType = packet[0] & 0x07 ;
			
			if (packet_count == 2){// next frame is RX downlink temetry
				state = MiLo_DWLNK_TLM1;
				return 5400;
			}
			else{
				if(SportHead != SportTail && upTLMcounter == 2){//next frame in uplink telemetry
					state = MiLo_UPLNK_TLM;
					upTLMcounter  = 0;//reset uplink telemetry counter
					break;
				}		
			}		
			state = MiLo_DATA1;			
			break;		
			case MiLo_UPLNK_TLM:	//Uplink telemetry
			packet_count = (packet_count + 1)%3;	
			MiLo_Telemetry_frame();
			nextChannel(1);
			SX1280_SetFrequencyReg(GetCurrFreq());		
			SX1280_WriteBuffer(0x00, packet, PayloadLength); 
			SX1280_SetMode(SX1280_MODE_TX);	
			state = MiLo_DWLNK_TLM1;// next frame is RX downlink temetry
			return 5400;//
			case MiLo_DWLNK_TLM1://downlink telemetry
			nextChannel(1);
			SX1280_SetFrequencyReg(GetCurrFreq());
			SX1280_SetTxRxMode(RX_EN);// do first to enable LNA
			SX1280_SetMode(SX1280_MODE_RX);
			packet_count = (packet_count + 1)%3;
			
			if(SportHead != SportTail)
			upTLMcounter++;	//increment using downlink TLM clock
			else
			upTLMcounter = 0;//reset counter
			
			state = MiLo_DWLNK_TLM2;
			return 7600; 
			case MiLo_DWLNK_TLM2:
			if(frameReceived)
			{
				uint8_t const FIFOaddr = SX1280_GetRxBufferAddr();
				SX1280_ReadBuffer(FIFOaddr, packet_in, PayloadLength);
				if((packet_in[0] == rx_tx_addr[3])&&packet_in[1] == rx_tx_addr[2])
				{	
					SX1280_GetLastPacketStats();
					frsky_process_telemetry(packet_in, PayloadLength);//check if valid telemetry packets
					memset(&packet_in[0], 0, PayloadLength );				
					frameReceived = false;
				}
			}
			
			state = MiLo_DATA1;
			return 1000;		
		}		
		return interval;		
	}
	
	void ICACHE_RAM_ATTR dioISR()
	{
		
		uint16_t irqStatus = SX1280_GetIrqStatus();
		
		SX1280_ClearIrqStatus(SX1280_IRQ_RADIO_ALL);
		
		if (irqStatus & SX1280_IRQ_TX_DONE)
		{
			SX1280_SetTxRxMode(TXRX_OFF);//TX_EN off and RX_EN pins off
			currOpmode = SX1280_MODE_FS; // radio goes to FS after TX
			
			
		}
		else
		if (irqStatus & (SX1280_IRQ_RX_DONE | SX1280_IRQ_CRC_ERROR | SX1280_IRQ_RX_TX_TIMEOUT))
		{
			uint8_t const fail =
			((irqStatus & SX1280_IRQ_CRC_ERROR) ? SX1280_RX_CRC_FAIL : SX1280_RX_OK) +
			((irqStatus & SX1280_IRQ_RX_TX_TIMEOUT) ? SX1280_RX_TIMEOUT : SX1280_RX_OK);
			// In continuous receive mode, the device stays in Rx mode
			if (timeout != 0xFFFF)
			{
				// From table 11-28, pg 81 datasheet rev 3.2
				// upon successsful receipt, when the timer is active or in single mode, it returns to STDBY_RC
				// but because we have AUTO_FS enabled we automatically transition to state SX1280_MODE_FS
				currOpmode = SX1280_MODE_FS;
			}
			if (fail == SX1280_RX_OK)
			{
				frameReceived = true;
			}	
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
	
	- Normal frame channels 1-8; frame rate 7ms
	0. Frame type(3bits) |telemetry down link frame counter(sequence) 5 bits(0-31)
	1. txid1 TXID on 16 bits
	2. txid2
	3. Model ID /Rx_Num(6 bits) | 1 bit flag activate Rx wifi|1bit for sync telem frame
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
	
	- Normal frame channels 9-16 separate; frame rate 7ms
	0. Frame type (3bits) | telemetry downlink frame counter(sequence) 5 bits(0-31)
	1. txid1 TXID on 16 bits
	2. txid2
	3.Model ID /Rx_Num 6 bits|1 bit flag activate Rx wifi|1bit for sync telem frame
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
	
	- TX uplink telemetry frame can be sent separate ;frame rate 7ms;1:6 telemetry data rate
	0. Frame type (3bits) | telemetry down link frame counter(sequence) 5 bits(0-31)
	1.txid1
	2.txid2
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
	
	- RX downlink telemetry frame sent separate at a fixed rate of 1:3;frame rate 7ms,
	
	0.txid1
	1.txid2
	2.RSSI/LQI/SNR/RXV alternate every ~80 ms update for each data
	3.telemetry frame counter(5bits)|3bits ID link data packet(RSSI/SNR /LQI)
	4.No. of bytes in sport frame(4 bits)|telemetry uplink counter sequence(4 bits)
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
	
	- Frame Sequence
	
	1- RC channels 1_8_1 
	2- RC channels 9_16
	3- downlink telemetry
	4- RC channels 1_8_1
	5 -uplink telemetry                 
	6- downlink telemetry
	7- RC channels 9_16
	8- RC channels 1_8_1
	9- downlink telemetry
	10- RC channels 9_16
	11- uplink telemetry               
	12- downlink telemetry
	13- RC channels 1_8_1
	14- RC channels 9_16
	15- downlink telemetry
	16- RC channels 1_8_1
	17  uplink telemetry 
	
	
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



