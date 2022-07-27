/**
 * OnDemandNonBlocking.ino
 * example of running the webportal or configportal manually and non blocking
 * trigger pin will start a webportal for 120 seconds then turn it off.
 * startAP = true will start both the configportal AP and webportal
 */
#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager

// include MDNS
#ifdef ESP8266
#include <ESP8266mDNS.h>
#elif defined(ESP32)
#include <ESPmDNS.h>
#endif

WiFiManager wm;
unsigned long previousMillis = 0;  
unsigned int  time_out   = 120; // seconds to run for
unsigned int  startTime;
bool portalRunning      = false;
bool startAP            = true; // start AP and webserver if true, else start only webserver
void doWiFiManager(void);

bool first_time = true;
bool second_time = true;

void setupWifiManager(){
startTime = millis();
  WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP  
  // put your setup code here, to run once
 // Serial.begin(115200);
  //Serial.setDebugOutput(true);
  //delay(1000);
  //Serial.println("\n Starting");
  wm.setHostname("MILO_TX");
  wm.autoConnect();
}


void startWifiManager() {//in loop
if(first_time){//only one time setup
setupWifiManager();
first_time = false;
}
//while(1){
  #ifdef ESP8266_PLATFORM
  MDNS.update();
  #endif
  doWiFiManager();
 // put your main code here, to run repeatedly:
//break;
//}
}



void doWiFiManager(){
  // is auto timeout portal running
  if(portalRunning){
    wm.process(); // do processing

    // check for timeout
    if((millis()-startTime) > (time_out*1000)){
      //Serial.println("portaltimeout");
      portalRunning = false;
      if(startAP){
        wm.stopConfigPortal();
      }
      else{
        wm.stopWebPortal();
      } 
   }
  }

if(second_time){  // is configuration portal requested?trigger
	
  if(!portalRunning){
    if(startAP){
      //Serial.println(" Starting Config Portal");
      wm.setConfigPortalBlocking(false);
      wm.startConfigPortal();
    }  
    else{
     // Serial.println("Starting Web Portal");
      wm.startWebPortal();
    }  
    portalRunning = true;
    startTime = millis();
  }
  second_time  = false;
}
}

