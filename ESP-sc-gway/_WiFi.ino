// 1-channel LoRa Gateway for ESP8266
// Copyright (c) 2016, 2017, 2018 Maarten Westenberg version for ESP8266
// Version 5.2.1
// Date: 2018-06-06
//
// 	based on work done by Thomas Telkamp for Raspberry PI 1ch gateway
//	and many others.
//
// All rights reserved. This program and the accompanying materials
// are made available under the terms of the MIT License
// which accompanies this distribution, and is available at
// https://opensource.org/licenses/mit-license.php
//
// Author: Maarten Westenberg (mw12554@hotmail.com)
//
// This file contains the LoRa filesystem specific code




// ----------------------------------------------------------------------------
// config.txt is a text file that contains lines(!) with WPA configuration items
// Each line contains an KEY vaue pair describing the gateway configuration
//
// ----------------------------------------------------------------------------
int WlanReadWpa() {
	
	readConfig( CONFIGFILE, &gwayConfig);

	if (gwayConfig.sf != (uint8_t) 0) sf = (sf_t) gwayConfig.sf;
	ifreq = gwayConfig.ch;
	debug = gwayConfig.debug;
	pdebug = gwayConfig.pdebug;
	_cad = gwayConfig.cad;
	_hop = gwayConfig.hop;
	gwayConfig.boots++;							// Every boot of the system we increase the reset
	
#if GATEWAYNODE==1
	if (gwayConfig.fcnt != (uint8_t) 0) frameCount = gwayConfig.fcnt+10;
#endif
	
#if WIFIMANAGER==1
	String ssid=gwayConfig.ssid;
	String pass=gwayConfig.pass;

	char ssidBuf[ssid.length()+1];
	ssid.toCharArray(ssidBuf,ssid.length()+1);
	char passBuf[pass.length()+1];
	pass.toCharArray(passBuf,pass.length()+1);
	Serial.print(F("WlanReadWpa: ")); Serial.print(ssidBuf); Serial.print(F(", ")); Serial.println(passBuf);
	
	strcpy(wpa[0].login, ssidBuf);				// XXX changed from wpa[0][0] = ssidBuf
	strcpy(wpa[0].passw, passBuf);
	
	Serial.print(F("WlanReadWpa: <")); 
	Serial.print(wpa[0].login); 				// XXX
	Serial.print(F(">, <")); 
	Serial.print(wpa[0].passw);
	Serial.println(F(">"));
#endif

}


// ----------------------------------------------------------------------------
// Print the WPA data of last WiFiManager to file
// ----------------------------------------------------------------------------
#if WIFIMANAGER==1
int WlanWriteWpa( char* ssid, char *pass) {

#if DUSB>=1
	if ( debug >=0 ) && ( pdebug & P_MAIN )) {
		Serial.print(F("WlanWriteWpa:: ssid=")); 
		Serial.print(ssid);
		Serial.print(F(", pass=")); 
		Serial.print(pass); 
		Serial.println();
	}
#endif
	// Version 3.3 use of config file
	String s((char *) ssid);
	gwayConfig.ssid = s;
	
	String p((char *) pass);
	gwayConfig.pass = p;

#if GATEWAYNODE==1	
	gwayConfig.fcnt = frameCount;
#endif
	gwayConfig.ch = ifreq;
	gwayConfig.sf = sf;
	gwayConfig.cad = _cad;
	gwayConfig.hop = _hop;
	
	writeConfig( CONFIGFILE, &gwayConfig);
	return 1;
}
#endif



// ----------------------------------------------------------------------------
// Function to join the Wifi Network
//	It is a matter of returning to the main loop() asap and make sure in next loop
//	the reconnect is done first thing. By default the system will reconnect to the
// samen SSID as it was connected to before.
// Parameters:
//		int maxTry: Number of reties we do:
//		0: Try forever. Which is normally what we want except for Setup maybe
//		1: Try once and if unsuccessful return(1);
//		x: Try x times
//
//  Returns:
//		On failure: Return -1
//		int number of retries necessary
//
//  XXX After a few retries, the ESP8266 should be reset. Note: Switching between 
//	two SSID's does the trick. Rettrying the same SSID does not.
// Workaround is found below: Let the ESP8266 forget the SSID
// ----------------------------------------------------------------------------
int WlanConnect(int maxTry) {
  
#if WIFIMANAGER==1
	WiFiManager wifiManager;
#endif

	unsigned char agains = 0;
	unsigned char wpa_index = (WIFIMANAGER >0 ? 0 : 1);		// Skip over first record for WiFiManager
	
	// So try to connect to WLAN as long as we are not connected.
	// The try parameters tells us how many times we try before giving up
	int i=0;
	
	if (WiFi.status() == WL_CONNECTED) return(1);

	// We try 5 times before giving up on connect
	while ( (WiFi.status() != WL_CONNECTED) && ( i< maxTry ) )
	{

	  // We try every SSID in wpa array until success
	  for (int j=wpa_index; (j< (sizeof(wpa)/sizeof(wpa[0]))) && (WiFi.status() != WL_CONNECTED ); j++)
	  {
		// Start with well-known access points in the list
		char *ssid		= wpa[j].login;
		char *password	= wpa[j].passw;
#if DUSB>=1
		Serial.print(i);
		Serial.print(':');
		Serial.print(j); 
		Serial.print(F(". WiFi connect SSID=")); 
		Serial.print(ssid);
		if (( debug>=1 ) && ( pdebug & P_MAIN )) {
			Serial.print(F(", pass="));
			Serial.print(password);
		}
		Serial.println();
#endif		
		// Count the number of times we call WiFi.begin
		gwayConfig.wifis++;

		//WiFi.disconnect();
		delay(1000);

		WiFi.persistent(false);
		WiFi.mode(WIFI_OFF);   // this is a temporary line, to be removed after SDK update to 1.5.4
		WiFi.mode(WIFI_STA);
		WiFi.begin(ssid, password);
		
		delay(9000);
		
		// We increase the time for connect but try the same SSID
		// We try for 10 times
		agains=1;
		while (((WiFi.status()) != WL_CONNECTED) && (agains < 10)) {
			agains++;
			delay(agains*500);
#if DUSB>=1
			if (( debug>=0 ) && ( pdebug & P_MAIN ))
				Serial.print(".");
#endif
		}
		
		// Check the connection status again
		switch (WiFi.status()) {
			case WL_CONNECTED:
#if DUSB>=1
				if (( debug>=0 ) && ( pdebug & P_MAIN ))
					Serial.println(F("WlanConnect:: CONNECTED"));				// 3
#endif
				return(1);
				break;
			case WL_IDLE_STATUS:
#if DUSB>=1
				if (( debug>=0 ) && ( pdebug & P_MAIN ))
					Serial.println(F("WlanConnect:: IDLE"));						// 0
#endif
				break;
			case WL_NO_SSID_AVAIL:
#if DUSB>=1
				Serial.println(F("WlanConnect:: NO SSID"));						// 1
#endif
				break;
			case WL_CONNECT_FAILED:
#if DUSB>=1
				if (( debug>=0 ) && ( pdebug & P_MAIN ))
					Serial.println(F("WlanConnect:: FAILED"));						// 4
#endif
				break;
			case WL_DISCONNECTED:
#if DUSB>=1
				if (( debug>=0 ) && ( pdebug & P_MAIN ))
					Serial.println(F("WlanConnect:: DISCONNECTED"));				// 6
#endif				
				break;
			case WL_SCAN_COMPLETED:
#if DUSB>=1
				if (( debug>=0 ) && ( pdebug & P_MAIN ))
					Serial.println(F("WlanConnect:: SCAN COMPLETE"));				// 2
#endif
				break;
			case WL_CONNECTION_LOST:
#if DUSB>=1
				if (( debug>=0 ) && ( pdebug & P_MAIN ))
					Serial.println(F("WlanConnect:: LOST"));						// 5
#endif
				break;
			default:
#if DUSB>=1
				if (( debug>=0 ) && ( pdebug & P_MAIN )) {
					Serial.print(F("WlanConnect:: code="));
					Serial.println(WiFi.status());
				}
#endif
				break;
		}

	  } //for
	  i++;													// Number of times we try to connect
	} //while

	// It should not be possible to be here while WL_CONNECTed
	if (WiFi.status() == WL_CONNECTED) {
#if DUSB>=1
		if (( debug>=3 ) && ( pdebug & P_MAIN )) {
			Serial.print(F("WLAN connected"));
			Serial.println();
		}
#endif
		writeGwayCfg(CONFIGFILE);
		return(1);
	}
	else {
#if WIFIMANAGER==1
#if DUSB>=1
		Serial.println(F("Starting Access Point Mode"));
		Serial.print(F("Connect Wifi to accesspoint: "));
		Serial.print(AP_NAME);
		Serial.print(F(" and connect to IP: 192.168.4.1"));
		Serial.println();
#endif
		wifiManager.autoConnect(AP_NAME, AP_PASSWD );
		//wifiManager.startConfigPortal(AP_NAME, AP_PASSWD );
		// At this point, there IS a Wifi Access Point found and connected
		// We must connect to the local SPIFFS storage to store the access point
		//String s = WiFi.SSID();
		//char ssidBuf[s.length()+1];
		//s.toCharArray(ssidBuf,s.length()+1);
		// Now look for the password
		struct station_config sta_conf;
		wifi_station_get_config(&sta_conf);

		//WlanWriteWpa(ssidBuf, (char *)sta_conf.password);
		WlanWriteWpa((char *)sta_conf.ssid, (char *)sta_conf.password);
#else
#if DUSB>=1
		if (( debug>=0) && ( pdebug & P_MAIN )) {
			Serial.println(F("WlanConnect:: Not connected after all"));
			Serial.print(F("WLAN retry="));
			Serial.print(i);
			Serial.print(F(" , stat="));
			Serial.print(WiFi.status() );						// Status. 3 is WL_CONNECTED
			Serial.println();
		}
#endif // DUSB
		return(-1);
#endif
	}

  
	yield();
  
	return(1);
}
