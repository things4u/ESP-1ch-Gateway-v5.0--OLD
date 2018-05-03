// 1-channel LoRa Gateway for ESP8266
// Copyright (c) 2016, 2017 Maarten Westenberg version for ESP8266
// Version 5.0.9
// Date: 2018-04-07
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


// ============================================================================
// LORA SPIFFS FILESYSTEM FUNCTIONS
//
// The LoRa supporting functions are in the section below



// ----------------------------------------------------------------------------
// Read the gateway configuration file
// ----------------------------------------------------------------------------
int readConfig(const char *fn, struct espGwayConfig *c) {

	int tries = 0;
	Serial.print(F("readConfig:: Starting "));

	if (!SPIFFS.exists(fn)) {
		Serial.print(F("ERROR:: readConfig, file="));
		Serial.print(fn);
		Serial.print(F(" does not exist .. Formatting"));
		SPIFFS.format();
		return(-1);
	}

	File f = SPIFFS.open(fn, "r");
	if (!f) {
		Serial.println(F("ERROR:: SPIFFS open failed"));
		return(-1);
	}

	while (f.available()) {
		
#if DUSB>=1
		if (debug>=0) {
			Serial.print('.');
		}
#endif
		// If we wait for more than 10 times, reformat the filesystem
		// We do this so that the system will be responsive (over OTA for example).
		//
		if (tries >= 10) {
			f.close();
			Serial.println(F("Formatting"));
			SPIFFS.format();
			f = SPIFFS.open(fn, "r");
			tries = 0;
		}
		
		String id =f.readStringUntil('=');						// C++ thing
		String val=f.readStringUntil('\n');
		
		if (id == "SSID") {										// WiFi SSID
			Serial.print(F("SSID=")); Serial.println(val);
			(*c).ssid = val;									// val contains ssid, we do NO check
		}
		//else if (id == "PASS") { 								// WiFi Password
		//	Serial.print(F("PASS=")); Serial.println(val); 
		//	(*c).pass = val;
		//}
		else if (id == "CH") { 									// Frequency Channel
			Serial.print(F("CH=")); Serial.println(val); 
			(*c).ch = (uint32_t) val.toInt();
		}
		else if (id == "SF") { 									// Spreading Factor
			Serial.print(F("SF  =")); Serial.println(val);
			(*c).sf = (uint32_t) val.toInt();
		}
		else if (id == "FCNT") {								// Frame Counter
			Serial.print(F("FCNT=")); Serial.println(val);
			(*c).fcnt = (uint32_t) val.toInt();
		}
		else if (id == "DEBUG") {								// Frame Counter
			Serial.print(F("DEBUG=")); Serial.println(val);
			(*c).debug = (uint8_t) val.toInt();
		}
		else if (id == "CAD") {									// CAD setting
			Serial.print(F("CAD=")); Serial.println(val);
			(*c).cad = (uint8_t) val.toInt();
		}
		else if (id == "HOP") {									// HOP setting
			Serial.print(F("HOP=")); Serial.println(val);
			(*c).hop = (uint8_t) val.toInt();
		}
		else if (id == "BOOTS") {								// BOOTS setting
			Serial.print(F("BOOTS=")); Serial.println(val);
			(*c).boots = (uint8_t) val.toInt();
		}
		else if (id == "RESETS") {								// RESET setting
			Serial.print(F("RESETS=")); Serial.println(val);
			(*c).resets = (uint8_t) val.toInt();
		}
		else if (id == "WIFIS") {								// WIFIS setting
			Serial.print(F("WIFIS=")); Serial.println(val);
			(*c).wifis = (uint8_t) val.toInt();
		}
		else if (id == "VIEWS") {								// VIEWS setting
			Serial.print(F("VIEWS=")); Serial.println(val);
			(*c).views = (uint8_t) val.toInt();
		}
		else if (id == "NODE") {								// NODE setting
			Serial.print(F("NODE=")); Serial.println(val);
			(*c).isNode = (uint8_t) val.toInt();
		}
		else if (id == "REFR") {								// REFR setting
			Serial.print(F("REFR=")); Serial.println(val);
			(*c).refresh = (uint8_t) val.toInt();
		}
		else if (id == "REENTS") {								// REENTS setting
			Serial.print(F("REENTS=")); Serial.println(val);
			(*c).reents = (uint8_t) val.toInt();
		}
		else if (id == "NTPERR") {								// NTPERR setting
			Serial.print(F("NTPERR=")); Serial.println(val);
			(*c).ntpErr = (uint8_t) val.toInt();
		}
		else if (id == "NTPETIM") {								// NTPERR setting
			Serial.print(F("NTPETIM=")); Serial.println(val);
			(*c).ntpErrTime = (uint32_t) val.toInt();
		}
		else if (id == "NTPS") {								// NTPS setting
			Serial.print(F("NTPS=")); Serial.println(val);
			(*c).ntps = (uint8_t) val.toInt();
		}
		else if (id == "FILENO") {								// log FILENO setting
			Serial.print(F("FILENO=")); Serial.println(val);
			(*c).logFileNo = (uint8_t) val.toInt();
		}
		else if (id == "FILEREC") {								// FILEREC setting
			Serial.print(F("FILEREC=")); Serial.println(val);
			(*c).logFileRec = (uint8_t) val.toInt();
		}
		else if (id == "FILENUM") {								// FILEREC setting
			Serial.print(F("FILENUM=")); Serial.println(val);
			(*c).logFileNum = (uint8_t) val.toInt();
		}
		else {
			tries++;
		}
	}
	f.close();
#if DUSB>=1
	if (debug>=0) {
		Serial.println('#');
	}
#endif
	Serial.println();
	return(1);
}

// ----------------------------------------------------------------------------
// Write the current gateway configuration to SPIFFS. First copy all the
// separate data items to the gwayConfig structure
//
// ----------------------------------------------------------------------------
int writeGwayCfg(const char *fn) {

	gwayConfig.sf = (uint8_t) sf;
	gwayConfig.ssid = WiFi.SSID();
	//gwayConfig.pass = WiFi.PASS();					// XXX We should find a way to store the password too
	gwayConfig.ch = ifreq;
	gwayConfig.debug = debug;
	gwayConfig.cad = _cad;
	gwayConfig.hop = _hop;
#if GATEWAYNODE==1
	gwayConfig.fcnt = frameCount;
#endif

	return(writeConfig(fn, &gwayConfig));
}

// ----------------------------------------------------------------------------
// Write the configuration as found in the espGwayConfig structure
// to SPIFFS
// ----------------------------------------------------------------------------
int writeConfig(const char *fn, struct espGwayConfig *c) {

	if (!SPIFFS.exists(fn)) {
		Serial.print("WARNING:: writeConfig, file not exists, formatting ");
		SPIFFS.format();
		// XXX make all initial declarations here if config vars need to have a value
		Serial.println(fn);
	}
	File f = SPIFFS.open(fn, "w");
	if (!f) {
		Serial.print("ERROR:: writeConfig, open file=");
		Serial.print(fn);
		Serial.println();
		return(-1);
	}

	f.print("SSID"); f.print('='); f.print((*c).ssid); f.print('\n'); 
	f.print("PASS"); f.print('='); f.print((*c).pass); f.print('\n');
	f.print("CH"); f.print('='); f.print((*c).ch); f.print('\n');
	f.print("SF");   f.print('='); f.print((*c).sf);   f.print('\n');
	f.print("FCNT"); f.print('='); f.print((*c).fcnt); f.print('\n');
	f.print("DEBUG"); f.print('='); f.print((*c).debug); f.print('\n');
	f.print("CAD");  f.print('='); f.print((*c).cad); f.print('\n');
	f.print("HOP");  f.print('='); f.print((*c).hop); f.print('\n');
	f.print("NODE");  f.print('='); f.print((*c).isNode); f.print('\n');
	f.print("BOOTS");  f.print('='); f.print((*c).boots); f.print('\n');
	f.print("RESETS");  f.print('='); f.print((*c).resets); f.print('\n');
	f.print("WIFIS");  f.print('='); f.print((*c).wifis); f.print('\n');
	f.print("VIEWS");  f.print('='); f.print((*c).views); f.print('\n');
	f.print("REFR");  f.print('='); f.print((*c).refresh); f.print('\n');
	f.print("REENTS");  f.print('='); f.print((*c).reents); f.print('\n');
	f.print("NTPETIM");  f.print('='); f.print((*c).ntpErrTime); f.print('\n');
	f.print("NTPERR");  f.print('='); f.print((*c).ntpErr); f.print('\n');
	f.print("NTPS");  f.print('='); f.print((*c).ntps); f.print('\n');
	f.print("FILEREC");  f.print('='); f.print((*c).logFileRec); f.print('\n');
	f.print("FILENO");  f.print('='); f.print((*c).logFileNo); f.print('\n');
	f.print("FILENUM");  f.print('='); f.print((*c).logFileNum); f.print('\n');
	
	f.close();
	return(1);
}

// ----------------------------------------------------------------------------
// Add a line with statitics to the log.
//
// We put the check in the function to protect against calling 
// the function without STAT_LOG being proper defined
// ToDo: Store the fileNo and the fileRec in the status file to save for 
// restarts
// ----------------------------------------------------------------------------
void addLog(const unsigned char * line, int cnt) 
{
#if STAT_LOG == 1
	char fn[16];
	
	if (gwayConfig.logFileRec > LOGFILEREC) {		// Have to make define for this
		gwayConfig.logFileRec = 0;					// In new logFile start with record 0
		gwayConfig.logFileNo++;						// Increase file ID
		gwayConfig.logFileNum++;					// Increase number of log files
	}
	gwayConfig.logFileRec++;
	
	// If we have too many logfies, delete the oldest
	//
	if (gwayConfig.logFileNum > LOGFILEMAX){
		sprintf(fn,"/log-%d", gwayConfig.logFileNo - LOGFILEMAX);
#if DUSB>=1
		Serial.print(F("addLog:: Too many logfile, deleting="));
		Serial.println(fn);
#endif
		SPIFFS.remove(fn);
		gwayConfig.logFileNum--;
	}
	
	// Make sure we have the right fileno
	sprintf(fn,"/log-%d", gwayConfig.logFileNo); 
	
	// If there is no SPIFFS, Error
	// Make sure to write the config record also
	if (!SPIFFS.exists(fn)) {
#if DUSB>=1
		Serial.print(F("ERROR:: addLog:: file="));
		Serial.print(fn);
		Serial.print(F(" does not exist .. rec="));
		Serial.print(gwayConfig.logFileRec);
		Serial.println();
#endif
	}
	
	File f = SPIFFS.open(fn, "a");
	if (!f) {
#if DUSB>=1
		Serial.println("file open failed=");
		Serial.println(fn);
#endif
	}
	
#if DUSB>=1
	if (debug>=1) {
		Serial.print(F("addLog:: fileno="));
		Serial.print(gwayConfig.logFileNo);
		Serial.print(F(", rec="));
		Serial.print(gwayConfig.logFileRec);

		Serial.print(F(": "));
		int i;
		for (i=0; i< 12; i++) {				// The first 12 bytes contain non printble characters
			Serial.print(line[i],HEX);
			Serial.print(' ');
		}
		Serial.print((char *) &line[i]);	// The rest if the buffer contains ascii

		Serial.println();
	}
#endif //DUSB
	f.write(line, cnt);			// write/append the line to the file
	f.print('\n');
	f.close();					// Close the file after appending to it

#endif //STAT_LOG
}

// ----------------------------------------------------------------------------
// Print (all) logfiles
//
// ----------------------------------------------------------------------------
void printLog()
{
	char fn[16];
	int i=0;
#if DUSB>=1
	while (i< LOGFILEMAX ) {
		sprintf(fn,"/log-%d", gwayConfig.logFileNo - i);
		if (!SPIFFS.exists(fn)) break;		// break the loop

		// Open the file for reading
		File f = SPIFFS.open(fn, "r");
		
		int j;
		for (j=0; j<LOGFILEREC; j++) {
			
			String s=f.readStringUntil('\n');
			if (s.length() == 0) break;

			Serial.println(s.substring(12));			// Skip the first 12 Gateway specific binary characters
			yield();
		}
		i++;
	}
#endif
} //printLog


// ----------------------------------------------------------------------------
// listDir
//	List the directory and put it in
// ----------------------------------------------------------------------------
void listDir(char * dir) 
{
#if DUSB>=1
	
#endif
}

