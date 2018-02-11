// 1-channel LoRa Gateway for ESP8266
// Copyright (c) 2016, 2017 Maarten Westenberg version for ESP8266
// Version 5.0.6
// Date: 2018-02-12
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
// This file contains the state machine code enabling to receive
// and transmit packages/messages.
// ========================================================================================
//

#if OLED>=1



void init_oLED() 
{
	// Initialising the UI will init the display too.
	display.init();
	display.flipScreenVertically();
	display.setFont(ArialMT_Plain_24);
	display.setTextAlignment(TEXT_ALIGN_LEFT);
	display.drawString(0, 24, "STARTING");
	display.display();
}

// Activate the OLED
//
void acti_oLED() 
{
	// Initialising the UI will init the display too.
	display.clear();
#if OLED==1
	display.setFont(ArialMT_Plain_24);
	display.drawString(0, 24, "READY");
#elif OLED==2
	display.setFont(ArialMT_Plain_16);
	display.drawString(0, 24, "READY");
#endif

	display.display();
}

// Print a message on the OLED.
// Note: The whole message must fit in the buffer
//
void msg_oLED(String tim, String sf) {
    display.clear();
    display.setFont(ArialMT_Plain_16);
    display.setTextAlignment(TEXT_ALIGN_LEFT);
	
	display.drawString(0, 48, "LEN: " );
//    display.drawString(40, 48, String((int)messageLength) );
    display.display();
	yield();
}


// Print the OLED address in use
//
void addr_oLED() 
{

	Serial.print(F("OLED_ADDR=0x"));
	Serial.println(OLED_ADDR, HEX);

}



#endif