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


// ----------------------------------------------------------------------------
// stateMachine handler of the state machine.
// We use ONE state machine for all kind of interrupts. This assures that we take
// the correct action upon receiving an interrupt.
//
// _event is the software interrupt: If set this function is executed from loop(),
// the function should itself take care of setting or resetting the variable.
//
// STATE MACHINE
// The program uses the following state machine (in _state), all states
// are done in interrupt routine, only the follow-up of S-RXDONE is done
// in the main loop() program. This is because otherwise the interrupt processing
// would take too long to finish
//
// So _state has one of the following state values:
//
// S-INIT=0, 	The commands in this state are executed only once
//	- Goto S_SCAN
//
// S-SCAN, 		CadScanner() part
//	- Upon CDDECT (int1) goto S_RX, 
//	- upon CDDONE (int0) goto S_CAD, walk through all SF until CDDETD
//	- Else stay in SCAN state
//
// S-CAD, 		
//	- Upon CDDECT (int1) goto S_RX, 
//	- Upon CDDONE (int0) goto S_SCAN, start with SF7 recognition again
//
// S-RX, 		Received CDDECT so message detected, RX cycle started. 
//	- Upon RXDONE (int0) package read. If read ok continue to read message
//	- upon RXTOUT (int1) error, goto S_SCAN
//
// S-TX			Transmitting a message
//	- Upon TXDONE goto S_SCAN
//
// S-TXDONE		Transmission complete by loop() now again in interrupt
//	- Set the Mask
//	- reset the Flags
//	- Goto either SCAN or RX
//
// This interrupt routine has been kept as simple and short as possible.
// If we receive an interrupt that does not below to a _state then print error.
// 
// NOTE: We may clear the interrupt but leave the flag for the moment. 
//	The eventHandler should take care of repairing flags between interrupts.
// ----------------------------------------------------------------------------

void stateMachine()
{
	// Determine what interrupt flags are set
	//
	uint8_t flags = readRegister(REG_IRQ_FLAGS);
	uint8_t mask  = readRegister(REG_IRQ_FLAGS_MASK);
	uint8_t intr  = flags & ( ~ mask );				// Only react on non masked interrupts
	uint8_t rssi;

	// If there is NO interrupt and if _hop we wait until this is one
	// or the wait time is over.
	// That means if hop we will ONLY execute the state machine below
	// when having an interrupt value and therefore a _state
	//
	if (intr == 0x00) 
	{
		// If we hop we have to make sure that we allow enought time to detect
		// CDDONE or CDECT. But if we do not receive interrupts, we have to schedule
		// another hop after EVENT_WAIT microseconds. 				
		// The process is such that we scan on SF7 (the shortest) preamble and if
		// nothing detected within a scan, we switch to another frequency.
		//
		if (_hop) {
		
			// Reset the IRQ registers. We clear the flag to accept all interrupts
			// and we clear all interrupts.
			writeRegister(REG_IRQ_FLAGS_MASK, (uint8_t) 0x00);
			writeRegister(REG_IRQ_FLAGS, (uint8_t) 0xFF);	
			
			while (_event==0)
			{
				hop();									// next frequency, set sf to SF7
				cadScanner();							// Reset to SF7

				// Wait for first CDDONE or CDETD interrupt to come in
				// This is tricky for hopping as hopping is NOT interrupt driven.
				// XXX All such timers are in seconds, or micros is used for real-time

				if ( (( micros() - hopTime ) > _STAT_INTERVAL ) ||
					 (( micros() - hopTime ) > _PULL_INTERVAL ) )
				{
					_event=0;
					return;
				}
				yield();								// XXX 03/01/2018
				delayMicroseconds(300);					// Allow CDDETD be noticed after CDDONE. XXX 150
				intr  = readRegister(REG_IRQ_FLAGS) | intr;
				if (intr!=0) _event=1;
			}
			
			// We received a real interrupt, so do nothing with either _event
			// or intr and let handle by state machine

			_state=S_CAD;
#if DUSB>=1
			if (debug>=1) {
				Serial.print("EVENT=0x");
				Serial.print(intr,HEX);
				Serial.print(F(", F="));
				Serial.print(ifreq);
				Serial.print(F(", SF="));
				Serial.print(sf);
				Serial.print(F(", E="));
				Serial.print(_event);
				Serial.print(F(", S="));
				Serial.print(_state);
				Serial.print(F(", t="));
				Serial.print( micros() - hopTime );
				Serial.println();
			}
#endif
			_event=0;							// If we received an interrupt, do the state machine below.
			
		}// hop
		
		// If not hopping make sure to return without doing anything
		// cause we only act on interrupts in this mode ((_event!=0) && (intr!=0))
		// 
		else {
			_event=0;
			//return;									// XXX Does this work as all are Freq 1 message when hopping
		}
	}// intr==0
	
	// This is the actual state machine of the gateway
	// and its next actions are depending on the state we are in.
	// For hop situations we do not get interrupts, so we have to
	// simulate and generate events ourselves.
	//
	switch (_state) 
	{
	
	  // --------------------------------------------------------------
	  // If the state is init, we are starting up.
	  // The initLoraModem() function is already called in setup();
	  //
	  case S_INIT:
#if DUSB>=2
		if (debug >= 1) { 
			Serial.println(F("S_INIT")); 
		}
#endif
		// new state, needed to startup the radio (to S_SCAN)
		writeRegister(REG_IRQ_FLAGS, 0xFF );		// Clear ALL interrupts
		_event=0;
	  break;

	  
	  // --------------------------------------------------------------
	  // In S_SCAN we measure a high RSSI this means that there (probably) is a message
	  // coming in at that freq. But not necessarily on the current SF.
	  // If so find the right SF with CDDETD. 
	  //
	  case S_SCAN:
	    //
		// Intr==IRQ_LORA_CDDETD_MASK
		// We detected a message on this frequency and SF when scanning
		// We clear both CDDETD and swich to reading state to read the message
		//
		if (intr & IRQ_LORA_CDDETD_MASK) {

			_state = S_RX;								// Set state to receiving
			opmode(OPMODE_RX_SINGLE);					// set reg 0x01 to 0x06

			// Set RXDONE interrupt to dio0, RXTOUT to dio1
			writeRegister(REG_DIO_MAPPING_1, (
				MAP_DIO0_LORA_RXDONE | 
				MAP_DIO1_LORA_RXTOUT | 
				MAP_DIO2_LORA_NOP | 
				MAP_DIO3_LORA_CRC ));
			
			// Since new state is S_RX, accept no interrupts except RXDONE or RXTOUT		
			writeRegister(REG_IRQ_FLAGS_MASK, (uint8_t) ~(
				IRQ_LORA_RXDONE_MASK | 
				IRQ_LORA_RXTOUT_MASK | 
				IRQ_LORA_HEADER_MASK | 
				IRQ_LORA_CRCERR_MASK));
			
			delayMicroseconds( RSSI_WAIT );				// Wait some microseconds less
			// Starting with version 5.0.1 the waittime is dependent on the SF
			// So for SF12 we wait longer (2^7 == 128 uSec) and for SF7 4 uSec.
			//delayMicroseconds( (0x01 << ((uint8_t)sf - 5 )) );
			rssi = readRegister(REG_RSSI);				// Read the RSSI
			_rssi = rssi;								// Read the RSSI in the state variable

			writeRegister(REG_IRQ_FLAGS, 0xFF );		// reset all interrupt flags
			_event = 0;									// Make 0, as soon aswe have an interrupt
#if DUSB>=1
			if (debug>=2) {
				Serial.println(F("SCAN:: CDDETD"));
			}
#endif
			detTime = micros();
		}//if

		// CDDONE
		// We received a CDDONE int telling us that we received a message on this
		// frequency and possibly on one of its SF.
		// If so, we switch to CAD state where we will wait for CDDETD event.
		//
		else if (intr & IRQ_LORA_CDDONE_MASK) {

			opmode(OPMODE_CAD);
			rssi = readRegister(REG_RSSI);				// Read the RSSI

			// We choose the generic RSSI as a sorting mechanism for packages/messages
			// The pRSSI (package RSSI) is calculated upon successful reception of message
			// So we expect that this value makes little sense for the moment with CDDONE.
			// Set the rssi as low as the noise floor. Lower values are not recognized then.
			// Every cycle starts with ifreq==0 and sf=SF7
			//
			if ( rssi > RSSI_LIMIT )					// Is set to 35
			{
#if DUSB>=1
				if (debug>=2) {
					Serial.println(F("S_SCAN:: -> CAD"));
				}
#endif
				_state = S_CAD;							// promote next level
				_event=0;								// next CDDONE by interrupt XXXXX
			}
			
			// If the RSSI is not big enough we skip the CDDONE
			// and go back to scanning
			else {
#if DUSB>=1
				if (debug>=2) {
					Serial.print("S_SCAN:: rssi=");
					Serial.println(rssi);
				}
#endif
				_state = S_SCAN;
				_event=1;								// loop() scan until CDDONE
			}

			// Clear the CADDONE flag
			writeRegister(REG_IRQ_FLAGS, 0xFF);
			
		}//SCAN CDDONE 
		
		// So if we are here then we are in S_SCAN and the interrupt is not
		// CDDECT or CDDONE. it is probably soft interrupt _event==1
		// So if _hop we change the frequency and restart the
		// interrupt in order to check for CDONE on other frequencies
		// if _hop we start at the next frequency, hop () sets the sf to SF7.
		// If we are at the end of all frequencies, reset frequencies and sf
		// and go to S_SCAN state.
		//
		// Note:: We should make sure that all frequencies are scanned in a row
		// and when we switch to ifreq==0 we should stop for a while
		// to allow system processing.
		// We should make sure that we enable webserver etc every once in a while.
		// We do this by changing _event to 1 in loop() only for _hop and
		// use _event=0 for non hop.
		//
		else if (intr == 0x00) 
		{
			//_state = S_SCAN;						// Do this state again but now for other freq.
			if (! _hop) _event = 0;					// XXX 26/12/2017 !!! NEED
		}
		
		// Unkown Interrupt, so we have an error
		//
		else {
#if DUSB>=1
			Serial.print(F("SCAN unknown intr="));
			Serial.println(intr,HEX);
#endif
			_state=S_SCAN;
			writeRegister(REG_IRQ_FLAGS, 0xFF);
		}
		
	  break; // S_SCAN

	  
	  // --------------------------------------------------------------
	  // S_CAD: In CAD mode we scan every SF for high RSSI until we have a DETECT.
	  // Reason is the we received a CADDONE interrupt so we know a message is received
	  // on the frequency but may be on another SF.
	  //
	  // If message is of the right frequency and SF, IRQ_LORA_CDDETD_MSAK interrupt
	  // is raised, indicating that we can start beging reading the message from SPI.
	  //
	  // DIO0 interrupt IRQ_LORA_CDDONE_MASK in state S_CAD==2 means that we might have
	  // a lock on the Freq but not the right SF. So we increase the SF
	  //
	  case S_CAD:

		// Intr=IRQ_LORA_CDDETD_MASK
		// We have to set the sf based on a strong RSSI for this channel
		//
		if (intr & IRQ_LORA_CDDETD_MASK) {

			// Set RXDONE interrupt to dio0, RXTOUT to dio1
			writeRegister(REG_DIO_MAPPING_1, (
				MAP_DIO0_LORA_RXDONE | 
				MAP_DIO1_LORA_RXTOUT | 
				MAP_DIO2_LORA_NOP |
				MAP_DIO3_LORA_CRC ));
			
			// Accept no interrupts except RXDONE or RXTOUT
			writeRegister(REG_IRQ_FLAGS_MASK, (uint8_t) ~(
				IRQ_LORA_RXDONE_MASK | 
				IRQ_LORA_RXTOUT_MASK |
				IRQ_LORA_HEADER_MASK |
				IRQ_LORA_CRCERR_MASK ));

			_state = S_RX;								// Set state to start receiving
			opmode(OPMODE_RX_SINGLE);					// set reg 0x01 to 0x06, initiate READ
			
			delayMicroseconds( RSSI_WAIT );				// Wait some microseconds less
			//delayMicroseconds( (0x01 << ((uint8_t)sf - 5 )) );
			rssi = readRegister(REG_RSSI);				// Read the RSSI
			_rssi = rssi;								// Read the RSSI in the state variable

			//writeRegister(REG_IRQ_FLAGS, IRQ_LORA_CDDETD_MASK | IRQ_LORA_RXDONE_MASK);
			writeRegister(REG_IRQ_FLAGS, 0xFF );		// reset all CAD Detect interrupt flags
			
			if (_hop) {
				_event=0;								// if CDECT, state=S_RX so we wait for intr
#if DUSB>=1
				if (debug>=1) {
					Serial.print(F("S_CAD:: hop CDECT freq="));
					Serial.print(ifreq);
					Serial.print(F(", sf="));
					Serial.println(sf);
				}
#endif
			}
			else{
				_event=1;								// XXX was 0;
			}
#if DUSB>=1
			if (debug>=2) {
				Serial.println(F("CAD:: CDDETD"));
			}
#endif
			detTime=micros();
		}// CDDETD
		
		// Intr == CADDONE
		// So we scan this SF and if not high enough ... next
		//
		else if (intr & IRQ_LORA_CDDONE_MASK) {
			// If this is not SF12, increment the SF and try again
			// We expect on other SF get CDDETD
			//
			if (((uint8_t)sf) < SF12) {
				sf = (sf_t)((uint8_t)sf+1);				// Increment sf
				setRate(sf, 0x04);						// Set SF with CRC==on

				opmode(OPMODE_CAD);						// Scanning mode
				
				delayMicroseconds(RSSI_WAIT);
				rssi = readRegister(REG_RSSI);			// Read the RSSI

				// reset interrupt flags for CAD Done
				writeRegister(REG_IRQ_FLAGS, IRQ_LORA_CDDONE_MASK | IRQ_LORA_CDDETD_MASK);
				//writeRegister(REG_IRQ_FLAGS, 0xFF );	// This will prevent the CDDETD from being read
				_event=0;								// XXXXX 171215
#if DUSB>=1
				if (debug>=2) {
					Serial.print(F("S_CAD:: CDONE, SF="));
					Serial.println(sf);
				}
#endif
			}

			// If we reach SF12, we should go back to SCAN state
			//
			else {
				_state = S_SCAN;						// As soon as we reach SF12 do something
				cadScanner();							// Which will reset SF to SF7

				//writeRegister(REG_IRQ_FLAGS, IRQ_LORA_CDDONE_MASK);
				writeRegister(REG_IRQ_FLAGS, 0xFF );
				_event=1;								// reset soft intr, to state machine again
#if DUSB>=1		
				if (debug>=2) {
					Serial.print(F("CAD->SCAN:0x"));
					Serial.print(ifreq);
					Serial.print(F(":SF"));
					Serial.print(sf);
					Serial.println();
				}
#endif
			}
		} //CADDONE

		// if this interrupt is not CDECT or CDDONE then probably is 0x00
		// This means _event was set but there was no real interrupt (yet).
		// So we clear _event and wait for next (soft) interrupt.
		// We stay in the CAD state because CDDONE means something is 
		// coming on this frequency so we wait on CDECT.
		//
		else if (intr == 0x00) {
			_event=0;										// Stay in CAD _state until real interrupt
		}
		
		// else we do not recognize the interrupt. We print an error
		// and restart scanning. If hop we even start at ifreq==1
		//
		else {
#if DUSB>=1
			if (debug>=0) { 
				Serial.print(F("CAD: Unknown interrupt=")); 
				Serial.println(intr);
			}
#endif
			_state = S_SCAN;
			cadScanner();									// Scan and set SF7
			writeRegister(REG_IRQ_FLAGS, (uint8_t) 0xFF);	// Reset all interrupts
			_event=0;
		}
	  break; //S_CAD

	  
	  // --------------------------------------------------------------
	  // If we receive an interrupt on dio0 state==S_RX
	  // it should be a RxDone interrupt
	  // So we should handle the received message
	  //
	  case S_RX:
	
		if (intr & IRQ_LORA_RXDONE_MASK) {

			// We have to check for CRC error which will be visible AFTER RXDONE is set.
			// CRC errors might indicate tha the reception is not OK.
			// Could be CRC error or message too large.
			// CRC error checking requires DIO3
			//
			if (intr & IRQ_LORA_CRCERR_MASK) {
#if DUSB>=1
				if ((debug>=1)&&(intr & IRQ_LORA_CRCERR_MASK)) Serial.println(F("CRC err"));
#endif
				if (_cad) {
					_state = S_SCAN;
					cadScanner();
				}
				else {
					_state = S_RX;
					rxLoraModem();
				}
				
				writeRegister(REG_IRQ_FLAGS_MASK, (uint8_t) 0x00);	// Reset the interrupt mask
				// Reset interrupts
				writeRegister(REG_IRQ_FLAGS, (uint8_t)(
					IRQ_LORA_RXDONE_MASK | 
					IRQ_LORA_RXTOUT_MASK | 
					IRQ_LORA_HEADER_MASK | 
					IRQ_LORA_CRCERR_MASK ));
					
				_event=0;
				break;
			}
unsigned long ffTime = micros();			
			// There should not be an error in the message
			//
			LoraUp.payLoad[0]= 0x00;
			if((LoraUp.payLength = receivePkt(LoraUp.payLoad)) <= 0) {
#if DUSB>=1
				if (debug>=0) {
					Serial.println(F("sMachine:: Error S-RX"));
				}
#endif
			}
#if DUSB>=1
			if (debug>=1) {
				Serial.print(F("RXDONE="));
				Serial.println(ffTime - detTime);
			}
#endif
				
			// Do all register processing in this section
			uint8_t value = readRegister(REG_PKT_SNR_VALUE);	// 0x19; 
			if ( value & 0x80 ) {								// The SNR sign bit is 1
				
				value = ( ( ~value + 1 ) & 0xFF ) >> 2;			// Invert and divide by 4
				LoraUp.snr = -value;
			}
			else {
				// Divide by 4
				LoraUp.snr = ( value & 0xFF ) >> 2;
			}
	
			LoraUp.prssi = readRegister(REG_PKT_RSSI);			// read register 0x1A, packet rssi
    
			// Correction of RSSI value based on chip used.	
			if (sx1272) {										// Is it a sx1272 radio?
				LoraUp.rssicorr = 139;
			} else {											// Probably SX1276 or RFM95
				LoraUp.rssicorr = 157;
			}
				
			LoraUp.sf = readRegister(REG_MODEM_CONFIG2) >> 4;

			// If read was successful, read the package from the LoRa bus
			//
			if (receivePacket() <= 0) {							// read is not successful
#if DUSB>=1
				Serial.println(F("sMach:: Error receivePacket"));
#endif
			}
			
			// Set the modem to receiving BEFORE going back to user space.
			// 
			if (_cad) {
				_state = S_SCAN;
				cadScanner();
			}
			else {
				_state = S_RX;
				rxLoraModem();
			}
			
			writeRegister(REG_IRQ_FLAGS_MASK, (uint8_t) 0x00);
			writeRegister(REG_IRQ_FLAGS, (uint8_t) 0xFF);		// Reset the interrupt mask
			_event=0;
		}
		
		// RX TIMEOUT: We did receive message receive timeout
		//
		else if (intr & IRQ_LORA_RXTOUT_MASK) {
			
			// Make sure we reset all interrupts//
			writeRegister(REG_IRQ_FLAGS_MASK, (uint8_t) 0x00 );
			writeRegister(REG_IRQ_FLAGS, (uint8_t) 0xFF);			// reset all interrupts
			
			// For the modem in cad state we reset to SF7
			// If a timeout occurs here we reset the cadscanner
			//
			if (_cad) {									// XXX 01/01/2018 
				// Set the state to CAD scanning
#if DUSB>=1
				if (debug>=2) {
					Serial.print(F("RXTOUT:: f="));
					Serial.print(ifreq);
					Serial.print(F(", sf="));
					Serial.print(sf);
					Serial.print(F(", tim="));
					Serial.println(micros() - detTime);
				}
#endif
				_state = S_SCAN;
				cadScanner();							// Start the scanner after RXTOUT
				_event=0;
			}
			
			// If not in cad mode we are in single channel single sf mode.
			//
			else {
				_state = S_RX;							// 
				rxLoraModem();
				_event=0;
			}

		}
		
		// The interrupt received is not RXDONE nor RXTOUT
		// therefore we restart the scanning sequence (catch all)
		// XXX This should not be possible, It is always one of the two...
		else {
#if DUSB>=1
			if (debug>=1) {
				Serial.print(F("S_RX:: no RXDONE or RXTOUT but="));
				Serial.println(intr);
			}
#endif
			//initLoraModem();
			//_event=0;
		}

	  break; // S_RX

	  
	  // --------------------------------------------------------------  
	  // Start te transmissoion of a message in state S-TX
	  // We use TXDONE as the state to read the message.
	  //
	  case S_TX:
	  
		if (intr == 0x00) {
#if DUSB>=1
			Serial.println(F("TX:00"));
			writeRegister(REG_IRQ_FLAGS, (uint8_t) 0xFF);				// reset interrupt flags
			_event=0;
			return;
#endif
		}
		
	  	// Initiate the transmission of the buffer (in Interrupt space)
		// We react on ALL interrupts if we are in TX state.
		txLoraModem(
			LoraDown.payLoad,
			LoraDown.payLength,
			LoraDown.tmst,
			LoraDown.sfTx,
			LoraDown.powe,
			LoraDown.fff,
			LoraDown.crc,
			LoraDown.iiq
		);
		
#if DUSB>=2
		if (debug>=0) { 
			Serial.println(F("S_TX, ")); 
		}
#endif
		_state = S_TXDONE;
		writeRegister(REG_IRQ_FLAGS_MASK, (uint8_t) 0x00);
		writeRegister(REG_IRQ_FLAGS, (uint8_t) 0xFF);				// reset interrupt flags
		_event=1;
	  break; // S_TX

	  
	  // ---------------------------------------------------
	  // AFter the transmission is completed by the hardware, 
	  // the interrupt TXDONE is raised telling us that the tranmission
	  // was successful.
	  // If we receive an interrupt on dio0 _state==S_TX it is a TxDone interrupt
	  // Do nothing with the interrupt, it is just an indication.
	  // sendPacket switch back to scanner mode after transmission finished OK
	  //
	  case S_TXDONE:
		if (intr & IRQ_LORA_TXDONE_MASK) {
#if DUSB>=1
			Serial.println(F("TXDONE interrupt"));
#endif
			// After transmission reset to receiver
			if (_cad) {
				// Set the state to CAD scanning
				_state = S_SCAN;
				cadScanner();										// Start the scanner after TX cycle
			}
			else {
				_state = S_RX;
				rxLoraModem();		
			}
		
			writeRegister(REG_IRQ_FLAGS_MASK, (uint8_t) 0x00);
			writeRegister(REG_IRQ_FLAGS, (uint8_t) 0xFF);			// reset interrupt flags
#if DUSB>=1
			if (debug>=1) {
				Serial.println(F("TXDONE handled"));
				if (debug>=2) Serial.flush();
			}
#endif
			_event=0;
		}
		
		// If a soft _event==0 interrupt and no transmission finished:
		else {
#if DUSB>=1
			if (debug>=0) {
				Serial.print(F("TXDONE unknown interrupt="));
				Serial.println(intr);
				if (debug>=2) Serial.flush();
			}
#endif
			writeRegister(REG_IRQ_FLAGS, (uint8_t) 0xFF);			// reset interrupt flags
			_event=0;
		}
		
	  break; // S_TXDONE	  

	  
	  // --------------------------------------------------------------
	  // If _STATE is in undefined state
	  // If such a thing happens, we should re-init the interface and 
	  // make sure that we pick up next interrupt
	  default:
#if DUSB>=1
		if (debug >= 0) { 
			Serial.print("E state="); 
			Serial.println(_state);	
		}
#endif
		if (_cad) {
			_state = S_SCAN;
			cadScanner();
		}
		else
		{
			_state = S_RX;
			rxLoraModem();
		}
		writeRegister(REG_IRQ_FLAGS_MASK, (uint8_t) 0x00);
		writeRegister(REG_IRQ_FLAGS, (uint8_t) 0xFF);				// Reset all interrupts
		_event=0;
	  break;
	}
	
	return;
}
