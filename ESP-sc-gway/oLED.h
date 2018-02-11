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
// This file contains a number of compile-time settings and definitions for OLED support.
//
// ----------------------------------------------------------------------------------------

#if OLED>=1

#define OLED_SCL 5								// GPIO5 / D1
#define OLED_SDA 4								// GPIO4 / D2

#if OLED==1
#include "SSD1306.h"
#define OLED_ADDR 0x3C							// Default 0x3C for 0.9", for 1.3" it is 0x78
SSD1306  display(OLED_ADDR, OLED_SDA, OLED_SCL);// i2c ADDR & SDA, SCL on wemos
#endif

#if OLED==2
#include "SH1106.h"
//#define OLED_ADDR 0x78						// Default for 1.3" it is 0x78
#define OLED_ADDR 0x3C							// Default 0x3C for 0.9", for 1.3" it is 0x78
SH1106  display(OLED_ADDR, OLED_SDA, OLED_SCL);	// i2c ADDR & SDA, SCL on wemos
#endif




#endif