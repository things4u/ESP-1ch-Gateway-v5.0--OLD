#Testing the Gateway

## Introduction

This document describes the testing of the single channel gateway. These parameters are set in gthe loraModem.h file and determine its behaviour for switching from S_SCAN state to S_CAD state and the fallback from S_CAD state to scanning if the rssi drop dramatically.

## Optimize for STD

For the standard mode, the folowing values are used for parameter setting:

- RSSI_LIMIT 40
- RSSI_WAIT	 275
- 

## Optimize for CAD

For the standard mode, the folowing values are used for parameter setting:


## Optimize for HOP

For the HOP mode, the folowing values are used for parameter setting:

- RSSI_LIMIT 39 for the S_SCAN state, and -5 for the S_CAD state
- RSSI_WAIT	 250
- RSSI_WAIT_DOWN 225; microseconds to wait before reading RSSI when in S_CAD mode to decide going back to S_SCAN
- 

