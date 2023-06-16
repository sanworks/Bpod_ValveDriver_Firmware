/*
  ----------------------------------------------------------------------------

  This file is part of the Sanworks Bpod_Gen2 repository
  Copyright (C) 2023 Sanworks LLC, Stony Brook, New York, USA

  ----------------------------------------------------------------------------

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, version 3.

  This program is distributed  WITHOUT ANY WARRANTY and without even the
  implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

//!! IMPORTANT: When compiling for Valve Driver 2 (Teensy 4.0) select 150MHz from Tools > CPU Speed

#include "ArCOM.h" // ArCOM is a serial interface wrapper developed by Sanworks, to streamline transmission of datatypes and arrays over serial

// SETUP MACROS TO COMPILE FOR TARGET DEVICE:
#define HARDWARE_VERSION 0 // Use: 1 = Valve Driver rev 1.0, 2 = AIM rev 2.0

#define FIRMWARE_VERSION 2 // Increments with each firmware release

// Validate macros
#if (HARDWARE_VERSION < 1) || (HARDWARE_VERSION > 2)
  #error Error! HARDWARE_VERSION must be either 1 or 2
#endif

#if HARDWARE_VERSION == 2
  #ifdef F_CPU
    #if F_CPU != 150000000
      #error Error! You must set the CPU Speed to 150MHz from the 'Tools' menu
    #endif
  #endif
#endif

#if HARDWARE_VERSION == 1
  ArCOM USBCOM(SerialUSB); // Creates an ArCOM object called USBCOM, wrapping SerialUSB (SAMD21 Mini) or Serial (Teensy 4)
#else
  ArCOM USBCOM(Serial); 
#endif
ArCOM StateMachineCOM(Serial1); // Creates an ArCOM object called StateMachineCOM, wrapping Serial1
char moduleName[] = "ValveModule"; // Name of module for manual override UI and state machine assembler

byte opCode = 0; 
byte opSource = 0;
boolean newOp = false;
byte channel = 0; 
byte circuitRevision = 0;
#if HARDWARE_VERSION == 1
  const byte enablePin = 4;
  const byte inputChannels[8] = {5, 6, 8, 9, 13, 12, 11, 10}; // Arduino pins wired to valve driver IC channels
#else
  const byte enablePin = 7;
  const byte inputChannels[8] = {8, 9, 10, 11, 12, 16, 15, 14}; // Arduino pins 
  const byte circuitRevisionArray[2] = {2,23};
  byte StateMachineSerialBuf[192] = {0}; // Extra memory for state machine serial buffer
#endif
const byte outputChannels[8] = {3, 2, 1, 0, 7, 6, 5, 4}; // Screw terminal channels 0-7, for each arduino pin in inputChannels
byte valveState[8] = {0};
void setup() {
  // put your setup code here, to run once:
  #if HARDWARE_VERSION == 2
    Serial1.addMemoryForRead(StateMachineSerialBuf, 192);
  #endif
  Serial1.begin(1312500); //1312500 //2625000
  // Read hardware revision from circuit board (an array of grounded pins indicates revision in binary, grounded = 1, floating = 0)
  circuitRevision = 0;
  #if HARDWARE_VERSION == 2
    for (int i = 0; i < 2; i++) {
      pinMode(circuitRevisionArray[i], INPUT_PULLUP);
      circuitRevision += pow(2, i)*digitalRead(circuitRevisionArray[i]);
      pinMode(circuitRevisionArray[i], INPUT);
    }
    circuitRevision = 3-circuitRevision;
  #endif
  pinMode(enablePin, OUTPUT);
  digitalWrite(enablePin, HIGH);
  for (int i = 0; i < 8; i++) {
    pinMode(inputChannels[i], OUTPUT);
    digitalWrite(inputChannels[i], 0);
  }
  #if HARDWARE_VERSION == 2 // Blink twice to acknowledge startup
    pinMode(13, OUTPUT);
    for (int i = 0; i < 2; i++) {
      digitalWrite(13, HIGH);
      delay(200);
      digitalWrite(13, LOW);
      delay(200);
    }
    pinMode(13, INPUT);
  #endif
}

void loop() {
  if (USBCOM.available()>0) {
    opCode = USBCOM.readByte();
    opSource = 0; newOp = true;
  } else if (StateMachineCOM.available()) {
    opCode = StateMachineCOM.readByte();
    opSource = 1; newOp = true;
  }
  if (newOp) {
    newOp = false;
    switch (opCode) {
      case 255:
        if (opSource == 1) {
          returnModuleInfo();
        } else if (opSource == 0) {
          USBCOM.writeByte(254); // Confirm
          USBCOM.writeUint32(FIRMWARE_VERSION); // Send firmware version
        }
      break;
      case 'O': // Open channel
        if (opSource == 0) {
          channel = USBCOM.readByte();
        } else {
          channel = StateMachineCOM.readByte();
        }
        channel = ascii2Num(channel)-1; // if channel is character 1-8 (ASCII 49-56), convert to 1-8
        digitalWrite(inputChannels[outputChannels[channel]], HIGH);
        valveState[channel] = 1;
      break;
      case 'C': // Close channel
        if (opSource == 0) {
          channel = USBCOM.readByte();
        } else {
          channel = StateMachineCOM.readByte();
        }
        channel = ascii2Num(channel)-1; // if channel is character 1-8 (ASCII 49-56), convert to 1-8
        digitalWrite(inputChannels[outputChannels[channel]], LOW);
        valveState[channel] = 0;
      break;
      case 'B': // Set valve states as bits of 1 byte
        if (opSource == 0) {
          channel = USBCOM.readByte();
        } else {
          channel = StateMachineCOM.readByte();
        }
        for (int i = 0; i < 8; i++) {
          valveState[i] = bitRead(channel, i);
          digitalWrite(inputChannels[outputChannels[i]], valveState[i]);
        }
        if (opSource == 0) {
          USBCOM.writeByte(1);
        }
      break;
      default: // Toggle channel; toggle op Codes = 1-8 or characters 1-8
        channel = ascii2Num(opCode);
        if ((channel < 9) && (channel > 0)) {
          channel = channel - 1;
          valveState[channel] = 1 - valveState[channel];
          digitalWrite(inputChannels[outputChannels[channel]], valveState[channel]);
        }
      break;
    }
  }
}

byte ascii2Num(byte value) { // Convert ascii numeric channels to numeric channels
  if ((value > 48) && (value < 57)) {
    value -= 48;
  }
  return value;
}

void returnModuleInfo() {
  boolean fsmSupportsHwInfo = false;
  delayMicroseconds(100);
  if (StateMachineCOM.available() == 1) { // FSM firmware v23 or newer sends a second info request byte to indicate that it supports additional ops
    if (StateMachineCOM.readByte() == 255) {fsmSupportsHwInfo = true;}
  }
  StateMachineCOM.writeByte('A'); // Acknowledge
  StateMachineCOM.writeUint32(FIRMWARE_VERSION); // 4-byte firmware version
  StateMachineCOM.writeByte(sizeof(moduleName)-1); // Length of module name
  StateMachineCOM.writeCharArray(moduleName, sizeof(moduleName)-1); // Module name
  if (fsmSupportsHwInfo) {
    StateMachineCOM.writeByte(1); // 1 if more info follows, 0 if not
    StateMachineCOM.writeByte('V'); // Op code for: Hardware major version
    StateMachineCOM.writeByte(HARDWARE_VERSION); 
    StateMachineCOM.writeByte(1); // 1 if more info follows, 0 if not
    StateMachineCOM.writeByte('v'); // Op code for: Hardware minor version
    StateMachineCOM.writeByte(circuitRevision); 
  }
  StateMachineCOM.writeByte(0); // 1 if more info follows, 0 if not
} 
