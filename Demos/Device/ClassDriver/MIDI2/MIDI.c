/*
             LUFA Library
     Copyright (C) Dean Camera, 2014.

  dean [at] fourwalledcubicle [dot] com
           www.lufa-lib.org
*/

/*
  Copyright 2014  Dean Camera (dean [at] fourwalledcubicle [dot] com)

  Permission to use, copy, modify, distribute, and sell this
  software and its documentation for any purpose is hereby granted
  without fee, provided that the above copyright notice appear in
  all copies and that both that the copyright notice and this
  permission notice and warranty disclaimer appear in supporting
  documentation, and that the name of the author not be used in
  advertising or publicity pertaining to distribution of the
  software without specific, written prior permission.

  The author disclaims all warranties with regard to this
  software, including all implied warranties of merchantability
  and fitness.  In no event shall the author be liable for any
  special, indirect or consequential damages or any damages
  whatsoever resulting from loss of use, data or profits, whether
  in an action of contract, negligence or other tortious action,
  arising out of or in connection with the use or performance of
  this software.
*/

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include <string.h>
#include <stdint.h>
typedef uint8_t uchar;

#include "Descriptors.h"

#include <LUFA/Drivers/Board/Buttons.h>
#include <LUFA/Drivers/USB/USB.h>
#include <LUFA/Platform/Platform.h>
#include "MIDI.h"

#define HW_CDC_BULK_OUT_SIZE     8
#define HW_CDC_BULK_IN_SIZE      8

static USB_ClassInfo_MIDI_Device_t Keyboard_MIDI_Interface = {
	.Config = {
		.StreamingInterfaceNumber = INTERFACE_ID_AudioStream,
		.DataINEndpoint = {
			.Address          = MIDI_STREAM_IN_EPADDR,
			.Size             = MIDI_STREAM_EPSIZE,
			.Banks            = 1,
		},
		.DataOUTEndpoint = {
			.Address          = MIDI_STREAM_OUT_EPADDR,
			.Size             = MIDI_STREAM_EPSIZE,
			.Banks            = 1,
		},
	},
};

/* TxLED = PD5 */
#define LEDTx_ON  { PORTD &= ~0b00100000; }
#define LEDTx_OFF { PORTD |=  0b00100000; }
/* RxLED = PD4 */
#define LEDRx_ON  { PORTD &= ~0b00010000; }
#define LEDRx_OFF { PORTD |=  0b00010000; }
#define    RX_SIZE        (HW_CDC_BULK_IN_SIZE)
static uchar utxrdy = false;    /* USB Packet ready in utx_buf */
static uchar rx_buf[RX_SIZE];   /* tempory buffer */
static uchar utx_buf[RX_SIZE];  /* BULK_IN buffer */

#define    TX_SIZE        (HW_CDC_BULK_OUT_SIZE<<2)
#define    TX_MASK        (TX_SIZE-1)
static uchar uwptr = 0, irptr = 0;
static uchar tx_buf[TX_SIZE];

void parseUSBMidiMessage(uchar *data, uchar len) {
  uchar cin = (*data) & 0x0f;	/* CABLE NOを無視する */
  uchar i;

  LEDRx_ON

  if (cin > 1) {		/* ignore cin == 0 and cin == 1 */
    for (i = 1 ; i < 4 ; i++) {
      tx_buf[uwptr++] = *(data + i);
      uwptr &= TX_MASK;
      if (i == 1) {
	if ((cin == 5) || /* single byte system common */
	    (cin == 15))  /* single byte */
	  break;
      }
      if (i == 2) {
	if ((cin == 2) ||  /* two-byte system common */
	    (cin == 6) ||  /* system ex end with 2 bytes */
	    (cin == 12) || /* program change */
	    (cin == 13))   /* channel pressure */
	  break;
      }
    }
  }

  LEDRx_OFF

  if (len > 4) {
    parseUSBMidiMessage(data+4, len-4);
  }
}

uchar parseSerialMidiMessage(uchar RxByte) {
  static uchar PC = 0;
  static uchar SysEx = false;
  static uchar stateTransTable[] = {
    0, 				/* 0 dummy */
    0,				/* 1 dummy */
    3,				/* 2->3 NOTE OFF (3) */
    2 | 0x80,			/* 3->2 */
    5,				/* 4->5 NOTE ON (3) */
    4 | 0x80,			/* 5->4 */
    7,				/* 6->7 Polyphonic key pressure (3) */
    6 | 0x80,			/* 7->6 */
    9,				/* 8->9 Control Change (3) */
    8 | 0x80,			/* 8->9 */
    10 | 0x80,			/* 10->10 program change (2) */
    0,				/* dummy */
    12 | 0x80,			/* 12->12 Channel Pressure (2) */
    0,				/* 13 dummy */
    15,				/* 14->15 Pitch Bend (3) */
    14 | 0x80			/* 15->14 */
  };

  LEDRx_ON

  if(SysEx){  /* MIDI System Message */
    if(RxByte == 0xf7){		/* MIDI_EndSysEx */
      SysEx = false;
    }
    LEDRx_OFF
    return false;
  }
  if (RxByte >= 0xF8){		/* Single Byte Message */
    utx_buf[0] = 0x0f;
    utx_buf[1] = RxByte;
    utx_buf[2] = 0;
    utx_buf[3] = 0;
    LEDRx_OFF
    return true;
  }

  if(RxByte > 0x7F){		/* Channel message */
    if(RxByte == 0xf0){		/* MIDI_StartSysEx */
      SysEx = true;
      LEDRx_OFF
      return false;
    }
    PC = 0;
  }

  if (PC == 0) {
    PC = (((RxByte >> 4) & 0x07) + 1) * 2;
    // conversion
    // 0x80 -> 2, 0x90 -> 4, 0xa0 -> 6, 0xb0 -> 8, 0xc0 -> 10, 0xd0 -> 12, 0xe0 -> 14
    rx_buf[0] = RxByte >> 4;
    rx_buf[1] = RxByte;
    rx_buf[3] = 0;
  } else {
    uchar tt = stateTransTable[PC];
    rx_buf[(PC & 1) + 2] = RxByte;
    PC = tt & 0x0f;
    if ((tt & 0x80) != 0) {
      memcpy(utx_buf, rx_buf, 4);
      LEDRx_OFF
      return true;
    }
  }
  LEDRx_OFF
  return false;
}

static void sendMIDIPacket(uint8_t Channel, uint8_t MIDICommand, uint8_t MIDIPitch) {
	if (MIDICommand) {
		MIDI_EventPacket_t MIDIEvent = (MIDI_EventPacket_t) {
			.Event       = MIDI_EVENT(0, MIDICommand),
			.Data1       = MIDICommand | Channel,
			.Data2       = MIDIPitch,
			.Data3       = MIDI_STANDARD_VELOCITY,
		};
		MIDI_Device_SendEventPacket(&Keyboard_MIDI_Interface, &MIDIEvent);
		MIDI_Device_Flush(&Keyboard_MIDI_Interface);
	}
}
static uint8_t Channel = MIDI_CHANNEL(1);

/** Configures the board hardware and chip peripherals for the demo's functionality. */
void SetupHardware(void) {
#if (ARCH == ARCH_AVR8)
	/* Disable watchdog if enabled by bootloader/fuses */
	MCUSR &= ~(1 << WDRF);
	wdt_disable();

	/* Disable clock division */
	clock_prescale_set(clock_div_1);
#elif (ARCH == ARCH_XMEGA)
	/* Start the PLL to multiply the 2MHz RC oscillator to 32MHz and switch the CPU core to run from it */
	XMEGACLK_StartPLL(CLOCK_SRC_INT_RC2MHZ, 2000000, F_CPU);
	XMEGACLK_SetCPUClockSource(CLOCK_SRC_PLL);

	/* Start the 32MHz internal RC oscillator and start the DFLL to increase it to 48MHz using the USB SOF as a reference */
	XMEGACLK_StartInternalOscillator(CLOCK_SRC_INT_RC32MHZ);
	XMEGACLK_StartDFLL(CLOCK_SRC_INT_RC32MHZ, DFLL_REF_INT_USBSOF, F_USB);

	PMIC.CTRL = PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm;
#endif

	DDRD |= 0b00110000; /* PD5,4 = OUTPUT for LED */
	PORTD |= 0b00110000;          /* turn off LED (PD4, PD5) */
	/* FIXME UBRR1H */
	UBRR1L = 31;                  /* 31250Hz at 16MHz clock */
	UCSR1B = (1<<RXEN1) | (1<<TXEN1);
	USB_Init();
}

/** Event handler for the library USB Connection event. */
void EVENT_USB_Device_Connect(void) {
}

/** Event handler for the library USB Disconnection event. */
void EVENT_USB_Device_Disconnect(void) {
}

/** Event handler for the library USB Configuration Changed event. */
void EVENT_USB_Device_ConfigurationChanged(void) {
	bool ConfigSuccess = true;
	ConfigSuccess &= MIDI_Device_ConfigureEndpoints(&Keyboard_MIDI_Interface);
}

/** Event handler for the library USB Control Request reception event. */
void EVENT_USB_Device_ControlRequest(void) {
	MIDI_Device_ProcessControlRequest(&Keyboard_MIDI_Interface);
}

/** Main program entry point. This routine contains the overall program flow, including initial
 *  setup of all components and the main program loop.
 */
int main(void) {
  SetupHardware();
  GlobalInterruptEnable();
  for (;;){ 
    /* receive from Serial MIDI line */
    if( UCSR1A & (1<<RXC1)) {
      utxrdy |= parseSerialMidiMessage(UDR1);
    }

    /* send packets to USB MIDI */
    if( utxrdy ) {
      MIDI_Device_SendEventPacket(&Keyboard_MIDI_Interface, (MIDI_EventPacket_t *)&utx_buf);
      MIDI_Device_Flush(&Keyboard_MIDI_Interface);
      utxrdy = false;
    }

    /* receive from USB MIDI */
    MIDI_EventPacket_t ReceivedMIDIEvent;
    while (MIDI_Device_ReceiveEventPacket(&Keyboard_MIDI_Interface, &ReceivedMIDIEvent)) {
      /* for each MIDI packet w/ 4 bytes */
      parseUSBMidiMessage((uchar *)&ReceivedMIDIEvent, 4);
    }
      
    /* send to Serial MIDI line  */
    if( (UCSR1A & (1<<UDRE1)) && uwptr!=irptr ) {
      UDR1 = tx_buf[irptr++];
      irptr &= TX_MASK;
    }

    MIDI_Device_USBTask(&Keyboard_MIDI_Interface);
    USB_USBTask();
  }
  return 0;
}
