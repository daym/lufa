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

#include "Descriptors.h"

#include <LUFA/Drivers/Board/Buttons.h>
#include <LUFA/Drivers/USB/USB.h>
#include <LUFA/Platform/Platform.h>
#include "MIDI.h"

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

#define COLS 6
static uint8_t columnPins[COLS] = {_BV(1), _BV(2), _BV(3), _BV(4), _BV(0), _BV(5)}; /* PORTC */

static uint16_t keystate[COLS] = {0};

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

/* PORTC is the octave selector, and driver.
   PD0 and PD1 are free.
   PIND, PINB are the (half)tone selectors, 12 of them, pins 2,3,4,5,6,7,8,9,10,11,12,13. */
static void Keyboard_Scan(void) {
	for(uint8_t c = 0; c < COLS; ++c) {
#if 0
		if(columnPins[c] == _BV(0)) /* no idea why I'm not allowed to do that. USB device vanishes when I try. */
			continue;
#endif
		uint16_t prevstate, newstate;
		DDRC |= columnPins[c]; /* set octave output */
		/* breaks; am not allowed to do that! */
		PORTC &=~ columnPins[c]; /* pulse column */
#if 0
		prevstate = keystate[c];
		newstate = ((uint16_t) PIND | ((uint16_t) PINB << 8)) >> 2;
		if(prevstate != newstate) { /* send MIDI event */
			uint8_t note;
			uint16_t changes = newstate ^ prevstate;
			for(note = 1; note < 12; ++note) { /* FIXME note 0 seems to change continously */
				if((changes & (1 << note)) != 0) {
					sendMIDIPacket(Channel, (newstate & _BV(note)) == 0 ? MIDI_COMMAND_NOTE_ON : MIDI_COMMAND_NOTE_OFF, 36 + c * 12 + note);
				}
			}
			keystate[c] = newstate;
		}
#endif
		/* TODO debounce */
		PORTC |= columnPins[c]; /* end pulse */
		DDRC &=~ columnPins[c]; /* end pulse */ /* breaks it */
	}
}

static void Keyboard_Init(void) {
	DDRB = 0; /* set input */
	DDRD &= 3; /* set input */ /* don't touch RX/TX */
	PORTB = 0xFF; /* activate pull-ups */
	PORTD |= 0xFF &~ 3; /* activate pull-ups */ /* don't touch RX/TX */
	DDRC = 0; /* input for now */
	PORTC = 0xFF; /* pullup for now */
	uint8_t c;
	for(c = 0; c < COLS; ++c)
		keystate[c] = 0xFF;
}


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

	/* Hardware Initialization */
	USB_Init();
	Keyboard_Init();
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
	for (;;) {
		Keyboard_Scan();
		MIDI_EventPacket_t ReceivedMIDIEvent;
		while (MIDI_Device_ReceiveEventPacket(&Keyboard_MIDI_Interface, &ReceivedMIDIEvent))
		{
			if ((ReceivedMIDIEvent.Event == MIDI_EVENT(0, MIDI_COMMAND_NOTE_ON)) && (ReceivedMIDIEvent.Data3 > 0))
			  ; /*LEDs_SetAllLEDs(ReceivedMIDIEvent.Data2 > 64 ? LEDS_LED1 : LEDS_LED2);*/
			else
			 ; /* LEDs_SetAllLEDs(LEDS_NO_LEDS);*/
		}

		MIDI_Device_USBTask(&Keyboard_MIDI_Interface);
		USB_USBTask();
	}
	return 0;
}
