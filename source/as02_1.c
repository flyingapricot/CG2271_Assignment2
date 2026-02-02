/*
 * Copyright 2016-2025 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    as02_1.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"
/*insert other include files here. */

/*insert other definitions and declarations here. */

// LED pin numbers
#define RED_PIN		31	// PTE31
#define GREEN_PIN	5	// PTD5
#define BLUE_PIN	29	// PTE29

typedef enum tl {
	RED, GREEN, BLUE
} TLED;

//TODO
void setMCGIRClk() {
    // Clear CLKS
	// MCG_C1_CLKS_MASK = 0b 1100 0000
	MCG->C1 = (MCG->C1 & ~MCG_C1_CLKS_MASK);

    // Set IRCLKEN to enable LIRC
	// MCG_C1_IRCLKEN_MASK = 0b 0000 0010
	MCG->C1 |= MCG_C1_IRCLKEN_MASK;

    // Choose the 2MHz Clock
	// MCG_C2_IRCS_MASK = 0b 0000 0001
	MCG->C2 |= MCG_C2_IRCS_MASK;

    // Set FRCDIV and LIRC_DIV2
	// Set FRCDIV's division factor to 1 (since we are dividing using LIRC_DIV2)
	// MCG_SC_FCRDIV_MASK = 0b0000 1110
	MCG->SC &= ~MCG_SC_FCRDIV_MASK;

	// Set LIRC_DIV2's division factor to 2
	// MCG_MC_LIRC_DIV2_MASK = 0b0000 0111
	MCG->MC &= ~MCG_MC_LIRC_DIV2_MASK; // Clears the LIRC_DIV2 Bits
	MCG->MC |= (0b001 & MCG_MC_LIRC_DIV2_MASK); // Set division factor to 2

}

//void initTimer()
//
//void startTimer()
//
//void stopTimer()
//
////count needs to be volatile.
//volatile int count = 0;
//void TPM0_IRQHandler()
//
//void initGPIO()

void ledOn(TLED led) {
	switch(led) {
	case RED:
		GPIOE->PCOR |= (1 << RED_PIN);
		break;

	case GREEN:
		GPIOD->PCOR |= (1 << GREEN_PIN);
		break;

	case BLUE:
		GPIOE->PCOR |= (1 << BLUE_PIN);
		break;
	}
}

void ledOff(TLED led) {
	switch(led) {
	case RED:
		GPIOE->PSOR |= (1 << RED_PIN);
		break;

	case GREEN:
		GPIOD->PSOR |= (1 << GREEN_PIN);
		break;

	case BLUE:
		GPIOE->PSOR |= (1 << BLUE_PIN);
		break;
	}
}

int main(void) {

    /* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
    /* Init FSL debug console. */
    BOARD_InitDebugConsole();
#endif

//    initGPIO();
//    initTimer();
//    PRINTF("TIMER DEMO\r\n");
//    startTimer();
//
//    ledOff(RED);
//    ledOff(GREEN);
//    ledOff(BLUE);
//
//    /* Enter an infinite loop, just incrementing a counter. */
//    while(1) {
//		switch(count) {
//		case 0:
//			ledOn(RED);
//			break;
//		case 1:
//			ledOff(RED);
//			break;
//		case 2:
//			ledOn(GREEN);
//			break;
//		case 3:
//			ledOff(GREEN);
//			break;
//		case 4:
//			ledOn(BLUE);
//			break;
//		case 5:
//			ledOff(BLUE);
//			break;
//		default:
//			count=0;
//		}
//    }
    return 0 ;
}
