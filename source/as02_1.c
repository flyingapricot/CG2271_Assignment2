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
#define SWITCH_PIN  4 // PTA4

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
	MCG->C2 &= ~MCG_C2_IRCS_MASK;

    // Set FRCDIV and LIRC_DIV2
	// Set FRCDIV's division factor to 1 (since we are dividing using LIRC_DIV2)
	// MCG_SC_FCRDIV_MASK = 0b0000 1110
	MCG->SC &= ~MCG_SC_FCRDIV_MASK;

	// Set LIRC_DIV2's division factor to 2
	// MCG_MC_LIRC_DIV2_MASK = 0b0000 0111
	MCG->MC &= ~MCG_MC_LIRC_DIV2_MASK; // Clears the LIRC_DIV2 Bits
	MCG->MC |= (0b001 & MCG_MC_LIRC_DIV2_MASK); // Set division factor to 2

}

void initTimer() {

    //setMCGIRClk(); //uncomment this after question 2

    // Turn on clock gating for TPM0
	// SIM_SCGC6_TPM0_MASK = 0b00000001 00000000 00000000 00000000
	SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK;

    // Disable TPM0 Interrupt
	// TPM_SC_TOIE_MASK = 0b0100 0000
	TPM0->SC &= ~TPM_SC_TOIE_MASK;

    // Clear TPM clock source and select MCGIRCLK

	// Clear TPM clock source
	// SIM_SOPT2_TPMSRC_MASK = 0b00000011 00000000 00000000 00000000
	SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;

	// Select MCGIRCLK
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC_MASK;

    // Turn off TPM0 and clear Prescale counter

	// Turn off TPMO
	// TPM_SC_CMOD_MASK = 0b00000000 00000000 00000000 00011000
	TPM0->SC &= ~TPM_SC_CMOD_MASK;

	// Clear Prescale Counter
	TPM0->CNT = 0;

    //Set Prescale Counter
	TPM0->SC &= ~TPM_SC_PS_MASK; // Clear PS bits
	TPM0->SC |= (0b011 & TPM_SC_PS_MASK); // Set PS bits to 0b011

    //Initialize counter to 0
	TPM0->CNT = 0;

    //Initialize modulo
	TPM0->MOD = 62499;

    //Set priority to highest and enable IRQ
	NVIC_SetPriority(TPM0_IRQn,0);
	NVIC_EnableIRQ(TPM0_IRQn);

	// Re-enable TPM Overflow Interrupt
	TPM0->SC |= TPM_SC_TOIE_MASK;
}

void startTimer() {
	TPM0->SC &= ~TPM_SC_CMOD_MASK; // Clear CMOD bits
	TPM0->SC |= TPM_SC_CMOD(1); // CMOD = 01
}

void stopTimer() {
	TPM0->SC &= ~TPM_SC_CMOD_MASK; // CMOD = 00;
}

//count needs to be volatile.
volatile int count = 0;
void TPM0_IRQHandler() {
	// Clear Pending IRQ
	NVIC_ClearPendingIRQ(TPM0_IRQn);

	// Check if TOF is set
	// TPM__SC_TOF_MASK = 0b1000 0000
	if(TPM0->SC & TPM_SC_TOF_MASK) {
		// If TOP is set,
		count = (count + 1)  % 6;

		// 2. reset counter
		TPM0->CNT = 0;

		// and 3. Clear TOF bit
		TPM0->SC |= TPM_SC_TOF_MASK;
	}

}

void initGPIO() {
    // Enable clocks for PORTA (switch), PORTD (green LED), PORTE (red/blue LEDs)
    SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK
               | SIM_SCGC5_PORTD_MASK
               | SIM_SCGC5_PORTE_MASK;

    // 2) Set pin mux to GPIO for LEDs
    PORTE->PCR[RED_PIN]   &= ~PORT_PCR_MUX_MASK;
    PORTE->PCR[RED_PIN]   |=  PORT_PCR_MUX(1);

    PORTD->PCR[GREEN_PIN] &= ~PORT_PCR_MUX_MASK;
    PORTD->PCR[GREEN_PIN] |=  PORT_PCR_MUX(1);

    PORTE->PCR[BLUE_PIN]  &= ~PORT_PCR_MUX_MASK;
    PORTE->PCR[BLUE_PIN]  |=  PORT_PCR_MUX(1);

    // 3) Set LED pins as outputs
    GPIOE->PDDR |= (1u << RED_PIN) | (1u << BLUE_PIN);
    GPIOD->PDDR |= (1u << GREEN_PIN);

    // 4) Turn LEDs OFF initially (active low => write HIGH to turn off)
    GPIOE->PSOR |= (1u << RED_PIN) | (1u << BLUE_PIN);
    GPIOD->PSOR |= (1u << GREEN_PIN);

    // 5) Configure switch pin as GPIO input with pull-up (active low)
    PORTA->PCR[SWITCH_PIN] &= ~PORT_PCR_MUX_MASK;
    PORTA->PCR[SWITCH_PIN] |=  PORT_PCR_MUX(1);

    PORTA->PCR[SWITCH_PIN] &= ~PORT_PCR_PS_MASK;
    PORTA->PCR[SWITCH_PIN] |=  PORT_PCR_PS(1);    // pull-up

    PORTA->PCR[SWITCH_PIN] &= ~PORT_PCR_PE_MASK;
    PORTA->PCR[SWITCH_PIN] |=  PORT_PCR_PE(1);    // enable pull

    GPIOA->PDDR &= ~(1u << SWITCH_PIN);   // input
}

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

    initGPIO();
    initTimer();
    PRINTF("TIMER DEMO\r\n");
    startTimer();

    ledOff(RED);
    ledOff(GREEN);
    ledOff(BLUE);

    /* Enter an infinite loop, just incrementing a counter. */
    while(1) {
		switch(count) {
		case 0:
			ledOn(RED);
			break;
		case 1:
			ledOff(RED);
			break;
		case 2:
			ledOn(GREEN);
			break;
		case 3:
			ledOff(GREEN);
			break;
		case 4:
			ledOn(BLUE);
			break;
		case 5:
			ledOff(BLUE);
			break;
		default:
			count=0;
		}
    }
    return 0 ;
}
