//*
 * Copyright 2016-2025 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    as02_2.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"
/* TODO: insert other include files here. */

/* TODO: insert other definitions and declarations here. */

//LED pin numbers
#define RED_PIN		31	//PTE31
#define GREEN_PIN	5	//PTD5
#define BLUE_PIN	29	//PTE29

#define SWITCH_PIN 4 //PTA4

typedef enum tl {
	RED, GREEN, BLUE
} TLED;

//TODO
void setMCGIRClk() {
}

void initTimer() {
}

void startTimer() {
}

void stopTimer() {
}

void initPWM() {
    //turn on clock gating to TPM0

    //turn on clock gating to PORTD and PORTE
    //configure the RGB LED MUX
    //set pins to output
    //setup TPM0:
    //turn off TPM0 by clearing the Clock Mode
    //clear and set the prescalar
    //set centre-aligned PWM mode
    //initialize count to 0
    //choose and initialize modulo
    //Configure TPM0 channels.
    //IMPORTANT: Configure a REVERSE PWM signal!!!
    //i.e. it sets when counting up and clears when counting
    //down. This is because the LEDs are active low.
}

void startPWM() {
}

void stopPWM() {
}

void setPWM(int LED, int percent) {
}

volatile int val = 0;

void TPM1_IRQHandler(){

	NVIC_ClearPendingIRQ(TPM1_IRQn);

	static int dir = 1;

	if(TPM1->STATUS & TPM_STATUS_TOF_MASK) {
		val += dir;
		if (val > 0 && val <= 100) {
			setPWM(RED, val);
		} else if (val > 100 && val <= 200) {
			setPWM(GREEN, val - 100);
		} else if (val > 200 && val <= 300) {
			setPWM(BLUE, val - 200);
		}
		if (val >= 300 || val <= 0) {
			dir *= -1;
		}

//			PRINTF("redVal=%d, greenVal=%d, blueVal=%d\r\n",
//					redVal, greenVal, blueVal);
			PRINTF("val=%d\r\n", val);

			TPM1->CNT=0;
			TPM1->STATUS |= TPM_STATUS_TOF_MASK;
		}
}

void initButton() {
    //Disable interrupts
    NVIC_DisableIRQ(PORTA_IRQn);

    //Enable clock gating to PORTA
    SIM->SCGC5 |= (SIM_SCGC5_PORTA_MASK);

    //Configure MUX of PTA4
	PORTA->PCR[SWITCH_PIN] &= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[SWITCH_PIN] |= PORT_PCR_MUX(1);

	//Set pullup resistor
	PORTA->PCR[SWITCH_PIN] &= ~PORT_PCR_PS_MASK;
	PORTA->PCR[SWITCH_PIN] |= PORT_PCR_PS(1);
	PORTA->PCR[SWITCH_PIN] &= ~PORT_PCR_PE_MASK;
	PORTA->PCR[SWITCH_PIN] |= PORT_PCR_PE(1);

	//Set as input
	GPIOA->PDDR &= ~(1 << SWITCH_PIN);

	//Configure the interrupt for falling edge
	PORTA->PCR[SWITCH_PIN] &= ~PORT_PCR_IRQC_MASK;
	PORTA->PCR[SWITCH_PIN] |= PORT_PCR_IRQC(0b1010);

	//Set NVIC priority to 0,
	//highest priority
	NVIC_SetPriority(PORTA_IRQn, 0);


	//Clear pending interrupts and enable interrupts
	NVIC_ClearPendingIRQ(PORTA_IRQn);
	NVIC_EnableIRQ(PORTA_IRQn);
}

void PORTA_IRQHandler() {
    NVIC_ClearPendingIRQ(PORTA_IRQn);

	//We don't actually have to test that it is
	//PTA4 since there is only one interrupt
	//configured, but this shows how to do it.
	if(PORTA->ISFR & (1 << SWITCH_PIN)) {
        stopPWM();
        TPM0->CNT = 0;
            // TODO: write modulo for 20Hz
        if (TPM0->MOD == ) {
            // TODO: write modulo for 250Hz
        	TPM0->MOD = ;
        } else {
            // TODO: write modulo for 20Hz
        	TPM0->MOD = ;
        }
        startPWM();
	}

    //Write a 1 to clear the ISFR bit.
    PORTA->ISFR |= (1 << SWITCH_PIN);

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

    //Set the TPM clock
    initTimer();
    initPWM();
    initButton();
    setPWM(RED, 0);
    setPWM(GREEN, 0);
    setPWM(BLUE, 0);
    PRINTF("PWM DEMO\r\n");
    startPWM();
    startTimer();

    /* Enter an infinite loop, just incrementing a counter. */
    int i = 0;
    while(1) {
    	i++;
    }
    return 0 ;
}
