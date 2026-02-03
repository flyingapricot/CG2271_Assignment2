/*
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
    // Clear CLKS
	// MCG_C1_CLKS_MASK = 0b 1100 0000
	MCG->C1 = (MCG->C1 & ~MCG_C1_CLKS_MASK);

    // Set IRCLKEN to enable LIRC
	// MCG_C1_IRCLKEN_MASK = 0b 0000 0010
	MCG->C1 |= MCG_C1_IRCLKEN_MASK;

    // Choose the 8MHz Clock
	// MCG_C2_IRCS_MASK = 0b 0000 0001
	MCG->C2 |= MCG_C2_IRCS_MASK;

    // Set FRCDIV and LIRC_DIV2
	// Set FRCDIV's division factor to 1 (since we are dividing using LIRC_DIV2)
	// MCG_SC_FCRDIV_MASK = 0b0000 1110
	MCG->SC &= ~MCG_SC_FCRDIV_MASK;

	// Set LIRC_DIV2's division factor to 1
	// MCG_MC_LIRC_DIV2_MASK = 0b0000 0111
	MCG->MC &= ~MCG_MC_LIRC_DIV2_MASK; // Sets to 000 -> divide by 1
}

void initTimer() {

    setMCGIRClk();

    // Turn on clock gating for TPM1
	// SIM_SCGC6_TPM1_MASK = 0b0010 0000 0000 0000 0000 0000 0000
	SIM->SCGC6 |= SIM_SCGC6_TPM1_MASK;

    // Disable TPM1 Interrupt
	// TPM_SC_TOIE_MASK = 0b0100 0000
	TPM1->SC &= ~TPM_SC_TOIE_MASK;

    // Clear TPM clock source and select MCGIRCLK

	// Clear TPM clock source
	// SIM_SOPT2_TPMSRC_MASK = 0b00000011 00000000 00000000 00000000
	SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;

	// Select MCGIRCLK
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC_MASK;

    // Turn off TPM 1and clear Prescaler

	// Turn off TPM1
	// TPM_SC_CMOD_MASK = 0b00000000 00000000 00000000 00011000
	TPM1->SC &= ~TPM_SC_CMOD_MASK;

    //Clear And Set Prescale Counter
	TPM1->SC &= ~TPM_SC_PS_MASK; // Clear PS bits
	TPM1->SC |= (0b111 & TPM_SC_PS_MASK); // Set PS bits to 0b111 (128)

    //Initialize counter to 0
	TPM1->CNT = 0;

    //Initialize modulo
	TPM1->MOD = 624;

    //Set priority to highest and enable IRQ
	NVIC_SetPriority(TPM1_IRQn,0);
	NVIC_EnableIRQ(TPM1_IRQn);

	// Re-enable TPM Overflow Interrupt
	TPM1->SC |= TPM_SC_TOIE_MASK;
}

void startTimer() {
    TPM1->SC &= ~TPM_SC_CMOD_MASK;
    TPM1->SC |= TPM_SC_CMOD(1);
}

void stopTimer() {
    TPM1->SC &= ~TPM_SC_CMOD_MASK;
}

void initPWM() {
    // Turn on clock gating to TPM0
	// SIM_SCGC6_TPM0_MASK = 0b00000001 00000000 00000000 00000000
	SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK;

    // Turn on clock gating to PORTD and PORTE
	SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;

    // Configure the RGB LED MUX
	PORTE->PCR[31] &= ~PORT_PCR_MUX_MASK; // Clear the MUX Field
	PORTE->PCR[31] |= PORT_PCR_MUX(3);

	PORTE->PCR[29] &= ~PORT_PCR_MUX_MASK; // Clear the MUX Field
	PORTE->PCR[29] |= PORT_PCR_MUX(3);

	PORTD->PCR[5] &= ~PORT_PCR_MUX_MASK; // Clear the MUX Field
	PORTD->PCR[5] |= PORT_PCR_MUX(4);

    // Set pins to output
	GPIOE->PDDR |= (1u << 31) | (1u << 29); // Red, Blue
	GPIOD->PDDR |= (1u << 5); // Green

    //setup TPM0:

    // Turn off TPM0 by clearing the Clock Mode
	// TPM_SC_CMOD_MASK = 0b00000000 00000000 00000000 00011000
	TPM0->SC &= ~TPM_SC_CMOD_MASK;

    //Clear and Set Prescaler
	TPM0->SC &= ~TPM_SC_PS_MASK; // Clear PS bits
	TPM0->SC |= (0b111 & TPM_SC_PS_MASK); // Prescaler to 128

    // Set centre-aligned PWM mode
	TPM0->SC |= TPM_SC_CPWMS_MASK;

    // Initialize count to 0
	TPM0->CNT = 0;

    // Choose and initialize modulo
	TPM0->MOD = 1562; // for 250 Hz it should be 125, 20Hz 1562

    //Configure TPM0 channels.

    //IMPORTANT: Configure a REVERSE PWM signal!!!
    //i.e. it sets when counting up and clears when counting
    //down. This is because the LEDs are active low.
	// BLUE: TPM0_CH2
	TPM0->CONTROLS[2].CnSC &= ~(TPM_CnSC_MSA_MASK | TPM_CnSC_MSB_MASK |
	                           TPM_CnSC_ELSA_MASK | TPM_CnSC_ELSB_MASK);
	TPM0->CONTROLS[2].CnSC |=  TPM_CnSC_MSB_MASK | TPM_CnSC_ELSA_MASK;

	// RED: TPM0_CH4
	TPM0->CONTROLS[4].CnSC &= ~(TPM_CnSC_MSA_MASK | TPM_CnSC_MSB_MASK |
	                           TPM_CnSC_ELSA_MASK | TPM_CnSC_ELSB_MASK);
	TPM0->CONTROLS[4].CnSC |=  TPM_CnSC_MSB_MASK | TPM_CnSC_ELSA_MASK;

	// GREEN: TPM0_CH5
	TPM0->CONTROLS[5].CnSC &= ~(TPM_CnSC_MSA_MASK | TPM_CnSC_MSB_MASK |
	                           TPM_CnSC_ELSA_MASK | TPM_CnSC_ELSB_MASK);
	TPM0->CONTROLS[5].CnSC |=  TPM_CnSC_MSB_MASK | TPM_CnSC_ELSA_MASK;
}

void startPWM() {
	//set CMOD for TPM0
	TPM0->SC &= ~TPM_SC_CMOD_MASK; // Clear CMOD bits
	TPM0->SC |= TPM_SC_CMOD(1); // CMOD = 01
}

void stopPWM() {
	//mask CMOD for TPM0
	TPM0->SC &= ~TPM_SC_CMOD_MASK; // CMOD = 00;
}

void setPWM(int LED, int percent) {
	if(percent < 0) percent = 0;
	if(percent > 100) percent = 100;

    uint16_t value = (percent * TPM0->MOD) / 100;

    switch (LED) {
        case RED:
            TPM0->CONTROLS[4].CnV = value;
            break;

        case GREEN:
            TPM0->CONTROLS[5].CnV = value;
            break;

        case BLUE:
            TPM0->CONTROLS[2].CnV = value;
            break;

        default:
            printf("invalid LED.\r\n");
            break;
    }
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
        if (TPM0->MOD == 1562) {
            // TODO: write modulo for 250Hz
        	TPM0->MOD = 125;
        } else {
            // TODO: write modulo for 20Hz
        	TPM0->MOD = 1562;
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
    return 0;
}
