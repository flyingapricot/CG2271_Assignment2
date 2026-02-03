/*
 * Copyright 2016-2025 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    as02_3.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"

/*
 * @brief   Application entry point.
 */

// LED pin numbers
#define RED_PIN		31	// PTE31
#define GREEN_PIN	5	// PTD5
#define BLUE_PIN	29	// PTE29

#define SWITCH_PIN 4 //PTA4

typedef enum tl {
	RED, GREEN, BLUE
} TLED;

/*
 * @brief   Application entry point.
 */
// Configure ADC_SE0 on PTE20 and ADC_SE4a on PTE21

#define ADC_SE0			0
#define ADC_SE0_PIN	20
#define ADC_SE4a			4
#define ADC_SE4_PIN	21
#define ADC_SE12  12
#define ADC_SE13  13

volatile int g_redPct = 0, g_greenPct = 0, g_bluePct = 0;


//TODO
void initADC() {
    // Disable & clear interrupt
	NVIC_DisableIRQ(ADC0_IRQn);
	NVIC_ClearPendingIRQ(ADC0_IRQn);

    // Enable clock gating to relevant configurations
	SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK;

    // Set pins from Q3 to ADC
	// Choosing PTB2 and PTB3
	SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;
	PORTB->PCR[2] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[3] &= ~PORT_PCR_MUX_MASK;

    // Configure the ADC
    // Enable ADC interrupt
    // Select single-ended ADC

    // Set 12 bit conversion
	ADC0->CFG1 &= ~ADC_CFG1_MODE_MASK;
	ADC0->CFG1 |= ADC_CFG1_MODE(1);

    // Select software conversion trigger
	ADC0->SC2 &= ~ADC_SC2_ADTRG_MASK;

    // Configure alternate voltage reference
	ADC0->SC2 &= ~ADC_SC2_REFSEL_MASK;
	ADC0->SC2 |= ADC_SC2_REFSEL(1);

	// Don't use averaging
	ADC0->SC3 &= ~ADC_SC3_AVGE_MASK;

    // Switch off continuous conversion.
	ADC0->SC3 &= ~ADC_SC3_ADCO_MASK;

    // Set highest priority
	NVIC_SetPriority(ADC0_IRQn, 0);
	NVIC_EnableIRQ(ADC0_IRQn);

}

int result[2];

void startADC(int channel) {
    ADC0->SC1[0] = ADC_SC1_AIEN_MASK | ADC_SC1_ADCH(channel); // SC1A
}

//TODO! This is NOT going to work.
void ADC0_IRQHandler(){
	static int turn=0;

	NVIC_ClearPendingIRQ(ADC0_IRQn);
	if(ADC0->SC1[0] & ADC_SC1_COCO_MASK) {
		result[turn] = ADC0->R[0];
		//PRINTF("Turn = %d, Result = %d\r\n", turn, result[turn]);
		turn = 1 - turn;
		if(turn == 0) {
			startADC(ADC_SE12);
		} else {
			startADC(ADC_SE13);
		}
	}
}

// Configure the MCG Internal Reference Clock
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

void setTPMClock(){
    setMCGIRClk(); // sets MCGIRCLK to 8MHz, div1

    // Select MCGIRCLK as TPM clock source (TPMSRC = 0b11)
    SIM->SOPT2 = (SIM->SOPT2 & ~SIM_SOPT2_TPMSRC_MASK) | SIM_SOPT2_TPMSRC(3);
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
	TPM0->MOD = 125; // 250 Hz, for 20Hz 1562

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

    PRINTF("Hello World\r\n");
    setTPMClock();
    initPWM();
	initButton();
    setPWM(RED, 0);
    setPWM(GREEN, 0);
    setPWM(BLUE, 0);
    startPWM();

    initADC();
    //startADC(ADC_SE0);
    startADC(ADC_SE12);
    /* Force the counter to be placed into memory. */
    volatile static int i = 0 ;

    /* Enter an infinite loop, just incrementing a counter. */
    while(1) {
        i++ ;

        int x = result[1] - 2048;   // left/right offset

        int redPct = 0;
        int greenPct = 0;
        int bluePct = 0;


        // deadzone around center
        if (x > 600) {
            // Move right is Red
            redPct = (x * 100) / 2047;
            greenPct = 0;
            bluePct = 0;
        }
        else if (x < -600) {
            // Move left is Green
            greenPct = (-x * 100) / 2047;
            redPct = 0;
            bluePct = 0;
        }
        else {
            // Center is Blue
            bluePct = 50;
            redPct = 0;
            greenPct = 0;
        }

        g_redPct = redPct;
        g_greenPct = greenPct;
        g_bluePct = bluePct;

        setPWM(RED, redPct);
        setPWM(GREEN, greenPct);
        setPWM(BLUE, bluePct);


//        // Sample way to convert values
//        int red = (int) (result[0] / 4096.0 * 255.0);
//        int green = (int) (result[1] / 4096.0 * 255.0);
//        int blue = (red + green) / 2;
//
//        PRINTF("RED = %d GREEN = %d BLUE = %d\r\n", red, green, blue);
//        setPWM(RED, red);
//        setPWM(GREEN, green);
//        setPWM(BLUE, blue);
        /* 'Dummy' NOP to allow source level single stepping of
            tight while() loop */
        __asm volatile ("nop");
    }
    return 0 ;
}

