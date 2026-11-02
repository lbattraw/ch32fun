/*
 * CH32V006: polled ADC while a servo sweeps.
 *
 * ADC setup and the PD0 blink are from examples_x00x/adc_polled.
 * The sweep is from ch32-libraries/Servo examples/servo_sweep, on the
 * TIM2 backend (no interrupts; the timer holds the pulse).
 *
 * Remap group 1 puts TIM2 CH1 on PC1. The analog input is PC4
 * (ADC channel 2), which is free of the servo, the PD0 blink, and SWIO.
 * 03-27-2023 E. Brombaugh (ADC)
 */

#define SERVO_TIM2_REMAP 1

#include "ch32fun.h"
#include "Servo.h"
#include <stdio.h>

static_assert( SERVO_TIM2_CH1_PIN == PC1, "remap group 1 must put TIM2 CH1 on PC1" );

/*
 * initialize adc for polling
 */
void adc_init( void )
{
	// ADCCLK = 24 MHz => RCC_ADCPRE divide by 2
	// ADC_CLK_MODE = 0 -- HB clock divides to make ADC clock
	// ADC_CLK_ADJ = 0 -- 1/2 duty cycle high level ADC clock
	RCC->CFGR0 = (RCC->CFGR0 & ~(RCC_ADCPRE | RCC_CFGR0_ADC_CLK_MODE | RCC_CFGR0_ADC_CLK_ADJ)) | RCC_ADCPRE_DIV2;

	// Enable GPIOC and ADC. PC4 is analog input channel 2.
	RCC->PB2PCENR |= RCC_ADCEN | RCC_IOPCEN;

	GPIOC->CFGLR &= ~(GPIO_CFGLR_MASK<<(4*4));
	GPIOC->CFGLR |= (GPIO_CFGLR_IN_ANALOG<<(4*4));

	// Reset the ADC to init all regs
	RCC->PB2PRSTR |= RCC_ADC1RST;
	RCC->PB2PRSTR &= ~RCC_ADC1RST;

	// Turn on ADC module
	ADC1->CTLR2 |= ADC_ADON;

	// Set up single conversion on channel 2 (PC4)
	ADC1->RSQR1 = 0;
	ADC1->RSQR2 = 0;
	ADC1->RSQR3 = 2;	// 0-9 for 8 ext inputs and two internals

	// set sampling time for channel 2
	ADC1->SAMPTR2 &= ~(ADC_SMP0<<(3*2));
	ADC1->SAMPTR2 |= 7<<(3*2);	// 0:7 => 3.5/7.5/11.5/19.5/35.5/55.5/71.5/239.5 cycles

	// set External trigger event to SWSTART
	ADC1->CTLR2 = ADC1->CTLR2 & ~ADC_EXTSEL_SWSTART;

	// should be ready for SW conversion now
}

/*
 * start conversion, wait and return result
 */
uint16_t adc_get( void )
{
	// start sw conversion (auto clears)
	ADC1->CTLR2 |= ADC_FLAG_STRT;

	// wait for conversion complete
	while(!(ADC1->STATR & ADC_EOC));

	// get result
	return ADC1->RDATAR;
}

/*
 * One sweep step: update the pulse, blink PD0 on a 100 ms / 100 ms cadence
 * (five 20 ms steps each), poll the ADC, and print.
 */
static void sample_and_step( Servo &servo, uint16_t us, uint32_t &count )
{
	servo.writeMicroseconds( us );

	if( (count % 10u) < 5u )
		GPIOD->BSHR = 1;		// PD0 on
	else
		GPIOD->BSHR = (1u << 16);	// PD0 off

	printf( "Count: %lu adc: %d us: %u\n\r", count++, adc_get(), (unsigned)us );
	Delay_Ms( 20 );
}

/*
 * entry
 */
int main()
{
	uint32_t count = 0;

	SystemInit();

	printf( "\r\r\n\nadc_servo_sweep example\n\r" );

	printf( "initializing adc..." );
	adc_init();
	printf( "done.\n\r" );

	// Enable GPIOs
	RCC->APB2PCENR |= RCC_APB2Periph_GPIOD;

	// GPIO D0 Push-Pull
	GPIOD->CFGLR &= ~(0xf<<(4*0));
	GPIOD->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_PP)<<(4*0);

	// Local Servo: its constructor runs here, so constructors support is unnecessary.
	Servo servo;
	servo.attach( SERVO_TIM2_CH1_PIN );

	printf( "looping...\n\r" );
	while(1)
	{
		for( uint16_t us = SERVO_MIN_US; us < SERVO_MAX_US; us += 10 )
			sample_and_step( servo, us, count );
		for( uint16_t us = SERVO_MAX_US; us > SERVO_MIN_US; us -= 10 )
			sample_and_step( servo, us, count );
	}
}
