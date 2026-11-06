/*
 * CH32V006 ESC throttle from the adc_servo_sweep pinout.
 *
 * PD4 is the ESC pulse (TIM2 CH1, remap group 0, 1000-2000 us).
 * PC4 is the analog throttle (ADC channel 2). PD0 is a status LED.
 * PD1 stays SWIO for debug printf.
 *
 * Startup holds minimum throttle so the ESC can arm, samples PC4 to
 * learn the stick's travel, then maps that span onto the pulse width.
 */

#include "ch32fun.h"
#include "Servo.h"
#include <stdio.h>

static_assert( SERVO_TIM2_REMAP == 0, "ESC pulse is TIM2 CH1 in remap group 0" );
static_assert( SERVO_TIM2_CH1_PIN == PD4, "remap group 0 must put TIM2 CH1 on PD4" );

#define ADC_FULL_SCALE 4095u
#define PROBE_MIN_SPAN 64u
#define ARM_MS         2000u
#define PROBE_MS       3000u
#define STEP_MS        20u

static uint16_t probe_lo = ADC_FULL_SCALE;
static uint16_t probe_hi = 0;

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
	ADC1->RSQR3 = 2;

	// set sampling time for channel 2
	ADC1->SAMPTR2 &= ~(ADC_SMP0<<(3*2));
	ADC1->SAMPTR2 |= 7<<(3*2);	// 0:7 => 3.5/7.5/11.5/19.5/35.5/55.5/71.5/239.5 cycles

	// set External trigger event to SWSTART
	ADC1->CTLR2 = ADC1->CTLR2 & ~ADC_EXTSEL_SWSTART;
}

/*
 * start conversion, wait and return the 12-bit result
 */
uint16_t adc_get( void )
{
	ADC1->CTLR2 |= ADC_FLAG_STRT;

	while(!(ADC1->STATR & ADC_EOC));

	return (uint16_t)(ADC1->RDATAR & ADC_FULL_SCALE);
}

static void led_set( int on )
{
	GPIOD->BSHR = on ? 1u : (1u << 16);
}

static void note_probe( uint16_t adc )
{
	if( adc < probe_lo ) probe_lo = adc;
	if( adc > probe_hi ) probe_hi = adc;
}

/*
 * Map a sample onto the ESC pulse. A probe that barely moved falls back
 * to the full 12-bit range so the stick still has somewhere to go.
 */
static uint16_t adc_to_us( uint16_t adc )
{
	uint16_t lo = probe_lo;
	uint16_t hi = probe_hi;

	if( hi < lo || (uint16_t)(hi - lo) < PROBE_MIN_SPAN )
	{
		lo = 0;
		hi = ADC_FULL_SCALE;
	}
	if( adc <= lo ) return SERVO_MIN_US;
	if( adc >= hi ) return SERVO_MAX_US;

	return (uint16_t)( SERVO_MIN_US +
		(uint32_t)(adc - lo) * (SERVO_MAX_US - SERVO_MIN_US) / (uint32_t)(hi - lo) );
}

static void step_delay( void )
{
	Delay_Ms( STEP_MS );
}

/*
 * entry
 */
int main()
{
	SystemInit();

	printf( "\r\r\n\nesc_control_auto_probe\n\r" );

	printf( "initializing adc..." );
	adc_init();
	printf( "done.\n\r" );

	RCC->APB2PCENR |= RCC_APB2Periph_GPIOD;

	// GPIO D0 Push-Pull
	GPIOD->CFGLR &= ~(0xf<<(4*0));
	GPIOD->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_PP)<<(4*0);

	Servo esc;
	esc.attach( SERVO_TIM2_CH1_PIN );
	esc.writeMicroseconds( SERVO_MIN_US );
	led_set( 1 );

	printf( "arming at %u us\n\r", (unsigned)SERVO_MIN_US );
	for( uint32_t step = 0; step < ARM_MS / STEP_MS; step++ )
	{
		uint16_t adc = adc_get();
		if( (step % 25u) == 0 )
			printf( "arm adc: %u us: %u\n\r", adc, (unsigned)SERVO_MIN_US );
		step_delay();
	}

	printf( "probing analog input, hold min throttle on the ESC\n\r" );
	for( uint32_t step = 0; step < PROBE_MS / STEP_MS; step++ )
	{
		uint16_t adc = adc_get();
		note_probe( adc );
		led_set( (step % 10u) < 5u );
		if( (step % 5u) == 0 )
			printf( "probe adc: %u min: %u max: %u\n\r", adc, probe_lo, probe_hi );
		esc.writeMicroseconds( SERVO_MIN_US );
		step_delay();
	}

	printf( "probe done min: %u max: %u\n\r", probe_lo, probe_hi );
	if( probe_hi < probe_lo || (uint16_t)(probe_hi - probe_lo) < PROBE_MIN_SPAN )
		printf( "span below %u counts, control uses 0..%u\n\r",
			(unsigned)PROBE_MIN_SPAN, (unsigned)ADC_FULL_SCALE );

	printf( "running\n\r" );
	uint32_t step = 0;
	while(1)
	{
		uint16_t adc = adc_get();
		uint16_t us = adc_to_us( adc );
		esc.writeMicroseconds( us );
		led_set( (step % 50u) < 25u );
		if( (step % 5u) == 0 )
			printf( "run adc: %u us: %u\n\r", adc, us );
		step++;
		step_delay();
	}
}
