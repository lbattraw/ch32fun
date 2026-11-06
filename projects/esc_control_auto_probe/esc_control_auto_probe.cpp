/*
 * CH32V006: ESC throttle and a spindle-probe arm on two TIM2 pulses.
 * Only one pulse is enabled at a time.
 *
 * PD4 (TIM2 CH1) drives the ESC. Its width tracks PC4, ADC channel 2,
 * from SERVO_MIN_US at 0 counts to SERVO_MAX_US at full scale.
 * PD3 (TIM2 CH2) drives the probe arm.
 *
 * After the ADC stays under ADC_IDLE_PERCENT of full scale for five
 * seconds, the ESC pulse turns off and the arm swings from minimum to
 * maximum. When the ADC rises back above that level, the arm swings
 * home. 1.5 seconds after it arrives, its pulse turns off and the ESC
 * turns back on, ramping from minimum to the width the ADC asks for.
 */

#include "ch32fun.h"
#include "Servo.h"
#include <stdio.h>

static_assert( SERVO_TIM2_REMAP == 0, "both pulses use TIM2 remap group 0" );
static_assert( SERVO_TIM2_CH1_PIN == PD4, "ESC pulse is TIM2 CH1 on PD4" );
static_assert( SERVO_TIM2_CH2_PIN == PD3, "probe arm is TIM2 CH2 on PD3" );
static_assert( SERVO_SMOOTH == 1, "the arm and the ESC handoff ramp in the library" );

#define ADC_FULL_SCALE 4095u
#ifndef ADC_IDLE_PERCENT
#define ADC_IDLE_PERCENT 5
#endif
#define ADC_IDLE_THRESHOLD ((ADC_FULL_SCALE * ADC_IDLE_PERCENT) / 100u)

#define IDLE_HOLD_MS     5000u
#define RETRACT_DWELL_MS 1500u
#define STEP_MS          20u
#define SWING_US_PER_S   1000u

static_assert( ADC_IDLE_THRESHOLD > 0, "idle threshold must be at least one count" );

enum Mode
{
	MODE_ESC,
	MODE_EXTEND,
	MODE_RETRACT,
	MODE_DWELL,
	MODE_RESUME
};

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

static uint16_t adc_to_us( uint16_t adc )
{
	return (uint16_t)( SERVO_MIN_US +
		(uint32_t)adc * (SERVO_MAX_US - SERVO_MIN_US) / ADC_FULL_SCALE );
}

static void led_set( int on )
{
	GPIOD->BSHR = on ? 1u : (1u << 16);
}

static int adc_is_idle( uint16_t adc )
{
	return adc < ADC_IDLE_THRESHOLD;
}

static int adc_is_active( uint16_t adc )
{
	return adc > ADC_IDLE_THRESHOLD;
}

static void enable_esc( Servo &esc, uint16_t us, int ramp_from_min )
{
	if( ramp_from_min )
	{
		esc.setSpeed( SWING_US_PER_S );
		esc.writeMicroseconds( SERVO_MIN_US );
		esc.attach( SERVO_TIM2_CH1_PIN );
		esc.writeMicroseconds( us );
	}
	else
	{
		esc.setSpeed( 0 );
		esc.writeMicroseconds( us );
		esc.attach( SERVO_TIM2_CH1_PIN );
	}
}

static void enable_probe( Servo &probe, uint16_t us )
{
	probe.setSpeed( SWING_US_PER_S );
	probe.writeMicroseconds( SERVO_MIN_US );
	probe.attach( SERVO_TIM2_CH2_PIN );
	probe.writeMicroseconds( us );
}

/*
 * entry
 */
int main()
{
	SystemInit();

	printf( "\r\r\n\nesc_control_auto_probe\n\r" );
	printf( "idle threshold %u counts (%u%%)\n\r",
		(unsigned)ADC_IDLE_THRESHOLD, (unsigned)ADC_IDLE_PERCENT );

	printf( "initializing adc..." );
	adc_init();
	printf( "done.\n\r" );

	RCC->APB2PCENR |= RCC_APB2Periph_GPIOD;

	// GPIO D0 Push-Pull. PD3 and PD4 are configured by the servo library.
	GPIOD->CFGLR &= ~(0xf<<(4*0));
	GPIOD->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_PP)<<(4*0);

	Servo esc;
	Servo probe;
	enable_esc( esc, adc_to_us( adc_get() ), 0 );

	Mode mode = MODE_ESC;
	uint32_t low_ms = 0;
	uint32_t dwell_ms = 0;
	uint32_t step = 0;

	while(1)
	{
		uint16_t adc = adc_get();
		uint16_t us = adc_to_us( adc );
		int report = (step % 5u) == 0;

		switch( mode )
		{
		case MODE_ESC:
			esc.writeMicroseconds( us );
			led_set( 1 );
			if( adc_is_idle( adc ) )
			{
				low_ms += STEP_MS;
				if( low_ms >= IDLE_HOLD_MS )
				{
					esc.detach();
					enable_probe( probe, SERVO_MAX_US );
					low_ms = 0;
					mode = MODE_EXTEND;
					printf( "extend\n\r" );
				}
			}
			else
			{
				low_ms = 0;
			}
			if( report )
				printf( "esc adc: %u us: %u low: %lu\n\r", adc, us, low_ms );
			break;

		case MODE_EXTEND:
			led_set( (step % 10u) < 5u );
			if( adc_is_active( adc ) )
			{
				probe.writeMicroseconds( SERVO_MIN_US );
				mode = MODE_RETRACT;
				printf( "retract\n\r" );
			}
			if( report )
				printf( "extend adc: %u\n\r", adc );
			break;

		case MODE_RETRACT:
			probe.writeMicroseconds( SERVO_MIN_US );
			led_set( (step % 10u) < 5u );
			if( !probe.isMoving() )
			{
				dwell_ms = 0;
				mode = MODE_DWELL;
				printf( "dwell\n\r" );
			}
			if( report )
				printf( "retract adc: %u\n\r", adc );
			break;

		case MODE_DWELL:
			led_set( (step % 10u) < 5u );
			dwell_ms += STEP_MS;
			if( dwell_ms >= RETRACT_DWELL_MS )
			{
				probe.detach();
				enable_esc( esc, us, 1 );
				mode = MODE_RESUME;
				printf( "resume\n\r" );
			}
			if( report )
				printf( "dwell adc: %u ms: %lu\n\r", adc, dwell_ms );
			break;

		case MODE_RESUME:
			esc.writeMicroseconds( us );
			led_set( 1 );
			if( !esc.isMoving() )
			{
				esc.setSpeed( 0 );
				low_ms = 0;
				mode = MODE_ESC;
				printf( "esc\n\r" );
			}
			else if( adc_is_idle( adc ) )
			{
				low_ms += STEP_MS;
				if( low_ms >= IDLE_HOLD_MS )
				{
					esc.detach();
					enable_probe( probe, SERVO_MAX_US );
					low_ms = 0;
					mode = MODE_EXTEND;
					printf( "extend\n\r" );
				}
			}
			else
			{
				low_ms = 0;
			}
			if( report )
				printf( "resume adc: %u us: %u\n\r", adc, us );
			break;
		}

		step++;
		Delay_Ms( STEP_MS );
	}
}
