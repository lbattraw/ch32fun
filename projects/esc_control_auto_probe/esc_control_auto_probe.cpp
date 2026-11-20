/*
 * CH32V006: ESC throttle and a spindle-probe arm on two TIM2 pulses.
 * Only one pulse is enabled at a time.
 *
 * Motor drive is the default. PD4 (TIM2 CH1) drives the ESC and tracks
 * PC4, ADC channel 2, from SERVO_MIN_US at 0 counts to SERVO_MAX_US at
 * full scale. PD3 (TIM2 CH2) drives the probe arm. After the throttle
 * stays under ADC_IDLE_PERCENT for five seconds, the ESC pulse turns
 * off and the arm swings out. Above that level the arm swings home,
 * and 1.5 seconds after it arrives the ESC ramps back on.
 *
 * A rising edge on the mode pin leaves motor drive and runs ATC. The
 * ESC sits at minimum for 10 seconds, then turns off and the arm goes
 * to maximum. The pump runs until PD5, ADC channel 5, reaches
 * PRESSURE_PERCENT of full scale. Success waits for another mode edge,
 * opens the discharge valve for one second, and returns to motor drive.
 * If the pressure is still short after three seconds, the pump stops,
 * the error output turns on, and the valve opens only after the manual
 * discharge input. Another mode edge then returns to motor drive.
 */

#include "ch32fun.h"
#include "Servo.h"
#include <stdio.h>

static_assert( SERVO_TIM2_REMAP == 0, "both pulses use TIM2 remap group 0" );
static_assert( SERVO_TIM2_CH1_PIN == PD4, "ESC pulse is TIM2 CH1 on PD4" );
static_assert( SERVO_TIM2_CH2_PIN == PD3, "probe arm is TIM2 CH2 on PD3" );
static_assert( SERVO_SMOOTH == 1, "the arm and the ESC handoff ramp in the library" );

#define ADC_FULL_SCALE 4095u
#define ADC_THROTTLE_CH 2u
#define ADC_PRESSURE_CH 5u

#ifndef ADC_IDLE_PERCENT
#define ADC_IDLE_PERCENT 5
#endif
#define ADC_IDLE_THRESHOLD ((ADC_FULL_SCALE * ADC_IDLE_PERCENT) / 100u)

#ifndef PRESSURE_PERCENT
#define PRESSURE_PERCENT 50
#endif
#define PRESSURE_THRESHOLD ((ADC_FULL_SCALE * PRESSURE_PERCENT) / 100u)

#define IDLE_HOLD_MS     5000u
#define RETRACT_DWELL_MS 1500u
#define ATC_SPINDOWN_MS  10000u
#define ATC_PRESSURE_MS  3000u
#define DISCHARGE_MS     1000u
#define STEP_MS          20u
#define SWING_US_PER_S   1000u

#define MODE_PIN      0	/* PD0, active high */
#define LED_PIN       2	/* PD2 */
#define PRESSURE_PIN  5	/* PD5, ADC channel 5 */
#define DISCHARGE_IN  0	/* PC0, active high */
#define PUMP_PIN      1	/* PC1 */
#define VALVE_PIN     3	/* PC3 */
#define ERROR_PIN     6	/* PC6 */

static_assert( ADC_IDLE_THRESHOLD > 0, "idle threshold must be at least one count" );
static_assert( PRESSURE_THRESHOLD > ADC_IDLE_THRESHOLD, "pressure target is above the idle level" );

enum Mode
{
	MODE_ESC,
	MODE_EXTEND,
	MODE_RETRACT,
	MODE_DWELL,
	MODE_RESUME,
	MODE_ATC_SPINDOWN,
	MODE_ATC_PUMP,
	MODE_ATC_HOLD,
	MODE_ATC_ERROR,
	MODE_ATC_DISCHARGE,
	MODE_ATC_WAIT_MODE
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

	// Enable GPIOC, GPIOD, and ADC.
	RCC->PB2PCENR |= RCC_ADCEN | RCC_IOPCEN | RCC_IOPDEN;

	// PC4 is throttle channel 2. PD5 is pressure channel 5.
	GPIOC->CFGLR &= ~(GPIO_CFGLR_MASK<<(4*4));
	GPIOC->CFGLR |= (GPIO_CFGLR_IN_ANALOG<<(4*4));
	GPIOD->CFGLR &= ~(GPIO_CFGLR_MASK<<(4*PRESSURE_PIN));
	GPIOD->CFGLR |= (GPIO_CFGLR_IN_ANALOG<<(4*PRESSURE_PIN));

	// Reset the ADC to init all regs
	RCC->PB2PRSTR |= RCC_ADC1RST;
	RCC->PB2PRSTR &= ~RCC_ADC1RST;

	// Turn on ADC module
	ADC1->CTLR2 |= ADC_ADON;

	// Set up single conversion. The channel is written before each sample.
	ADC1->RSQR1 = 0;
	ADC1->RSQR2 = 0;
	ADC1->RSQR3 = ADC_THROTTLE_CH;

	// Slow sample time for the throttle and pressure channels.
	ADC1->SAMPTR2 &= ~((ADC_SMP0<<(3*ADC_THROTTLE_CH)) | (ADC_SMP0<<(3*ADC_PRESSURE_CH)));
	ADC1->SAMPTR2 |= (7<<(3*ADC_THROTTLE_CH)) | (7<<(3*ADC_PRESSURE_CH));

	// set External trigger event to SWSTART
	ADC1->CTLR2 = ADC1->CTLR2 & ~ADC_EXTSEL_SWSTART;
}

/*
 * start conversion, wait and return the 12-bit result
 */
uint16_t adc_read( uint8_t channel )
{
	ADC1->RSQR3 = channel;
	ADC1->CTLR2 |= ADC_FLAG_STRT;

	while(!(ADC1->STATR & ADC_EOC));

	return (uint16_t)(ADC1->RDATAR & ADC_FULL_SCALE);
}

static uint16_t adc_to_us( uint16_t adc )
{
	return (uint16_t)( SERVO_MIN_US +
		(uint32_t)adc * (SERVO_MAX_US - SERVO_MIN_US) / ADC_FULL_SCALE );
}

static void pin_output( GPIO_TypeDef *port, int pin )
{
	port->CFGLR &= ~(0xfu<<(4*pin));
	port->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_PP)<<(4*pin);
	port->BSHR = 1u<<(16+pin);
}

static void pin_input_pulldown( GPIO_TypeDef *port, int pin )
{
	port->CFGLR &= ~(0xfu<<(4*pin));
	port->CFGLR |= GPIO_CFGLR_IN_PUPD<<(4*pin);
	port->BSHR = 1u<<(16+pin);	/* ODR 0 selects the pull-down */
}

static void pin_write( GPIO_TypeDef *port, int pin, int on )
{
	port->BSHR = 1u<<(on ? pin : (16+pin));
}

static int pin_high( GPIO_TypeDef *port, int pin )
{
	return (port->INDR & (1u<<pin)) != 0;
}

static int rose( int level, int *was_high )
{
	int edge = level && !*was_high;
	*was_high = level;
	return edge;
}

static void led_set( int on )
{
	pin_write( GPIOD, LED_PIN, on );
}

static int adc_is_idle( uint16_t adc )
{
	return adc < ADC_IDLE_THRESHOLD;
}

static int adc_is_active( uint16_t adc )
{
	return adc > ADC_IDLE_THRESHOLD;
}

static int motor_mode( Mode mode )
{
	return mode <= MODE_RESUME;
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

static void probe_at_max( Servo &probe )
{
	probe.setSpeed( 0 );
	probe.writeMicroseconds( SERVO_MAX_US );
	probe.attach( SERVO_TIM2_CH2_PIN );
	probe.writeMicroseconds( SERVO_MAX_US );
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
	printf( "pressure target %u counts (%u%%)\n\r",
		(unsigned)PRESSURE_THRESHOLD, (unsigned)PRESSURE_PERCENT );

	printf( "initializing adc..." );
	adc_init();
	printf( "done.\n\r" );

	RCC->APB2PCENR |= RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD;

	pin_output( GPIOD, LED_PIN );
	pin_input_pulldown( GPIOD, MODE_PIN );
	pin_input_pulldown( GPIOC, DISCHARGE_IN );
	pin_output( GPIOC, PUMP_PIN );
	pin_output( GPIOC, VALVE_PIN );
	pin_output( GPIOC, ERROR_PIN );

	Servo esc;
	Servo probe;
	uint16_t throttle = adc_read( ADC_THROTTLE_CH );
	enable_esc( esc, adc_to_us( throttle ), 0 );

	Mode mode = MODE_ESC;
	int mode_was_high = 0;
	int discharge_was_high = 0;
	int return_to_motor = 0;
	uint32_t low_ms = 0;
	uint32_t dwell_ms = 0;
	uint32_t phase_ms = 0;
	uint32_t step = 0;

	while(1)
	{
		throttle = adc_read( ADC_THROTTLE_CH );
		uint16_t us = adc_to_us( throttle );
		uint16_t pressure = adc_read( ADC_PRESSURE_CH );
		int mode_edge = rose( pin_high( GPIOD, MODE_PIN ), &mode_was_high );
		int discharge_edge = rose( pin_high( GPIOC, DISCHARGE_IN ), &discharge_was_high );
		int report = (step % 5u) == 0;

		if( motor_mode( mode ) && mode_edge )
		{
			probe.detach();
			enable_esc( esc, SERVO_MIN_US, 0 );
			pin_write( GPIOC, PUMP_PIN, 0 );
			pin_write( GPIOC, VALVE_PIN, 0 );
			phase_ms = 0;
			mode = MODE_ATC_SPINDOWN;
			printf( "atc spindown\n\r" );
		}
		else switch( mode )
		{
		case MODE_ESC:
			esc.writeMicroseconds( us );
			led_set( 1 );
			if( adc_is_idle( throttle ) )
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
				printf( "esc adc: %u us: %u low: %lu\n\r", throttle, us, low_ms );
			break;

		case MODE_EXTEND:
			led_set( (step % 10u) < 5u );
			if( adc_is_active( throttle ) )
			{
				probe.writeMicroseconds( SERVO_MIN_US );
				mode = MODE_RETRACT;
				printf( "retract\n\r" );
			}
			if( report )
				printf( "extend adc: %u\n\r", throttle );
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
				printf( "retract adc: %u\n\r", throttle );
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
				printf( "dwell adc: %u ms: %lu\n\r", throttle, dwell_ms );
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
			else if( adc_is_idle( throttle ) )
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
				printf( "resume adc: %u us: %u\n\r", throttle, us );
			break;

		case MODE_ATC_SPINDOWN:
			esc.writeMicroseconds( SERVO_MIN_US );
			led_set( 1 );
			phase_ms += STEP_MS;
			if( phase_ms >= ATC_SPINDOWN_MS )
			{
				esc.detach();
				probe_at_max( probe );
				pin_write( GPIOC, PUMP_PIN, 1 );
				phase_ms = 0;
				mode = MODE_ATC_PUMP;
				printf( "atc pump\n\r" );
			}
			else if( report )
				printf( "atc spindown ms: %lu\n\r", phase_ms );
			break;

		case MODE_ATC_PUMP:
			led_set( (step % 10u) < 5u );
			if( pressure >= PRESSURE_THRESHOLD )
			{
				mode = MODE_ATC_HOLD;
				printf( "atc hold pressure: %u\n\r", pressure );
			}
			else
			{
				phase_ms += STEP_MS;
				if( phase_ms >= ATC_PRESSURE_MS )
				{
					pin_write( GPIOC, PUMP_PIN, 0 );
					pin_write( GPIOC, ERROR_PIN, 1 );
					mode = MODE_ATC_ERROR;
					printf( "error pressure: %u\n\r", pressure );
				}
				else if( report )
					printf( "atc pump pressure: %u\n\r", pressure );
			}
			break;

		case MODE_ATC_HOLD:
			led_set( (step % 10u) < 5u );
			if( mode_edge )
			{
				pin_write( GPIOC, PUMP_PIN, 0 );
				pin_write( GPIOC, VALVE_PIN, 1 );
				phase_ms = 0;
				return_to_motor = 1;
				mode = MODE_ATC_DISCHARGE;
				printf( "discharge\n\r" );
			}
			else if( report )
				printf( "atc hold pressure: %u\n\r", pressure );
			break;

		case MODE_ATC_ERROR:
			led_set( (step % 10u) < 5u );
			if( discharge_edge )
			{
				pin_write( GPIOC, VALVE_PIN, 1 );
				phase_ms = 0;
				return_to_motor = 0;
				mode = MODE_ATC_DISCHARGE;
				printf( "discharge\n\r" );
			}
			else if( report )
				printf( "atc error pressure: %u\n\r", pressure );
			break;

		case MODE_ATC_DISCHARGE:
			led_set( (step % 10u) < 5u );
			phase_ms += STEP_MS;
			if( phase_ms >= DISCHARGE_MS )
			{
				pin_write( GPIOC, VALVE_PIN, 0 );
				if( return_to_motor )
				{
					probe.detach();
					pin_write( GPIOC, ERROR_PIN, 0 );
					low_ms = 0;
					enable_esc( esc, us, 1 );
					mode = MODE_RESUME;
					printf( "resume\n\r" );
				}
				else
				{
					mode = MODE_ATC_WAIT_MODE;
					printf( "atc wait mode\n\r" );
				}
			}
			break;

		case MODE_ATC_WAIT_MODE:
			led_set( (step % 10u) < 5u );
			if( mode_edge )
			{
				probe.detach();
				pin_write( GPIOC, ERROR_PIN, 0 );
				pin_write( GPIOC, PUMP_PIN, 0 );
				pin_write( GPIOC, VALVE_PIN, 0 );
				low_ms = 0;
				enable_esc( esc, us, 1 );
				mode = MODE_RESUME;
				printf( "resume\n\r" );
			}
			else if( report )
				printf( "atc wait mode\n\r" );
			break;
		}

		step++;
		Delay_Ms( STEP_MS );
	}
}
