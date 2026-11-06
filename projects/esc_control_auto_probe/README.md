# ESC control with a spindle-probe arm

CH32V006 project based on `examples_x00x/adc_servo_sweep`. Two TIM2 pulses share one timer. Only one of them is enabled at a time. `Servo.h` and `Servo.cpp` come from that example (ISC license, Servo commit `27b30675a200a4a856ba0e0eeaf7556e644fe4e1`).

## Pins

| Pin | Function |
| --- | --- |
| PC4 | Analog input, ADC channel 2. 0 V to 3.3 V. |
| PD4 | ESC signal. TIM2 CH1. 1000 µs at 0 counts, 2000 µs at full scale. |
| PD3 | Probe arm. TIM2 CH2. Swings between the same 1000 µs and 2000 µs limits. |
| PD0 | On while the ESC pulse is enabled. Blinks 100 ms while the arm pulse is enabled. |
| PD1 | SWIO. Leave this for programming and debug `printf`. |

Give the ESC its own motor supply and tie the grounds together. Both signals are 3.3 V. Remove the propeller before the first run. The arm must be clear of the spindle at the minimum pulse.

## What it does

The ESC pulse follows the ADC directly. The idle level defaults to 5% of full scale (204 counts of 4095). Override it with `ADC_IDLE_PERCENT` on the compiler command line.

1. While the ESC pulse is enabled, the ADC has to stay under that level for 5 seconds. The timer resets if the input rises to the level or above.
2. The ESC pulse is then disabled and the arm swings from 1000 µs to 2000 µs at 1000 µs per second.
3. When the ADC goes above the idle level, the arm swings back to 1000 µs at the same rate.
4. 1.5 seconds after the arm reaches minimum, its pulse is disabled. The ESC pulse is enabled at 1000 µs and ramps, at 1000 µs per second, to the width the ADC is asking for. After that ramp, the ESC tracks the ADC on the next frame.

## Use

```
make build
```

`make` programs the chip with minichlink. Debug `printf` lines are tagged `esc`, `extend`, `retract`, `dwell`, and `resume`.
