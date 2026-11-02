# Polled ADC with a servo sweep

CH32V006 example that runs the polled ADC from `examples_x00x/adc_polled` while a servo sweeps the way `examples/servo_sweep` does in [ch32-libraries/Servo](https://github.com/ch32-libraries/Servo).

The ADC is single-channel and software-triggered. Each conversion busy-waits until the result is ready, then the 12-bit sample is printed. At the same time TIM2 generates the servo pulse in hardware, so the pulse width does not depend on the ADC wait or on `printf`. The sweep walks the pulse from 1000 µs to 2000 µs and back in 10 µs steps, one step every 20 ms. PD0 is on for 100 ms and off for 100 ms, the same cadence as `adc_polled`.

`Servo.h` and `Servo.cpp` in this directory are the ISC-licensed library at commit `27b30675a200a4a856ba0e0eeaf7556e644fe4e1`. See `LICENSE`.

## Pins

| Pin | Function |
| --- | --- |
| PC4 | Analog input, ADC channel 2. 0 V to 3.3 V. |
| PC1 | Servo signal. TIM2 CH1 after remap group 1. |
| PD0 | Blinks 100 ms on, 100 ms off. |
| PD1 | SWIO. Leave this for programming and debug `printf`. |

PC4 is an ADC pin on every CH32V006 package, and remap group 1 does not use it for TIM2. Give the servo its own supply and tie the grounds together. The signal is 3.3 V.

## Use

```
make build
```

`make` programs the chip with minichlink. Each line on the debug printf output is the sample count, the ADC reading, and the pulse width in microseconds.
