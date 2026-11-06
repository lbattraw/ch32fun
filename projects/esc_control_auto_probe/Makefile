all : flash

TARGET:=esc_control_auto_probe
TARGET_EXT:=cpp

TARGET_MCU?=CH32V006
ADDITIONAL_C_FILES+=../../examples_x00x/adc_servo_sweep/Servo.cpp

include ../../ch32fun/ch32fun.mk

# After the include: ch32fun.mk sets CFLAGS with `?=`.
# SERVO_TIM2_REMAP is a command-line define so Servo.cpp and this file
# select the same TIM2 pin. No exceptions: ch32fun's libgcc has no abort.
CFLAGS+=-I../../examples_x00x/adc_servo_sweep -DSERVO_TIM2_REMAP=0 -DSERVO_SMOOTH=1 \
	-fno-exceptions -fno-unwind-tables -fno-asynchronous-unwind-tables

flash : cv_flash
clean : cv_clean
