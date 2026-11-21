all : flash

TARGET:=adc_servo_sweep
TARGET_EXT:=cpp

TARGET_MCU?=CH32V006
ADDITIONAL_C_FILES+=Servo.cpp

include ../../ch32fun/ch32fun.mk

# After the include: ch32fun.mk sets CFLAGS with `?=`.
# No exceptions or RTTI: ch32fun links a freestanding libgcc with no abort/malloc.
CFLAGS+=-I. -fno-exceptions -fno-unwind-tables -fno-asynchronous-unwind-tables

flash : cv_flash
clean : cv_clean
