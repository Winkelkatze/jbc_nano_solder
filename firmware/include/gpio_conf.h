#pragma once

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/exti.h>

// GPIOA
#define GPIO_ENCODER_A  GPIO4
#define GPIO_ENCODER_B  GPIO5
#define GPIO_ENCODER_SW GPIO6
#define GPIO_IDLE_SW    GPIO7

// GBIOB
#define GPIO_HEATER     GPIO1

#define EXTI_ENCODER_A  EXTI4
#define EXTI_ENCODER_SW EXTI6
#define EXTI_IDLE_SW    EXTI7
