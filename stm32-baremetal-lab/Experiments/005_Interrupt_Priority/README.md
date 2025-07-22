# Experiment 005 - Interrupt Priority

## Description
This experiment demonstrates how NVIC interrupt priority and preemption work on the STM32F103. Two external buttons generate interrupts on EXTI0 and EXTI4. By assigning different NVIC priority levels to each interrupt, preemptive behavior is observed when both interrupts are triggered in close succession.

## Concepts Practiced
- External interrupt setup using EXTI and AFIO
- NVIC interrupt enabling and priority configuration
- Interrupt preemption with nested handler execution
- GPIO output toggling via EXTI ISR

## Circuit
- LED1: PA0 – toggled by EXTI0 (Button 1)
- LED2: PA1 – toggled by EXTI4 (Button 2)
- Button1: PB0 with pull-up – triggers EXTI0
- Button2: PB4 with pull-up – triggers EXTI4

## Expected Behavior
- When only one button is pressed, its corresponding LED toggles.
- When both buttons are pressed nearly at the same time:
- LED1 toggles first, because EXTI0 has a higher NVIC priority than EXTI4.
- EXTI4 ISR is preempted if EXTI0 is triggered while EXTI4 is still running.

## Result
Successfully verified on NUCLEO-F103RB.  
NVIC priorities and nested interrupt behavior observed correctly.