# Experiment 004 - Button Interrupt

## Description
This experiment demonstrates how to use external interrupts (EXTI) with GPIO on the STM32F103RB. A button connected to pin **PC13** is configured to generate an interrupt on a falling edge. When the interrupt occurs, an onboard LED connected to **PA5** is toggled.

## Concepts Practiced
- GPIO interrupt mode configuration (GPIO_MODE_IT_FT)
- AFIO EXTI line mapping
- EXTI falling edge trigger and unmasking
- NVIC IRQ enabling and ISR implementation
- Pending bit clearing in EXTI
- Delay implementation inside ISR (for button debouncing)

## Circuit
- LED: PA5 (onboard LED)
- Button: PC13 (onboard button, pulled-up by default)

## Expected Behavior
When the button is pressed, it triggers an interrupt. The ISR toggles the LED and clears the EXTI pending bit. The LED state changes on each button press.

## Result
Successfully tested on NUCLEO-F103RB.
