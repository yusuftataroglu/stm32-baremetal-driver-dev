# Experiment 003 - Multiple External LEDs Control

## Description
This experiment demonstrates controlling multiple external LEDs connected to GPIO pins PC10, PC11, and PC12. The LEDs are turned ON simultaneously when a button connected to PA1 is pressed, and turned OFF when the button is released.

## Concepts Practiced
- GPIO output mode with multiple pins
- GPIO input with pull-up resistor
- Reading button state to control LEDs
- Writing to output port with bit masking

## Circuit
- LEDs: PC10, PC11, PC12 (external LEDs)
- Button: PA1 with pull-up resistor (external button)

## Expected Behavior
When the button is pressed, all three LEDs turn on simultaneously. When released, LEDs turn off.

## Result
Successfully tested on NUCLEO-F103RB.
