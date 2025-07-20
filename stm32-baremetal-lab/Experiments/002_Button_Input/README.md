# Experiment 002 - Button Input

## Description
This experiment uses the onboard button (PC13) as a digital input to toggle the onboard LED (PA5) when pressed.

## Concepts Practiced
- GPIO input mode
- Button debounce handling (basic delay)
- GPIO output toggle
- Bit masking

## Circuit
- LED: PA5 (onboard)
- Button: PC13 (onboard)

## Expected Behavior
When the button is pressed, the LED toggles state.

## Result
Successfully tested on NUCLEO-F103RB.