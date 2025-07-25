# stm32-baremetal-driver-dev
Bu repo, STM32F103RB için sıfırdan sürücü yazımı ve bare-metal uygulamaları içerir.


## Experiments

| ID   | Title                                                                                  | Description                                                                       |
|------|----------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------|
| 001  | [LED Toggle](stm32-baremetal-lab/Experiments/001_Led_Toggle)                           | Toggle onboard LED on PA5 withdelay                                               |
| 002  | [Button Input](stm32-baremetal-lab/Experiments/002_Button_Input)                       | Toggle LED when button (PC13) pressed                                             |
| 003  | [Multiple Leds](stm32-baremetal-lab/Experiments/003_Multiple_Leds)                     | Toggle external LEDs when external button connected to (PA1) pressed              |
| 004  | [Button Interrupt](stm32-baremetal-lab/Experiments/004_Button_Interrupt)               | Toggle onboard LED when button (PC13) interrupt occurs (EXTI15_10)                |
| 005  | [Interrupt Priority](stm32-baremetal-lab/Experiments/005_Interrupt_Priority)           | Demonstrates NVIC interrupt preemption by assigning priorities                    |
| 006  | [SPI Led Shift Register](stm32-baremetal-lab/Experiments/006_SPI_LED_ShiftRegister)    | Sending data via SPI to 74HC595 shift register to toggle Leds given pattern       |
| 007  | [SPI Loopback RX](stm32-baremetal-lab/Experiments/007_SPI_Loopback_RX)                 | Verifies SPI TX and RX using loopback by connecting MOSI to MISO (no slave used)  |
