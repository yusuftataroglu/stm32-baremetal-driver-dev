# stm32-baremetal-driver-dev

This repository documents the bare-metal driver development process for the STM32F103RB microcontroller.  
All drivers are implemented from scratch in C without relying on HAL or CMSIS libraries.

My goal is to deeply understand the STM32 architecture, peripheral register-level programming, and to build reusable low-level drivers suitable for professional embedded systems development.

Experiments are designed to validate each driver module and demonstrate their integration in practical scenarios.  
This repository serves both as a learning archive and a professional portfolio.


**Açıklama (TR)**  
Bu depo, STM32F103RB mikrodenetleyicisi için bare-metal (donanıma yakın seviye) sürücü geliştirme sürecini belgelendirir.
Tüm sürücüler HAL veya CMSIS gibi hazır kütüphaneler kullanılmadan, sıfırdan C dilinde yazılmıştır.

Amacım; STM32 mimarisini, çevresel donanım birimlerinin (peripheral) register seviyesindeki kontrolünü derinlemesine öğrenmek ve profesyonel gömülü sistem geliştirme projelerinde kullanılabilecek yeniden kullanılabilir sürücüler oluşturmaktır.

Her deney, ilgili sürücünün işlevselliğini test etmek ve farklı modüllerin bir arada nasıl çalıştığını göstermek için tasarlanmıştır.
Bu depo hem kişisel bir öğrenme arşivi hem de profesyonel bir portföy çalışması olarak kullanılmaktadır.

---

## Experiments

| ID   | Title                                                                                  | Description                                                                                       |
|------|----------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------|
| 001  | [LED Toggle](stm32-baremetal-lab/Experiments/001_Led_Toggle)                           | Toggle onboard LED on PA5 with delay                                                              |
| 002  | [Button Input](stm32-baremetal-lab/Experiments/002_Button_Input)                       | Toggle LED when button (PC13) pressed                                                             |
| 003  | [Multiple LEDs](stm32-baremetal-lab/Experiments/003_Multiple_Leds)                     | Toggle external LEDs when external button connected to PA1 is pressed                             |
| 004  | [Button Interrupt](stm32-baremetal-lab/Experiments/004_Button_Interrupt)               | Toggle onboard LED using external interrupt from PC13 (EXTI15_10)                                 |
| 005  | [Interrupt Priority](stm32-baremetal-lab/Experiments/005_Interrupt_Priority)           | Demonstrates NVIC preemption with different interrupt priority levels                             |
| 006  | [SPI LED Shift Register](stm32-baremetal-lab/Experiments/006_SPI_LED_ShiftRegister)    | Control LEDs through SPI using 74HC595 shift register                                             |
| 007  | [SPI Loopback RX](stm32-baremetal-lab/Experiments/007_SPI_Loopback_RX)                 | Test SPI transmit and receive in full-duplex mode with MISO-MOSI loopback                         |
| 008  | [SPI IT Loopback](stm32-baremetal-lab/Experiments/008_SPI_IT_Loopback)                 | Full-duplex SPI communication using interrupts, tested with loopback wiring and LED indicators.   |

---
