# Experiment 008 - SPI Interrupt-Based Loopback

## Description
This experiment tests SPI full-duplex communication using interrupt-driven data transfer. The SPI1 peripheral operates in master mode, and its MOSI and MISO pins are physically connected to form a loopback configuration. Data is sent and received using non-blocking, interrupt-based functions.

Two onboard LEDs are used as indicators:
LED on PA0: toggles when transmission is completed.

LED on PA1: toggles when received data matches the transmitted data.
If a mismatch occurs, the LED on PA1 is turned on permanently to indicate a failure.

## Concepts Practiced
- Interrupt-driven SPI communication
- SPI TX and RX completion callbacks
- Full-duplex communication in loopback mode
- Peripheral and NVIC configuration for SPI
- Simple memory comparison using memcmp()

## Circuit
- SPI1 configured as master
- PA5 (SCK) – not connected
- PA6 (MISO) connected to PA7 (MOSI) via jumper wire
- PA0: Transmit indicator LED
- PA1: Receive indicator LED
- No external slave device required

# Expected Behavior
- Transmit and receive operations are initiated using interrupts.
- When transmission completes, LED on PA0 toggles 10 times.
- When reception completes, received data is compared to the transmitted string "Interrupt SPI!".
- If the comparison is successful, LED on PA1 toggles 10 times.
- If the comparison fails, LED on PA1 is set HIGH and stays on.

## Result
Interrupt-based SPI communication was successfully verified on NUCLEO-F103RB.
The loopback test confirmed correct operation of the following functions:

SPI_SendData_IT()
SPI_ReceiveData_IT()
SPI_IRQHandling()
SPI_ApplicationEventCallback()

No issues were observed in interrupt triggering or data integrity during the test.
This test forms a foundation for future SPI applications involving real-time data exchange with external devices.