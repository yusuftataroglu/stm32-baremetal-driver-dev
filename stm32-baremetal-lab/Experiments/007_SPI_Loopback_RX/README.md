# Experiment 007 - SPI Loopback Test

## Description
This experiment verifies the SPI transmit and receive functions using a loopback setup. The MOSI and MISO pins of SPI1 are physically connected, allowing data sent out to be received back immediately.

> NOTE! In this test, SPI_ReceiveData() was temporarily modified to not send dummy data (0xFF) during reception. This simplification works because the master has already sent valid data through SPI_SendData(), and the loopback wire returns that data to MISO.  
>
> In a real-world scenario with an external slave device, dummy data transmission must be enabled during reception to generate SPI clock cycles.

## Concepts Practiced
- Full-duplex SPI communication
- SPI send and receive API validation
- Loopback testing without an external slave
- Clock/data flow debugging

## Circuit
- SPI1 (Master Mode)
- PA5 (SCK) – unused
- PA6 (MISO) connected to PA7 (MOSI)
- No slave device required

## Expected Behavior
- Transmitted data (e.g., 0xAB) is received back through the MISO line.
- If the received byte matches the transmitted byte, loopback is considered successful.
- A debug LED or breakpoint is used to confirm the result.

## Result
Loopback test successfully verified on NUCLEO-F103RB.  
SPI_SendData() and SPI_ReceiveData() functions worked as expected when looped back.  
Dummy transmit logic will be re-enabled for future real-world SPI tests involving slave devices.
