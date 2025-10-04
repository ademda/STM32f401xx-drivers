# STM32f401xx-drivers
In this repository you find UART/USART, SPI, I2C, GPIO, RCC and EXTI drivers for STM32f407xx MCU with documentation
# STM32F401xx Peripheral Drivers

This repository contains custom peripheral drivers for the STM32F401xx microcontroller family, developed from scratch without using HAL libraries.

## Table of Contents
- [Overview](#overview)
- [GPIO Driver](#gpio-driver)
- [SPI Driver](#spi-driver)
- [I2C Driver](#i2c-driver)
- [USART Driver](#usart-driver)
- [RCC Driver](#rcc-driver)
- [EXTI Configuration](#exti-configuration)
- [Usage Examples](#usage-examples)
- [Project Structure](#project-structure)

## Overview

This project provides low-level drivers for STM32F401xx peripherals, offering direct register-level control with a clean API interface. All drivers are built without dependencies on ST's HAL libraries.

**Supported Peripherals:**
- GPIO (General Purpose Input/Output)
- SPI (Serial Peripheral Interface)
- I2C (Inter-Integrated Circuit)
- USART (Universal Synchronous/Asynchronous Receiver/Transmitter)
- RCC (Reset and Clock Control)
- EXTI (External Interrupt)

---

## GPIO Driver

### Configuration Registers (CRx)

#### MODER (Mode Register)
Controls the I/O direction mode of each pin:
- `00`: Input mode
- `01`: General purpose output mode
- `10`: Alternate function mode
- `11`: Analog mode

#### OTYPER (Output Type Register)
Configures the output type:
- `0`: Push-pull
- `1`: Open-drain

#### OSPEEDR (Output Speed Register)
Controls the I/O output speed:
- `00`: Low speed
- `01`: Medium speed
- `10`: High speed
- `11`: Very high speed

#### PUPDR (Pull-up/Pull-down Register)
Configures internal pull resistors:
- `00`: No pull-up, no pull-down
- `01`: Pull-up
- `10`: Pull-down
- `11`: Reserved

#### AFRL/AFRH (Alternate Function Registers)
Selects alternate function for pins (AF0-AF15).

### Status Registers

#### IDR (Input Data Register)
- **Read-only** register containing input values of GPIO pins
- Each bit represents the input state of corresponding pin

#### ODR (Output Data Register)
- **Read/Write** register for output data
- Writing sets output value for pins configured as outputs

### Key Features
- Pin-level configuration control
- Interrupt support via EXTI
- Alternate function mapping
- Atomic bit set/reset operations

---

## SPI Driver

### Configuration Registers

#### CR1 (Control Register 1)
- **CPHA** [0]: Clock phase
- **CPOL** [1]: Clock polarity  
- **MSTR** [2]: Master selection
- **BR[2:0]** [5:3]: Baud rate control
- **SPE** [6]: SPI enable
- **LSBFIRST** [7]: Frame format
- **SSI** [8]: Internal slave select
- **SSM** [9]: Software slave management
- **RXONLY** [10]: Receive only
- **DFF** [11]: Data frame format (8/16-bit)
- **CRCNEXT** [12]: CRC transfer next
- **CRCEN** [13]: Hardware CRC calculation enable
- **BIDIOE** [14]: Output enable in bidirectional mode
- **BIDIMODE** [15]: Bidirectional data mode enable

#### CR2 (Control Register 2)
- **RXDMAEN** [0]: RX buffer DMA enable
- **TXDMAEN** [1]: TX buffer DMA enable
- **SSOE** [2]: SS output enable
- **FRF** [4]: Frame format (SPI/TI)
- **ERRIE** [5]: Error interrupt enable
- **RXNEIE** [6]: RX buffer not empty interrupt enable
- **TXEIE** [7]: TX buffer empty interrupt enable

### Status Register (SR)
- **RXNE** [0]: Receive buffer not empty
- **TXE** [1]: Transmit buffer empty
- **CHSIDE** [2]: Channel side (I2S)
- **UDR** [3]: Underrun flag (I2S)
- **CRCERR** [4]: CRC error flag
- **MODF** [5]: Mode fault
- **OVR** [6]: Overrun flag
- **BSY** [7]: Busy flag
- **FRE** [8]: Frame error (TI mode)

### Supported Modes
- Master/Slave operation
- Full-duplex, Half-duplex, Simplex
- 8-bit and 16-bit data frames
- Hardware/Software NSS management

---

## I2C Driver

### Configuration Registers

#### CR1 (Control Register 1)
- **PE** [0]: Peripheral enable
- **SMBUS** [1]: SMBus mode
- **SMBTYPE** [3]: SMBus type
- **ENARP** [4]: ARP enable
- **ENPEC** [5]: PEC enable
- **ENGC** [6]: General call enable
- **NOSTRETCH** [7]: Clock stretching disable
- **START** [8]: Start generation
- **STOP** [9]: Stop generation
- **ACK** [10]: Acknowledge enable
- **POS** [11]: Acknowledge/PEC position
- **PEC** [12]: Packet error checking
- **ALERT** [13]: SMBus alert
- **SWRST** [15]: Software reset

#### CR2 (Control Register 2)
- **FREQ[5:0]** [5:0]: Peripheral clock frequency
- **ITERREN** [8]: Error interrupt enable
- **ITEVTEN** [9]: Event interrupt enable
- **ITBUFEN** [10]: Buffer interrupt enable
- **DMAEN** [11]: DMA requests enable
- **LAST** [12]: DMA last transfer

#### CCR (Clock Control Register)
- **CCR[11:0]** [11:0]: Clock control register
- **DUTY** [14]: FM mode duty cycle
- **FS** [15]: Fast mode selection

### Status Registers

#### SR1 (Status Register 1)
- **SB** [0]: Start bit
- **ADDR** [1]: Address sent/matched
- **BTF** [2]: Byte transfer finished
- **ADD10** [3]: 10-bit header sent
- **STOPF** [4]: Stop detection
- **RXNE** [6]: Data register not empty
- **TXE** [7]: Data register empty
- **BERR** [8]: Bus error
- **ARLO** [9]: Arbitration lost
- **AF** [10]: Acknowledge failure
- **OVR** [11]: Overrun/Underrun
- **PECERR** [12]: PEC error
- **TIMEOUT** [14]: Timeout error
- **SMBALERT** [15]: SMBus alert

#### SR2 (Status Register 2)
- **MSL** [0]: Master/Slave
- **BUSY** [1]: Bus busy
- **TRA** [2]: Transmitter/Receiver
- **GENCALL** [4]: General call address
- **SMBDEFAULT** [5]: SMBus device default address
- **SMBHOST** [6]: SMBus host header
- **DUALF** [7]: Dual flag
- **PEC[7:0]** [15:8]: Packet error checking register

### Communication Modes
- Standard mode (up to 100 kHz)
- Fast mode (up to 400 kHz)
- 7-bit and 10-bit addressing
- Master and Slave operation

---

## USART Driver

### Configuration Registers

#### CR1 (Control Register 1)
- **SBK** [0]: Send break
- **RWU** [1]: Receiver wakeup
- **RE** [2]: Receiver enable
- **TE** [3]: Transmitter enable
- **IDLEIE** [4]: IDLE interrupt enable
- **RXNEIE** [5]: RXNE interrupt enable
- **TCIE** [6]: Transmission complete interrupt enable
- **TXEIE** [7]: TXE interrupt enable
- **PEIE** [8]: PE interrupt enable
- **PS** [9]: Parity selection
- **PCE** [10]: Parity control enable
- **WAKE** [11]: Wakeup method
- **M** [12]: Word length
- **UE** [13]: USART enable
- **OVER8** [15]: Oversampling mode

#### CR2 (Control Register 2)
- **ADD[3:0]** [3:0]: Address of USART node
- **LBDL** [5]: LIN break detection length
- **LBDIE** [6]: LIN break detection interrupt enable
- **LBCL** [8]: Last bit clock pulse
- **CPHA** [9]: Clock phase
- **CPOL** [10]: Clock polarity
- **CLKEN** [11]: Clock enable
- **STOP[1:0]** [13:12]: Stop bits
- **LINEN** [14]: LIN mode enable

#### CR3 (Control Register 3)
- **EIE** [0]: Error interrupt enable
- **IREN** [1]: IrDA mode enable
- **IRLP** [2]: IrDA low-power
- **HDSEL** [3]: Half-duplex selection
- **NACK** [4]: Smartcard NACK enable
- **SCEN** [5]: Smartcard mode enable
- **DMAR** [6]: DMA enable receiver
- **DMAT** [7]: DMA enable transmitter
- **RTSE** [8]: RTS enable
- **CTSE** [9]: CTS enable
- **CTSIE** [10]: CTS interrupt enable
- **ONEBIT** [11]: One sample bit method enable

### Status Register (SR)
- **PE** [0]: Parity error
- **FE** [1]: Framing error
- **NE** [2]: Noise detected flag
- **ORE** [3]: Overrun error
- **IDLE** [4]: IDLE line detected
- **RXNE** [5]: Read data register not empty
- **TC** [6]: Transmission complete
- **TXE** [7]: Transmit data register empty
- **LBD** [8]: LIN break detection flag
- **CTS** [9]: CTS flag

### BRR (Baud Rate Register)
- **DIV_Fraction[3:0]** [3:0]: Fraction of USARTDIV
- **DIV_Mantissa[11:0]** [15:4]: Mantissa of USARTDIV

### Communication Features
- Asynchronous/Synchronous modes
- Hardware flow control (RTS/CTS)
- Multi-processor communication
- LIN (Local Interconnect Network) support
- IrDA SIR support
- Smartcard mode

---

## RCC Driver

### Configuration Registers

#### CR (Clock Control Register)
- **HSION** [0]: HSI clock enable
- **HSIRDY** [1]: HSI clock ready flag
- **HSITRIM[4:0]** [7:3]: HSI clock trimming
- **HSICAL[7:0]** [15:8]: HSI clock calibration
- **HSEON** [16]: HSE clock enable
- **HSERDY** [17]: HSE clock ready flag
- **HSEBYP** [18]: HSE clock bypass
- **CSSON** [19]: Clock security system enable
- **PLLON** [24]: Main PLL enable
- **PLLRDY** [25]: Main PLL ready flag
- **PLLI2SON** [26]: PLLI2S enable
- **PLLI2SRDY** [27]: PLLI2S ready flag

#### PLLCFGR (PLL Configuration Register)
- **PLLM[5:0]** [5:0]: Division factor for main PLL input clock
- **PLLN[8:0]** [14:6]: Main PLL multiplication factor for VCO
- **PLLP[1:0]** [17:16]: Main PLL division factor for main system clock
- **PLLSRC** [22]: Main PLL entry clock source
- **PLLQ[3:0]** [27:24]: Main PLL division factor for USB OTG FS, SDIO clocks

#### CFGR (Clock Configuration Register)
- **SW[1:0]** [1:0]: System clock switch
- **SWS[1:0]** [3:2]: System clock switch status
- **HPRE[3:0]** [7:4]: AHB prescaler
- **PPRE1[2:0]** [12:10]: APB Low speed prescaler (APB1)
- **PPRE2[2:0]** [15:13]: APB high-speed prescaler (APB2)
- **RTCPRE[4:0]** [20:16]: HSE division factor for RTC clock
- **MCO1[1:0]** [22:21]: Microcontroller clock output 1
- **MCO1PRE[2:0]** [26:24]: MCO1 prescaler
- **MCO2PRE[2:0]** [29:27]: MCO2 prescaler
- **MCO2[1:0]** [31:30]: Microcontroller clock output 2

### Clock Enable Registers
- **AHB1ENR**: AHB1 peripheral clock enable register
- **AHB2ENR**: AHB2 peripheral clock enable register  
- **APB1ENR**: APB1 peripheral clock enable register
- **APB2ENR**: APB2 peripheral clock enable register

---

## EXTI Configuration

The External Interrupt/Event Controller (EXTI) manages external interrupts and events from GPIO pins and other sources.

### Configuration Registers

#### IMR (Interrupt Mask Register)
- **MR[15:0]**: Interrupt mask for lines 0-15
- `1`: Interrupt request from line x is unmasked
- `0`: Interrupt request from line x is masked

#### EMR (Event Mask Register)  
- **MR[15:0]**: Event mask for lines 0-15
- `1`: Event request from line x is unmasked
- `0`: Event request from line x is masked

#### RTSR (Rising Trigger Selection Register)
- **TR[15:0]**: Rising trigger event configuration for lines 0-15
- `1`: Rising edge trigger enabled
- `0`: Rising edge trigger disabled

#### FTSR (Falling Trigger Selection Register)
- **TR[15:0]**: Falling trigger event configuration for lines 0-15
- `1`: Falling edge trigger enabled  
- `0`: Falling edge trigger disabled

#### SWIER (Software Interrupt Event Register)
- **SWIER[15:0]**: Software interrupt for lines 0-15
- `1`: Generates an interrupt/event request
- `0`: No action

#### PR (Pending Register)
- **PR[15:0]**: Pending bit for lines 0-15
- `1`: Selected trigger request occurred
- `0`: No trigger request occurred
- **Clear**: Write `1` to clear the bit

### SYSCFG EXTICR Configuration

The SYSCFG_EXTICR registers select which GPIO port is connected to each EXTI line:

#### EXTICR1 (External Interrupt Configuration Register 1)
- **EXTI3[3:0]** [15:12]: EXTI 3 configuration
- **EXTI2[3:0]** [11:8]: EXTI 2 configuration  
- **EXTI1[3:0]** [7:4]: EXTI 1 configuration
- **EXTI0[3:0]** [3:0]: EXTI 0 configuration

#### EXTICR2-4
Similar configuration for EXTI lines 4-15.

**Port Selection Values:**
- `0000`: PA[x] pin
- `0001`: PB[x] pin  
- `0010`: PC[x] pin
- `0011`: PD[x] pin
- `0100`: PE[x] pin
- `0101`: PF[x] pin
- `0110`: PG[x] pin
- `0111`: PH[x] pin

### EXTI Line Mapping
- **Lines 0-15**: Connected to GPIO pins (PA0-PA15, PB0-PB15, etc.)
- **Line 16**: Connected to PVD output
- **Line 17**: Connected to RTC Alarm event  
- **Line 18**: Connected to USB OTG FS Wakeup event
- **Line 21**: Connected to RTC Tamper and TimeStamp events
- **Line 22**: Connected to RTC Wakeup event

### NVIC Integration
Each EXTI line maps to specific NVIC interrupt vectors:
- **EXTI0**: EXTI0_IRQn (IRQ 6)
- **EXTI1**: EXTI1_IRQn (IRQ 7)
- **EXTI2**: EXTI2_IRQn (IRQ 8)  
- **EXTI3**: EXTI3_IRQn (IRQ 9)
- **EXTI4**: EXTI4_IRQn (IRQ 10)
- **EXTI9_5**: EXTI9_5_IRQn (IRQ 23) - Lines 5-9
- **EXTI15_10**: EXTI15_10_IRQn (IRQ 40) - Lines 10-15

---

## Usage Examples

### GPIO Example
```c
#include "stm32f401xx_gpio_driver.h"

// Configure PA5 as output (LED)
GPIO_Handle_t gpioLed;
gpioLed.pGPIOx = GPIOA;
gpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
gpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
gpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
gpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
gpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

GPIO_PeriClockControl(GPIOA, ENABLE);
GPIO_Init(&gpioLed);
GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_NO_5, GPIO_PIN_SET);
```

### USART Example  
```c
#include "stm32f401xx_usart_driver.h"

// Configure USART2
USART_Handle_t usart2Handle;
usart2Handle.pUSARTx = USART2;
usart2Handle.USART_Config.USART_Baud = USART_STD_BAUD_115200;
usart2Handle.USART_Config.USART_Mode = USART_MODE_TXRX;
usart2Handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
usart2Handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
usart2Handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
usart2Handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;

USART_Init(&usart2Handle);
USART_PeripheralControl(USART2, ENABLE);

// Send data
uint8_t data[] = "Hello World!";
USART_SendData(&usart2Handle, data, strlen((char*)data));
```

### EXTI Example
```c
#include "stm32f401xx_gpio_driver.h"

// Configure PC13 for external interrupt
GPIO_Handle_t gpioBtn;
gpioBtn.pGPIOx = GPIOC;
gpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
gpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT; // Falling edge trigger
gpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
gpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

GPIO_PeriClockControl(GPIOC, ENABLE);
GPIO_Init(&gpioBtn);

// Configure NVIC
GPIO_IRQPriorityConfig(IRQ_NO_EXTI15_10, NVIC_IRQ_PRI15);
GPIO_IRQInterruptConfig(IRQ_NO_EXTI15_10, ENABLE);
```

---

## Project Structure

```
stm32f4xx_drivers/
├── drivers/
│   ├── Inc/
│   │   ├── stm32f401xx.h                    # MCU specific header
│   │   ├── stm32f401xx_gpio_driver.h        # GPIO driver header  
│   │   ├── stm32f401xx_spi_driver.h         # SPI driver header
│   │   ├── stm32f401xx_i2c_driver.h         # I2C driver header
│   │   ├── stm32f401xx_usart_driver.h       # USART driver header
│   │   └── stm32f401xx_rcc_driver.h         # RCC driver header
│   └── Src/
│       ├── stm32f401xx_gpio_driver.c        # GPIO driver implementation
│       ├── stm32f401xx_spi_driver.c         # SPI driver implementation  
│       ├── stm32f401xx_i2c_driver.c         # I2C driver implementation
│       ├── stm32f401xx_usart_driver.c       # USART driver implementation
│       └── stm32f401xx_rcc_driver.c         # RCC driver implementation
├── Src/
│   ├── 001led_toggle.c                      # GPIO LED toggle example
│   ├── 002_spi_send_data.c                  # SPI data transmission example
│   ├── i2c_master_tx_testing.c              # I2C master transmission test
│   └── usart_tx.c                           # USART transmission test
├── Inc/                                     # Application headers
├── Startup/
│   └── startup_stm32f401retx.s              # Startup assembly file
└── README.md                                # This file
```

---

## Hardware Requirements

- **STM32F401RE Nucleo Board** or compatible STM32F401xx development board
- **Logic Analyzer** (optional, for signal verification)
- **Oscilloscope** (optional, for advanced debugging)
- **External devices** for testing (LEDs, buttons, sensors, etc.)

---

## Build Instructions

1. **Import Project**: Import into STM32CubeIDE or compatible IDE
2. **Build Configuration**: Select Debug or Release configuration  
3. **Compile**: Build the project using the provided Makefile or IDE
4. **Flash**: Program the target device using ST-LINK debugger
5. **Debug**: Use integrated debugger for testing and verification

---

## Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/new-driver`)
3. Commit changes (`git commit -am 'Add new driver feature'`)
4. Push to branch (`git push origin feature/new-driver`)
5. Create Pull Request

---

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

## Author

**STM32F401xx Driver Development**
- Custom peripheral drivers written from scratch
- Register-level programming without HAL dependencies
- Optimized for educational and professional use

---

## Acknowledgments

- STM32F401xx Reference Manual (RM0368)
- STM32F401xB/STM32F401xC Datasheet  
- ARM Cortex-M4 Technical Reference Manual
- STM32CubeIDE Development Environment
