Raspberry Pi Pico USB-HW_and_PIO-UART Bridge
=================================
This expands Noltari's project to add 4 additional UARTs using the pico PIOs.
It requires a "fix" in tinyusb - ~/pico/pico-sdk/lib/tinyusb/src/device/usbd.c (thanks hippy)

#define CFG_TUD_EP_MAX          15

The PIO uarts are currently all 8x1. They will support but not check odd/even/no parity bit

Raspberry Pi Pico USB-UART Bridge
=================================

This program bridges the Raspberry Pi Pico HW UARTs to two independent USB CDC serial devices in order to behave like any other USB-to-UART Bridge controllers.

Disclaimer
----------

This software is provided without warranty, according to the MIT License, and should therefore not be used where it may endanger life, financial stakes, or cause discomfort and inconvenience to others.

Raspberry Pi Pico Pinout
------------------------

| Raspberry Pi Pico GPIO | Function |
|:----------------------:|:--------:|
| GPIO0 (Pin 1)          | UART0 TX |
| GPIO1 (Pin 2)          | UART0 RX |
| GPIO4 (Pin 6)          | UART1 TX |
| GPIO5 (Pin 7)          | UART1 RX |
| GPIO8 (Pin 11)         | UART2 TX |
| GPIO9 (Pin 12)         | UART2 RX |
| GPIO12 (Pin 16)        | UART3 TX |
| GPIO13 (Pin 17)        | UART3 RX |
| GPIO16 (Pin 21)        | UART4 TX |
| GPIO17 (Pin 22)        | UART4 RX |
| GPIO20 (Pin 26)        | UART5 TX |
| GPIO21 (Pin 27)        | UART5 RX |

