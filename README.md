# STM32F103C8_USB_CH340
Simple lightweight STM32F103C8 USB interface based on the CH340 USB to serial converter

No fancy pancy coding and no HAL. No error checking or USB suspend, just what is needed to get it to work.

Provides a simple interface to send or read a character via the USB interface.

The STM32F103C8T6 (bluepill) needs to run on either 48MHz or 72MHz to be able te generate the needed 48MHz for
the USB device.

This sample project just echo's the characters send from the host back to the host.

To embed it in one owns project just take the usb.c and usb.h file
