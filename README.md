# STM32F103C8_USB_CH340
Simple lightweight STM32F103C8 USB interface based on the CH340 USB to serial converter

No fancy pancy coding and no HAL. No error checking or USB suspend, just what is needed to get it to work.

Provides a simple interface to send or read a character via the USB interface.

The STM32F103C8T6 (for example a bluepill board) needs to run on either 48MHz or 72MHz to be able te generate the 
needed 48MHz for the USB device.

This simple project just echo's the characters send from the host back to the host.

To embed it in one owns project just take the usb.c, usb.h and the stm32f103_db.h file. The latter contains some of
the hardware peripheral defines and structures found in many of the ST include files.

To compile one can use the compile.sh script.

With the burn.sh script one can write it to the target. A st-link v2 jtag interface in combination with openocd 
is needed for this.

The debug.sh script can be used to start openocd for debugging. Much information on how to debug with
openocd can be found on the net.
