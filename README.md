**evofw3** is an opensource alternative to the HGI80

It uses an avr microcontroller to interface to a TI cc1101 radio.

The code supports 16MHZ atmega328 and 8/16 MHz atmega32u4 platforms.
The atmega328 uses a SW Uart to interface to the cc1101 while the atmega32u4 uses a HW Uart.

**While evofw3 supports the 16MHz atmega328 the receive performance of the atmega32u4 is siginifacntly better.**
 
**evofw3** is currently constructed to be built and be programmed with the Arduino GUI.
In order to support different platforms it is necessary to use custom board definitions.

You need to be running at least version 1.8.13 of the Arduino GUI.

To access these board definitions in the Arduino GUI


Select *File*, *Preferences*

Paste the following link into *Additional Board Manager URLs*
https://raw.githubusercontent.com/ghoti57/evofw3_avr/master/package_evofw3_boards_index.json

Close *Preferences*

Select *Tools*, *Board*, *Boards Manager*

type **evofw3** in the search bar

Click on install in *Evofw3 avr boards*

Close *Boards Manager*


In *Tools*, *Board* you should now see *Evofw3 avr boards*.

Use *Tools*, *Board*, *Evofw3 avr boards* to select the appropriate avr controller type

Use *Tools*, *Processor* to select the correct speed variant

Use *Tools*, *Pinout* to select the appropriate connection variant.


Note that the first time you program an avr controller with **evofw3** the COM port may change.

You can buy an Atmega32U4 + 868MHz CC1101 USB stick here
https://indalo-tech.onlineweb.shop/
