_**NOTE** This is currently configured to run at 38400 baud, which will need a
modified Domoticz to run. I cannot reliably make it run at 115200 which Domoticz
natively expects_

packages:

-- to compile
make
gcc-avr
avr-libc

-- to flash
avrdude
