# stm32wb5mm-dk-bunkbed-control-unit
STM32WB5MM-DK Bunkbed Control Unit

## Arduino setup

To setup this project please do the following:
- Install the STM32duino board package
- Select the board "Discovery" and then under Tools->Board Part Number select STM32WB5MM-DK.

You will need to additionally install several arduino libraries via the library management UI.

Adafruit Neopixel
Adafruit VL53L0X
STM32Duino ISM330DHCX
STM32Duino STTS22H
U8g2
DFRobot DF2301Q

Once this is done build and launch.

The bunkbed_control_unit.ino is commented with additional information such as the pins in use for the LED lights and relay which can be further configured as needed.
