# BLE_LCD_PWM

This project is for the Nucleo STM32F401RE board, it implements a custom protocol which is currently controlled via an iOS swift application which can be found here:

https://github.com/DanXS/BLEControl

The application was created using STM32CubeMX and SystemWorkbench which can be downloaded from:

http://www.st.com/en/development-tools/stm32-software-development-tools.html?querycriteria=productId=SC2106

STM32CubeMX helps you configure the pins on the chip and the clock, interupts etc via the UI and then generates code to get you started.  It uses STM's hardware abstraction layer classes (HAL) as opposed the the standard peripheral libraries which should make the application more portable to other MCU's while also potentially making the application larger that in could be if you went direct to the hardware.

One side effect of this is that custom code should be implemented between the user begin/user end comment blocks.  This makes for rather untidy looking code with lots of excess comments but also allows you to make changes to your configuration via cube and generate new code from it without it deleting the code you implement.

The basic pinout and configuration can be found in the accompanying BLE_LCD_PWM.pdf file.  Essentially it uses two timers each with 4 channels for PWM, which allows for 8 PWM driven accessories to be connected.  It also uses 2 I2C pins to control a simple two line LCD, as I guide to implementing the LCD control messages I adapted the code from a python library written by Eric McNiece which can be found here:

https://github.com/emcniece/rpi-lcd/blob/master/RPi_I2C_driver.py

It is implemented in the lcd.c/lcd.h files.  His version was more advanced and allowed for custom bit patterns to be passed as well as simple letters and numbers, but here I just wanted to be able to display basic text.

The Protocol so far just consists the following commands:

* INIT - Initialise the device - resets any variables/buffer pointers etc
* ANALOG_OUT_EN - Enable or disable specific PWM channel specifing the index which then map the the PWM timer/channels
* SERVO_VAL - Set the PWM value for the servo/brushless motor etc
* PWM_VAL - Set the PWM value for a general (virtual) analog signal e.g to fade an LED or to drive the speed of a DC motor
* LCD_TEXT - Write a line of text to the lcd, just 2 lines in this implementation but easy to add support for a 4 line lcd
* LCD_CLEAR - Clear the lcd

The format of each command follows this form:

Command | Length | Data

The command is a byte representing the command, a byte with the length of the message, then the data for the specific message.  For example the LCD_TEXT command data would include and index for the line number and the text data terminated with a '\0' or 0x00.  The SERVO_VAL command data would include the index of the output signal PWM channel (0..7) in this case followed by two bytes for the high and low order byte of the PWM value.  Here I use a granularity of 1000, so the two bytes define that value which is then passed to the PWM clock reset counter register.

The difference between the SERVO_VAL and PWM_VAL commands is just in the overall pulse width for each phase or duty cycle.  The frequency is 50Hz or 20ms per cycle, to control a servo the pulse with should range from 1ms to 2ms. For the PWM_VAL command instead the pulse can range can be anywhere along the full 20ms cycle, the effect of this is that as the pulse width gets longer, overall the resulting voltage increases if you where to smooth out the square wave.  At rates as fast as this most analog devices will not notice it is a square wave at all and just treat it as a single voltage level.  It is possible to add resisters/capacitors to smooth out the signal if required.

The basic functionaly of the program is that it continously tries to read from a circular buffer which is populated via the uart device - in this case the BLE module.  An interupt is generated from the UART source that at the same time fills the circular buffer.  Commands are read off and executed as the become available.

Please also check out the youtube of it working:

https://www.youtube.com/watch?v=zFuNHCO-w5U
