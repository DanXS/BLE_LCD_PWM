#include "stm32f4xx_hal.h"
#include "lcd.h"


void dev_lcd_write_cmd(I2C_HandleTypeDef* hi2c, uint8_t cmd) {

	if (cmd == 0xFF) {
		return;
	}
	while(hi2c->State != HAL_I2C_STATE_READY) {
		if (hi2c->State == HAL_I2C_STATE_ERROR) {
			return;
		}
		else if (hi2c->State == HAL_I2C_STATE_ABORT) {
			return;
		}
		else if (hi2c->State == HAL_I2C_STATE_TIMEOUT) {
			return;
		}
	}

	HAL_StatusTypeDef status = HAL_I2C_Master_Transmit_IT(hi2c, (uint16_t)(LCD_ADDRESS<<1), &cmd, 1);
	HAL_Delay(1);
}


void lcd_strobe(I2C_HandleTypeDef* hi2c, uint8_t data) {
	dev_lcd_write_cmd(hi2c, data | LCD_EN | LCD_BACKLIGHT);
	dev_lcd_write_cmd(hi2c, ((data & ~LCD_EN) | LCD_BACKLIGHT));
}

void lcd_write_four_bits(I2C_HandleTypeDef* hi2c, uint8_t data) {
	dev_lcd_write_cmd(hi2c, data | LCD_BACKLIGHT);
	lcd_strobe(hi2c, data);
}

void lcd_write(I2C_HandleTypeDef* hi2c, uint8_t cmd, uint8_t mode) {
	lcd_write_four_bits(hi2c, mode | (cmd & 0xF0));
	lcd_write_four_bits(hi2c, mode | ((cmd << 4) & 0xF0));
}

void lcd_display_string(I2C_HandleTypeDef* hi2c, char* str, int line) {
   switch(line) {
   case 1:
	   lcd_write(hi2c, 0x80, 0x00);
	   break;
   case 2:
	   lcd_write(hi2c, 0xC0, 0x00);
	   break;
   default:
	   return;
   }
   int len = strlen(str);
   for(int i = 0; i < len; i++) {
      lcd_write(hi2c, str[i], LCD_RS);
   }
}

void lcd_clear(I2C_HandleTypeDef* hi2c) {
   lcd_write(hi2c, LCD_CLEARDISPLAY, 0x00);
   lcd_write(hi2c, LCD_RETURNHOME, 0x00);
}

void initLCD(I2C_HandleTypeDef* hi2c) {
    lcd_write(hi2c, 0x03, 0x00);
    lcd_write(hi2c, 0x03, 0x00);
    lcd_write(hi2c, 0x03, 0x00);
    lcd_write(hi2c, 0x02, 0x00);

    lcd_write(hi2c, LCD_FUNCTIONSET | LCD_2LINE | LCD_5x8DOTS | LCD_4BITMODE, 0x00);
    lcd_write(hi2c, LCD_DISPLAYCONTROL | LCD_DISPLAYON, 0x00);
    lcd_write(hi2c, LCD_CLEARDISPLAY, 0x00);
    lcd_write(hi2c, LCD_ENTRYMODESET | LCD_ENTRYLEFT, 0x00);
	HAL_Delay(200);
}
