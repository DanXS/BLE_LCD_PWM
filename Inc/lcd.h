
// LCD Address
#define LCD_ADDRESS 0x27

// commands
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

// flags for display entry mode
#define LCD_ENTRYRIGHT 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// flags for display on/off control
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

// flags for display/cursor shift
#define LCD_DISPLAYMOVE 0x08
#define LCD_CURSORMOVE 0x00
#define LCD_MOVERIGHT 0x04
#define LCD_MOVELEFT 0x00

// flags for function set
#define LCD_8BITMODE 0x10
#define LCD_4BITMODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00

// flags for backlight control
#define LCD_BACKLIGHT 0x08
#define LCD_NOBACKLIGHT 0x00

#define LCD_EN 0b00000100 // Enable bit
#define LCD_RW 0b00000010 // Read/Write bit
#define LCD_RS 0b00000001 // Register select bit

void initLCD(I2C_HandleTypeDef* hi2c);
void lcd_clear(I2C_HandleTypeDef* hi2c);
void lcd_display_string(I2C_HandleTypeDef* hi2c, char* str, int line);
void lcd_write(I2C_HandleTypeDef* hi2c, uint8_t cmd, uint8_t mode);
void lcd_write_four_bits(I2C_HandleTypeDef* hi2c, uint8_t data);
void lcd_strobe(I2C_HandleTypeDef* hi2c, uint8_t data);


