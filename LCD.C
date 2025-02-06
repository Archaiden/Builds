// implementation of an LCD driver interfacing with a 16x2 LCD using a 4-bit or 8-bit parallel 

#include <avr/io.h>
#include <util/delay.h>
#include "lcd.h"

#define LCD_CTRL PORTB
#define LCD_DATA PORTC
#define RS PB0
#define RW PB1
#define EN PB2

void lcd_command(unsigned char cmnd)
{
    LCD_DATA = cmnd;          // Send the command to the data port
    LCD_CTRL &= ~(1 << RS);   // Set RS = 0 
    LCD_CTRL &= ~(1 << RW);   // Set RW = 0 to write to LCD 
    LCD_CTRL |= (1 << EN);    // Pulse EN pin high and then low to latch the command into the LCD.
    _delay_ms(1);             // Wait for 1 ms
    LCD_CTRL &= ~(1 << EN);   // Set EN = 0 (disable pulse)
}

void lcd_clear()
{
    lcd_command(0x01);
    _delay_ms(2);
}

void lcd_gotoxy(uint8_t x, uint8_t y)
{
    uint8_t pos = 0x80 + (y - 1) * 0x40 + (x - 1);
    lcd_command(pos);
}

void lcd_print(char *str)
{
    while (*str)
    {
        LCD_DATA = *str++;
        LCD_CTRL |= (1 << RS);
        LCD_CTRL &= ~(1 << RW);
        LCD_CTRL |= (1 << EN);
        _delay_ms(1);
        LCD_CTRL &= ~(1 << EN);
    }
}

void lcd_init()
{
    DDRC = 0xFF; // Set PORTC as output for data
    DDRB |= (1 << RS) | (1 << RW) | (1 << EN); // Control pins as output

    _delay_ms(15);
    lcd_command(0x38); // 8-bit, 2-line
    _delay_ms(1);
    lcd_command(0x0C); // Display on, cursor off
    _delay_ms(1);
    lcd_clear();
}

