#ifndef LCD_H
#define LCD_H

void lcd_init();
void lcd_clear();
void lcd_gotoxy(uint8_t x, uint8_t y);
void lcd_print(char *str);

#endif
