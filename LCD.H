#ifndef LCD_H   
#define LCD_H

void lcd_init();                                //Initalises LCD
void lcd_clear();                              // Clears the LCD 
void lcd_gotoxy(uint8_t x, uint8_t y);        //Move Cursor
void lcd_print(char *str);                   // Print string to character array

#endif
