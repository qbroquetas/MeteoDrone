#pragma once
#include <stm32f3xx.h>
#include <gpio_config.h>
#include <stdio.h>
#include <stdlib.h>

#define LCDRS_pin 10
#define LCDRS_port GPIOA
#define LCDRW_pin 11
#define LCDRW_port GPIOA
#define LCDE_pin 5
#define LCDE_port GPIOB

#define LCD0_pin 12
#define LCD0_port GPIOA
#define LCD1_pin 9
#define LCD1_port GPIOC
#define LCD2_pin 8
#define LCD2_port GPIOA //
#define LCD3_pin 9
#define LCD3_port GPIOA
#define LCD4_pin 7
#define LCD4_port GPIOC
#define LCD5_pin 6
#define LCD5_port GPIOB
#define LCD6_pin 7
#define LCD6_port GPIOB
#define LCD7_pin 11
#define LCD7_port GPIOC

#define delay_before_enable 5
#define delay_before_disable 5
#define delay_after_disable 5


void brute_delay (int delay_time)
{
  int volatile j = 0;
  for (int i = 0; i < delay_time; i++)
    {
      j++;
    }
}

void LCD_send_bit (GPIO_TypeDef* port, int pin, bool bit_state)
{
  if (bit_state)
    {
      port->BSRR |= (1 << pin);
    }
  else port->BRR |= (1 << pin);
}

bool LCD_read_bit (GPIO_TypeDef* port, int pin)
{
  return port->IDR & (1 << pin);
}

void LCD_enable ()
{
  brute_delay (delay_before_enable);
  LCD_send_bit(LCDE_port, LCDE_pin, 1);  
}

void LCD_switch_to_write ()
{
  set_pin_output(LCD0_port, LCD0_pin);
  set_pin_output(LCD1_port, LCD1_pin);
  set_pin_output(LCD2_port, LCD2_pin);
  set_pin_output(LCD3_port, LCD3_pin);
  set_pin_output(LCD4_port, LCD4_pin);
  set_pin_output(LCD5_port, LCD5_pin);
  set_pin_output(LCD6_port, LCD6_pin);
  set_pin_output(LCD7_port, LCD7_pin);
  
  LCD_send_bit(LCDRW_port, LCDRW_pin, 0);
}

void LCD_switch_to_read ()
{
  set_pin_input(LCD0_port, LCD0_pin);
  set_pin_input(LCD1_port, LCD1_pin);
  set_pin_input(LCD2_port, LCD2_pin);
  set_pin_input(LCD3_port, LCD3_pin);
  set_pin_input(LCD4_port, LCD4_pin);
  set_pin_input(LCD5_port, LCD5_pin);
  set_pin_input(LCD6_port, LCD6_pin);
  set_pin_input(LCD7_port, LCD7_pin);
  
  LCD_send_bit(LCDRW_port, LCDRW_pin, 1);
}

void LCD_command_mode ()
{
  LCD_send_bit(LCDRS_port,  LCDRS_pin, 0);
}

void LCD_character_mode ()
{
  LCD_send_bit(LCDRS_port,  LCDRS_pin, 1);
}


void LCD_send_byte (char byte)
{
  LCD_send_bit (LCD0_port, LCD0_pin, byte & 0b00000001);
  LCD_send_bit (LCD1_port, LCD1_pin, byte & 0b00000010);
  LCD_send_bit (LCD2_port, LCD2_pin, byte & 0b00000100);
  LCD_send_bit (LCD3_port, LCD3_pin, byte & 0b00001000);
  LCD_send_bit (LCD4_port, LCD4_pin, byte & 0b00010000);
  LCD_send_bit (LCD5_port, LCD5_pin, byte & 0b00100000);
  LCD_send_bit (LCD6_port, LCD6_pin, byte & 0b01000000);
  LCD_send_bit (LCD7_port, LCD7_pin, byte & 0b10000000);
  brute_delay (delay_before_disable);
  // Turn off Enable pin
  LCD_send_bit (LCDE_port, LCDE_pin, 0);
  brute_delay (delay_after_disable);
}


void LCD_write_character (char character)
{
  LCD_switch_to_write ();
  LCD_character_mode ();
  LCD_enable();
  LCD_send_byte (character);
}


void LCD_write_command (char character)
{
  LCD_switch_to_write ();
  LCD_command_mode ();
  LCD_enable();
  LCD_send_byte (character);
}

void LCD_initialise ()
{
  set_pin_output(LCDRS_port, LCDRS_pin);
  set_pin_output(LCDRW_port, LCDRW_pin);
  set_pin_output(LCDE_port, LCDE_pin);
  
  LCD_write_command (0b0000111100); // set 8bit mode, 2 lines, size 5x11
  LCD_write_command (0b0000001100); // display ON, cursor and c. position OFF
  LCD_write_command (0b0000000110); // set no shift, cursor moves right
  LCD_write_command (0b0000000001); // clear display
  brute_delay(1100);
}

void LCD_cursor_on ()
{
  LCD_write_command (0b0000001111); // display ON, cursor ON and c. position OFF
}

void LCD_cursor_off ()
{
  LCD_write_command (0b0000001100); // display ON, cursor and c. position OFF
}

void LCD_clear_display ()
{
  LCD_write_command (0b0000000001); // clear display
  brute_delay(1100);
}

int LCD_get_raw_cursor_position ()
{
  LCD_switch_to_read ();
  LCD_command_mode ();
  LCD_enable();
  bool bit0 = LCD_read_bit(LCD0_port, LCD0_pin);
  bool bit1 = LCD_read_bit(LCD1_port, LCD1_pin);
  bool bit2 = LCD_read_bit(LCD2_port, LCD2_pin);
  bool bit3 = LCD_read_bit(LCD3_port, LCD3_pin);
  bool bit4 = LCD_read_bit(LCD4_port, LCD4_pin);
  bool bit5 = LCD_read_bit(LCD5_port, LCD5_pin);
  bool bit6 = LCD_read_bit(LCD6_port, LCD6_pin);
  brute_delay(delay_before_disable);
  LCD_send_bit (LCDE_port, LCDE_pin, 0);
  brute_delay (delay_after_disable);
  
  return bit0 + (bit1 << 1) + (bit2 << 2) + (bit3 << 3) + (bit4 << 4) + (bit5 << 5) + (bit6 << 6); 
}

void LCD_set_raw_cursor_position (int raw_cursor_position)
{
  LCD_write_command(raw_cursor_position + 0b0010000000);
}
      
  
void LCD_set_cursor_position(int line, int cursor_position)
{
  if (line == 1)
    {
      LCD_write_command(cursor_position + 0b0010000000);
    }
  else if (line == 2)
    {
      LCD_write_command(0x40 + cursor_position + 0b0010000000);
    }
}

void LCD_write_on_cursor (char* string)
{
  while (*string)
    {
      LCD_write_character (*string++);
    }
}  

void LCD_write_1st_line (char* line)
{
  LCD_write_command (0b0010000000); // set cursor to 1st line
  while (*line)
    {
      LCD_write_character (*line++);
    }
}

void LCD_write_2nd_line (char* line)
{
  LCD_write_command (0b0011000000); // set cursor to 2nd line
  while (*line)
    {      
      LCD_write_character (*line++);
    }
}

void LCD_write_space ()
{
  LCD_write_character(32);
}

// write int to LCD
void LCD_write_int (int number, uint8_t max_digits, char* volatile unit)
{
  int digits = 0;
  int abs_number = abs(number);
  if (number < 0) {digits = 1;}
  if (abs_number < 10) {digits += 1;}
  else if (abs_number < 100) {digits += 2;}
  else if (abs_number < 1000) {digits += 3;}
  else if (abs_number < 10000) {digits += 4;}
  else if (abs_number < 100000) {digits += 5;}
  else if (abs_number < 1000000) {digits += 6;}
  else if (abs_number < 10000000) {digits += 7;}
  else if (abs_number < 100000000) {digits += 8;}
         
  int cursor_position = LCD_get_raw_cursor_position();
  char number_string[digits];
  char* p_number_string = &number_string[0];
  int empty_spaces = max_digits - digits;
  sprintf (number_string, "%d", number);
  for (int i = 0; i < empty_spaces ; i++)
    {
      LCD_write_space();
    }
  while (*p_number_string)
    {
      LCD_write_character (*p_number_string++);
    }

  while (*unit)
    {
      LCD_write_character(*unit++);
    }
  
  LCD_set_raw_cursor_position (cursor_position);
}

// write float to LCD
void LCD_write_float (float number, uint8_t precision, char* volatile unit)
{
  int cursor_position = LCD_get_raw_cursor_position();
  char number_string[precision];
  char* p_number_string = &number_string[0];
  sprintf (number_string, "%f", number);
  for (int i = 0; i < precision + 1 && *p_number_string ; i++)
    {
      LCD_write_character (*p_number_string++);
    }
  while (*unit)
    {
      LCD_write_character(*unit++);
    }
  LCD_set_raw_cursor_position (cursor_position);
}
