//    This file is part of Meteo Drone.

//    Meteo Drone is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.

//    Meteo Drone is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.

//   You should have received a copy of the GNU General Public License
//   along with Meteo Drone.  If not, see <http://www.gnu.org/licenses/>.


#pragma once
#include <stm32f3xx.h>
#include <gpio_config.h>
#include <LCD_lib.h>



//Temperature sensor raw value at 30 degrees C, VDDA=3.3V
#define TEMP30_CAL ((uint16_t*) ((uint32_t) 0x1FFFF7B8))

//Temperature sensor raw value at 110 degrees C, VDDA=3.3V
#define TEMP110_CAL ((uint16_t*) ((uint32_t) 0x1FFFF7C2))

//Internal voltage reference raw value at 30 degrees C, VDDA=3.3V
#define VREFINT_CAL ((uint16_t*) ((uint32_t) 0x1FFFF7BA))

#define LED_pin 5
#define LED_port GPIOA
#define USER_BUTTON_pin 13
#define USER_BUTTON_port GPIOC
#define CONFIG_BUTTON_pin 3
#define CONFIG_BUTTON_port GPIOC
#define LEFT_BUTTON_pin 2
#define LEFT_BUTTON_port GPIOC
#define RIGHT_BUTTON_pin 0
#define RIGHT_BUTTON_port GPIOA
#define ENTER_BUTTON_pin 1
#define ENTER_BUTTON_port GPIOA
#define LCD_SUPPLY_pin 2
#define LCD_SUPPLY_port GPIOD

#define INTERNAL_TEMPERATURE_BUFFER_SIZE 16
#define INTERNAL_TEMPERATURE_BUFFER_MASK (INTERNAL_TEMPERATURE_BUFFER_SIZE - 1)
#define SYSTICK_FREQUENCY 100

class meteo_drone
{

public:

  meteo_drone();
  void initialise ();
  void set_awake_time (int a_time) {awake_time = a_time * SYSTICK_FREQUENCY;};
  void set_polling_rate (int p_rate) {polling_rate = p_rate * SYSTICK_FREQUENCY;};
  void set_no_sleep (int n_sleep) {no_sleep = n_sleep;};
  void systick_enable(int frequency);
  void delay (uint32_t deciseconds);
  void led_tick (GPIO_TypeDef* port, int pin, int ticks);
  void update_analog_supply_voltage ();
  void update_internal_temperature ();
  float vdda_independent_voltage (int raw_sample);
  void mode_sleep ();
  void mode_internal_sensors ();
  void mode_time();
  void mode_config ();
  // void goto_last_mode ();
  void state_awaiting_command (int milliseconds);
  int check_button_press ();
  void restart_sleep_timer();
  void check_sleep();
  uint32_t volatile ticks;
  bool volatile user_button_pushed;
  bool volatile config_button_pushed;
  bool volatile left_button_pushed;
  bool volatile right_button_pushed;
  bool volatile enter_button_pushed;
  bool volatile button_pressed;
  int volatile uptime_since_action;
  int volatile uptime_since_sleep;
  //bool goto_mode_config;
  //bool goto_last_mode;
  char current_mode;
  char fallback_mode;
  void goto_next_mode();
  void goto_previous_mode();

private:
  
  int test;
  int adc1_calibration_factor;
  int awake_time;
  int polling_rate;
  int raw_reference_voltage;
  int raw_temperature;
  float analog_supply_voltage;
  float internal_temperature;
  bool in_mode_config;
  bool in_mode_sleep;
  int configuration_screen;
  bool configuration_submenu;
  int config_time;
  int max_config_time;
  bool set_config_value;
  int submenu_option;
  int submenu_value;
  bool no_sleep;
  float internal_temperature_buffer[INTERNAL_TEMPERATURE_BUFFER_SIZE];
  float* p_internal_temperature_buffer = &internal_temperature_buffer[0];
};


meteo_drone::meteo_drone() :
  adc1_calibration_factor(0),
  ticks(0),
  user_button_pushed(false),
  config_button_pushed(false),
  left_button_pushed(false),
  right_button_pushed(false),
  enter_button_pushed(false),
  awake_time(60),
  polling_rate(2000),
  raw_reference_voltage(0),
  raw_temperature(0),
  analog_supply_voltage(0),
  internal_temperature(0),
  configuration_screen(1),
  configuration_submenu(false),
  config_time(0),
  max_config_time(20),
  set_config_value(false),
  submenu_option(0),
  submenu_value(0),
  no_sleep(false),
  // goto_mode_config(false),
  // goto_last_mode(false),
  button_pressed(false),
  uptime_since_action(0),
  uptime_since_sleep(0),
  test(0)
{
  for (int i = 0; i < INTERNAL_TEMPERATURE_BUFFER_SIZE; i++)
    {
      internal_temperature_buffer[i] = 0;
    }
}

// Initialisation routine for STM32F334R8
void meteo_drone::initialise ()
{
  // SYSCLOCK prescaler to 1
  RCC->CFGR &= ~(RCC_CFGR_HPRE);

  // Enable SysTick and its interrupt

  systick_enable(SYSTICK_FREQUENCY);

  // make sure ADC is disabled
  ADC1->CR &= ~(ADC_CR_ADEN);

  // Enbale ADC and temperature sensor clock
  RCC->AHBENR |= RCC_AHBENR_ADC12EN;
  RCC->AHBENR |= RCC_AHBENR_TSCEN;

  // Enable internal connection between temperature sensor and ADC1_16
  // Enable VREF in ADC IN 18
  ADC12_COMMON->CCR |= ADC_CCR_TSEN;
  ADC12_COMMON->CCR |= ADC_CCR_VREFEN;

  // Set ADC's to AHB Clock option (CKMODE) with prescaling of 1
  // Set ADC PLL clock prescaler to 0, so AHB clock can be used
  ADC12_COMMON->CCR |= (0b01 << 16);
  RCC->CFGR2 &= ~(0b11111 << 4);

  // enable ADC voltage regulator
  ADC1->CR &= ~(ADC_CR_ADVREGEN_1);
  ADC1->CR &= ~(ADC_CR_ADVREGEN_0);
  ADC1->CR |= ADC_CR_ADVREGEN_0;
  delay(10);

  // Set single ended calibration
  // Set ADC IN 15 to single ended mode
  ADC1->CR &= ~(ADC_CR_ADCALDIF);
  ADC1->DIFSEL &= ~(ADC_DIFSEL_DIFSEL_15);

  // Start calibration
  ADC1->CR |= ADC_CR_ADCAL;

  // Set LED and USER_BUTTON pins
  set_pin_output(LED_port, LED_pin);
  set_pin_input(USER_BUTTON_port, USER_BUTTON_pin);
  set_pin_input_pulldown(CONFIG_BUTTON_port, CONFIG_BUTTON_pin);
  set_pin_input_pulldown(LEFT_BUTTON_port, LEFT_BUTTON_pin);
  set_pin_input_pulldown(RIGHT_BUTTON_port, RIGHT_BUTTON_pin);
  set_pin_input_pulldown(ENTER_BUTTON_port, ENTER_BUTTON_pin);
  set_pin_output(LCD_SUPPLY_port, LCD_SUPPLY_pin);

  LCD_SUPPLY_port->BSRR |= (1 << LCD_SUPPLY_pin);
  brute_delay(10000);

  //wait until calibration is complete
  LCD_initialise ();
  do
    {
      LCD_write_1st_line ("ADC CAL");
      delay(20);
      LCD_set_cursor_position(1, 7);
      LCD_write_on_cursor (".");
      delay(20);
      LCD_set_cursor_position(1, 8);
      LCD_write_on_cursor (".");
      delay(20);
      LCD_set_cursor_position(1, 9);
      LCD_write_on_cursor (".");
      delay(20);
    }
  while ((ADC1->CR & ADC_CR_ADCAL)!= 0);

  adc1_calibration_factor = ADC1->CALFACT & ADC_CALFACT_CALFACT_S;
  LCD_write_2nd_line("CAL ");
  LCD_write_int(adc1_calibration_factor, 2, "");
  delay(70);

  // Enable ADC
  ADC1->CR |= ADC_CR_ADEN;

  LCD_clear_display();
  // Check if ADC is ready
  do
    {
      LCD_write_1st_line ("ENABLE ADC");
      delay(20);
      LCD_set_cursor_position(1, 10);
      LCD_write_on_cursor (".");
      delay(20);
      LCD_set_cursor_position(1, 11);
      LCD_write_on_cursor (".");
      delay(20);
      LCD_set_cursor_position(1, 12);
      LCD_write_on_cursor (".");
      delay(20);
    }
  while ((ADC1->CR & ADC_ISR_ADRDY) == 0);
  LCD_write_2nd_line("ADC READY!");
  delay(70);

  // Set right-aligned data in the ADC_DR register
  ADC1->CFGR &= ~ADC_CFGR_ALIGN;

  //Enable SYSCFG clock (for EXTI lines)
  RCC->APB2ENR |= 1;

  // Unmask EXTI13
  // Set EXTI13 to GPIOC pin

  EXTI->IMR |= EXTI_IMR_MR13;
  EXTI->IMR |= EXTI_IMR_MR0;
  EXTI->IMR |= EXTI_IMR_MR1;
  EXTI->IMR |= EXTI_IMR_MR2;
  EXTI->IMR |= EXTI_IMR_MR3;
  SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PA;
  SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI1_PA;
  SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI2_PC;
  SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI3_PC;
  SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI13_PC;

  // Falling voltage triggers interrput in EXTI13
  EXTI->FTSR |= EXTI_FTSR_TR13;
  EXTI->RTSR |= EXTI_RTSR_RT0;
  EXTI->RTSR |= EXTI_RTSR_RT1;
  EXTI->RTSR |= EXTI_RTSR_RT2;
  EXTI->RTSR |= EXTI_RTSR_RT3;

  //Enable EXTI15_10 Interrupt
  NVIC_SetPriority (EXTI15_10_IRQn, 1);
  NVIC_EnableIRQ (EXTI15_10_IRQn);
  NVIC_SetPriority(EXTI0_IRQn, 1);
  NVIC_EnableIRQ(EXTI0_IRQn);
  NVIC_SetPriority(EXTI1_IRQn, 1);
  NVIC_EnableIRQ(EXTI1_IRQn);
  NVIC_SetPriority(EXTI2_TSC_IRQn, 1);
  NVIC_EnableIRQ(EXTI2_TSC_IRQn);
  NVIC_SetPriority(EXTI3_IRQn, 1);
  NVIC_EnableIRQ(EXTI3_IRQn);
}

// Enable systick clock and its interrupt
void meteo_drone::systick_enable (int frequency)
{
  uint32_t systick_error = SysTick_Config (SystemCoreClock / frequency);
}

// MCU enters sleep mode and is only woken up by systick to check time
void meteo_drone::delay (uint32_t centiseconds)
{
  int tick_target = ticks + centiseconds;
  while (ticks < tick_target)
    {
      __WFI();
    }
}

// blink led function. systick is configured to generate periodic interrups
void meteo_drone::led_tick (GPIO_TypeDef* port, int pin, int ticks)
{
  for (int i = 0; i < ticks; i++)
    {
      port->BSRR = (1 << pin);
      delay (10);
      port->BSRR = (1 << (pin + 16));
      delay (30);
    }
  delay (50);
}

// Update analog supply voltage variable
void meteo_drone::update_analog_supply_voltage ()
{
  analog_supply_voltage = 3.3 * *VREFINT_CAL / raw_reference_voltage;
}

// update MCU internal temperature variable
void meteo_drone::update_internal_temperature ()
{
  float compensated_temperature = vdda_independent_voltage (raw_temperature);
  float voltage_at_25deg = 1.43;
  internal_temperature = ((voltage_at_25deg - compensated_temperature) / 0.0043) + 25;
}

// calculate input voltage for a specific sample value
// independently of the reference voltage used in the ADC
float meteo_drone::vdda_independent_voltage (int raw_sample)
{
  return raw_sample * (analog_supply_voltage / 4095);
}


float calculate_buffer_average (float *buffer, int buffer_size)
{
  float accumulated_value = 0;
  for (int i = 0; i < buffer_size; i++)
    {
      accumulated_value += *buffer++;
    }
  return accumulated_value / buffer_size;
}

void meteo_drone::restart_sleep_timer ()
{
  uptime_since_action = 0;
}

void meteo_drone::check_sleep ()
{
  if (uptime_since_action >= awake_time && no_sleep == false)
    {
      current_mode = 's';
    }
}
    

void meteo_drone::mode_sleep ()
{
  // prepare for sleep
  LCD_clear_display();
  LCD_write_1st_line("ENTERING");
  LCD_write_2nd_line("SLEEP");
  LCD_set_cursor_position(2, 5);
  state_awaiting_command(45);
  LCD_write_on_cursor(".");
  state_awaiting_command(45);
  check_button_press();
  if (current_mode != 's')
    {
      return;
    }
  LCD_write_on_cursor(".");
  state_awaiting_command(45);
  check_button_press();
  if (current_mode != 's')
    {
      return;
    }
  LCD_write_on_cursor(".");
  state_awaiting_command(45);
  check_button_press();
  if (current_mode != 's')
    {
      return;
    }
  LCD_write_on_cursor(".");
  state_awaiting_command(45);
  check_button_press();
  if (current_mode != 's')
    {
      return;
    }
  LCD_write_on_cursor(".");
  state_awaiting_command(45);
  check_button_press();
  if (current_mode != 's')
    {
      return;
    }
  LCD_clear_display();
  LCD_SUPPLY_port->BSRR |= (1 << (16 + LCD_SUPPLY_pin));
  SysTick->CTRL = 0;
  __WFI();

  // wake-up routine
  systick_enable(SYSTICK_FREQUENCY);
  restart_sleep_timer();
  uptime_since_sleep = 0;
  ADC1->CALFACT |= adc1_calibration_factor;
  LCD_SUPPLY_port->BSRR |= (1 << LCD_SUPPLY_pin);
  brute_delay(10000);
  check_button_press();
}

void meteo_drone::mode_internal_sensors ()
{
  LCD_clear_display();
  delay(10);
  LCD_write_1st_line("INTERNAL SENSORS");
  state_awaiting_command(120);
  check_button_press();

  // Set ADC sampling time to slowest
  // Set two conversions in ADC regular sequence, CH18, 16.
  ADC1->SMPR2 |= (0b111 << 18);
  ADC1->SMPR2 |= (0b111 << 24);
  ADC1->SQR1 &= ~(0b1111);
  ADC1->SQR1 |= 0b0001;
  ADC1->SQR1 |= (18 << 6);
  ADC1->SQR1 |= (16 << 12);
  LCD_clear_display();
  LCD_write_1st_line("MCU Temp:");
  LCD_write_2nd_line("Vdda in:");

  // Fill temperature buffer
  for (int i = 0; i < INTERNAL_TEMPERATURE_BUFFER_SIZE ; i++)
    {
      ADC1->CR |= ADC_CR_ADSTART;
      while ((ADC1->ISR & ADC_ISR_EOC) == 0)
	{
	}
      raw_reference_voltage = ADC1->DR;
      update_analog_supply_voltage();
      while ((ADC1->ISR & ADC_ISR_EOS) == 0)
	{
	}
      raw_temperature = ADC1->DR;
      update_internal_temperature ();
      internal_temperature_buffer[i] = internal_temperature;
    }

  // ADC Regular conversions
  int buffer_position = 0;
  float buffer_average = 0;
  while (current_mode == 'i')
    {
      ADC1->CR |= ADC_CR_ADSTART;
      while ((ADC1->ISR & ADC_ISR_EOC) == 0)
	{
	}
      raw_reference_voltage = ADC1->DR;
      update_analog_supply_voltage();
      LCD_set_cursor_position(2, 10);
      LCD_write_float(analog_supply_voltage, 4, "V");
      while ((ADC1->ISR & ADC_ISR_EOS) == 0)
	{
	}
      raw_temperature = ADC1->DR;
      update_internal_temperature ();
      p_internal_temperature_buffer[buffer_position] = internal_temperature;
      buffer_position++;
      buffer_position &= INTERNAL_TEMPERATURE_BUFFER_MASK;
      buffer_average = calculate_buffer_average
	(p_internal_temperature_buffer, INTERNAL_TEMPERATURE_BUFFER_SIZE);
      LCD_set_cursor_position(1, 11);
      LCD_write_float(buffer_average, 3, "C");
      state_awaiting_command(polling_rate);
      check_button_press();
      check_sleep();
    }
}



void meteo_drone::mode_time ()
{
  LCD_clear_display();
  delay(10);
  LCD_write_1st_line("      TIME      ");
  state_awaiting_command(120);
  check_button_press();
  
  LCD_clear_display();
  LCD_write_1st_line("Uptime:");
  LCD_write_2nd_line("Awake:");

  while (current_mode == 't')
    {
      LCD_set_cursor_position(1, 8);
      LCD_write_int(ticks * (1 / (float)SYSTICK_FREQUENCY), 7, "s");
      LCD_set_cursor_position(2, 7);
      LCD_write_int(uptime_since_sleep * (1 / (float)SYSTICK_FREQUENCY), 8, "s" );
      state_awaiting_command(1 * SYSTICK_FREQUENCY);
      check_button_press();
      check_sleep();
    }
}




// configuration mode
void meteo_drone::mode_config()
{
  LCD_clear_display();
  delay(10);
  LCD_write_1st_line(" CONFIGURATION  ");
  state_awaiting_command(120);
  check_button_press();

  while (current_mode == 'c')
    {
      if (configuration_screen == 1)
	{
	  submenu_option = 0;
	  LCD_clear_display();
	  LCD_write_1st_line("POLLING RATE [s]");
	  while (current_mode == 'c' && configuration_submenu == false &&
		 configuration_screen == 1)
	    {
	      LCD_write_2nd_line("ENTER: Set Value");
	      state_awaiting_command(2000);
	      check_button_press();
	    }
	  while (configuration_submenu && configuration_screen == 1)
	    {
	      LCD_write_2nd_line("1 2 4 8 16 32 64");
	      LCD_cursor_on();
	      if (submenu_option > 6)
		{
		  submenu_option = 6;
		}
	      else if (submenu_option < 0)
		{
		  submenu_option = 0;
		}
	      switch (submenu_option)
		{
		case 0:
		  LCD_set_cursor_position(2, 0);
		  break;
		case 1:
		  LCD_set_cursor_position(2, 2);
		  break;
		case 2:
		  LCD_set_cursor_position(2, 4);
		  break;
		case 3:
		  LCD_set_cursor_position(2, 6);
		  break;
		case 4:
		  LCD_set_cursor_position(2, 8);
		  break;
		case 5:
		  LCD_set_cursor_position(2, 11);
		  break;
		case 6:
		  LCD_set_cursor_position(2, 14);
		  break;
		}
	      state_awaiting_command(20000);
	      check_button_press();

	      if (set_config_value)
		{
		  set_config_value = false;
		  switch (submenu_option)
		    {
		    case 0:
		      set_polling_rate(1);
		      break;
		    case 1:
		      set_polling_rate(2);
		      break;
		    case 2:
		      set_polling_rate(4);
		      break;
		    case 3:
		      set_polling_rate(8);
		      break;
		    case 4:
		      set_polling_rate(16);
		      break;
		    case 5:
		      set_polling_rate(32);
		      break;
		    case 6:
		      set_polling_rate(64);
		      break;
		    }
		  LCD_cursor_off();
		  LCD_clear_display();
		  LCD_write_2nd_line("VALUE SET");
		  delay(100);
		}
	      LCD_cursor_off();
	    }
	}

      else if (configuration_screen == 2)
	{
	  submenu_option = 0;
	  LCD_clear_display();
	  LCD_write_1st_line("AWAKE TIME [min]");
	  while (current_mode == 'c' && configuration_submenu == false
		 && configuration_screen == 2)
	    {
	      LCD_write_2nd_line("ENTER: Set Value");
	      state_awaiting_command(20000);
	      check_button_press();
	    }
	  while (configuration_submenu && configuration_screen == 2)
	    {
	      LCD_write_2nd_line("0.5 1 2 5 10 OFF");
	      LCD_cursor_on();
	      if (submenu_option > 5)
		{
		  submenu_option = 5;
		}
	      else if (submenu_option < 0)
		{
		  submenu_option = 0;
		}
	      switch (submenu_option)
		{
		case 0:
		  LCD_set_cursor_position(2, 0);
		  break;
		case 1:
		  LCD_set_cursor_position(2, 4);
		  break;
		case 2:
		  LCD_set_cursor_position(2, 6);
		  break;
		case 3:
		  LCD_set_cursor_position(2, 8);
		  break;
		case 4:
		  LCD_set_cursor_position(2, 10);
		  break;
		case 5:
		  LCD_set_cursor_position(2, 13);
		  break;
		}
	      state_awaiting_command(20000);
	      check_button_press();

	      if (set_config_value)
		{
		  set_config_value = false;
		  switch (submenu_option)
		    {
		    case 0:
		      set_awake_time(30);
		      set_no_sleep(false);
		      break;
		    case 1:
		      set_awake_time(60);
		      set_no_sleep(false);
		      break;
		    case 2:
		      set_awake_time(120);
		      set_no_sleep(false);
		      break;
		    case 3:
		      set_awake_time(300);
		      set_no_sleep(false);
		      break;
		    case 4:
		      set_awake_time(600);
		      set_no_sleep(false);
		      break;
		    case 5:
		      set_no_sleep(true);
		      break;
		    }
		  LCD_cursor_off();
		  LCD_clear_display();
		  LCD_write_2nd_line("VALUE SET");
		  delay(100);
		}
	      LCD_cursor_off();
	    }
	}

      else if (configuration_screen == 3)
	{
	  LCD_clear_display();
	  LCD_write_1st_line("THREE CONFIG");
	  LCD_write_2nd_line("1 2 3 4");
	  while (current_mode == 'c' && configuration_submenu == false
		 && configuration_screen == 3)
	    {
	      state_awaiting_command(20000);
	      check_button_press();
	    }
	  while (configuration_submenu && configuration_screen == 3)
	    {
	      LCD_set_cursor_position(2, 0);
	      LCD_cursor_on();
	      state_awaiting_command(20000);
	      check_button_press();
	      LCD_cursor_off();
	    }
	}
    }
  
  current_mode = fallback_mode;
}


// this method waits for user commands for a specified amount of time.
// to be used when a delay that still listens to user entry is needed.
void meteo_drone::state_awaiting_command (int centiseconds)
{
  int tick_target = ticks + centiseconds;
  while (ticks < tick_target)
    {
      if (button_pressed)
	{
	  return;
	}
      __WFI();
    }
}


int meteo_drone::check_button_press()
{
  button_pressed = false;
  if (user_button_pushed && current_mode == 's')
    {
      restart_sleep_timer();
      user_button_pushed = false;
      current_mode = fallback_mode;
      return 1;
    }
  
  else if (config_button_pushed)
    {
      restart_sleep_timer();
      config_button_pushed = false;
      if (current_mode == 's')
	{
	  current_mode = fallback_mode;
	  return 0;
	}      
      else if (current_mode == 'c' && configuration_submenu)
	{
	  set_config_value = false;
	  configuration_submenu = false;
	}
      else if (current_mode == 'c' && configuration_submenu == false)
	{
	  current_mode = fallback_mode; 
	}
      else
	{
	  current_mode = 'c';
	}
      return 1;
    }

  else if (left_button_pushed)
    {
      restart_sleep_timer();
      left_button_pushed = false;
      if (current_mode == 's')
	{
	  current_mode = fallback_mode;
	}
      else if (current_mode == 'c')
	{
	  if (configuration_submenu == false)
	    {
	      if (configuration_screen <= 1)
		{
		  // do nothing, leftmost screen is already selected!
		}
	      else if (configuration_screen > 1)
		{
		  configuration_screen--;
		}
	    }
	  else if (configuration_submenu)
	    {
	      submenu_option--;
	    }
	}

      else goto_previous_mode();
      
      return 1;
    }

  else if (right_button_pushed)
    {
      restart_sleep_timer();
      right_button_pushed = false;
      
      if (current_mode == 's')
	{
	  current_mode = fallback_mode;
	}     
      else if (current_mode == 'c')
	{
	  if (configuration_submenu == false)
	    {
	      if (configuration_screen >= 3)
		{
		  // do nothing, rightmost screen is already selected!
		}
	      else if (configuration_screen < 3)
		{
		  configuration_screen++;
		}
	    }
	  else if (configuration_submenu)
	    {
	      submenu_option++;
	    }
	}
      
      else goto_next_mode();
      
      return 1;
    }

  else if (enter_button_pushed)
    {
      restart_sleep_timer();
      enter_button_pushed = false;
      if (current_mode == 's')
	{
	  current_mode = fallback_mode;
	}
      else if (current_mode == 'c')
	{
	  if (configuration_submenu == false)
	    {
	      configuration_submenu = true;
	    }
	  else if (configuration_submenu)
	    {
	      set_config_value = true;
	      configuration_submenu = false;
	    }
	}
      return 1;
    }
  
  else return 0;
}


void meteo_drone::goto_next_mode()
{
  switch (current_mode)
    {
    case 'i':
      current_mode = 't';
      break;
    }
}

void meteo_drone::goto_previous_mode()
{
  switch (current_mode)
    {
    case 't':
      current_mode = 'i';
      break;
    }
}
