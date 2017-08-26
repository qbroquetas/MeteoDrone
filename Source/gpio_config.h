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

void set_pin_output (GPIO_TypeDef* port, int pin)
{
  // enable clock for the specified port
  if (port == GPIOA)
    {
      RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    }
  else if (port == GPIOB)
    {
      RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    }
  else if (port == GPIOC)
    {
      RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    }
  else if (port == GPIOD)
    {
      RCC->AHBENR |= RCC_AHBENR_GPIODEN;
    }
  else if (port == GPIOF)
    {
      RCC->AHBENR |= RCC_AHBENR_GPIOFEN;
    }

  // Configure pin as output
  port->MODER &= ~(0b11 << (pin << 1));
  port->MODER |= (0b01 << (pin << 1));
  // Configure pin as push-pull
  port->OTYPER &= ~(1 << pin);
  // Set maximum speed 
  port->OSPEEDR |= (0b11 << (pin << 1));
  // Set no pull-up or pull-down
  port->PUPDR &= ~(0b11 << (pin << 1));    
}

void set_pin_input (GPIO_TypeDef* port, int pin)
{
  // enable clock for the specified port
  if (port == GPIOA)
    {
      RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    }
  else if (port == GPIOB)
    {
      RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    }
  else if (port == GPIOC)
    {
      RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    }
  else if (port == GPIOD)
    {
      RCC->AHBENR |= RCC_AHBENR_GPIODEN;
    }
  else if (port == GPIOF)
    {
      RCC->AHBENR |= RCC_AHBENR_GPIOFEN;
    }
  
  // Configure GPIO pin as input for push button
  port->MODER &= ~(0b11 << (pin << 1));
  // Set GPIOC pin to no pull-up or pull-down
  port->PUPDR &= ~(0b11 << (pin << 1));
}


void set_pin_output_pulldown (GPIO_TypeDef* port, int pin)
{
  // enable clock for the specified port
  if (port == GPIOA)
    {
      RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    }
  else if (port == GPIOB)
    {
      RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    }
  else if (port == GPIOC)
    {
      RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    }
  else if (port == GPIOD)
    {
      RCC->AHBENR |= RCC_AHBENR_GPIODEN;
    }
  else if (port == GPIOF)
    {
      RCC->AHBENR |= RCC_AHBENR_GPIOFEN;
    }

  // Configure pin as output
  port->MODER &= ~(0b11 << (pin << 1));
  port->MODER |= (0b01 << (pin << 1));
  // Configure pin as push-pull
  port->OTYPER &= ~(1 << pin);
  // Set maximum speed 
  port->OSPEEDR |= (0b11 << (pin << 1));
  // Set pull-down
  port->PUPDR &= ~(0b11 << (pin << 1));
  port->PUPDR |= (0b10 << (pin << 1));
}

void set_pin_input_pulldown (GPIO_TypeDef* port, int pin)
{
  // enable clock for the specified port
  if (port == GPIOA)
    {
      RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    }
  else if (port == GPIOB)
    {
      RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    }
  else if (port == GPIOC)
    {
      RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    }
  else if (port == GPIOD)
    {
      RCC->AHBENR |= RCC_AHBENR_GPIODEN;
    }
  else if (port == GPIOF)
    {
      RCC->AHBENR |= RCC_AHBENR_GPIOFEN;
    }
  
  // Configure GPIO pin as input for push button
  port->MODER &= ~(0b11 << (pin << 1));
  // Set GPIOC pin to pull-down
  port->PUPDR &= ~(0b11 << (pin << 1));
  port->PUPDR |= (0b10 << (pin << 1));
}


void set_pin_output_pullup (GPIO_TypeDef* port, int pin)
{
  // enable clock for the specified port
  if (port == GPIOA)
    {
      RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    }
  else if (port == GPIOB)
    {
      RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    }
  else if (port == GPIOC)
    {
      RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    }
  else if (port == GPIOD)
    {
      RCC->AHBENR |= RCC_AHBENR_GPIODEN;
    }
  else if (port == GPIOF)
    {
      RCC->AHBENR |= RCC_AHBENR_GPIOFEN;
    }

  // Configure pin as output
  port->MODER &= ~(0b11 << (pin << 1));
  port->MODER |= (0b01 << (pin << 1));
  // Configure pin as push-pull
  port->OTYPER &= ~(1 << pin);
  // Set maximum speed 
  port->OSPEEDR |= (0b11 << (pin << 1));
  // Set pull-up
  port->PUPDR &= ~(0b11 << (pin << 1));
  port->PUPDR |= (0b01 << (pin << 1));
}

void set_pin_input_pullup (GPIO_TypeDef* port, int pin)
{
  // enable clock for the specified port
  if (port == GPIOA)
    {
      RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    }
  else if (port == GPIOB)
    {
      RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    }
  else if (port == GPIOC)
    {
      RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    }
  else if (port == GPIOD)
    {
      RCC->AHBENR |= RCC_AHBENR_GPIODEN;
    }
  else if (port == GPIOF)
    {
      RCC->AHBENR |= RCC_AHBENR_GPIOFEN;
    }
  
  // Configure GPIO pin as input for push button
  port->MODER &= ~(0b11 << (pin << 1));
  // Set GPIOC pin to pull-up
  port->PUPDR &= ~(0b11 << (pin << 1));
  port->PUPDR |= (0b01 << (pin << 1));
}
    
