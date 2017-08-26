#include <stm32f3xx.h>
#include <meteo_drone.hpp>

meteo_drone station;

int main() {

  // initialisation parameters
  // time and rate in seconds
  station.initialise();
  station.set_awake_time(120);
  station.set_no_sleep(false);
  station.set_polling_rate(4);
  station.current_mode = 'i';
  
  // mode cycling and switching
  while (1)
    {
      switch(station.current_mode)
	{
	case 's':
	  station.mode_sleep();
	  break;
	case 'c':
	  station.mode_config();
	  break;
	case 'i':
	  station.fallback_mode = 'i';
	  station.mode_internal_sensors();
	  break;
	case 't':
	  station.fallback_mode = 't';
	  station.mode_time();
	}
    }
}

extern "C" void SysTick_Handler (void)
{
  station.ticks++;
  station.uptime_since_action++;
  station.uptime_since_sleep++;
}

extern "C" void EXTI15_10_IRQHandler (void)
{
  // check that interrupt source is EXTI13 and interrupt is pending
  if ((EXTI->IMR & EXTI_IMR_MR13) && (EXTI->PR & EXTI_PR_PR13))
    {
      station.user_button_pushed = true;
      station.button_pressed = true;
      // clear pending status
      EXTI->PR |= EXTI_PR_PR13;
    }
}

extern "C" void EXTI0_IRQHandler (void)
{
  NVIC_DisableIRQ(EXTI0_IRQn);
  brute_delay(20000);  
  if (EXTI->RTSR & EXTI_RTSR_RT0)
    {     
      if (EXTI->PR & EXTI_PR_PR0)
	{
	  EXTI->RTSR &= ~(EXTI_RTSR_RT0);
	  EXTI->FTSR |= EXTI_FTSR_TR0;
	  station.enter_button_pushed = true;
	  station.button_pressed = true;
	}
    }
  
  else if (EXTI->FTSR & EXTI_FTSR_TR0)
    {
      if (EXTI->PR & EXTI_PR_PR0)
	{
	  EXTI->FTSR &= ~(EXTI_FTSR_TR0);
	  EXTI->RTSR |= EXTI_RTSR_RT0;
	}
    }
  
  EXTI->PR |= EXTI_PR_PR0;
  NVIC_EnableIRQ(EXTI0_IRQn);
}

extern "C" void EXTI1_IRQHandler (void)
{
  NVIC_DisableIRQ(EXTI1_IRQn);
  brute_delay(20000);
  if (EXTI->RTSR & EXTI_RTSR_RT1)
    {     
      if (EXTI->PR & EXTI_PR_PR1)
	{
	  EXTI->RTSR &= ~(EXTI_RTSR_RT1);
	  EXTI->FTSR |= EXTI_FTSR_TR1;
	  station.right_button_pushed = true;
	  station.button_pressed = true;
	}
    }
  
  else if (EXTI->FTSR & EXTI_FTSR_TR1)
    {
      if (EXTI->PR & EXTI_PR_PR1)
	{
	  EXTI->FTSR &= ~(EXTI_FTSR_TR1);
	  EXTI->RTSR |= EXTI_RTSR_RT1;
	}
    }
  
  EXTI->PR |= EXTI_PR_PR1;
  NVIC_EnableIRQ(EXTI1_IRQn);
}

  

extern "C" void EXTI2_TSC_IRQHandler (void)
{
  NVIC_DisableIRQ(EXTI2_TSC_IRQn);
  brute_delay(20000);
  if (EXTI->RTSR & EXTI_RTSR_RT2)
    {     
      if (EXTI->PR & EXTI_PR_PR2)
	{
	  EXTI->RTSR &= ~(EXTI_RTSR_RT2);
	  EXTI->FTSR |= EXTI_FTSR_TR2;
	  station.left_button_pushed = true;
	  station.button_pressed = true;
	}
    }
  
  else if (EXTI->FTSR & EXTI_FTSR_TR2)
    {
      if (EXTI->PR & EXTI_PR_PR2)
	{
	  EXTI->FTSR &= ~(EXTI_FTSR_TR2);
	  EXTI->RTSR |= EXTI_RTSR_RT2;
	}
    }
  
  EXTI->PR |= EXTI_PR_PR2;
  NVIC_EnableIRQ(EXTI2_TSC_IRQn);
}


extern "C" void EXTI3_IRQHandler (void)
{
  NVIC_DisableIRQ(EXTI3_IRQn);
  brute_delay(20000);
  if (EXTI->RTSR & EXTI_RTSR_RT3)
    {     
      if (EXTI->PR & EXTI_PR_PR3)
	{
	  EXTI->RTSR &= ~(EXTI_RTSR_RT3);
	  EXTI->FTSR |= EXTI_FTSR_TR3;
	  station.config_button_pushed = true;
	  station.button_pressed = true;
	}
    }
  
  else if (EXTI->FTSR & EXTI_FTSR_TR3)
    {
      if (EXTI->PR & EXTI_PR_PR3)
	{
	  EXTI->FTSR &= ~(EXTI_FTSR_TR3);
	  EXTI->RTSR |= EXTI_RTSR_RT3;
	}
    }
  
  EXTI->PR |= EXTI_PR_PR3;
  NVIC_EnableIRQ(EXTI3_IRQn);
}
