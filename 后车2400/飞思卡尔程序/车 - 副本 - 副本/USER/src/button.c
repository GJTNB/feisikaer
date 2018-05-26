#include "button.h"

uint8 b_value=0;

uint8 get_button_value(void)
{
  if(PTC16_IN==0){
    pit_delay_ms(PIT0,10);
    if(PTC16_IN==0){
      return 1;
    }
  }
  if(PTC18_IN==0){
    pit_delay_ms(PIT0,10);
    if(PTC18_IN==0){
      return 2;
    }
  }
  if(PTC17_IN==0){
    pit_delay_ms(PIT0,10);
    if(PTC17_IN==0){
      return 3;
    }
  }
  if(PTC19_IN==0){
    pit_delay_ms(PIT0,10);
    if(PTC19_IN==0){
      return 4;
    }
  }
  if(PTD0_IN==0){
    pit_delay_ms(PIT0,10);
    if(PTD0_IN==0){
      return 5;
    }
  }
  
  return 0;
}

void set_b_value(void)
{
  b_value=0;
}