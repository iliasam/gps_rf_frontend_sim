#include "stm32f10x.h"
#include "init_periph.h"
#include "simulator.h"


int main()
{
  init_all_periph();
  sim_generate_data();
  sim_start_tx_data();
  
  while(1)
  {
    sim_handling();
  }

  return 0;
}

