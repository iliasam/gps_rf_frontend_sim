# gps_rf_frontend_sim
Simple GPS RF front-end simulator based on STM32  
  
This device is based on STM32F103C8T6 (blue pill).  
It generates serial data using SPI (CLK + MOSI), and emulating 1-bit ADC of GPS RF front-end.  
Software is emulating receiving of one GPS satellite, which is sending constant navigation data (subframe 1).  
  
CLK frequency is 16 MHz, data IF frequency is 4 MHz, which is close to parameters of MAX2769.  
PA5 is CLK  
PA7 is MOSI  

