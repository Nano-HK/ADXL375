# ADXL375 nRF52
This code is executed and tested on the following:
- nRF52840 DK
- NRF52 SDK v17
- ADXL375 EVK
The purpose of this repo is to give a good starting point to develop ADXL375 on nRF52. It is up to you to make it fit your requirements. Do tell if you find any issues. 

The code requires the following libraries from nRF52 SDK:
- nrfx_gpiote (Pin interrupt)
  - nrf_drv_gpiote.h
- twi (I2C)
  - nrf_drv_twi.h
  
  # ADXL375 EVK 
  The code is written to handle single shock detection, shock configurations, interrupt configs, data rate and ADXL375 modes. 
