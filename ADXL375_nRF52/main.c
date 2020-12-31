/**
 * This file contain the main application for the ADXL375 Accelerometer
 * It shows the initialization, I2C and Interrupt configuration for the device from host nRF52.
 * 
 * 
 * 
 * 
 */

#include <stdio.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "nrf_drv_gpiote.h"
#include "ADXL375.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


#define ADXL375_PIN_INT1    11

void irq_int1_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    NRF_LOG_INFO("IRQ Triggered \n");
    ADXL375_read_interrupt();
}

void gpio_init(void)
{
    ret_code_t err_code;
    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);
    
    nrf_drv_gpiote_in_config_t config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
    config.sense = NRF_GPIOTE_POLARITY_LOTOHI;
    config.pull = NRF_GPIO_PIN_PULLDOWN;

    err_code = nrf_drv_gpiote_in_init(ADXL375_PIN_INT1, &config, irq_int1_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(ADXL375_PIN_INT1, true);
}

/**
 * @brief Function for main application entry.
 */
int main(void)
{
    int16_t z,x,y = 0;
    uint8_t threshold = 10;
    uint8_t shock_dur_ms = 10;
    uint16_t shock_latency_ms = 200;

    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    NRF_LOG_INFO("ADXL375 Example started.");

    gpio_init();
    ADXL375_Init();
    ADXL375_clear_interrupts();
    ADXL375_offsets_set(0,0,0);
    ADXL375_config_shock(threshold, shock_dur_ms, shock_latency_ms);
    ADXL375_irq_configurations();
    ADXL375_setDataRate(ADXL375_DR_3200HZ);
    ADXL375_setMode(ADXL375_MEASURING_MODE);
 
    while (true)
    {
        ADXL375_acc_data_read(&x,  &y,  &z);
        NRF_LOG_INFO("X-Axis: %d  Y-Axis: %d   Z-Axis: %d", x, y, z);
        NRF_LOG_FLUSH();
        nrf_delay_ms(250);
    }
}

/** @} */
