#include <stdio.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"
#include "ADXL375.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
/* TWI instance ID. */
#define TWI_INSTANCE_ID     0

/* TWI instance. */
static const nrf_drv_twi_t adxl375_i2c = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

/**
 * @brief TWI events handler.
 */
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
  
}

void ADXL375_Init(void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t i2c_adxl375_config = {
       .scl                = 26,
       .sda                = 27,
       .frequency          = NRF_DRV_TWI_FREQ_400K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&adxl375_i2c, &i2c_adxl375_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&adxl375_i2c);    
   
}

void ADXL375_setMode(uint8_t mode)
{
    ret_code_t err_code;
    //Set mode by default to Measurement Mode
    uint8_t tx_data[2] = {ADXL375_REG_POWER_CTL, mode}; 
    // Put the ADXL375 in Measurement Mode by writing 0x08 to the POWER_CTL Register
    err_code = ADXL375_I2C_writeRegister(ADXL375_I2C_LOW_ADDRESS, tx_data, sizeof(tx_data) );
}


void ADXL375_setDataRate(uint8_t rate)
{
    uint8_t tx_data[2] = {ADXL375_REG_BW_RATE, rate};
    ret_code_t err_code;
    err_code = ADXL375_I2C_writeRegister(ADXL375_I2C_LOW_ADDRESS, tx_data, sizeof(tx_data));
    APP_ERROR_CHECK(err_code);
    
    uint8_t rx_data = 0x00;
    //This read is just for control (checking if write was successful)
    err_code = ADXL375_I2C_readRegister(ADXL375_I2C_LOW_ADDRESS, ADXL375_REG_BW_RATE, &rx_data, 1);
     
    APP_ERROR_CHECK(err_code);
  
    if(rx_data == rate)
    {
        NRF_LOG_INFO("Read Rate %d ", rx_data);
        NRF_LOG_FLUSH();
        NRF_LOG_INFO("Setting Data rate Sucessful!");
        NRF_LOG_FLUSH();
    }


}

void ADXL375_irq_configurations(void)
{
    uint8_t int_map = 0;
    uint8_t tx_data[2] = {ADXL375_REG_INT_MAP, 0x9F};  // Allowing single and double chock occurs on INT1 
    ADXL375_I2C_writeRegister(ADXL375_I2C_LOW_ADDRESS, tx_data, sizeof(tx_data));
    ADXL375_I2C_readRegister(ADXL375_I2C_LOW_ADDRESS, ADXL375_REG_INT_MAP, &int_map, 1);
    NRF_LOG_INFO("INT MAP %x", int_map);
    NRF_LOG_FLUSH();

    //Interrupt enable
    tx_data[0] = ADXL375_REG_INT_ENABLE;
    tx_data[1] = 0x60;                  // Enabling single and double shock interrupt on INT1
    ADXL375_I2C_writeRegister(ADXL375_I2C_LOW_ADDRESS, tx_data, sizeof(tx_data));


}

void ADXL375_clear_interrupts(void)
{
    //ADXL375 Interrupts are cleared by reading the INT_SOURCE register.
    //This is done because otherwise the interrupt pin (INT1/INT2) will always be active high 
    uint8_t tmp = 0;
    ADXL375_I2C_readRegister(ADXL375_I2C_LOW_ADDRESS, ADXL375_REG_INT_SOURCE, &tmp, 1);
    
}

void ADXL375_read_interrupt(void)
{
    uint8_t int_source = 0;
    uint8_t shock_status = 0;
    int16_t x,z,y = 0;
    ADXL375_I2C_readRegister(ADXL375_I2C_LOW_ADDRESS, ADXL375_REG_INT_SOURCE, &int_source, 1);

    if (int_source & ADXL375_SINGLE_SCHOCK_DETECTED)
    {
        NRF_LOG_INFO("SINGLE SHOCK DETECTED");
    }
    if (int_source & ADXL375_DOUBLE_SHOCK_DETECTED)
    {
        NRF_LOG_INFO("DOUBLE SHOCK DETECTED ------------------------*****----");
    }
    
    //Read the first axis indicated in the shock or activity
    ADXL375_I2C_readRegister(ADXL375_I2C_LOW_ADDRESS, ADXL375_REG_ACT_SHOCK_STATUS, &shock_status, 1);

    if (shock_status & 0x04)        //Check if the X-axis was the first axis shock occured on
    {
        NRF_LOG_INFO("Shock occured first on X axis");
    }
    if (shock_status & 0x02)        //Check if the Y-axis was the first axis
    {
        NRF_LOG_INFO("Shock occured first on Y axis");
    }
    if (shock_status & 0x01)        //Check if the Z-axis was the first axis
    {
        NRF_LOG_INFO("Shock occured first on Z axis");
    }

    ADXL375_acc_data_read(&x,  &y,  &z);
    NRF_LOG_INFO("X-Axis: %d  Y-Axis: %d   Z-Axis: %d", x, y, z);
}

void ADXL375_offsets_set(int8_t x_offset, int8_t y_offset, int8_t z_offset)
{
    ret_code_t ret_code;
    
    int8_t tx_data[4] = {ADXL375_REG_OFSX, x_offset,y_offset,z_offset};
    
    ret_code = ADXL375_I2C_writeRegister(ADXL375_I2C_LOW_ADDRESS, tx_data, sizeof(tx_data));
    
    APP_ERROR_CHECK(ret_code);
    
    uint8_t rx_data[3] ={0x00,0x00,0x00};
    
    ret_code = ADXL375_I2C_readRegister(ADXL375_I2C_LOW_ADDRESS, ADXL375_REG_OFSX, rx_data, sizeof(rx_data)); 
    
    APP_ERROR_CHECK(ret_code);
    
    
    if( rx_data[0] == x_offset &&
        rx_data[1] == y_offset &&
        rx_data[2] == z_offset  )
    {
        NRF_LOG_INFO("Setting Offsets Sucessful!");
    }
    else
    {
        NRF_LOG_INFO("Setting Offsets Failed!");
    }
}

void ADXL375_shock_axes(void)
{
    uint8_t tx_data[2] = {ADXL375_REG_SHOCK_AXES, 0x07}; //0x07 means enabling x, y and z axis for shock detection
    ADXL375_I2C_writeRegister(ADXL375_I2C_LOW_ADDRESS, tx_data, sizeof(tx_data));
}

void ADXL375_setFIFOMode(void)
{
    uint8_t tx_data[2]={ADXL375_REG_FIFO_CTL, 00};
    ADXL375_I2C_writeRegister(ADXL375_I2C_LOW_ADDRESS, tx_data, sizeof(tx_data));

    //tx_data[1] = 0xC0;
    //ADXL375_I2C_writeRegister(ADXL375_I2C_LOW_ADDRESS, tx_data, sizeof(tx_data));
}

void ADXL375_config_shock(uint8_t threshold, uint8_t shock_dur_ms, uint16_t shock_latency_ms)
{
    uint8_t threshold_scaled_value = (threshold*1000/ADXL375_THRESH_SHOCK_SCALE); // threshold MAXIMUM input value is approx 198g (780mg/LSB), see datasheet
    uint8_t dur_scaled_value = (shock_dur_ms*1000/ADXL375_SHOCK_DUR_SCALE);          // scale shock duration time conversion, MAXIMUM 159ms 
    uint8_t latency_scaled_val = (shock_latency_ms*100/ADXL375_SHOCK_LATENCY_SCALE); // scale shock latency in ms, MAXIMUM latency 318ms 

    //Write the shock threshold configuration
    uint8_t tx_data[2] = {ADXL375_REG_THRESH_SHOCK, threshold_scaled_value};
    ADXL375_I2C_writeRegister(ADXL375_I2C_LOW_ADDRESS, tx_data, sizeof(tx_data));

    //Write the shock duration configuration
    tx_data[0] = ADXL375_REG_DUR;
    tx_data[1] = dur_scaled_value;
    ADXL375_I2C_writeRegister(ADXL375_I2C_LOW_ADDRESS, tx_data, sizeof(tx_data));

#if ADXL375_DOUBLE_SHOCK
    //Write the shock latency configuration
    tx_data[0] = ADXL375_REG_LATENT;
    tx_data[1] = latency_scaled_val;
    ADXL375_I2C_writeRegister(ADXL375_I2C_LOW_ADDRESS, tx_data, sizeof(tx_data));
#endif
    ADXL375_shock_axes();
    ADXL375_setFIFOMode();
    
}


uint8_t ADXL375_readDataRate(void)
{
    uint8_t data = 0;
    ADXL375_I2C_readRegister(ADXL375_I2C_LOW_ADDRESS, ADXL375_REG_BW_RATE, &data, 1);
    return data;
}

void ADXL375_DeviceID(uint8_t* hex)
{
    uint8_t pdata = 0;
    ADXL375_I2C_readRegister(ADXL375_I2C_LOW_ADDRESS, ADXL375_REG_DEVID, &pdata, 1);
    *hex = pdata;
}


void ADXL375_acc_data_read(int16_t * x_value, int16_t * y_value, int16_t * z_value)
{
    ret_code_t ret_code;
    
    uint8_t rx_buffer[6];
    
    // The DATAX0 register contains the acceleration data
    ret_code = ADXL375_I2C_readRegister(ADXL375_I2C_LOW_ADDRESS, ADXL375_REG_DATAX0, rx_buffer, sizeof(rx_buffer)); 
    
    APP_ERROR_CHECK(ret_code);
    
    // The reason it is done in this manner is because the buffer field 0 and 1, represents the x value
    // and you must left shift the second buffer, so it occupies the 8 MSB bits (same logic with y and z value)
    *x_value = (rx_buffer[1]<<8)|rx_buffer[0];
    *y_value = (rx_buffer[3]<<8)|rx_buffer[2];
    *z_value = (rx_buffer[5]<<8)|rx_buffer[4];

    // This is done because the outputs for each axis is expressed in LSBs and instead are converted to acceleration (g)
    // by multiplying each value by the 49 mg/LSB scale factor
    *x_value = *x_value * ADXL375_XYZ_READ_SCALE_FACTOR;
    *y_value = *y_value * ADXL375_XYZ_READ_SCALE_FACTOR;
    *z_value = *z_value * ADXL375_XYZ_READ_SCALE_FACTOR;
}


ret_code_t ADXL375_I2C_readRegister(uint8_t slave_addr, uint8_t reg_addr,  uint8_t * pdata, uint32_t bytes)
{
    ret_code_t err_code;
    err_code = nrf_drv_twi_tx(&adxl375_i2c,slave_addr, &reg_addr, 1, false);
    if(err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    err_code = nrf_drv_twi_rx(&adxl375_i2c,slave_addr, pdata, bytes);
    
    return err_code;
}


ret_code_t ADXL375_I2C_writeRegister(uint8_t regAddr, uint8_t* pdata, uint32_t bytes)
{
    ret_code_t err_code;
    err_code = nrf_drv_twi_tx(&adxl375_i2c, regAddr, pdata, bytes, false);
    APP_ERROR_CHECK(err_code);
}