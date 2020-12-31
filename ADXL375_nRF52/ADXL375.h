#include <stdint.h>
#include "app_error.h"
/* Common addresses definition for ADXL375 sensor. */
#define ADXL375_REG_DEVID               0x00    // Device ID
#define ADXL375_REG_THRESH_SHOCK        0x1D    // Shock threshold
#define ADXL375_REG_OFSX                0x1E    // X-axis offset
#define ADXL375_REG_OFSY                0x1F    // Y-axis offset
#define ADXL375_REG_OFSZ                0x20    // Z-axis offset
#define ADXL375_REG_DUR                 0x21    // Shock duration
#define ADXL375_REG_LATENT              0x22    // Shock latency
#define ADXL375_REG_WINDOW              0x23    // Tap window
#define ADXL375_REG_THRESH_ACT          0x24    // Activity threshold
#define ADXL375_REG_THRESH_INACT        0x25    // Inactivity threshold
#define ADXL375_REG_TIME_INACT          0x26    // Inactivity time
#define ADXL375_REG_ACT_INACT_CTL       0x27    // Axis enable control for activity and inactivity detection
#define ADXL375_REG_SHOCK_AXES          0x2A    // Axis control for single/double tap
#define ADXL375_REG_ACT_SHOCK_STATUS    0x2B    // Source for single/double tap
#define ADXL375_REG_BW_RATE             0x2C    // Data rate and power mode control
#define ADXL375_REG_POWER_CTL           0x2D    // Power-saving features control
#define ADXL375_REG_INT_ENABLE          0x2E    // Interrupt enable control
#define ADXL375_REG_INT_MAP             0x2F    // Interrupt mapping control
#define ADXL375_REG_INT_SOURCE          0x30    // Source of interrupts
#define ADXL375_REG_DATA_FORMAT         0x31    // Data format control
#define ADXL375_REG_DATAX0              0x32    // X-axis data 0
#define ADXL375_REG_DATAX1              0x33    // X-axis data 1
#define ADXL375_REG_DATAY0              0x34    // Y-axis data 0
#define ADXL375_REG_DATAY1              0x35    // Y-axis data 1
#define ADXL375_REG_DATAZ0              0x36    // Z-axis data 0
#define ADXL375_REG_DATAZ1              0x37    // Z-axis data 1
#define ADXL375_REG_FIFO_CTL            0x38    // FIFO control
#define ADXL375_REG_FIFO_STATUS         0x39    // FIFO status

#define ADXL375_XYZ_READ_SCALE_FACTOR   49      // scaling factor when reading xyz data
#define ADXL375_THRESH_SHOCK_SCALE      780     // scaling factor for shock threshold register
#define ADXL375_SHOCK_DUR_SCALE         625     // scaling factor for shock duration
#define ADXL375_SHOCK_LATENCY_SCALE     125     // scaling factor for latency duration to detect double shock

#define ADXL375_FIFO_MODE_BYPASS        0b00
#define ADXL375_FIFO_MODE_FIFO          0b01
#define ADXL375_FIFO_MODE_STREAM        0b10
#define ADXL375_FIFO_MODE_TRIGGER       0b11

// when ALT ADDRESS pin is LOW
#define ADXL375_I2C_LOW_READ            0xA7
#define ADXL375_I2C_LOW_WRITE           0xA6
#define ADXL375_I2C_LOW_ADDRESS         0x53
 
//when ALT ADDRESS pin is HIGH
#define ADXL375_I2C_HIGH_READ           0x3B
#define ADXL375_I2C_HIGH_WRITE          0x3A
#define ADXL375_I2C_HIGH_ADDRESS        0x1D

#define ADXL375_TRIGGER_INT1_PIN        0
#define ADXL375_TRIGGER_INT2_PIN        1

//Data rate (DR) defines for ADXL375, see datasheet as reference.
#define ADXL375_DR_3200HZ      0x0F
#define ADXL375_DR_1600HZ      0x0E
#define ADXL375_DR_800HZ       0x0D
#define ADXL375_DR_400HZ       0x0C
#define ADXL375_DR_200HZ       0x0B
#define ADXL375_DR_100HZ       0x0A
#define ADXL375_DR_50HZ        0x09
#define ADXL375_DR_25HZ        0x08
#define ADXL375_DR_12HZ5       0x07
#define ADXL375_DR_6HZ25       0x06

/**
 * ADXL375 operating modes (see datasheet)
 */
#define ADXL375_MEASURING_MODE          0x08
#define ADLX375_SLEEP_MODE              0x04
#define ADLX375_AUTOSLEEP_MODE          0x10

/**
 * Defines related to the shock functionality of ADXL375
 * 
 * */
#define ADXL375_DOUBLE_SHOCK            1       //Set this define to 1 if want to include double shock
#define ADXL375_EN_ONLY_SINGLE_SHOCK    0x20    //Enables only single shock detection
#define ADXL375_EN_ONLY_DOUBLE_SHOCK    0x40    //Enables only double shock detection
#define ADXL375_EN_ALL_SHOCK            0x60    //Enables both double and single shock detection

#define ADXL375_SINGLE_SCHOCK_DETECTED  0x40    //To identify if single shock is detected
#define ADXL375_DOUBLE_SHOCK_DETECTED   0x20    //-''-

/********************************************************************************************************/

void ADXL375_Init(void);
void ADXL375_DeviceID(uint8_t* hex);
void ADXL375_setDataRate(uint8_t rate);
void ADXL375_setMode(uint8_t mode);
void ADXL375_offsets_set(int8_t x_offset, int8_t y_offset, int8_t z_offset);
void ADXL375_configurations(void);
void ADXL375_irq_configurations(void);
void ADXL375_config_shock(uint8_t threshold, uint8_t shock_dur_ms, uint16_t shock_latency_ms);
void ADXL375_acc_data_read(int16_t * x_value, int16_t * y_value, int16_t * z_value);
void ADXL375_read_interrupt(void);
void ADXL375_clear_interrupts(void);
uint8_t ADXL375_readDataRate(void);
ret_code_t ADXL375_I2C_readRegister(uint8_t slave_addr, uint8_t reg_addr,  uint8_t * pdata, uint32_t bytes);
ret_code_t ADXL375_I2C_writeRegister(uint8_t regAddr, uint8_t* pdata, uint32_t bytes);