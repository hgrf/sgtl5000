#include "sgtl5000.h"

#include "esp_log.h"

static const char *TAG = "CODEC";

#define READ_BIT                1
#define WRITE_BIT               0                   
#define ACK_CHECK_EN            1
#define ACK_CHECK_DIS           0
#define ACK_VAL                 0x0
#define NACK_VAL                0x1     

#define VALIDATE_REG_WRITES         1

// NOTE: a very clean register map is available at
// https://github.com/PaulStoffregen/Audio/blob/master/control_sgtl5000.cpp
#define SGTL5000_REG_CHIP_ID        0x0000
#define SGTL5000_REG_DIG_POWER      0x0002
#define SGTL5000_REG_CLK_CTRL       0x0004
#define SGTL5000_REG_I2S_CTRL       0x0006
#define SGTL5000_REG_SSS_CTRL       0x000a
#define SGTL5000_REG_ADCDAC_CTRL    0x000e
#define SGTL5000_REG_DAC_VOL        0x0010
#define SGTL5000_REG_ANA_HP_CTRL    0x0022
#define SGTL5000_REG_ANA_CTRL       0x0024
#define SGTL5000_REG_LINREG_CTRL    0x0026
#define SGTL5000_REG_REF_CTRL       0x0028
#define SGTL5000_REG_LINE_OUT_CTRL  0x002c
#define SGTL5000_REG_LINE_OUT_VOL   0x002e
#define SGTL5000_REG_ANA_POWER      0x0030
#define SGTL5000_REG_CLK_TOP_CTRL   0x0034
#define SGTL5000_REG_SHORT_CTRL     0x003c

static struct {
    i2c_port_t i2c_port;
    uint8_t i2c_address;
} hw_config;

static esp_err_t sgtl5000_read_reg(uint16_t reg_addr, uint16_t *data_rd)
{
    uint8_t buffer[2];
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (hw_config.i2c_address << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, (reg_addr >> 8) & 0xff, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr & 0xff, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (hw_config.i2c_address << 1) | READ_BIT, ACK_CHECK_EN);
    i2c_master_read(cmd, buffer, 1, ACK_VAL);
    i2c_master_read_byte(cmd, buffer + 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(hw_config.i2c_port, cmd, 100 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if(ret == ESP_OK)
        *data_rd = ((uint16_t) buffer[0] << 8) | buffer[1];

    return ret;
}

static esp_err_t sgtl5000_write_reg(uint16_t reg_addr, uint16_t val)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (hw_config.i2c_address << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, (reg_addr >> 8) & 0xff, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr & 0xff, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, (val >> 8) & 0xff, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, val & 0xff, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(hw_config.i2c_port, cmd, 100 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

#if VALIDATE_REG_WRITES
    if(ret != ESP_OK) {
        ESP_LOGE(TAG, "Register write failed at address 0x%04X", reg_addr);
        return ret;
    }
    
    uint16_t new_reg_val;
    ret = sgtl5000_read_reg(reg_addr, &new_reg_val);

    if(ret != ESP_OK) {
        ESP_LOGE(TAG, "Register validation read failed at address 0x%04X", reg_addr);
        return ret;
    }

    if(new_reg_val != val) {
        ESP_LOGE(TAG, "Register value validation failed at address 0x%04X: got 0x%04X instead of 0x%04X",
                    reg_addr, new_reg_val, val);
    }
#endif

    return ret;
}

esp_err_t sgtl5000_init(i2c_port_t port, uint8_t i2c_address)
{
    uint16_t reg_val;
    esp_err_t ret;

    hw_config.i2c_port = port;
    hw_config.i2c_address = i2c_address;

    ret = sgtl5000_read_reg(SGTL5000_REG_CHIP_ID, &reg_val);

    /* validate chip ID */
    if(ret == ESP_OK) {
        // expect 0xA0XX where XX is the revision number
        if((reg_val & 0xFF00) == 0xA000) {
            ESP_LOGI(TAG, "Chip ID correct: 0x%04X", reg_val);
        } else {
            ESP_LOGE(TAG, "Initialization failed. Wrong chip ID: 0x%04X", reg_val);
            return -1;
        }
    } else {
        ESP_LOGE(TAG, "Initialization failed. Could not read chip ID.");
        return -1;
    }

    /* configuration adapted from https://github.com/IoTBits/ESP32_SGTL5000_driver/blob/7082bb53fe462d0c419ea3f227842d07249ec6fd/components/audiobit/audiobit.c */

    /* digital power control: enable I2S data in and DAC */
    ret |= sgtl5000_write_reg(SGTL5000_REG_DIG_POWER, 0x0021);

    /* 44.1 kHz sample rate, with CLKM = 256*Fs = 11.289600 MHz */
    ret |= sgtl5000_write_reg(SGTL5000_REG_CLK_CTRL, 0x0004);

    /* I2S mode = 0 (I2S), LRALIGN = 0, LRPOL = 0
     * 16 bits/sample, I2S is slave, SCLK rate is 32*Fs
     */
    ret |= sgtl5000_write_reg(SGTL5000_REG_I2S_CTRL, 0x0130);

    /* I2S in -> DAC output, rest left at default */
    ret |= sgtl5000_write_reg(SGTL5000_REG_SSS_CTRL, 0x0010);

    /* unmute DAC, no volume ramp enabled */
    ret |= sgtl5000_write_reg(SGTL5000_REG_ADCDAC_CTRL, 0x0000);

    /* set DAC volume to 0 dB (left and right channel) */
    ret |= sgtl5000_write_reg(SGTL5000_REG_DAC_VOL, 0x3C3C);

    /* set headphone volume to -17dB each */
    ret |= sgtl5000_write_reg(SGTL5000_REG_ANA_HP_CTRL, 0x3A3A);

    /* mute ADC, disable zero cross detector (both headphone and ADC),
     * unmute headphones, mute lineout
     */
    ret |= sgtl5000_write_reg(SGTL5000_REG_ANA_CTRL, 0x0101);

    /* analog ground voltage = 1.575 V (max.) = approx. VDDA/2 (1.65 V)
     * bias current adjustment +12.5%
     */
    ret |= sgtl5000_write_reg(SGTL5000_REG_REF_CTRL, 0x01F2);

    /* stereo DAC, VDDD regulator off (supplied externally), start-up circuit
     * off, VDDC charge pump off, PLL off, analog ground buffer on, stereo ADC,
     * reference bias on, headphone amplifiers on, DAC on, capless headphone on,
     * ADC off, line-out off
     */
    ret |= sgtl5000_write_reg(SGTL5000_REG_ANA_POWER, 0x40FC);

    /* set headphone volume to 0 dB */
    ret |= sgtl5000_write_reg(SGTL5000_REG_ANA_HP_CTRL, 0x1818);

    if(ret != ESP_OK) {
        ESP_LOGE(TAG, "Initialization failed");
    } else {
        ESP_LOGI(TAG, "Initialization successful");
    }

    return ret;
}