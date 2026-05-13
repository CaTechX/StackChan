#include "LTR553.h"
#include <esp_log.h>
#include <driver/i2c_master.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


static const char *TAG = "LTR553";

// -- register map (LTR-553ALS-WA) --
static constexpr uint8_t REG_ALS_CONTR     = 0x80;
static constexpr uint8_t REG_PS_CONTR      = 0x81;
static constexpr uint8_t REG_PS_LED        = 0x82;
static constexpr uint8_t REG_PS_N_PULSES   = 0x83;
static constexpr uint8_t REG_PS_MEAS_RATE  = 0x84;
static constexpr uint8_t REG_ALS_MEAS_RATE = 0x85;
static constexpr uint8_t REG_PART_ID       = 0x86;
static constexpr uint8_t REG_ALS_CH1_LO    = 0x88; // ALS CH1 (IR only)
static constexpr uint8_t REG_ALS_CH1_HI    = 0x89;
static constexpr uint8_t REG_ALS_CH0_LO    = 0x8A; // ALS CH0 (visible + IR)
static constexpr uint8_t REG_ALS_CH0_HI    = 0x8B;
static constexpr uint8_t REG_PS_DATA_LO    = 0x8E;
static constexpr uint8_t REG_PS_DATA_HI    = 0x8F;

// -- ALS_CONTR bits --
static constexpr uint8_t ALS_MODE     = 1 << 0;
static constexpr uint8_t ALS_SW_RESET = 1 << 1;
static constexpr uint8_t ALS_GAIN_B0  = 1 << 2;
static constexpr uint8_t ALS_GAIN_B1  = 1 << 3;
// 00 = 1X, 01 = 2X, 10 = 4X, 11 = 8X

// -- PS_CONTR bits --
static constexpr uint8_t PS_MODE = 1 << 0;

LTR553::LTR553(i2c_master_bus_handle_t bus, uint8_t addr)
    : m_bus(bus), m_addr(addr) {}

LTR553::~LTR553() {
    if (m_dev) {
        i2c_master_bus_rm_device(m_dev);
    }
}

bool LTR553::begin() {
    // Add I2C device
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = m_addr,
        .scl_speed_hz    = 100000,
    };
    if (i2c_master_bus_add_device(m_bus, &dev_cfg, &m_dev) != ESP_OK) {
        ESP_LOGE(TAG, "failed to add i2c device at 0x%02X", m_addr);
        return false;
    }

    // Verify part ID
    uint8_t id = readReg(REG_PART_ID);
    ESP_LOGI(TAG, "part id: 0x%02X (expect 0x92)", id);

    // Soft-reset
    writeReg(REG_ALS_CONTR, ALS_SW_RESET);
    vTaskDelay(pdMS_TO_TICKS(10));

    // -- ALS config: active, 1X gain, 100 ms integration --
    writeReg(REG_ALS_CONTR, ALS_MODE);                    // active, 1X
    writeReg(REG_ALS_MEAS_RATE, 0x01);                    // 100 ms integration
    vTaskDelay(pdMS_TO_TICKS(10));

    // -- PS config: active, LED on, 1 pulse, 12.5 ms --
    writeReg(REG_PS_CONTR, PS_MODE);
    vTaskDelay(pdMS_TO_TICKS(10));
    writeReg(REG_PS_LED, 0x7F);                           // max LED current
    writeReg(REG_PS_N_PULSES, 0x01);                      // 1 pulse
    writeReg(REG_PS_MEAS_RATE, 0x02);                     // 12.5 ms
    // Enable the PS diode and set gain
    writeReg(REG_PS_CONTR, PS_MODE | (1 << 2) | (1 << 4));// active, diode=1, high gain
    vTaskDelay(pdMS_TO_TICKS(50));

    ESP_LOGI(TAG, "LTR-553ALS-WA initialized");
    return true;
}

uint8_t LTR553::readReg(uint8_t reg) {
    uint8_t value = 0;
    if (i2c_master_transmit_receive(m_dev, &reg, 1, &value, 1, 100) != ESP_OK) {
        ESP_LOGW(TAG, "read reg 0x%02X failed", reg);
    }
    return value;
}

bool LTR553::writeReg(uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {reg, value};
    esp_err_t ret = i2c_master_transmit(m_dev, buf, 2, 100);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "write reg 0x%02X <- 0x%02X failed: %s",
                 reg, value, esp_err_to_name(ret));
        return false;
    }
    return true;
}

bool LTR553::readALS(uint16_t &ch0, uint16_t &ch1) {
    ch0 = (uint16_t)readReg(REG_ALS_CH0_LO) | ((uint16_t)readReg(REG_ALS_CH0_HI) << 8);
    ch1 = (uint16_t)readReg(REG_ALS_CH1_LO) | ((uint16_t)readReg(REG_ALS_CH1_HI) << 8);
    ESP_LOGD(TAG, "ALS  ch0=%u  ch1=%u", ch0, ch1);
    return true;
}

bool LTR553::readPS(uint16_t &ps_data) {
    ps_data = (uint16_t)readReg(REG_PS_DATA_LO) | ((uint16_t)readReg(REG_PS_DATA_HI) << 8);
    ESP_LOGD(TAG, "PS  data=%u", ps_data);
    return true;
}

float LTR553::calcLux(uint16_t ch0, uint16_t ch1) {
    // CH0 = visible + IR,  CH1 = IR only
    // Compensation table from Lite-On application note
    float ratio = (ch0 + ch1) > 0
                      ? (float)ch1 / (float)(ch0 + ch1)
                      : 0.0f;
    float comp;
    if (ratio < 0.45f)      comp = 1.0f;    // mostly visible
    else if (ratio < 0.64f) comp = 0.75f;
    else if (ratio < 0.85f) comp = 0.5f;
    else                    comp = 0.339f;   // mostly IR

    // 1X gain → gain_factor = 1.0
    return (float)ch0 * comp;
}
