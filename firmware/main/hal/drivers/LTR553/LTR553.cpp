#include "LTR553.h"
#include <esp_log.h>
#include <driver/i2c_master.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


static const char *TAG = "LTR553";

// -- register map (LTR-553ALS-WA, based on LTR5XX official library for LTR55X) --
static constexpr uint8_t REG_ALS_CONTR      = 0x80;
static constexpr uint8_t REG_PS_CONTR       = 0x81;
static constexpr uint8_t REG_PS_LED         = 0x82;
static constexpr uint8_t REG_PS_N_PULSES    = 0x83;
static constexpr uint8_t REG_PS_MEAS_RATE   = 0x84;
static constexpr uint8_t REG_ALS_MEAS_RATE  = 0x85;
static constexpr uint8_t REG_PART_ID        = 0x86;
static constexpr uint8_t REG_MANUFACT_ID    = 0x87;
static constexpr uint8_t REG_ALS_DATA_CH1_0 = 0x88; // ALS CH1 (IR only)
static constexpr uint8_t REG_ALS_DATA_CH1_1 = 0x89;
static constexpr uint8_t REG_ALS_DATA_CH0_0 = 0x8A; // ALS CH0 (visible + IR)
static constexpr uint8_t REG_ALS_DATA_CH0_1 = 0x8B;
static constexpr uint8_t REG_ALS_PS_STATUS  = 0x8C; // status register
static constexpr uint8_t REG_PS_DATA_LOW    = 0x8D; // *** was 0x8E (wrong!)
static constexpr uint8_t REG_PS_DATA_HIGH   = 0x8E; // *** was 0x8F (wrong!)

// -- ALS_CONTR bits (0x80) --
static constexpr uint8_t ALS_MODE     = 1 << 0;
static constexpr uint8_t ALS_SW_RESET = 1 << 1;
static constexpr uint8_t ALS_GAIN_B0  = 1 << 2;
static constexpr uint8_t ALS_GAIN_B1  = 1 << 3;
// 00 = 1X, 01 = 2X, 10 = 4X, 11 = 8X

// -- PS_CONTR bits (0x81) --
// Bit 1 = PS active (per LTR5XX library: PS_MODE_MASK = 0x02, shift = 1)
//        Pre-0.1 driver had PS_MODE at bit 0 — that was ALS_CONTR's bit layout copied over!
static constexpr uint8_t PS_MODE_BIT  = 1 << 1;
// Bit 4 = PS gain (0 = 1X, 1 = 16X)
static constexpr uint8_t PS_GAIN_HIGH = 1 << 4;

// -- Status register bits (0x8C) --
// Bit 7 = PS data valid (LTR5XX: LTR5XX_VALID_PS_DATA_MASK = 0x80)
static constexpr uint8_t STATUS_PS_VALID = 1 << 7;

// -- PS data masks --
static constexpr uint8_t PS_DATA_LOW_MASK  = 0xFF;  //  8 bits
static constexpr uint8_t PS_DATA_HIGH_MASK = 0x07;  //  3 bits → 11-bit total

// -- Default configuration values (matching LTR55X defaults from LTR5XX library) --
static constexpr uint8_t LED_PULSE_FREQ     = 0x03;  // 60 KHz
static constexpr uint8_t LED_DUTY_CYCLE     = 0x03;  // 100 %
static constexpr uint8_t LED_PEAK_CURRENT   = 0x04;  // 100 mA (0x07 = reserved/invalid!)
static constexpr uint8_t CFG_PS_N_PULSES    = 8;     // 8 pulses (was 1 — too weak)
static constexpr uint8_t CFG_PS_MEAS_RATE   = 0x02;  // 100 ms per LTR55X table
static constexpr uint8_t CFG_ALS_GAIN       = 0x00;  // 1X
static constexpr uint8_t CFG_ALS_MEAS_RATE  = 0x01;  // bits[2:0] = 100 ms


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

    // Part ID check
    uint8_t id  = readReg(REG_PART_ID);
    uint8_t mfg = readReg(REG_MANUFACT_ID);
    ESP_LOGI(TAG, "part id=0x%02X  mfg id=0x%02X", id, mfg);

    // Soft-reset ALS
    writeReg(REG_ALS_CONTR, ALS_SW_RESET);
    vTaskDelay(pdMS_TO_TICKS(10));

    // Put PS into standby before configuring (matching LTR5XX begin() pattern)
    writeReg(REG_PS_CONTR, 0);
    vTaskDelay(pdMS_TO_TICKS(1));

    // -- Configure PS LED --
    // Register 0x82 layout:
    //   bits[7:5] = LED pulse frequency  (LTR5XX: 0x03 = 60 KHz)
    //   bits[4:3] = LED duty cycle       (LTR5XX: 0x03 = 100 %)
    //   bits[2:0] = LED peak current     (LTR5XX: 0x04 = 100 mA)
    uint8_t led_val = (LED_PULSE_FREQ << 5) | (LED_DUTY_CYCLE << 3) | LED_PEAK_CURRENT;
    writeReg(REG_PS_LED, led_val);

    // Number of IR pulses per measurement
    writeReg(REG_PS_N_PULSES, CFG_PS_N_PULSES & 0x0F);

    // PS measurement rate (bits[3:0]; 0x02 = 100 ms)
    writeReg(REG_PS_MEAS_RATE, CFG_PS_MEAS_RATE & 0x0F);

    // -- Configure ALS --
    // ALS_CONTR: keep gain bits, leave mode standby until activated below
    writeReg(REG_ALS_CONTR, (CFG_ALS_GAIN << 2));
    // ALS_MEAS_RATE: bits[5:3] = integration time, bits[2:0] = measurement rate
    writeReg(REG_ALS_MEAS_RATE, CFG_ALS_MEAS_RATE);

    // -- Activate both sensors --
    // PS_CONTR: bit 1 = active, bit 4 = high gain (16X)
    writeReg(REG_PS_CONTR, PS_MODE_BIT | PS_GAIN_HIGH);
    // ALS_CONTR: set bit 0 while preserving gain bits
    writeReg(REG_ALS_CONTR, ALS_MODE | (CFG_ALS_GAIN << 2));

    // Wait for first measurement to complete (100 ms rate + margin)
    vTaskDelay(pdMS_TO_TICKS(120));

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
    ch0 = (uint16_t)readReg(REG_ALS_DATA_CH0_0)
        | ((uint16_t)readReg(REG_ALS_DATA_CH0_1) << 8);
    ch1 = (uint16_t)readReg(REG_ALS_DATA_CH1_0)
        | ((uint16_t)readReg(REG_ALS_DATA_CH1_1) << 8);
    return true;
}

bool LTR553::readPS(uint16_t &ps_data) {
    // Optional: check status register for valid PS data before reading
    // uint8_t status = readReg(REG_ALS_PS_STATUS);
    // if (!(status & STATUS_PS_VALID)) return false;

    uint8_t lo = readReg(REG_PS_DATA_LOW)  & PS_DATA_LOW_MASK;
    uint8_t hi = readReg(REG_PS_DATA_HIGH) & PS_DATA_HIGH_MASK;
    ps_data = ((uint16_t)hi << 8) | lo;
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
