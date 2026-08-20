#pragma once

#include <cstdint>
#include <driver/i2c_master.h>

class LTR553 {
public:
    LTR553(i2c_master_bus_handle_t bus, uint8_t addr = 0x23);
    ~LTR553();

    bool begin();
    bool readALS(uint16_t &ch0, uint16_t &ch1);
    bool readPS(uint16_t &ps_data);
    float calcLux(uint16_t ch0, uint16_t ch1);

private:
    uint8_t readReg(uint8_t reg);
    bool writeReg(uint8_t reg, uint8_t value);

    i2c_master_dev_handle_t m_dev = nullptr;
    i2c_master_bus_handle_t m_bus;
    uint8_t m_addr;
};
