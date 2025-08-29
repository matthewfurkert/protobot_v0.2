#ifndef I2C_COMMS_HPP
#define I2C_COMMS_HPP

#include "smbus.hpp"
#include <cstdint>
#include <vector>
#include <stdexcept>
#include <memory>
#include <cmath>

inline std::shared_ptr<SMBus> open_bus(int bus_number) {
    return std::make_shared<SMBus>(bus_number);
}

class Sensor {
    uint8_t address_;

    std::vector<uint8_t> i2c_read(SMBus& bus, uint8_t reg, uint8_t count) const {
        try {
            // Combine write and read into a single transaction to avoid repeated START conditions
            I2cMsg write_msg = I2cMsg::write(address_, {reg});
            I2cMsg read_msg = I2cMsg::read(address_, count);
            
            // Single i2cRdwr call with both write and read messages
            bus.i2cRdwr({write_msg, read_msg});
            
            auto data = read_msg.getData();
            if (data.size() != count) {
                throw std::runtime_error("Read size mismatch");
            }
            return data;
        } catch (const std::exception& e) {
            throw std::runtime_error("I2C read failed: " + std::string(e.what()));
        }
    }

public:
    explicit Sensor(uint8_t address) : address_(address) {}

    uint16_t read_raw_angle(SMBus& bus) const {
        auto data = i2c_read(bus, 0x08, 2);
        return (static_cast<uint16_t>(data[0]) << 8) | data[1];
    }

    uint16_t read_magnitude(SMBus& bus) const {
        auto data = i2c_read(bus, 0x1B, 2);
        return (static_cast<uint16_t>(data[0]) << 8) | data[1];
    }

    double get_angle_radians(SMBus& bus) const {
        return static_cast<double>(read_raw_angle(bus)) * 2.0 * M_PI / 4096.0;
    }

};

#endif // I2C_COMMS_HPP