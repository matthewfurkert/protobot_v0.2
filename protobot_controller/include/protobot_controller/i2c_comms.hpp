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
public:
    // Constructor: Takes only the I2C address of the sensor
    Sensor(uint8_t address) : address(address) {}

    // Reads the raw angle (12-bit value) from the sensor
    uint16_t read_raw_angle(std::shared_ptr<SMBus> bus) {
        std::vector<uint8_t> data = bus->readI2cBlockData(address, 0x0C, 2);
        if (data.size() != 2) {
            throw std::runtime_error("Failed to read 2 bytes for angle");
        }
        return (static_cast<uint16_t>(data[0]) << 8) | data[1];
    }

    // Reads the magnitude from the sensor
    uint16_t read_magnitude(std::shared_ptr<SMBus> bus) {
        std::vector<uint8_t> data = bus->readI2cBlockData(address, 0x1B, 2);
        if (data.size() != 2) {
            throw std::runtime_error("Failed to read 2 bytes for magnitude");
        }
        return (static_cast<uint16_t>(data[0]) << 8) | data[1];
    }

    // Computes and returns the angle in degrees
    double get_angle_degrees(std::shared_ptr<SMBus> bus) {
        uint16_t raw_angle = read_raw_angle(bus);
        return static_cast<double>(raw_angle) * 360.0 / 4096.0;
    }

    // Computes and returns the angle in radians
    double get_angle_radians(std::shared_ptr<SMBus> bus) {
        uint16_t raw_angle = read_raw_angle(bus);
        return static_cast<double>(raw_angle) * 2.0 * M_PI / 4096.0;
    }

private:
    uint8_t address; // I2C address of the sensor
};

#endif // I2C_COMMS_HPP