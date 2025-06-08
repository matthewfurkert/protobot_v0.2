#ifndef SMBUS_HPP
#define SMBUS_HPP

#include <cstdint>
#include <vector>
#include <string>
#include <stdexcept>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <cstring>
#include <iostream>

// Commands from uapi/linux/i2c-dev.h
#define I2C_SLAVE           0x0703  // Use this slave address
#define I2C_SLAVE_FORCE     0x0706  // Use this slave address, even if it is already in use by a driver!
#define I2C_FUNCS           0x0705  // Get the adapter functionality mask
#define I2C_RDWR            0x0707  // Combined R/W transfer (one STOP only)
#define I2C_SMBUS           0x0720  // SMBus transfer. Takes pointer to i2c_smbus_ioctl_data
#define I2C_PEC             0x0708  // != 0 to use PEC with SMBus

// SMBus transfer read or write markers from uapi/linux/i2c.h
#define I2C_SMBUS_WRITE     0
#define I2C_SMBUS_READ      1

// Size identifiers uapi/linux/i2c.h
#define I2C_SMBUS_QUICK             0
#define I2C_SMBUS_BYTE              1
#define I2C_SMBUS_BYTE_DATA         2
#define I2C_SMBUS_WORD_DATA         3
#define I2C_SMBUS_PROC_CALL         4
#define I2C_SMBUS_BLOCK_DATA        5
#define I2C_SMBUS_BLOCK_PROC_CALL   7
#define I2C_SMBUS_I2C_BLOCK_DATA    8
#define I2C_SMBUS_BLOCK_MAX         32

// I2C functionality flags
enum class I2cFunc : uint32_t {
    I2C = 0x00000001,
    ADDR_10BIT = 0x00000002,
    PROTOCOL_MANGLING = 0x00000004,
    SMBUS_PEC = 0x00000008,
    NOSTART = 0x00000010,
    SLAVE = 0x00000020,
    SMBUS_BLOCK_PROC_CALL = 0x00008000,
    SMBUS_QUICK = 0x00010000,
    SMBUS_READ_BYTE = 0x00020000,
    SMBUS_WRITE_BYTE = 0x00040000,
    SMBUS_READ_BYTE_DATA = 0x00080000,
    SMBUS_WRITE_BYTE_DATA = 0x00100000,
    SMBUS_READ_WORD_DATA = 0x00200000,
    SMBUS_WRITE_WORD_DATA = 0x00400000,
    SMBUS_PROC_CALL = 0x00800000,
    SMBUS_READ_BLOCK_DATA = 0x01000000,
    SMBUS_WRITE_BLOCK_DATA = 0x02000000,
    SMBUS_READ_I2C_BLOCK = 0x04000000,
    SMBUS_WRITE_I2C_BLOCK = 0x08000000,
    SMBUS_HOST_NOTIFY = 0x10000000,
    SMBUS_BYTE = 0x00060000,
    SMBUS_BYTE_DATA = 0x00180000,
    SMBUS_WORD_DATA = 0x00600000,
    SMBUS_BLOCK_DATA = 0x03000000,
    SMBUS_I2C_BLOCK = 0x0c000000,
    SMBUS_EMUL = 0x0eff0008
};

// Bitwise operators for I2cFunc enum
inline I2cFunc operator|(I2cFunc a, I2cFunc b) {
    return static_cast<I2cFunc>(static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
}

inline I2cFunc operator&(I2cFunc a, I2cFunc b) {
    return static_cast<I2cFunc>(static_cast<uint32_t>(a) & static_cast<uint32_t>(b));
}

inline bool operator!=(I2cFunc a, I2cFunc b) {
    return static_cast<uint32_t>(a) != static_cast<uint32_t>(b);
}

// Use the structures from Linux kernel headers
// union i2c_smbus_data and struct i2c_smbus_ioctl_data are already defined in linux/i2c.h and linux/i2c-dev.h

// I2C message structure for combined transactions
class I2cMsg {
private:
    std::vector<uint8_t> buffer;
    
public:
    uint16_t addr;
    uint16_t flags;
    uint16_t len;
    uint8_t* buf;
    
    I2cMsg() : addr(0), flags(0), len(0), buf(nullptr) {}
    
    ~I2cMsg() {
        // Buffer is managed by std::vector, no manual cleanup needed
    }
    
    // Create a read message
    static I2cMsg read(uint16_t address, uint16_t length) {
        I2cMsg msg;
        msg.addr = address;
        msg.flags = I2C_M_RD;
        msg.len = length;
        msg.buffer.resize(length);
        msg.buf = msg.buffer.data();
        return msg;
    }
    
    // Create a write message
    static I2cMsg write(uint16_t address, const std::vector<uint8_t>& data) {
        I2cMsg msg;
        msg.addr = address;
        msg.flags = 0;
        msg.len = data.size();
        msg.buffer = data;
        msg.buf = msg.buffer.data();
        return msg;
    }
    
    // Get data as vector
    std::vector<uint8_t> getData() const {
        return std::vector<uint8_t>(buf, buf + len);
    }
    
    // Get data as string (for debugging)
    std::string toString() const {
        std::string result;
        for (uint16_t i = 0; i < len; ++i) {
            if (buf[i] >= 32 && buf[i] <= 126) {  // Printable ASCII
                result += static_cast<char>(buf[i]);
            }
        }
        return result;
    }
};

// struct i2c_rdwr_ioctl_data is already defined in linux/i2c-dev.h

// Main SMBus class
class SMBus {
private:
    int fd;
    I2cFunc funcs;
    int address;
    bool force;
    bool force_last;
    int pec;
    
    void setAddress(int addr, bool force_addr = false) {
        bool use_force = force_addr || force;
        if (address != addr || force_last != use_force) {
            int cmd = use_force ? I2C_SLAVE_FORCE : I2C_SLAVE;
            if (ioctl(fd, cmd, addr) < 0) {
                throw std::runtime_error("Failed to set I2C slave address");
            }
            address = addr;
            force_last = use_force;
        }
    }
    
    I2cFunc getFuncs() {
        uint64_t f;
        if (ioctl(fd, I2C_FUNCS, &f) < 0) {
            throw std::runtime_error("Failed to get I2C functionality");
        }
        return static_cast<I2cFunc>(f);
    }
    
public:
    SMBus(int bus = -1, bool force_enable = false) 
        : fd(-1), funcs(static_cast<I2cFunc>(0)), address(-1), 
          force(force_enable), force_last(false), pec(0) {
        if (bus >= 0) {
            open(bus);
        }
    }
    
    SMBus(const std::string& device_path, bool force_enable = false)
        : fd(-1), funcs(static_cast<I2cFunc>(0)), address(-1),
          force(force_enable), force_last(false), pec(0) {
        open(device_path);
    }
    
    ~SMBus() {
        close();
    }
    
    // Disable copy constructor and assignment operator
    SMBus(const SMBus&) = delete;
    SMBus& operator=(const SMBus&) = delete;
    
    void open(int bus) {
        std::string filepath = "/dev/i2c-" + std::to_string(bus);
        open(filepath);
    }
    
    void open(const std::string& filepath) {
        if (fd >= 0) {
            close();
        }
        
        fd = ::open(filepath.c_str(), O_RDWR);
        if (fd < 0) {
            throw std::runtime_error("Failed to open I2C device: " + filepath);
        }
        
        try {
            funcs = getFuncs();
        } catch (...) {
            ::close(fd);
            fd = -1;
            throw;
        }
    }
    
    void close() {
        if (fd >= 0) {
            ::close(fd);
            fd = -1;
            pec = 0;
            address = -1;
            force_last = false;
        }
    }
    
    bool isOpen() const {
        return fd >= 0;
    }
    
    I2cFunc getFunctionality() const {
        return funcs;
    }
    
    void enablePec(bool enable = true) {
        if ((funcs & I2cFunc::SMBUS_PEC) == static_cast<I2cFunc>(0)) {
            throw std::runtime_error("SMBUS_PEC is not supported");
        }
        pec = enable ? 1 : 0;
        if (ioctl(fd, I2C_PEC, pec) < 0) {
            throw std::runtime_error("Failed to set PEC mode");
        }
    }
    
    bool getPec() const {
        return pec != 0;
    }
    
    void writeQuick(int i2c_addr, bool force_addr = false) {
        setAddress(i2c_addr, force_addr);
        
        union i2c_smbus_data data;
        struct i2c_smbus_ioctl_data msg = {I2C_SMBUS_WRITE, 0, I2C_SMBUS_QUICK, &data};
        
        if (ioctl(fd, I2C_SMBUS, &msg) < 0) {
            throw std::runtime_error("SMBus write_quick failed");
        }
    }
    
    uint8_t readByte(int i2c_addr, bool force_addr = false) {
        setAddress(i2c_addr, force_addr);
        
        union i2c_smbus_data data;
        struct i2c_smbus_ioctl_data msg = {I2C_SMBUS_READ, 0, I2C_SMBUS_BYTE, &data};
        
        if (ioctl(fd, I2C_SMBUS, &msg) < 0) {
            throw std::runtime_error("SMBus read_byte failed");
        }
        
        return data.byte;
    }
    
    void writeByte(int i2c_addr, uint8_t value, bool force_addr = false) {
        setAddress(i2c_addr, force_addr);
        
        union i2c_smbus_data data;
        struct i2c_smbus_ioctl_data msg = {I2C_SMBUS_WRITE, value, I2C_SMBUS_BYTE, &data};
        
        if (ioctl(fd, I2C_SMBUS, &msg) < 0) {
            throw std::runtime_error("SMBus write_byte failed");
        }
    }
    
    uint8_t readByteData(int i2c_addr, uint8_t reg, bool force_addr = false) {
        setAddress(i2c_addr, force_addr);
        
        union i2c_smbus_data data;
        struct i2c_smbus_ioctl_data msg = {I2C_SMBUS_READ, reg, I2C_SMBUS_BYTE_DATA, &data};
        
        if (ioctl(fd, I2C_SMBUS, &msg) < 0) {
            throw std::runtime_error("SMBus read_byte_data failed");
        }
        
        return data.byte;
    }
    
    void writeByteData(int i2c_addr, uint8_t reg, uint8_t value, bool force_addr = false) {
        setAddress(i2c_addr, force_addr);
        
        union i2c_smbus_data data;
        data.byte = value;
        struct i2c_smbus_ioctl_data msg = {I2C_SMBUS_WRITE, reg, I2C_SMBUS_BYTE_DATA, &data};
        
        if (ioctl(fd, I2C_SMBUS, &msg) < 0) {
            throw std::runtime_error("SMBus write_byte_data failed");
        }
    }
    
    uint16_t readWordData(int i2c_addr, uint8_t reg, bool force_addr = false) {
        setAddress(i2c_addr, force_addr);
        
        union i2c_smbus_data data;
        struct i2c_smbus_ioctl_data msg = {I2C_SMBUS_READ, reg, I2C_SMBUS_WORD_DATA, &data};
        
        if (ioctl(fd, I2C_SMBUS, &msg) < 0) {
            throw std::runtime_error("SMBus read_word_data failed");
        }
        
        return data.word;
    }
    
    void writeWordData(int i2c_addr, uint8_t reg, uint16_t value, bool force_addr = false) {
        setAddress(i2c_addr, force_addr);
        
        union i2c_smbus_data data;
        data.word = value;
        struct i2c_smbus_ioctl_data msg = {I2C_SMBUS_WRITE, reg, I2C_SMBUS_WORD_DATA, &data};
        
        if (ioctl(fd, I2C_SMBUS, &msg) < 0) {
            throw std::runtime_error("SMBus write_word_data failed");
        }
    }
    
    uint16_t processCall(int i2c_addr, uint8_t reg, uint16_t value, bool force_addr = false) {
        setAddress(i2c_addr, force_addr);
        
        union i2c_smbus_data data;
        data.word = value;
        struct i2c_smbus_ioctl_data msg = {I2C_SMBUS_WRITE, reg, I2C_SMBUS_PROC_CALL, &data};
        
        if (ioctl(fd, I2C_SMBUS, &msg) < 0) {
            throw std::runtime_error("SMBus process_call failed");
        }
        
        return data.word;
    }
    
    std::vector<uint8_t> readBlockData(int i2c_addr, uint8_t reg, bool force_addr = false) {
        setAddress(i2c_addr, force_addr);
        
        union i2c_smbus_data data;
        struct i2c_smbus_ioctl_data msg = {I2C_SMBUS_READ, reg, I2C_SMBUS_BLOCK_DATA, &data};
        
        if (ioctl(fd, I2C_SMBUS, &msg) < 0) {
            throw std::runtime_error("SMBus read_block_data failed");
        }
        
        uint8_t length = data.block[0];
        return std::vector<uint8_t>(data.block + 1, data.block + 1 + length);
    }
    
    void writeBlockData(int i2c_addr, uint8_t reg, const std::vector<uint8_t>& data_vec, bool force_addr = false) {
        if (data_vec.size() > I2C_SMBUS_BLOCK_MAX) {
            throw std::invalid_argument("Data length cannot exceed " + std::to_string(I2C_SMBUS_BLOCK_MAX) + " bytes");
        }
        
        setAddress(i2c_addr, force_addr);
        
        union i2c_smbus_data data;
        data.block[0] = data_vec.size();
        std::memcpy(data.block + 1, data_vec.data(), data_vec.size());
        
        struct i2c_smbus_ioctl_data msg = {I2C_SMBUS_WRITE, reg, I2C_SMBUS_BLOCK_DATA, &data};
        
        if (ioctl(fd, I2C_SMBUS, &msg) < 0) {
            throw std::runtime_error("SMBus write_block_data failed");
        }
    }
    
    std::vector<uint8_t> blockProcessCall(int i2c_addr, uint8_t reg, const std::vector<uint8_t>& data_vec, bool force_addr = false) {
        if (data_vec.size() > I2C_SMBUS_BLOCK_MAX) {
            throw std::invalid_argument("Data length cannot exceed " + std::to_string(I2C_SMBUS_BLOCK_MAX) + " bytes");
        }
        
        setAddress(i2c_addr, force_addr);
        
        union i2c_smbus_data data;
        data.block[0] = data_vec.size();
        std::memcpy(data.block + 1, data_vec.data(), data_vec.size());
        
        struct i2c_smbus_ioctl_data msg = {I2C_SMBUS_WRITE, reg, I2C_SMBUS_BLOCK_PROC_CALL, &data};
        
        if (ioctl(fd, I2C_SMBUS, &msg) < 0) {
            throw std::runtime_error("SMBus block_process_call failed");
        }
        
        uint8_t length = data.block[0];
        return std::vector<uint8_t>(data.block + 1, data.block + 1 + length);
    }
    
    std::vector<uint8_t> readI2cBlockData(int i2c_addr, uint8_t reg, uint8_t length, bool force_addr = false) {
        if (length > I2C_SMBUS_BLOCK_MAX) {
            throw std::invalid_argument("Desired block length over " + std::to_string(I2C_SMBUS_BLOCK_MAX) + " bytes");
        }
        
        setAddress(i2c_addr, force_addr);
        
        union i2c_smbus_data data;
        data.byte = length;
        struct i2c_smbus_ioctl_data msg = {I2C_SMBUS_READ, reg, I2C_SMBUS_I2C_BLOCK_DATA, &data};
        
        if (ioctl(fd, I2C_SMBUS, &msg) < 0) {
            throw std::runtime_error("SMBus read_i2c_block_data failed");
        }
        
        return std::vector<uint8_t>(data.block + 1, data.block + 1 + length);
    }
    
    void writeI2cBlockData(int i2c_addr, uint8_t reg, const std::vector<uint8_t>& data_vec, bool force_addr = false) {
        if (data_vec.size() > I2C_SMBUS_BLOCK_MAX) {
            throw std::invalid_argument("Data length cannot exceed " + std::to_string(I2C_SMBUS_BLOCK_MAX) + " bytes");
        }
        
        setAddress(i2c_addr, force_addr);
        
        union i2c_smbus_data data;
        data.block[0] = data_vec.size();
        std::memcpy(data.block + 1, data_vec.data(), data_vec.size());
        
        struct i2c_smbus_ioctl_data msg = {I2C_SMBUS_WRITE, reg, I2C_SMBUS_I2C_BLOCK_DATA, &data};
        
        if (ioctl(fd, I2C_SMBUS, &msg) < 0) {
            throw std::runtime_error("SMBus write_i2c_block_data failed");
        }
    }
    
    void i2cRdwr(const std::vector<I2cMsg>& msgs) {
        if (msgs.empty()) {
            throw std::invalid_argument("No messages provided");
        }
        
        // Convert I2cMsg objects to i2c_msg structs
        std::vector<struct i2c_msg> i2c_msgs;
        i2c_msgs.reserve(msgs.size());
        
        for (const auto& msg : msgs) {
            struct i2c_msg i2c_msg_struct;
            i2c_msg_struct.addr = msg.addr;
            i2c_msg_struct.flags = msg.flags;
            i2c_msg_struct.len = msg.len;
            i2c_msg_struct.buf = reinterpret_cast<__u8*>(msg.buf);
            i2c_msgs.push_back(i2c_msg_struct);
        }
        
        struct i2c_rdwr_ioctl_data ioctl_data;
        ioctl_data.msgs = i2c_msgs.data();
        ioctl_data.nmsgs = i2c_msgs.size();
        
        if (ioctl(fd, I2C_RDWR, &ioctl_data) < 0) {
            throw std::runtime_error("I2C combined transaction failed");
        }
    }
};

#endif // SMBUS_HPP