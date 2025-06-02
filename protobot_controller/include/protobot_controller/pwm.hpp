#ifndef PWM_H
#define PWM_H

#include <string>
#include <stdexcept>
#include <fstream>
#include <sstream>
#include <chrono>
#include <thread>
#include <unistd.h>
#include <sys/stat.h>
#include <string.h>

class PWMError : public std::runtime_error {
public:
    int errnum;
    PWMError(int errnum, const std::string& message)
        : std::runtime_error(message), errnum(errnum) {}
};

class PWM {
private:
    int _chip;
    int _channel;
    std::string _path;
    mutable int _period_ns;
    static const int PWM_STAT_RETRIES = 10;
    static constexpr double PWM_STAT_DELAY = 0.1; // in seconds

    void _open(int chip, int channel) {
        if (chip < 0) {
            throw std::invalid_argument("Invalid chip type, should be non-negative integer.");
        }
        if (channel < 0) {
            throw std::invalid_argument("Invalid channel type, should be non-negative integer.");
        }
        std::string chip_path = "/sys/class/pwm/pwmchip" + std::to_string(chip);
        std::string channel_path = chip_path + "/pwm" + std::to_string(channel);
        struct stat sb;
        if (stat(chip_path.c_str(), &sb) != 0 || !S_ISDIR(sb.st_mode)) {
            throw PWMError(0, "Opening PWM: PWM chip " + std::to_string(chip) + " not found.");
        }
        if (stat(channel_path.c_str(), &sb) != 0 || !S_ISDIR(sb.st_mode)) {
            // Export the PWM
            std::string export_path = chip_path + "/export";
            std::ofstream f_export(export_path);
            if (!f_export.is_open()) {
                throw PWMError(errno, "Exporting PWM channel: " + std::string(strerror(errno)));
            }
            f_export << channel << std::endl;
            if (f_export.fail()) {
                throw PWMError(errno, "Writing export: " + std::string(strerror(errno)));
            }
            f_export.close();
            // Wait for the channel directory to appear
            for (int i = 0; i < PWM_STAT_RETRIES; ++i) {
                if (stat(channel_path.c_str(), &sb) == 0 && S_ISDIR(sb.st_mode)) {
                    break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(PWM_STAT_DELAY * 1000)));
                if (i == PWM_STAT_RETRIES - 1) {
                    throw PWMError(0, "Exporting PWM: waiting for \"" + channel_path + "\" timed out");
                }
            }
            // Wait for the period file to be writable
            std::string period_path = channel_path + "/period";
            for (int i = 0; i < PWM_STAT_RETRIES; ++i) {
                if (access(period_path.c_str(), W_OK) == 0) {
                    break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(PWM_STAT_DELAY * 1000)));
                if (i == PWM_STAT_RETRIES - 1) {
                    throw PWMError(EACCES, "Opening PWM period: Period file not writable");
                }
            }
        }
        _chip = chip;
        _channel = channel;
        _path = channel_path;
        _period_ns = get_period_ns();
    }

    std::string _read_channel_attr(const std::string& attr) const {
        std::string file_path = _path + "/" + attr;
        std::ifstream f(file_path);
        if (!f.is_open()) {
            throw PWMError(errno, "Opening " + attr + ": " + std::string(strerror(errno)));
        }
        std::string value;
        std::getline(f, value);
        return value;
    }

    void _write_channel_attr(const std::string& attr, const std::string& value) {
        std::string file_path = _path + "/" + attr;
        std::ofstream f(file_path);
        if (!f.is_open()) {
            throw PWMError(errno, "Opening " + attr + ": " + std::string(strerror(errno)));
        }
        f << value << std::endl;
        if (f.fail()) {
            throw PWMError(errno, "Writing " + attr + ": " + std::string(strerror(errno)));
        }
    }

public:
    PWM(int chip, int channel) : _chip(-1), _channel(-1), _period_ns(0) {
        _open(chip, channel);
    }

    ~PWM() {
        close();
    }

    void close() {
        if (_channel != -1) {
            std::string unexport_path = "/sys/class/pwm/pwmchip" + std::to_string(_chip) + "/unexport";
            std::ofstream f_unexport(unexport_path);
            if (f_unexport.is_open()) {
                f_unexport << _channel << std::endl;
                f_unexport.close();
            }
            _chip = -1;
            _channel = -1;
        }
    }

    void enable() {
        set_enabled(true);
    }

    void disable() {
        set_enabled(false);
    }

    std::string get_devpath() const {
        return _path;
    }

    int get_chip() const {
        return _chip;
    }

    int get_channel() const {
        return _channel;
    }

    int get_period_ns() const {
        std::string period_str = _read_channel_attr("period");
        try {
            _period_ns = std::stoi(period_str);
            return _period_ns;
        } catch (const std::invalid_argument&) {
            throw PWMError(0, "Unknown period value: \"" + period_str + "\"");
        }
    }

    void set_period_ns(int period_ns) {
        if (period_ns < 0) {
            throw std::invalid_argument("Invalid period value, should be non-negative integer.");
        }
        _write_channel_attr("period", std::to_string(period_ns));
        _period_ns = period_ns;
    }

    int get_duty_cycle_ns() const {
        std::string duty_cycle_str = _read_channel_attr("duty_cycle");
        try {
            return std::stoi(duty_cycle_str);
        } catch (const std::invalid_argument&) {
            throw PWMError(0, "Unknown duty cycle value: \"" + duty_cycle_str + "\"");
        }
    }

    void set_duty_cycle_ns(int duty_cycle_ns) {
        if (duty_cycle_ns < 0) {
            throw std::invalid_argument("Invalid duty cycle value, should be non-negative integer.");
        }
        _write_channel_attr("duty_cycle", std::to_string(duty_cycle_ns));
    }

    double get_period() const {
        return static_cast<double>(get_period_ns()) / 1e9;
    }

    void set_period(double period) {
        if (period <= 0.0) {
            throw std::invalid_argument("Invalid period value, should be positive.");
        }
        set_period_ns(static_cast<int>(period * 1e9));
    }

    double get_duty_cycle() const {
        return static_cast<double>(get_duty_cycle_ns()) / _period_ns;
    }

    void set_duty_cycle(double duty_cycle) {
        if (duty_cycle < 0.0 || duty_cycle > 1.0) {
            throw std::invalid_argument("Invalid duty cycle value, should be between 0.0 and 1.0.");
        }
        int duty_cycle_ns = static_cast<int>(duty_cycle * _period_ns);
        set_duty_cycle_ns(duty_cycle_ns);
    }

    double get_frequency() const {
        return 1.0 / get_period();
    }

    void set_frequency(double frequency) {
        if (frequency <= 0.0) {
            throw std::invalid_argument("Invalid frequency value, should be positive.");
        }
        set_period(1.0 / frequency);
    }

    std::string get_polarity() const {
        return _read_channel_attr("polarity");
    }

    void set_polarity(const std::string& polarity) {
        std::string pol_lower = polarity;
        for (char& c : pol_lower) c = std::tolower(c);
        if (pol_lower != "normal" && pol_lower != "inversed") {
            throw std::invalid_argument("Invalid polarity, should be \"normal\" or \"inversed\".");
        }
        _write_channel_attr("polarity", pol_lower);
    }

    bool get_enabled() const {
        std::string enabled_str = _read_channel_attr("enable");
        if (enabled_str == "1") return true;
        if (enabled_str == "0") return false;
        throw PWMError(0, "Unknown enabled value: \"" + enabled_str + "\"");
    }

    void set_enabled(bool enabled) {
        _write_channel_attr("enable", enabled ? "1" : "0");
    }

    std::string to_string() const {
        std::ostringstream oss;
        oss << "PWM " << _channel << ", chip " << _chip
            << " (period=" << get_period() << " sec, duty_cycle=" << (get_duty_cycle() * 100)
            << "%, polarity=" << get_polarity() << ", enabled=" << (get_enabled() ? "true" : "false") << ")";
        return oss.str();
    }
};

#endif // PWM_H