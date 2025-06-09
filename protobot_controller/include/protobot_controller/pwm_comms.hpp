#ifndef SERVO_MOTOR_H
#define SERVO_MOTOR_H

#include "pwm.hpp"
#include <stdexcept>

class Motor {
private:
    PWM pwm;                // PWM object to control the motor
    int frequency;          // PWM frequency in Hz
    double current_speed;   // Current speed setting (0 to 1)
    bool is_active;         // Flag to check if the motor is active

public:
    /**
     * @brief Constructs a Motor object for a DC motor.
     * @param chip PWM chip number.
     * @param channel PWM channel number.
     * @param freq PWM frequency in Hz (default: 1000 Hz).
     */
    Motor(int chip, int channel, int freq = 1000)
        : pwm(chip, channel), frequency(freq), current_speed(0.0), is_active(false) {
        pwm.set_frequency(frequency);           // Set frequency
        pwm.set_polarity("normal");             // Set polarity to normal
        // PWM is not enabled here to keep the motor off initially
    }

    /**
     * @brief Sets the motor speed.
     * @param speed Desired speed (0.0 to 1.0).
     */
    void set_speed(double speed) {
        if (speed < 0.0) speed = 0.0;
        else if (speed > 1.0) speed = 1.0;
        pwm.set_duty_cycle(speed);
        current_speed = speed;
        if (!is_active) {
            pwm.enable();
            is_active = true;
        }
    }

    /**
     * @brief Gets the current speed setting of the motor.
     * @return Current speed (0.0 to 1.0).
     */
    double get_speed() const {
        return current_speed;
    }

    /**
     * @brief Activates the motor at full speed (100% duty cycle).
     */
    void activate() {
        set_speed(1.0);  // Sets duty cycle to 100% and enables PWM
    }

    /**
     * @brief Deactivates the motor, stopping it.
     */
    void deactivate() {
        if (is_active) {
            pwm.disable();
            is_active = false;
        }
    }
};

#endif // SERVO_MOTOR_H