#ifndef SERVO_MOTOR_H
#define SERVO_MOTOR_H

#include "pwm.hpp"
#include <stdexcept>

class Motor {
private:
    PWM pwm;                // PWM object to control the servo
    double min_pulse_width; // Minimum pulse width in seconds (e.g., 0.0005 for 0.5ms)
    double max_pulse_width; // Maximum pulse width in seconds (e.g., 0.0025 for 2.5ms)
    int frequency;          // PWM frequency in Hz
    double current_angle;   // Current angle of the servo in degrees
    bool is_active;         // Flag to check if the servo is active

    /**
     * @brief Converts an angle to the corresponding duty cycle.
     * @param angle Angle in degrees (0 to 180).
     * @return Duty cycle value between 0 and 1.
     */
    double angle_to_duty_cycle(double angle) const {
        double pulse_width = min_pulse_width + (max_pulse_width - min_pulse_width) * (angle / 180.0);
        return pulse_width * frequency;
    }

public:
    /**
     * @brief Constructs a Motor object. double angle_to_duty_cycle(double angle) const {
        double pulse_width = min_pulse_width + (max_pulse_width - min_pulse_width) * (angle / 180.0);
        return pulse_width * frequency;
    }
     * @param chip PWM chip number.
     * @param channel PWM channel number.
     * @param min_pw Minimum pulse width in seconds (default: 0.0005s or 0.5ms).
     * @param max_pw Maximum pulse width in seconds (default: 0.0025s or 2.5ms).
     */
    Motor(int chip, int channel, double min_pw = 0.0005, double max_pw = 0.0025)
        : pwm(chip, channel), min_pulse_width(min_pw), max_pulse_width(max_pw), frequency(50), current_angle(90.0), is_active(false) {
        pwm.set_frequency(frequency);           // Set frequency
        pwm.set_polarity("normal");             // Set polarity to normal
        // Do not set duty cycle or enable PWM here to keep the servo relaxed
    }

    /**
     * @brief Sets the servo to the specified angle.
     * @param angle Desired angle in degrees (clamped to 0-180).
     */
    void set_angle(double angle) {
        if (angle < 0.0) angle = 0.0;
        else if (angle > 180.0) angle = 180.0;
        double duty_cycle = angle_to_duty_cycle(angle);
        pwm.set_duty_cycle(duty_cycle);
        current_angle = angle;
        if (!is_active) {
            pwm.enable();
            is_active = true;
        }
    }

    /**
     * @brief Gets the current angle of the servo.
     * @return Current angle in degrees.
     */
    double get_angle() const {
        return current_angle;
    }

    /**
     * @brief Activates the servo to hold its current position.
     * If the servo has not been set to a specific angle, it will be set to 90 degrees.
     */
    void activate() {
        if (!is_active) {
            // Set to current angle or default to 90 degrees if not set
            double duty_cycle = angle_to_duty_cycle(current_angle);
            pwm.set_duty_cycle(duty_cycle);
            pwm.enable();
            is_active = true;
        }
    }

    /**
     * @brief Deactivates the servo, allowing it to relax.
     */
    void deactivate() {
        if (is_active) {
            pwm.disable();
            is_active = false;
        }
    }
};

#endif // SERVO_MOTOR_H