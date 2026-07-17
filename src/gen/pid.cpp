#include <cmath>

#include "gen/pid.hpp"

namespace gen {

float AsymptoticGains::at(float setpoint) const {
    const float magnitude = std::pow(std::fabs(setpoint), power);
    const float scaleMagnitude = std::pow(std::fabs(scale), power);
    const float denominator = magnitude + scaleMagnitude;
    if (denominator == 0.0f) return initial;
    return (final - initial) * magnitude / denominator + initial;
}

PID::PID(PIDConfig config)
    : config(config), proportionalGain(config.proportional.at(0.0f)) {}

float PID::update(float nextError) {
    error = nextError;

    if (std::fabs(error) > config.integralRange) {
        integral = 0.0f;
    } else if (config.integralSignReset && std::signbit(error) != std::signbit(previousError)) {
        integral = 0.0f;
    } else {
        integral += error * 0.01f;
    }

    const float derivative = (error - previousError) * 100.0f;
    previousError = error;
    return proportionalGain * error + config.kI * integral + config.kD * derivative;
}

void PID::reset(float initialError) {
    error = initialError;
    integral = 0.0f;
    previousError = initialError;
}

void PID::setTarget(float setpoint) { proportionalGain = config.proportional.at(setpoint); }

float PID::getError() const { return error; }

float PID::getProportionalGain() const { return proportionalGain; }

} // namespace gen
