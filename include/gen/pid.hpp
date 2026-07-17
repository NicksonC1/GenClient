#pragma once

namespace gen {

struct AsymptoticGains {
    float initial = 0.0f;
    float final = 0.0f;
    float scale = 1.0f;
    float power = 1.0f;

    float at(float setpoint) const;
};

struct PIDConfig {
    AsymptoticGains proportional{};
    float kI = 0.0f;
    float kD = 0.0f;
    float integralRange = 0.0f;
    bool integralSignReset = false;
};

class PID {
    public:
        explicit PID(PIDConfig config = {});

        float update(float error);
        void reset(float initialError = 0.0f);
        void setTarget(float setpoint);

        float getError() const;
        float getProportionalGain() const;

    private:
        PIDConfig config;
        float proportionalGain = 0.0f;
        float error = 0.0f;
        float integral = 0.0f;
        float previousError = 0.0f;
};

} // namespace gen
