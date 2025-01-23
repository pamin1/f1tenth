/**
 * @brief PID Controller class, header only
 * @author Prachit Amin
 */

#include <chrono>

class PID {
public:
    PID() : prevTime_(std::chrono::high_resolution_clock::now()) {};
    void calculateError(double setpoint, double processVar);
    double P(double kP);
    double I(double kI);
    double D(double kD);
private:
    double error;
    double prevError;
    double dt = 0;
    double integral = 0;
    double derivative = 0;
    std::chrono::time_point<std::chrono::high_resolution_clock> prevTime_;
};

void PID::calculateError(double setpoint, double processVar) {
    error = -(setpoint - processVar);
}

double PID::P(double kP) {
    return kP * error;
}

double PID::I(double kI) {
    auto currentTime_ = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> timeDiff_ = currentTime_ - prevTime_;
    dt = timeDiff_.count();

    integral += error * dt;
    prevTime_ = currentTime_;
    return kI * integral;
}

double PID::D(double kD) {
    derivative = (dt > 0) ? (error - prevError) / 2 : 0;
    return kD * derivative;
}