/**
 * @brief PID Controller class, header only
 * @author Prachit Amin
 */

#include <chrono>

class PID
{
public:
    PID(double millis) : dt(1 / millis) {};
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
};

void PID::calculateError(double setpoint, double processVar)
{
    error = -(setpoint - processVar);
}

double PID::P(double kP)
{
    return kP * error;
}

double PID::I(double kI)
{
    integral += error * dt;
    return kI * integral;
}

double PID::D(double kD)
{
    derivative = (error - prevError) / 2;
    return kD * derivative;
}