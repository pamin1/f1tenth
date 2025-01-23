/**
 * @brief PID Controller class, header only
 * @author Prachit Amin
 */

class PID {
public:
    PID(){};
    void calculateError(double setpoint, double processVar);
    double P(double kP);
    // double I();
    // double D();
private:
    double error;
};

void PID::calculateError(double setpoint, double processVar) {
    error = setpoint - processVar;
}

double PID::P(double kP) {
    return kP * error;
}
