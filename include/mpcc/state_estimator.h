/**
 * @brief Header only file for EKF implementation.
 * @author Prachit Amin
 */

#include <Eigen/Dense>
#include <iostream>
#include <cmath>

using namespace Eigen;

// EKF Class Definition
class EKF
{
private:
    // Define vehicle parameters
    double m;
    double I_z;
    double l_f;
    double l_r;
    double dt;

public:
    VectorXd x;         // state vector [X, Y, phi, v_x, v_y, omega]
    MatrixXd P;         // covariance matrix (uncertainty)
    MatrixXd Q;         // process noise
    MatrixXd R;         // measurement noise
    MatrixXd F;         // state transition matrix
    MatrixXd H;         // measurement model
    double C_f = 50000; // front cornering stiffness
    double C_r = 50000; // rear cornering stiffness

    EKF() : EKF(1.98, 0.14, 0.13, 0.13, 0.005) {};

    /**
     * @brief Initializes matrices needed to run an EKF on given vehicle parameters.
     * @param m     mass of the car (kg)
     * @param I_z   moment of inertia (kg*m^2)
     * @param l_f   distance to front axle (m)
     * @param l_r   distance to rear axle (m)
     * @param dt    time step (s)
     */
    EKF(double m, double I_z, double l_f, double l_r, double dt)
    {
        this->m = m;
        this->I_z = I_z;
        this->l_f = l_f;
        this->l_r = l_r;
        this->dt = dt;

        x = VectorXd(6);
        P = MatrixXd::Identity(6, 6);
        Q = MatrixXd::Identity(6, 6) * 0.5;
        R = MatrixXd::Identity(3, 3) * 0.5;
        H = MatrixXd::Zero(3, 6); // Initialize H with zeros (3 measurements, 6 state variables)

        // Map IMU measurements to state variables
        H(0, 3) = 1;  // Longitudinal acceleration (maps to v_x)
        H(1, 4) = 1;  // Lateral acceleration (maps to v_y)
        H(2, 5) = 1;  // Yaw rate (maps to omega)
    };

    /**
     * @brief Predicts the next state based on the steering and throttle inputs.
     * @param delta steering angle input
     * @param a throttle/braking input
     */
    void predict(double delta, double a)
    {
        // x = [X, Y, phi, v_x, v_y, omega]
        double phi = x(2);
        double v_x = x(3);
        double v_y = x(4);
        double omega = x(5);

        double F_fy = -C_f * (delta - atan((v_y + l_f * omega) / v_x));
        double F_ry = -C_r * atan((v_y - l_r * omega) / v_x);

        double v_x2 = pow(v_x, 2);
        double vy_omega_f = pow(v_y + l_f * omega, 2);
        double vy_omega_r = pow(v_y - l_r * omega, 2);

        // Nonlinear state update
        x(0) += (v_x * cos(phi) - v_y * sin(phi)) * dt;
        x(1) += (v_x * sin(phi) + v_y * cos(phi)) * dt;
        x(2) += omega * dt;
        x(3) += (a / m - F_fy * sin(delta) / m + v_y * omega) * dt;
        x(4) += (F_ry / m + F_fy * cos(delta) / m - v_x * omega) * dt;
        x(5) += (l_f * F_fy * cos(delta) - l_r * F_ry) / I_z * dt;

        // Jacobian (Linearization)
        MatrixXd F = MatrixXd::Zero(6, 6);
        F(0, 2) = (-v_x * sin(phi) - v_y * cos(phi)) * dt;
        F(0, 3) = cos(phi) * dt;
        F(0, 4) = -sin(phi) * dt;

        F(1, 2) = (v_x * cos(phi) - v_y * sin(phi)) * dt;
        F(1, 3) = sin(phi) * dt;
        F(1, 4) = cos(phi) * dt;

        F(2, 5) = 1;

        F(3, 3) = (C_f * sin(delta) / (m * (v_x2 + vy_omega_f))) * (v_y + l_f * omega);
        F(3, 4) = omega + (C_f * sin(delta) / (m * (v_x2 + vy_omega_f))) * v_x;
        F(3, 5) = v_y + (C_f * l_f * sin(delta) / (m * (v_x2 + vy_omega_f))) * v_x;

        F(4, 3) = -omega + (C_r / (m * (v_x2 + vy_omega_r))) * (v_y - l_r * omega) + (C_f * cos(delta) / (m * (v_x2 + vy_omega_f))) * (v_y + l_f * omega);
        F(4, 4) = (C_r * v_x / (m * (v_x2 + vy_omega_r))) + (C_f * cos(delta) / (m * (v_x2 + vy_omega_f))) * v_x;
        F(4, 5) = -v_x + (C_r * l_r / (m * (v_x2 + vy_omega_r))) * v_x + (C_f * l_f * cos(delta) / (m * (v_x2 + vy_omega_f))) * v_x;

        F(5, 3) = (l_f * C_f * cos(delta) / I_z) * (v_y + l_f * omega) / (v_x2 + vy_omega_f) - (l_r * C_r / I_z) * (v_y - l_r * omega) / (v_x2 + vy_omega_r);
        F(5, 4) = (l_f * C_f * cos(delta) / I_z) * v_x / (v_x2 + vy_omega_f) - (l_r * C_r / I_z) * v_x / (v_x2 + vy_omega_r);
        F(5, 5) = (l_f * l_f * C_f * cos(delta) / I_z) * v_x / (v_x2 + vy_omega_f) + (l_r * l_r * C_r / I_z) * v_x / (v_x2 + vy_omega_r);

        // Predict covariance
        P = F * P * F.transpose() + Q;
        return;
    };

    /**
     * @brief Updates the state and covariance estimates.
     * @param z measured value input. considering a 3 input IMU.
     */
    void update(const VectorXd &z)
    {
        if (z.size() != 3) {
            std::cout << "measured value was invalid\n";
        }
        VectorXd y = z - H * x; // Innovation
        MatrixXd S = H * P * H.transpose() + R;
        MatrixXd K = P * H.transpose() * S.inverse(); // Kalman gain

        x += K * y;
        P = (MatrixXd::Identity(6, 6) - K * H) * P;
    };
};