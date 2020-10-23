//
// Created by jake on 10/22/20.
//

#pragma once
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace std;

class EKF{

public:
    EKF(double deltaT): deltaT(deltaT) {

        P = Eigen::MatrixXd(5, 5); //covariance matrix
        X = Eigen::VectorXd(5); //state vector

        R = Eigen::MatrixXd::Identity(5, 5); //process noise
        R *= .1;

        Q = Eigen::MatrixXd::Identity(5, 5); //meas noise
        Q *= .1;

        numLandmarks = 0; //set the initial number of landmarks to zero

    }

    /**
     * Kalman prediction step
     * @param velocity : velocity command (straight line)
     * @param rotaionalVelocity : rotation command, applied at the end of the straight line move
     */
    void predict(double velocity, double rotaionalVelocity){

        //update the state vector using the nonlinear transition function
        X(0) = X(0) + deltaT * velocity * sin(X(2));
        X(1) = X(1) + deltaT * velocity * cos(X(2));
        X(2) = X(2) + deltaT * rotaionalVelocity;

        //get the jacobian matrix of the transition function G_t
        Eigen::MatrixXd G_t = Eigen::MatrixXd::Identity(5, 5);
        G_t(0, 2) = deltaT * velocity * cos(X(2));
        G_t(1, 2) = -deltaT * velocity * sin( X(2));

        //propagate the covariance matrix
        P = G_t * P * G_t.transpose() + R;
    }

    /**
     * The kalman filter update step for the case when we see some landmarks
     * @param ranges : vector of ranges to landmarks
     * @param bearings : vector of bearings to landmarks
     * @param labels : vector of landmark IDs
     */
    void landmarkUpdate(const vector<double> &ranges, const vector<double> &bearings){

        int i = 0; //dummy i

        double deltaX = ranges.at(i) * cos(bearings.at(i) + X(2)); //the differnce in x and y between the
        double deltaY = ranges.at(i) * sin(bearings.at(i) + X(2)); //robot and the landmark
        double q = deltaX * deltaX + deltaY * deltaY; // sum square difference, compute here to make things clean

        Eigen::MatrixXd H = Eigen::MatrixXd(2, 5); //Calculate the jacobian matrix for this time step
        H.row(0) << -1/sqrt(q) * deltaX, -1/sqrt(q) * deltaX, 0, 1/sqrt(q) * deltaX, 1/sqrt(q) * deltaY;
        H.row(1) << deltaY, -deltaX, 1, -deltaY, deltaX;

        Eigen::MatrixXd z_t = Eigen::VectorXd(2); //Meas vector
        z_t << deltaX + X(0), deltaY + X(1);

        Eigen::MatrixXd z_t_hat = Eigen::VectorXd(2); //Predicted meas values
        z_t_hat << X(3), X(4);

        Eigen::MatrixXd Q_meas = Eigen::MatrixXd(2, 2); //Meas noise
        Q_meas.row(0) << .1, 0;
        Q_meas.row(1) << 0, .1;

        Eigen::MatrixXd F = Eigen::MatrixXd(5, 5); //instanciate F
        F.row(0) << 1, 0, 0, 0, 0;
        F.row(1) << 0, 1, 0, 0, 0;
        F.row(2) << 0, 0, 1, 0, 0;
        F.row(3) << 0, 0, 0, 1, 0;
        F.row(4) << 0, 0, 0, 0, 1;

        Eigen::MatrixXd H_t = H * F; //map the jacobian into the high dim state
        Eigen::MatrixXd H_t_transpose = H_t.transpose();

        Eigen::MatrixXd K; // compute the kalman gain
        K = H_t * P * H_t.transpose() + Q_meas;
        K = K.inverse();
        K = P * H_t_transpose * K;

        X = K * (z_t - z_t_hat); //update the state vector
    }

    /**
     * The kalman filter update step for the case when we get some odometry readings
     * @param velocity : straight line speed
     * @param rotationalVelocity : rotational speed
     */
    void odomUpdate(double velocity, double rotationalVelocity){

    }

private:

    Eigen::MatrixXd P; //covariance matrix P
    Eigen::VectorXd X; // state vector
    Eigen::MatrixXd R; //process noise
    Eigen::MatrixXd Q; //meas noise
    const double deltaT; //timestep size
    int numLandmarks; //number of landmarks in the system

};
