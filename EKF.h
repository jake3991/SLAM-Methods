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

        P = Eigen::MatrixXd(3, 3); //covariance matrix
        X = Eigen::VectorXd(3); //state vector
        R = Eigen::MatrixXd(3, 3); //process noise
        R(0,0) = .1; //populate the process noise matrix
        R(1, 1) = .1;
        R(2, 2) = .1;


        cout << P(1,1) << endl;
    }

    //kalman prediction step
    void predict(double velocity, double rotaionalVelocity){

        //update the state vector using the nonlinear transition function
        X(0) = X(0) + deltaT * velocity * sin(X(2));
        X(1) = X(1) + deltaT * velocity * cos(X(2));
        X(2) = X(2) + deltaT * rotaionalVelocity;

        //get the jacobian matrix of te transition function G_t
        Eigen::MatrixXd G_t = Eigen::MatrixXd::Identity(3, 3);
        G_t(0, 2) = deltaT * velocity * cos(X(2));
        G_t(1, 2) = -deltaT * velocity * sin( X(2));

        //propagate the covariance matrix
        P = G_t * P * G_t.transpose() + R;
    }

private:

    Eigen::MatrixXd P; //covariance matrix P
    Eigen::VectorXd X; // state vector
    Eigen::MatrixXd R; //process noise
    const double deltaT; //timestep size





};
