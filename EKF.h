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
    EKF(double deltaT, int numLandmarks): deltaT(deltaT), numLandmarks(numLandmarks){
        P = Eigen::MatrixXd(3 + 2 * numLandmarks, 3 + 2 * numLandmarks); //covariance matrix
        X = Eigen::VectorXd(3 + 2 * numLandmarks); //state vector, x, y , theta and then x,y for each landmark

        R = Eigen::MatrixXd::Identity(3 + 2 * numLandmarks, 3 + 2 * numLandmarks); //process noise
        R *= .1;

        //Q = Eigen::MatrixXd::Identity(3 + 2 * numLandmarks, 3 + 2 * numLandmarks); //meas noise
        //Q *= .1;

    }

    /**
     * Kalman prediction step
     * @param velocity : velocity command (straight line)
     * @param rotaionalVelocity : rotation command, applied at the end of the straight line move
     */
    void predict(double velocity, double rotaionalVelocity){

        double thetaRadians = X(2) * (M_PI / 180.);
        //update the state vector using the nonlinear transition function
        X(0) = X(0) + deltaT * velocity * sin(thetaRadians);
        X(1) = X(1) + deltaT * velocity * cos(thetaRadians);
        X(2) = X(2) + deltaT * rotaionalVelocity;

        //get the jacobian matrix of the transition function G_t
        Eigen::MatrixXd G_t = Eigen::MatrixXd::Identity(3 + 2 * numLandmarks, 3 + 2 * numLandmarks);
        G_t(0, 2) = deltaT * velocity * cos(thetaRadians);
        G_t(1, 2) = -deltaT * velocity * sin(thetaRadians);

        //propagate the covariance matrix
        P = G_t * P * G_t.transpose() + R;

        xHist.push_back(X(0)); //log
        yHist.push_back(X(1));
    }

    /**
     * The kalman filter update step for the case when we see some landmarks
     * @param ranges : vector of ranges to landmarks
     * @param bearings : vector of bearings to landmarks
     * @param labels : vector of landmark IDs
     */
    void landmarkUpdate(const vector<double> &ranges, const vector<double> &bearings){

        for(int i = 0; i < ranges.size(); i++){

            double deltaX = ranges.at(i) * cos(deg2rad(bearings.at(i) + X(2))); //the differnce in x and y between the
            double deltaY = ranges.at(i) * sin(deg2rad(bearings.at(i) + X(2))); //robot and the landmark
            double q = deltaX * deltaX + deltaY * deltaY; // sum square difference, compute here to make things clean

            if(X(2 * i + 3) == 0){ //if we have never seen this landmark
                X(2* i + 3) = deltaX + X(0);
                X(2 * i + 4) = deltaY + X(1);

            }else { //perform a kalman update for the ith landmark

                Eigen::MatrixXd H = Eigen::MatrixXd(2, 3 + 2 *numLandmarks); //Calculate the jacobian matrix for this time step
                Eigen::MatrixXd H_state_vector = Eigen::MatrixXd::Zero(2,3); //get the jacobian of just the state vector
                H_state_vector.row(0) << -1 / sqrt(q) * deltaX, -1 / sqrt(q) *deltaX, 0; //push this into the larger jacobian
                H_state_vector.row(1) << deltaY, -deltaX, 1;
                H.block<2, 3>(0, 0) = H_state_vector;

                H(0, 2 * i + 3) = 1 / sqrt(q) * deltaX; //populate the jacobian matrix with the ith landmark
                H(0, 2 * i + 4) = 1 / sqrt(q) * deltaY;
                H(1, 2 * i + 3) = -deltaY;
                H(1, 2 * i + 4) = deltaX;

                Eigen::MatrixXd z_t = Eigen::VectorXd(2); //Meas vector
                z_t << deltaX + X(0), deltaY + X(1); //parse the meas vector

                Eigen::MatrixXd z_t_hat = Eigen::VectorXd(2); //Predicted meas values
                z_t_hat << X(2 * i + 3), X(2 * i + 4); //parse the predicted location of the landmark

                Eigen::MatrixXd Q_meas = Eigen::MatrixXd(2, 2); //Meas noise
                Q_meas.row(0) << .1, 0;
                Q_meas.row(1) << 0, .1;

                Eigen::MatrixXd F = 0. * Eigen::MatrixXd(3 + 2 * numLandmarks, 3 + 2 * numLandmarks);//instanciate F
                F(0, 0) = 1.;
                F(1, 1) = 1.;
                F(2, 2) = 1.;
                F(2 * i + 3, 2 * i + 3) = 1.;
                F(2 * i + 4, 2 * i + 4) = 1.;

                Eigen::MatrixXd H_t = H * F; //map the jacobian into the high dim state
                Eigen::MatrixXd H_t_transpose = H_t.transpose(); //compute this here, this way it's done once

                Eigen::MatrixXd K; // compute the kalman gain
                K = H_t * P * H_t_transpose + Q_meas;
                K = K.inverse();
                K = P * H_t_transpose * K;

                X = X + K * (z_t - z_t_hat); //update the state vector using kalman gain
            }
        }
    }

    /**
     * The kalman filter update step for the case when we get some odometry readings
     * @param velocity : straight line speed
     * @param rotationalVelocity : rotational speed
     */
    void odomUpdate(double velocity, double rotationalVelocity){

    }

    void plot(){
        plt::plot(xHist,yHist);
        plt::scatter(xHist,yHist,20, {{"marker", "o"}, {"color", "black"}});
        plt::xlim(- 10, 10);
        plt::ylim(-10 , 10);
        plt::show();
    }

    static double deg2rad(double val){
        return val * (M_PI / 180.);
    }

private:

    Eigen::MatrixXd P; //covariance matrix P
    Eigen::VectorXd X; // state vector
    Eigen::MatrixXd R; //process noise
    Eigen::MatrixXd Q; //meas noise
    const double deltaT; //timestep size
    const int numLandmarks; //number of landmarks in the system

    vector<double> xHist = vector< double> ();
    vector<double> yHist = vector< double> ();

};