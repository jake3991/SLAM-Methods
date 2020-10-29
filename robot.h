//
// Created by jake on 10/13/20.
//
#pragma once
#include "matplotlibcpp.h"

using namespace std;
namespace plt = matplotlibcpp;

class robot{

public:

    /**
     * Class constructor
     * @param x_ : initial x
     * @param y_ : initial y
     * @param theta_ : initial theta
     * @param sizeX_  : size of the world in x-axis
     * @param sizeY_  : size of the world in the y-axis
     * @param numLandmarks_ : number of point landmarks
     */
    robot(double x_, double y_, double theta_, int sizeX_, int sizeY_, int numLandmarks_)
                            : x(x_), y(y_), theta(theta_), sizeX(sizeX_), sizeY(sizeY_), numLandmarks(numLandmarks_){

        xLandmark = vector< double> (); //allocate the arrays for the landmarks
        yLandmark = vector< double> ();

        xHistory = vector<double> (); //allocate the arrays for the pose history
        yHistory = vector<double> ();
        thetaHistory = vector<double> ();

        for(int i = 0; i < numLandmarks; i++){ //generate some random landmarks inside the limits
            xLandmark.push_back((rand() % sizeX) - (sizeX / 2.));
            yLandmark.push_back((rand() % sizeY ) - (sizeY / 2.));
        }

        cout << xLandmark[0] << ":" << yLandmark[0] << endl;
    }

    /**
     * Move the robot
     * @param linearVelocity : velocity along the tangent line
     * @param rotationalVelocity : omega, rotational vel
     * @param deltaT : timestep size
     */
    void move(double linearVelocity, double rotationalVelocity, double deltaT){

        double thetaRadians = theta * (M_PI / 180.); //get theta in radians for sin and cos
        x = x + linearVelocity * deltaT * sin(thetaRadians); // update x
        y = y + linearVelocity * deltaT * cos(thetaRadians); //update y
        theta = theta + rotationalVelocity * deltaT; // update theta

        xHistory.push_back(x); //log the pose
        yHistory.push_back(y);
        thetaHistory.push_back(theta);
    }

    /**
     * Sense the landmarks
     * @param ranges : range meas vector
     * @param bearings : bearing meas vector
     */
    void sense(vector<double> &ranges, vector<double> &bearings){

        for(int i = 0; i < numLandmarks; i++){ //loop over all the landmarks

            double xLoc = xLandmark.at(i) - x; // do some trig to figure out range and bearing
            double yLoc = yLandmark.at(i) - y;
            double r = sqrt(xLoc * xLoc + yLoc * yLoc);
            double b = (atan2 (yLoc,xLoc) * 180 / M_PI) + theta;

            ranges.at(i) = r;  //change the meas vector
            bearings.at(i) = b;
        }
    }

    /**
     * Use matplotlib to plot the history of the robot and the landmark locations
     */
    void plot(){
        plt::scatter(xLandmark,yLandmark,50, {{"marker", "x"}, {"color", "green"}});
        plt::plot(xHistory,yHistory);
        plt::scatter(xHistory,yHistory,20, {{"marker", "o"}, {"color", "black"}});
        plt::xlim(- sizeX / 2, sizeX / 2);
        plt::ylim(- sizeY / 2, sizeY / 2);
        plt::show();

    }

private:

    double x, y, theta; //position varibles and the landmark count
    int sizeX, sizeY;
    int numLandmarks;

    vector<double> xLandmark; //define containers for the landmark coords
    vector<double> yLandmark;

    vector<double> xHistory;  //vectors to track the pose history of the robot
    vector<double> yHistory;
    vector<double> thetaHistory;

};
