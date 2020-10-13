//
// Created by jake on 10/13/20.
//
#pragma once
#include "matplotlibcpp.h"

class robot{

public:
    
    //class constructor, feed the initial position
    robot(double x_, double y_, double theta_, double sizeX_, double sizeY_, int numLandmarks_)
                            : x(x_), y(y_), theta(theta_), sizeX(sizeX_), numLandmarks(numLandmarks_){}

private:

    //position varibles
    double x, y, theta, sizeX, sizeY;
    int numLandmarks

    /**
     * Set the landmarks throughout the envirnment
     */
    void setLandmarks(){

    }

};
