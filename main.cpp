#include <iostream>
#include "matplotlibcpp.h"
#include "robot.h"
#include "EKF.h"


int main() {

    int number_of_landmarks = 10;

    EKF filter = EKF(1.0, number_of_landmarks);
    robot bot = robot(0,0,0,20,20,number_of_landmarks); // set up a new robot
    vector<double> ranges(number_of_landmarks, 0); //set up meas vectors
    vector<double> bearings(number_of_landmarks, 0);

    for(int i = 0; i < 10; i++){ //loop for n timesteps

        double vel = 1.;
        double rot = -5;

        bot.move(vel,rot,1); //move
        bot.sense(ranges, bearings); //sense

        filter.predict(vel, rot);
        filter.landmarkUpdate(ranges, bearings);
    }
    bot.plot(); // plot the trajectory and landmarks
    filter.plot();
    return 0;
}
