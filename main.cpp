#include <iostream>
#include "matplotlibcpp.h"
#include "robot.h"
#include "EKF.h"


int main() {

    EKF filter = EKF(1.0, 3);
    robot bot = robot(0,0,0,20,20,3); // set up a new robot
    vector<double> ranges(3, 0); //set up meas vectors
    vector<double> bearings(3, 0);

    for(int i = 0; i < 1; i++){ //loop for n timesteps

        double vel = 1.;
        double rot = -5;

        bot.move(vel,rot,1); //move
        bot.sense(ranges, bearings); //sense

        filter.predict(vel, rot);
        //filter.landmarkUpdate(ranges, bearings);
    }
    //bot.plot(); // plot the trajectory and landmarks
    //filter.plot();
    return 0;
}
