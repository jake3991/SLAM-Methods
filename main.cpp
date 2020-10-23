#include <iostream>
#include "matplotlibcpp.h"
#include "robot.h"
#include "EKF.h"


int main() {

    EKF filter = EKF(.2);



    robot bot = robot(0,0,0,20,20,3); // set up a new robot
    vector<double> ranges(3, 0); //set up meas vectors
    vector<double> bearings(3, 0);

    for(int i = 0; i < 10; i++){ //loop for n timesteps

        bot.move(1,-5,1); //move
        bot.sense(ranges, bearings); //sense

        filter.predict(1, 0);
    }
    bot.plot(); // plot the trajectory and landmarks
    return 0;
}
