/**
 * @file main.cpp
 * @author Ajinkya Parwekar
 * @author Karan Sutradhar
 * @brief The main.cpp file for Ackerman PID controller program.
 * It contains object creation and computation of control value.
 * @Copyright "Copyright 2020" <Ajinkya Parwekar>
 * @Copyright "Copyright 2020" <Karan Sutradhar>
 */

#include "pidcontroller.hpp"

/**
 * @brief main function of the program.
 * @param None.
 * @return 0.
 */

int main() {
    // Initialising Constructor (creating an object of the class)
    // stub implementation
  double kp = -10, ki = 0, kd = 0, carLen = 1;

  // Heading and velocity to be achieved. Heading should be -3.14 and +3.14
  double headingSp = 2, velSp = 1;

  // Initializing simulation
  double nIterations = 40, temp;
  double simulationTime = 0.05;
  // ackerman_sim simObj(simulationTime);

  // Initializing ackermann controller
  
  pidController obj(kp, kd, ki, simulationTime, true);

  double heading = 0, posX = 0, posY = 0;
  double *headingptr = &heading;
  double *posXptr = &posX;
  double *posYptr = &posY;

  obj.setSetPoints(headingSp, velSp);
  std::cout << "The heading setpoint is : " << headingSp << std::endl
            << "Enter a value and press enter...." << std::endl;
  std::cin >> temp;
  double lVel = velSp / 2, rVel = velSp / 2, steer = 0;
  double *lVelptr = &lVel;
  double *rVelptr = &rVel;
  double *steerptr = &steer;

  // Running simulation
  for (int k = 1; k < nIterations; k++) {
    std::cout << "Simulation Iteration No : \t " << k << std::endl;
    obj.compute(steerptr, lVelptr, rVelptr, posXptr, posYptr, headingptr,
                   carLen);
    std::cout << "Current Heading : " << *headingptr
              << ", Current X Position : " << *posXptr
              << ", Current Y Position : " << *posYptr << std::endl;
    obj.computePIDParameters(steerptr, headingptr, rVelptr, lVelptr);
    std::cout << "Steering angle PID output : " << *steerptr << std::endl
              << std::endl;
  }
  return 0;
}