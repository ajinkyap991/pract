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
  double kp = -12, ki = 0, kd = 0, carLen = 1;

  // Heading and velocity to be achieved. Heading should be between -3.14 and +3.14
  double headingSp = 2, velSp = 1;

  // Initializing simulation
  double nIterations = 40;
  double simulationTime = 0.05;
  // ackerman_sim simObj(simulationTime);

  // Initializing ackermann controller
  
  pidController obj(kp, kd, ki, simulationTime, true);

  double heading = 0, posX = 0, posY = 0;
  double *headingptr = &heading;
  double *posXptr = &posX;
  double *posYptr = &posY;

  obj.setSetPoints(headingSp, velSp);
  std::cout << "The heading setpoint is : " << headingSp << std::endl;
  double lSpeed = velSp / 2, rSpeed = velSp / 2, steer = 0;
  double *lSpeedptr = &lSpeed;
  double *rSpeedptr = &rSpeed;
  double *steerptr = &steer;

  // Running simulation
  for (int z = 1; z < nIterations; z++) {
    std::cout << "Simulation Iteration Number: \t" << z << std::endl;
    obj.compute(steerptr, lSpeedptr, rSpeedptr, posXptr, posYptr, headingptr,
                   carLen);
    std::cout << "Current Heading: " << *headingptr
              << ", Current X Position: " << *posXptr
              << ", Current Y Position: " << *posYptr << std::endl;
    obj.computePIDParameters(steerptr, headingptr, rSpeedptr, lSpeedptr);
    std::cout << "Steering angle PID output: " << *steerptr << std::endl
              << std::endl;
  }
  return 0;
}