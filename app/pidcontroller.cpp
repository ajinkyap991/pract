/**
 * @file pidcontroller.cpp
 * @author Ajinkya Parwekar
 * @author Karan Sutradhar
 * @brief The pidcontroller.cpp file for Ackerman PID controller program.
 * It contains Ackerman PID controller class methods definitions.
 * @Copyright "Copyright 2020" <Ajinkya Parwekar>
 * @Copyright "Copyright 2020" <Karan Sutradhar>
 */


#include "pidcontroller.hpp"

  /**
   * @brief Base Constructor for the PID Controller class.
   * @param None.
   * @return None.
   */

  pidController::pidController() {
    kp = 1;
    kd = 0;
    ki = 0;
    dt = 0.1;
    kb = 0;
    errorSum = 0;
    previousError = 0;
    integralError = 0;
    prevTime = std::chrono::system_clock::now();
    dtMode = false;
    setpoint = 0;
    firstRunFlag = true;
    baseline = 1;
    carLen = 1;
    arcRadius = 0;
    rWheelVel = 0;
    lWheelVel = 0;
    steeringAngle = 0;
    setpointSpeed = 2;
    setpointHeading = 0;
    dtSim = 0.05;
    posX = 0;
    posY = 0;
    updatedHeading = 0;
    leftWheelSpeed = 0;
    rightWheelSpeed = 0;
  }

  /**
   * @brief Constructor for PID controller class with three gain parameters and time variable.
   * @param kpValue Proportional Gain of PID controller.
   * @param kdValue Differential Gain of PID controller.
   * @param kiValue Integral Gain of PID controller.
   * @param dtValue time variable for controller.
   * @return None.
   */

  pidController::pidController(double kpValue, double kdValue,
    double kiValue, double dtValue, bool dtModeIn) {
    kp = kpValue;
    kd = kdValue;
    ki = kiValue;
    dt = dtValue;
    kb = 0;
    errorSum = 0;
    previousError = 0;
    integralError = 0;
    prevTime = std::chrono::system_clock::now();
    dtMode = dtModeIn;
    setpoint = 0;
    firstRunFlag = true;
    baseline = 1;
    carLen = 1;
    arcRadius = 0;
    rWheelVel = 0;
    lWheelVel = 0;
    steeringAngle = 0;
    setpointSpeed = 2;
    setpointHeading = 0;
    dtSim = 0.01;
    posX = 0;
    posY = 0;
    updatedHeading = 0;
    leftWheelSpeed = 0;
    rightWheelSpeed = 0;
  }

  /**
   * @brief Constructor for PID controller class with all private attributes.
   * @param kpValue Proportional Gain of PID controller.
   * @param kdValue Differential Gain of PID controller.
   * @param kiValue Integral Gain of PID controller.
   * @param kbValue Variable for back calculation to eliminate windup.
   * @param dtValue time variable for controller.
   * @param errorvalue error value for proportional gain.
   * @param previousErrorValue error value for differential gain.
   * @param integralErrorValue error value for integral gain.
   * @return None.
   */

  pidController::pidController(double kpValue, double kdValue, double kiValue,
    double kbValue, double dtValue, double errorValue,
    double previousErrorValue, double integralErrorValue) {
    kp = kpValue;
    kd = kdValue;
    ki = kiValue;
    kb = kbValue;
    dt = dtValue;
    errorSum = errorValue;
    previousError = previousErrorValue;
    integralError = integralErrorValue;
    prevTime = std::chrono::system_clock::now();
    dtMode = false;
    setpoint = 0;
    firstRunFlag = true;
    baseline = 1;
    carLen = 1;
    arcRadius = 0;
    rWheelVel = 0;
    lWheelVel = 0;
    steeringAngle = 0;
    setpointSpeed = 2;
    setpointHeading = 0;
    dtSim = 0.01;
    posX = 0;
    posY = 0;
    updatedHeading = 0;
    leftWheelSpeed = 0;
    rightWheelSpeed = 0;
  }

  /**
   * @brief Function to compute the output of the PID controller.
   * The output of the function is based on the three error values corresponding to three controller gains.
   * @param feedback value i.e. the actual value of the parameter being corrected.
   * @return controlAction Output calculated by the PID controller using the gain values.
   */

  double pidController::computeControlAction(double feedback) {
    // stub implementation
    double currentError, output, futureError;
    currentError = feedback - setpoint;
    errorSum += currentError;
    futureError = currentError - previousError;

  if (firstRunFlag == false) {
    // Updating dtVal
    if (dtMode == false) {
      std::chrono::system_clock::time_point currTime =
          std::chrono::system_clock::now();
      std::chrono::duration<double> elapsed_seconds;
      elapsed_seconds = currTime - prevTime;
      dt = elapsed_seconds.count();  // Updating dt
    }
    // Calculating the pid output
    output = kp * currentError + ki * errorSum * dt
        + kd * futureError / dt;

  } else {
    output = kp * currentError;
    firstRunFlag = false;
  }
  previousError = currentError;
  prevTime = std::chrono::system_clock::now();
  std::cout<<"The output value is: " << output<<std::endl;
  return output;
  }

  /**
   * @brief Function to set the proportional gain variable of the PID controller
   * @param kp (Proportional gain)
   * @return None.
   */

  void pidController::setKp(double kpIn) {
    kp = kpIn;
  }

  /**
   * @brief Function to set the differential gain variable of the PID controller
   * @param kd (Differential gain)
   * @return None.
   */

  void pidController::setKd(double kdIn) {
    kd = kdIn;
  }

  /**
   * @brief Function to set the integral gain variable of the PID controller
   * @param ki (Integral gain)
   * @return None.
   */

  void pidController::setKi(double kiIn) {
    ki = kiIn;
  }

  /**
   * @brief Function to set the back calculation variable of the PID controller
   * @param kb (for back calculation)
   * @return None.
   */

  void pidController::setKb(double kbIn) {
    kb = kbIn;
  }

  /**
   * @brief Function to set the setpoint value of the PID controller
   * @param setpoint
   * @return None.
   */

  void pidController::setSp(double spIn) {
    setpoint = spIn;
  }

  /**
   * @brief Function to set the time variable of the PID controller
   * @param dt (time variable)
   * @return None.
   */

  void pidController::setDt(double dtIn) {
    dt = dtIn;
  }

  /**
   * @brief Function to set the error value of the PID controller
   * @param error (for proportional gain error))
   * @return None.
   */

  void pidController::setError(double errorIn) {
    errorSum = errorIn;
  }

  /**
   * @brief Function to set the previous error of the PID controller
   * @param previousError (for differential gain error)
   * @return None.
   */

  void pidController::setPreviousError(double previousErrorIn) {
    previousError = previousErrorIn;
  }

  /**
   * @brief Function to set the integral error of the PID controller
   * @param integralError (for integral gain error)
   * @return None.
   */

  void pidController::setIntegralError(double integralErrorIn) {
    integralError = integralErrorIn;
  }

  /**
   * @brief Function to get the proportional gain variable of the PID controller
   * @param None
   * @return kp (Proportional gain)
   */

  double pidController::getKp() {
      return kp;
  }

  /**
   * @brief Function to get the differential gain variable of the PID controller
   * @param None
   * @return kd (Differential gain)
   */

  double pidController::getKd() {
      return kd;
  }

  /**
   * @brief Function to get the integral gain variable of the PID controller
   * @param None
   * @return ki (Integral gain)
   */

  double pidController::getKi() {
      return ki;
  }

  /**
   * @brief Function to get the back calculation variable of the PID controller
   * @param None
   * @return kb (for back calculation)
   */

  double pidController::getKb() {
    return kb;
  }

  /**
   * @brief Function to get the setpoint value of the PID controller
   * @param None
   * @return setpoint value
   */

  double pidController::getSp() {
    return setpoint;
  }

  /**
   * @brief Function to get the time variable value of the PID controller
   * @param None
   * @return dt (time variable)
   */

  double pidController::getDt() {
      return dt;
  }

  /**
   * @brief Function to get the proportional error value of the PID controller
   * @param None
   * @return error (for proportional gain error)
   */

  double pidController::getError() {
    return errorSum;
  }

  /**
   * @brief Function to get the previous error value of the PID controller
   * @param None
   * @return previousError (for differential gain error)
   */

  double pidController::getPreviousError() {
    return previousError;
  }

  /**
   * @brief Function to get the integral error value of the PID controller
   * @param None
   * @return integralError (for integral gain error)
   */

  double pidController::getIntegralError() {
    return integralError;
  }

  /**
   * @brief Function to compute the arc radius of the wheel from rotation point.
   * @param None.
   * @return arc radius.
   */

  void pidController::computeArcRadius() {
      // stub implementation
  	  arcRadius = carLen * (tan ((3.14/2) - steeringAngle));
  }

  /**
   * @brief Function to compute the wheel velocities of both wheels.
   * @param None.
   * @return wheel Speed.
   */

  void pidController::computeWheelSpeed() {
      // stub implementation
  	  leftWheelSpeed = setpointSpeed * ( 1 - (baseline / 2 * arcRadius));
  	  rightWheelSpeed = setpointSpeed * ( 1 + (baseline / 2 * arcRadius));
  }

  /**
   * @brief Function to compute the steering angle for the wheel from rotation point.
   * @param steering angle, Speed of right wheel, Speed of left wheel and heading output.
   * @return steering angle.
   */

  void pidController::computePIDParameters(double *steeringAngle, double *headingOutput,
    double *rightWheelSpeed, double *leftWheelSpeed) {
      // stub implementation
  	  *steeringAngle = pidController::steeringAngle;
  	  pidController::steeringAngle = computeControlAction(*headingOutput);
  	  computeArcRadius();
  	  computeWheelSpeed();
  	  std::cout<< "The speed of the right wheel is: " << *rightWheelSpeed << std::endl;
  	  std::cout<< "The speed of the left wheel is: " << *leftWheelSpeed << std::endl;
      // return steeringAngle + rightWheelSpeed
      // + leftWheelSpeed + headingOutput;
  }

  /**
   * @brief Function to generate the throttle output value.
   * @param None.
   * @return throttle output value.
   */

  double pidController::throttleOutput() {
    // stub implementation
    return 0;
  }

  /**
   * @brief Function to set the setpoint values.
   * @param setpoint Speed and setpoint heading.
   * @return None.
   */

  void pidController::setSetPoints(double setpointHeadingIn,
    double setpointSpeedIn) {
  	  setpointHeading = setpointHeadingIn;
  	  pidController::setSp(setpointHeadingIn);
  	  setpointSpeed = setpointSpeedIn;
  }

  void pidController::reset(){
  	errorSum = 0;
  	previousError = 0;
  	firstRunFlag = 0;
  }

  void pidController::compute(double *steeringAngle, double *rightWheelSpeed, double *leftWheelSpeed, double *posX,
  	double *posY, double *updateHeading, double carLen) {
  // vehicle center velocity
  double vehVel = (*rightWheelSpeed + *leftWheelSpeed) / 2 * carLen;
  if (*steeringAngle > (3.14 / 4))
    *steeringAngle = 3.14 / 4;
  else if (*steeringAngle < (-3.14 / 4))
    *steeringAngle = -3.14 / 4;

  if (*steeringAngle != 0) {
    // calculating turning radius
    double turnRad = vehVel * tan((3.14 / 2) - *steeringAngle);
    double angVel = vehVel / turnRad;  // calculating angular velocity
    *updateHeading = *updateHeading + angVel * dtSim;  // updating heading
  }
  *posX += -vehVel * dtSim * sin(*updateHeading);
  *posY += vehVel * dtSim * cos(*updateHeading);
}

  /**
   * Destructor for PID controller
   * @param None.
   * @return None.
   */

  // ~pidController() {
  // }
