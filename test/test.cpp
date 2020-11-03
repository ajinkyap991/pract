/**
 * @file test.cpp
 * @author Karan Sutradhar: Driver
 * @author Ajinkya Parwekar: Navigator
 * @brief Test code functions for PID obj using gtest
 * @Copyright "Copyright 2020" <Ajinkya Parwekar>
 * @Copyright "Copyright 2020" <Karan Sutradhar>
 */

#include <gtest/gtest.h>
#include "pidcontroller.hpp"

/**
 * @brief This test checks if the control law works as expected
 * @param objTest is the name of the group of tests
 * @param controlFunctionTest1 is the specific name to check the control function
 */

TEST(objTest, controlFunctionTest1) {
    pidController obj(2.7, 4.5, 6.3, 1.0, false);
    EXPECT_EQ(5.4, obj.computeControlAction(2));
}

/**
 * @brief This test checks if the obj has properly stored its parameters
 * @param objTest is the name of the group of tests
 * @param paramGetTest is the specific name to check the getControlParam function
 */

TEST(objTest, paramGetTest) {
    pidController obj(2.7, 4.5, 6.3, 1.0, true);
    EXPECT_EQ(2.7, obj.getKp());
    EXPECT_EQ(4.5, obj.getKd());
    EXPECT_EQ(6.3, obj.getKi());
    EXPECT_EQ(0.0, obj.getKb());
    EXPECT_EQ(1.0, obj.getDt());
    EXPECT_EQ(0.0, obj.getError());
    EXPECT_EQ(0.0, obj.getPreviousError());
    EXPECT_EQ(0.0, obj.getIntegralError());
}

/**
* @brief This test checks if the obj can change the inherent parameters
* @param objTest is the name of the group of tests
* @param paramSetTest is the specific name to check the getControlParam function
*/

TEST(objTest, paramSetTest) {
    pidController obj(2.7, 4.5, 6.3, 1.0, 1.0, 1.0, 1.0, 1.0);

    obj.setKp(5.6);
    obj.setKd(2.3);
    obj.setKi(3.6);
    obj.setKb(4.5);
    obj.setDt(2.0);
    obj.setError(5.9);
    obj.setPreviousError(6.2);
    obj.setIntegralError(5.3);
    obj.setSp(2.2);
    obj.setSetPoints(2.2, 5.0);
    EXPECT_EQ(5.6, obj.getKp());
    EXPECT_EQ(2.3, obj.getKd());
    EXPECT_EQ(3.6, obj.getKi());
    EXPECT_EQ(0.0, obj.getKb());
    EXPECT_EQ(2.0, obj.getDt());
    EXPECT_EQ(5.9, obj.getError());
    EXPECT_EQ(6.2, obj.getPreviousError());
    EXPECT_EQ(5.3, obj.getIntegralError());
    EXPECT_EQ(2.2, obj.getSp());
}

/**
* @brief This test covers the various void functions of the program
* @param objTest is the name of the group of tests
* @param unitTest1 is the specific name to check the void functions
*/

TEST(objTest, unitTest1) {
    pidController obj;
    obj.reset();
    obj.computeArcRadius();
    obj.computeWheelSpeed();
    EXPECT_EQ(0.8, obj.throttleOutput(0.8));
    EXPECT_EQ(0.5, obj.throttleOutput(10));
}

/**
* @brief This test covers the constraints function of the program
* @param objTest is the name of the group of tests
* @param unitTest2 is the specific name to check the constraint function
*/

TEST(objTest, unitTest2) {
    pidController obj;
    EXPECT_EQ(2.0, obj.constraints(2.2, 1.0, 2.0));
    EXPECT_EQ(1.0, obj.constraints(0.5, 1.0, 2.0));
    EXPECT_EQ(1.5, obj.constraints(1.5, 1.0, 2.0));
}

