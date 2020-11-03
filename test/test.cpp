/**
 * @file test.cpp
 * @author Ajinkya Parwekar: Driver
 * @author Karan Sutradhar: Navigator
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
    pidController obj(1, 0, 0, 0, 0, 0, 0, 0);
    EXPECT_EQ(2, obj.computeControlAction(2));
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

    EXPECT_EQ(5.6, obj.getKp());
    EXPECT_EQ(2.3, obj.getKd());
    EXPECT_EQ(3.6, obj.getKi());
    EXPECT_EQ(4.5, obj.getKb());
    EXPECT_EQ(2.0, obj.getDt());
    EXPECT_EQ(5.9, obj.getError());
    EXPECT_EQ(6.2, obj.getPreviousError());
    EXPECT_EQ(5.3, obj.getIntegralError());
}

/**
 * @brief This test checks if the control law works as expected
 * @param objTest is the name of the group of tests
 * @param getVeriableTest3 is the specific name to check the computeArcRadius function
 */

// TEST(objTest, getVeriableTest3) {
//     pidController obj(1, 0, 0, 0, true);
//     EXPECT_EQ(0, obj.computeArcRadius());
// }

/**
 * @brief This test checks if the control law works as expected
 * @param objTest is the name of the group of tests
 * @param getVeriableTest4 is the specific name to check the get veriable function
 */

// TEST(objTest, getVeriableTest4) {
//     pidController obj(1, 0, 0, 0, true);
//     EXPECT_EQ(0, obj.computeWheelSpeed());
// }

/**
 * @brief This test checks if the control law works as expected
 * @param objTest is the name of the group of tests
 * @param getVeriableTest5 is the specific name to check the get veriable function
 */

// TEST(objTest, getVeriableTest5) {
//     pidController obj(1, 0, 0, 0, true);
//     EXPECT_EQ(0, obj.computePIDParameters(0, 0, 0, 0));
// }

/**
 * @brief This test checks if the control law works as expected
 * @param objTest is the name of the group of tests
 * @param getVeriableTest6 is the specific name to check the get veriable function
 */

TEST(objTest, getVeriableTest6) {
    pidController obj;
    EXPECT_EQ(0, obj.throttleOutput());
}