/**
 * @file main.cpp
 * @author Ajinkya Parwekar: Driver
 * @author Karan Sutradhar: Navigator
 * @brief this program runs the unit tests defined in test.cpp
 * @Copyright "Copyright 2020" <Ajinkya Parwekar>
 * @Copyright "Copyright 2020" <Karan Sutradhar>
 */

#include <gtest/gtest.h>

/**
 * @brief main function of the test suite program.
 * @param argc.
 * @param argv.
 * @return RUN_ALL_TESTS()- Calling a function to run all test suites.
 */

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
