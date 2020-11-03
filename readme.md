# PID Controller for Ackermann Steering
[![Build Status](https://travis-ci.org/ajinkyap991/pract.svg?branch=master)](https://travis-ci.org/ajinkyap991/pract)
[![Coverage Status](https://coveralls.io/repos/github/ajinkyap991/pract/badge.svg?branch=master)](https://coveralls.io/github/ajinkyap991/pract?branch=master)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
---

## Overview and Description

We are implementing a PID based controller for an ackermann steering system working on a feedback loop with a maximum steering angle constraint. The inputs are robot compass heading and speed. The output are the steering angle and the throttle value demonstrating the convergence of the actual components to the set points. This project will implement a design of a PID controller working on a feedback mechanism which will compute the error between the set point variable and the actual variables. This project is implemented with C++ codes, we are considering a single class pidController. There are various inputs of proportional-integral–derivative components and we intend to find the throttle value and steering angle using the feedback loop of speed, compass heading and orientation.

Results and Performances to be included after the implementation.


## Authors

Midterm Phase I
 
Ajinkya Parwekar (Driver)
Karan Sutradhar (Navigator)


## Personal Description


Ajinkiya Parwekar:
A Research Assistant working to design and manufacture an automonous quadrotor with the AMAV (Autonomous Micro Aerial Vehicle) team. He is pursuing a Master's in Robotics from the University of Maryland and is expected to be graduated by Fall 2021. He has professional experience of one year as an Automation test Engineer from a firm in India. He is avid reader of books (especially Novels). Apart from studies he enjoys playing badminton and solving Rubik's cube blindfolded.  

Karan Sutradhar:
A Research Assistant working with the Department of Mechanical Engineering, his research involves building and developing Control Systems for thermal energy storage and their use in demand/response, energy trading. He is a part of the AMAV (Autonomous Micro Aerial Vehicle) team that builds the software for ARTag detection and Image segmentation in building of a automonous quadrotor. He is pursuing a Master's in Robotics from the University of Maryland and is expected to graduate by Fall 2021. He has professional experience of 3.4 years as an Industrial Robotics Engineer from two different firm in India. In his free time he enjoys playing traditional sports, swimming and he represented his country in the senior national football.



## Agile Iterative Process

[Agile Iterative Process Google Spreadsheet](https://docs.google.com/spreadsheets/d/1h2nwnI-me5BiQL5BhDgtjJHhiWp_mYoPdVkZe6hzyFI/edit?usp=sharing)

## Sprint Planning Notes
[Sprint Planning Notes Google Document](https://docs.google.com/document/d/1I3oo1O6Uo4DJCLdgcUL5v5L1ioNQ-vy0LTcZkPlc0Hs/edit?usp=sharing)

## License

MIT License Clause

## Motivation:

The recent developments in the field of robotics have gone from science fiction fantasy to world-bound reality. Robots are consistently substituting humans to do mundane, hazardous, repetitive tasks with higher precision. However, we understood with our experiences, research background that the applications of Robots are much broader than the aforementioned reasons. Yearn for constant learning and continuous advancement in the field of Autonomous Driving drove us to choose the Ackerman PID (proportional–integral–derivative) controller. Karan has been involved in the field of Automation and Control Systems in the research lab he is working for, Ajinkya has been involved with AMAV (autonomous Micro Aerial Vehicle) team in the feild for Design, Modeling and Control. The background and interests in Control systems in autonomous systems motivated us to opt for the project of the Ackerman steering control system.


## Abstract
```
Simple starter C++ project with:

- cmake
- googletest

```
## Steps to run the program
```
git clone --recursive https://github.com/dpiet/cpp-boilerplate
cd <path to repository>
mkdir build
cd build
cmake ..
make
Run tests: ./test/cpp-test
Run program: ./app/shell-app

```

## Cpplint check
```
cd  <path to repository>
cpplint $( find . -name \*.hpp -or -name \*.cpp | grep -vE -e "^./build/" -e "^./vendor/" -e "^./docs/" -e "^./results" )

```

Cppcheck check
```
cd <path to repository>
cppcheck --enable=all --std=c++11 -I include/ --suppress=missingIncludeSystem $( find . -name *.cpp | grep -vE -e "^./build/" -e "^./vendor/" )

```

## Doxygen File generation
```
sudo apt-get install doxygen
doxygen -g
Open Doxygen file and source file in "INPUT" prameter and add the include and app folder
Add "*.hpp *.cpp" in the "FILE_PATTERNS" parameter in the doxygen file
Run "doxygen ./Doxyfile" in te terminal
Open html folder
open index.html
```

## Building for code coverage
```
sudo apt-get install lcov
cmake -D COVERAGE=ON -D CMAKE_BUILD_TYPE=Debug ../
make
make code_coverage
```
This generates a index.html page in the build/coverage sub-directory that can be viewed locally in a web browser.





