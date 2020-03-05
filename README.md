# Capstone 2020 Canal Boat
University of Massachusetts, Lowell Spring 2020 Capstone Project

## About
This repository contains code for the UML Spring 2020 Capstone Project, an autonomous boat for cleaning up garbage in the historic Lowell power canals. The goal is to eventually have a vehicle which can pick up garbage in the canals with little-to-no operator intervention. 

The vehicle will be controlled by a single central process which will spawn child processes and threads to handle various tasks such as sensor management, motor control, and navigation. One of the key aspects of the program is that it is stable and self-recovering, so that an error can be handled without the operator's input. At the same time, it must follow standard safety procedures for autonomous machinery.

### Code Authors
- Timothy Volpe (timothy_volpe@student.uml.edu), Spring 2020

## Building
Originally, the code was supposed to be built on Windows and Linux, despite the fact that it wouldn't be able to control anything on a Windows system. This was scrapped, and as such there are still some reminants of this fact. However, in its current state it can only be built successfully on Linux. It has been compiled successfully and tested on Raspbian Buster. The C++14 standard is required.

In order to create the project files, CMake 3.3.0 is used. More recent versions may work. The required libraries are as follows:
- Boost 1.59.0, *only the headers are required!*

## Documentation
All of the code is documented using doxygen. The doxygen.cfg file is provided; in order to generate the docs, simply download doxygen and run it in the root directory with doxygen.cfg.

## Version History

### Version 0.1
First release. This version is very rudimentary and possibly unstable. It provides the ability to control the 60A motor controller via user input, provided it is connected to the Raspberry Pi's serial1 port.
- UART control of RoboClaw motor controllers
- User control of 60A motor controller via terminal commands
- Basic program architecture for further expansion
