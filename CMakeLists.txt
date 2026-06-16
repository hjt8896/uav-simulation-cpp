cmake_minimum_required(VERSION 4.1)
project(aircraft_sim)

set(CMAKE_CXX_STANDARD 17)

add_executable(aircraft_sim main.cpp
        RigidBody.cpp
        RigidBody.h
        BaseEffector.h
        MotorEffector.h
        MotorEffector.cpp
        CommonTypes.h)
