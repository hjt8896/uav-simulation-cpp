cmake_minimum_required(VERSION 4.1)
project(aircraft_sim)

set(CMAKE_CXX_STANDARD 17)

add_executable(aircraft_sim
#        main.cpp
        RigidBody.cpp
        RigidBody.h
        BaseEffector.h
        MotorEffector.h
        MotorEffector.cpp
        CommonTypes.h
        Mixer.cpp
        Mixer.h
        SMCController.cpp
        SMCController.h
        MRPSteering.cpp
        MRPSteering.h
        WindEffector.cpp
        WindEffector.h
        BMI088_Sensor.cpp
        BMI088_Sensor.h
        AttitudeUKF.cpp
        AttitudeUKF.h
        NotchFilter.h
        ukf_test.cpp)
