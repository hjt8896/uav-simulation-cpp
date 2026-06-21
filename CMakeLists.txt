cmake_minimum_required(VERSION 4.1)
project(aircraft_sim)
# 找到系统中的 GLFW3 库
find_package(glfw3 REQUIRED)
include_directories(${CMAKE_SOURCE_DIR}/mujoco-3.9.0/include)
# 2. 告诉链接器去哪里找动态库
link_directories(${CMAKE_SOURCE_DIR}/mujoco-3.9.0/lib)

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
        MathUtils.cpp
        MathUtils.h
        mujoco_sim.cpp
        VelocitySMC.cpp
        VelocitySMC.h
        AltitudeSMC.cpp
        AltitudeSMC.h
        LowPassFilter.h
        USQUE.cpp
        USQUE.h
        #                ukf_test.cpp
        )

target_link_libraries(
        aircraft_sim
        glfw
        GL
        mujoco
)