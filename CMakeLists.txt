cmake_minimum_required(VERSION 4.1)
project(aircraft_sim)
# 找到系统中的 GLFW3 库
find_package(glfw3 REQUIRED)
include_directories(${CMAKE_SOURCE_DIR}/mujoco-3.9.0/include)
# 添加 OSQP 生成代码的头文件路径
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/osqp_workspace/inc/public)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/osqp_workspace/inc/private)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/osqp_workspace)
# 将 OSQP 生成的源码收集起来
file(GLOB OSQP_SRC "${CMAKE_SOURCE_DIR}/osqp_workspace/src/*.c" workspace.c workspace.h)
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
        Barometer_Sensor.cpp
        Barometer_Sensor.h
        GPS_Sensor.cpp
        GPS_Sensor.h
        #                ukf_test.cpp
        ${OSQP_SRC}
        OuterLoopMPC.cpp
        OuterLoopMPC.h
        osqp_workspace/workspace.c
        osqp_workspace/workspace.h
)

target_link_libraries(
        aircraft_sim
        glfw
        GL
        mujoco
)