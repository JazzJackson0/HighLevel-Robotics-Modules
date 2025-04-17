#pragma once
#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include "../LinuxSerial/Serial.hpp"
using namespace Eigen;

#define I2C_NUM 1
#define I2C_SLAVE_ADDRS 0x55

class Odom {

    private:
        float left_distance;
        float right_distance;
        float trackwidth;
        float dt;
        int8_t serial_bus1;
        int8_t serial_bus2;
        VectorXf previous_pose;
        Serial *serial;

    public:

        Odom();

        Odom(float robot_trackwidth, float time_step);

        void Set_Trackwidth(float robot_trackwidth);

        VectorXf Get_NewPosition();

        VectorXf Get_NewVelocities();

        VectorXf Get_NewRawVelocities();
};








