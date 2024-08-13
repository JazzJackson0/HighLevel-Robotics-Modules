#include "Odometry.hpp"


Odom::Odom() {}

Odom::Odom(float robot_trackwidth, float time_step) : trackwidth(robot_trackwidth), dt(time_step) {

    serial = new Serial();

    // I2C
    serial_bus1 = serial->I2CInit(0, 0x0);
    serial_bus2 = serial->I2CInit(1, 0x0);

    // UART
    // serial_bus1 = serial->UARTInit(0);
    // serial_bus2 = serial->UARTInit(1);
}


void Odom::Set_Trackwidth(float robot_trackwidth) {

    trackwidth = robot_trackwidth;
}

VectorXf Odom::Get_NewPosition() {

    VectorXf new_position(3);
    int8_t *left_encoder_buffer;
    int8_t *right_encoder_buffer;
    serial->I2CRead(serial_bus1, left_encoder_buffer, 2);
    serial->I2CRead(serial_bus2, right_encoder_buffer, 2);
    left_distance = (float)(*(float*)&left_encoder_buffer);
    right_distance = (float)(*(float*)&right_encoder_buffer);

    float phi = left_distance - right_distance / trackwidth;
    float r_center = trackwidth / 2;
    VectorXf P(2);
    P << previous_pose[0] - r_center * std::cos(previous_pose[2]), previous_pose[0] - r_center * std::sin(previous_pose[2]);
    new_position << P[0] + r_center * std::cos(phi + previous_pose[2]), P[1] + r_center * std::sin(phi + previous_pose[2]), phi + previous_pose[2];
    return new_position;
}

VectorXf Odom::Get_NewVelocities() {

    VectorXf new_velocity(2);
    int8_t *left_encoder_buffer;
    int8_t *right_encoder_buffer;
    serial->I2CRead(serial_bus1, left_encoder_buffer, 2);
    serial->I2CRead(serial_bus2, right_encoder_buffer, 2);
    left_distance = (float)(*(float*)&left_encoder_buffer);
    right_distance = (float)(*(float*)&right_encoder_buffer);

    float phi = left_distance - right_distance / trackwidth;
    float r_center = trackwidth / 2;
    new_velocity << r_center/dt, phi/dt;

    return new_velocity;
}














