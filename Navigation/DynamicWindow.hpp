#pragma once
#include <iostream>
#include <cmath>
#include <limits>
#include <vector>
#include </usr/include/eigen3/Eigen/Dense>
using namespace Eigen;

struct Velocities {
    float trans_vel;
    float rot_vel;
    Velocities() {}
    Velocities(float t_vel, float r_vel) : trans_vel(t_vel), rot_vel(r_vel) {}
};

class DynamicWindowApproach {

    private:

        VectorXf Goal;
        VectorXf RobotPos;
        std::vector<VectorXf> PointCloud;
        float dist_nearing_goal;
        float smoothing;
        float heading_weight;
        float dist_weight;
        float vel_weight;
        float time_interval;
        Velocities PreviousVel;
        float MinVel;
        float MaxVel;
        float VelInterval;

        VectorXf Get_ClosestObstacle();

        float heading(float trans_vel, float rot_vel);

        float distance(float trans_vel, float rot_vel);

        float velocity(float trans_vel, float rot_vel);

        float ObjectiveFunction(float trans_vel, float rot_vel);

        std::vector<Velocities> Generate_CircularTrajectories();

        std::vector<Velocities> Choose_AdmissableVelocities(std::vector<Velocities> vels);

        std::vector<Velocities> Apply_DynamicWindow(std::vector<Velocities> vels);

        std::vector<Velocities> SearchSpace();

        VectorXf Optimize(std::vector<Velocities> vels);


    public:

        DynamicWindowApproach();

        DynamicWindowApproach(float smoothing_val, float heading_w, float dist_w, float vel_w);

        void Set_VelocityLimits(float min_vel, float max_vel, float vel_interval);

        void Set_Goal(VectorXf goal);

        VectorXf Run(VectorXf robot_pos, std::vector<VectorXf> point_cloud);
};