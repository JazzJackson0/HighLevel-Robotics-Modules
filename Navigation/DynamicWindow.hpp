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
};

class DynamicWindowApproach {

    private:

        float smoothing;
        float heading_weight;
        float dist_weight;
        float vel_weight;
        float time_interval = 0.1;

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

        VectorXf Run();
};