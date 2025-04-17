#pragma once
#include <iostream>
#include <cmath>
#include <limits>
#include <vector>
#include <chrono>
#include <Eigen/Dense>
#include "utils.hpp"
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
        VectorXf ClosestObstacle;
        PointCloud Cloud;
        float dist_nearing_goal;
        float smoothing;
        float heading_weight;
        float dist_weight;
        float vel_weight;
        float time_interval;
        Velocities PreviousVel;
        float MinTransVel;
        float MaxTransVel;
        float TransVelInterval;
        float MinRotVel;
        float MaxRotVel;
        float RotVelInterval;
        float current_trans_vel;
        float current_rot_vel;
        float trans_accel_max;
        float rot_accel_max;

        

        /**
         * @brief A measure of how much you are moving towards the goal. 
         *       Measures the angle between the robot and the goal direction for a given trajectory. 
         * 
         * @param trans_vel 
         * @param rot_vel 
         * @return float 
         */
        float heading(float trans_vel, float rot_vel);

        /**
         * @brief The distance to the closest obstacle on the given trajectory. 
         *      The smaller the distance to an obstacle, the higher the robot’s desire to move around it.
         * 
         * @param trans_vel 
         * @param rot_vel 
         * @return float 
         */
        float distance(float trans_vel, float rot_vel);

        /**
         * @brief The forward (linear) velocity of the robot, so as to maximize speed.
         * 
         * @param trans_vel 
         * @return float 
         */
        float velocity(float trans_vel);

        /**
         * @brief A Linear Combination of 3 Functions. Calculates the cost of a given velocity pair
         * 
         * @param trans_vel 
         * @param rot_vel 
         * @return float 
         */
        float ObjectiveFunction(float trans_vel, float rot_vel);

        /**
         * @brief Generate set of possible velocity combinations ranging from min to max of each volocity type.
         * 
         * @return std::vector<Velocities> 
         */
        std::vector<Velocities> Generate_CircularTrajectories();

        /**
         * @brief A velocity is considered admissible if the robot can stop before it reaches an obstacle.
         * 
         * @param vels 
         * @return std::vector<Velocities> 
         */
        std::vector<Velocities> Choose_AdmissableVelocities(std::vector<Velocities> vels);

        /**
         * @brief Restricts the admissible velocities to those that can be reached within a short time interval 
         *          given the limited accelerations of the robot.
         * 
         * @param vels 
         * @return std::vector<Velocities> 
         */
        std::vector<Velocities> Apply_DynamicWindow(std::vector<Velocities> vels);

        /**
         * @brief Determine a set of permissible velocities
         * 
         * @return std::vector<Velocities> 
         */
        std::vector<Velocities> SearchSpace();

        /**
         * @brief 
         * 
         * @param vels 
         * @return VectorXf 
         */
        VectorXf Optimize(std::vector<Velocities> vels);


    public:

        /**
         * @brief Construct a new Dynamic Window Approach object
         * 
         */
        DynamicWindowApproach();

        /**
         * @brief Construct a new Dynamic Window Approach object
         * 
         * @param smoothing_val 
         * @param heading_w 
         * @param dist_w 
         * @param vel_w 
         */
        DynamicWindowApproach(float smoothing_val, float heading_w, float dist_w, float vel_w);

        /**
         * @brief 
         * 
         * @param min_vel 
         * @param max_vel 
         * @param vel_interval 
         */
        void Set_TranslationalVelocityLimits(float min_vel, float max_vel, float vel_interval);

        /**
         * @brief 
         * 
         * @param min_vel 
         * @param max_vel 
         * @param vel_interval 
         */
        void Set_RotationalVelocityLimits(float min_vel, float max_vel, float vel_interval);

        /**
         * @brief 
         * 
         * @param _trans_accel_max 
         * @param _rot_accel_max 
         */
        void Set_MaxAccelerations(float _trans_accel_max, float _rot_accel_max);

        /**
         * @brief 
         * 
         * @param goal 
         */
        void Set_Goal(VectorXf goal);

        /**
         * @brief 
         * 
         * @param robot_pos 
         * @param point_cloud 
         * @param current_vels
         * @return VectorXf 
         */
        VectorXf Run(VectorXf robot_pos, PointCloud point_cloud, VectorXf current_vels);
};