#include "DynamicWindow.hpp"

// Private---------------------------------------------------------------------------------------------------------------------------------
float DynamicWindowApproach::heading(float trans_vel, float rot_vel) {

    float goal_angle = std::atan2(Goal[1], Goal[0]);
    float robot_angle = std::atan2(RobotPos[1], RobotPos[0]);
    float goal_robot_angle = goal_angle - robot_angle;

    return M_PI - goal_robot_angle;
}

float DynamicWindowApproach::distance(float trans_vel, float rot_vel) {

    float circle_trajectory_radius = trans_vel / rot_vel;
    float robot_angle = std::atan2(RobotPos[1], RobotPos[0]);
    float gamma = std::atan2(ClosestObstacle[1], ClosestObstacle[0]) - robot_angle;

    return std::abs(circle_trajectory_radius * gamma);
}

float DynamicWindowApproach::velocity(float trans_vel) {

    
    float dist = std::hypot((RobotPos[0] - Goal[0]), (RobotPos[1] - Goal[1]));
    float v_norm = std::abs(trans_vel);

    // Higher cost for faster vels
    if (dist > dist_nearing_goal)
        return v_norm / MaxTransVel;

    // Higher cost for slower vels
    return 1 - (v_norm / MaxTransVel);
}

float DynamicWindowApproach::ObjectiveFunction(float trans_vel, float rot_vel) {

    return smoothing * (heading_weight * heading(trans_vel, rot_vel) + 
        dist_weight * distance(trans_vel, rot_vel) + vel_weight * velocity(trans_vel));
}

std::vector<Velocities> DynamicWindowApproach::Generate_CircularTrajectories() {

    std::vector<Velocities> circular_trajectories;
    for (float i = MinTransVel; i < MaxTransVel + 0.01; i += TransVelInterval) {

        for (float j = MinRotVel; j < MaxRotVel + 0.01; j += RotVelInterval) {
            Velocities v = Velocities(i, j);
            circular_trajectories.push_back(v);
        }
    }

    // std::cout << "# of Circular Trajectories: " << circular_trajectories.size() << "\n";
    return circular_trajectories;
}

std::vector<Velocities> DynamicWindowApproach::Choose_AdmissableVelocities(std::vector<Velocities> vels) {

    float time_interval_inv = 1.0 / time_interval;
    std::vector<Velocities> admissable_velocities;
    for (int i = 0; i < vels.size(); i++) {
        
        // Determine Accelerations
        float trans_accel = (vels[i].trans_vel - PreviousVel.trans_vel) * time_interval_inv; 
        float rot_accel = (vels[i].rot_vel - PreviousVel.rot_vel) * time_interval_inv; 

        // Get Distance to closest obstacle
        float collision_dist = distance(vels[i].trans_vel, vels[i].rot_vel);

        // Determine Admissable Upper Limits
        float trans_vel_limit = std::sqrt(2 * collision_dist * std::abs(trans_accel));
        float rot_vel_limit = std::sqrt(2 * collision_dist * std::abs(rot_accel));
        
        if (vels[i].trans_vel <= trans_vel_limit && vels[i].rot_vel <= rot_vel_limit)
            admissable_velocities.push_back(vels[i]);  
    }

    // std::cout << "# of Admissable Velocities: " << admissable_velocities.size() << "\n";
    return admissable_velocities;
}

std::vector<Velocities> DynamicWindowApproach::Apply_DynamicWindow(std::vector<Velocities> vels) {

    std::vector<Velocities> within_window;

    // Calculate Upper & Lower Acheivable Velocity Limits of Window
    float trans_lower = current_trans_vel - (std::abs(trans_accel_max) * time_interval);
    float trans_upper = current_trans_vel + (std::abs(trans_accel_max) * time_interval);
    float rot_lower = current_rot_vel - (std::abs(rot_accel_max) * time_interval);
    float rot_upper = current_rot_vel + (std::abs(rot_accel_max) * time_interval);

    for (int i = 0; i < vels.size(); i++) {

        if (vels[i].trans_vel >= (trans_lower) && vels[i].trans_vel <= (trans_upper) 
            && vels[i].rot_vel >= (rot_lower) && vels[i].rot_vel <= (rot_upper))
            within_window.push_back(vels[i]);
    }

    // std::cout << "# Within Dynamic Window: " << within_window.size() << "\n";
    // std::cout << std::endl;
    return within_window;
}

std::vector<Velocities> DynamicWindowApproach::SearchSpace() {

    return Apply_DynamicWindow(Choose_AdmissableVelocities(Generate_CircularTrajectories()));
}

VectorXf DynamicWindowApproach::Optimize(std::vector<Velocities> vels) {

    VectorXf best_vel(2);

    if (vels.size() == 0) {
        std::cerr << "ERROR: No permissible DWA velocities available" << std::endl;
        best_vel << 0, 0;
        return best_vel;
    }

    float lowest_cost = std::numeric_limits<float>::max();
    float lowest_cost_idx = -1;
    for (int i = 0; i < vels.size(); i++) {

        float cost = ObjectiveFunction(vels[i].trans_vel, vels[i].rot_vel);

        if (cost < lowest_cost) {

            lowest_cost = cost;
            lowest_cost_idx = i;
        }
    }

    best_vel << vels[lowest_cost_idx].trans_vel, vels[lowest_cost_idx].rot_vel;
    PreviousVel.trans_vel = best_vel[0];
    PreviousVel.rot_vel = best_vel[1];

    return best_vel;
}



// Public---------------------------------------------------------------------------------------------------------------------------------

DynamicWindowApproach::DynamicWindowApproach() {}

DynamicWindowApproach::DynamicWindowApproach(float smoothing_val, float heading_w, float dist_w, float vel_w) : 
    smoothing(smoothing_val), heading_weight(heading_w), dist_weight(dist_w), vel_weight(vel_w) {

    time_interval = 0.1;
    dist_nearing_goal = 0.05;
    PreviousVel = Velocities(0.f, 0.f);
}

void DynamicWindowApproach::Set_TranslationalVelocityLimits(float min_vel, float max_vel, float vel_interval) {

    MinTransVel = min_vel;
    MaxTransVel = max_vel;
    TransVelInterval = vel_interval;

    if (min_vel == 0) { 
        std::cerr << "WARNING: Min Velocity must be greater than 0. Setting min vel to 0.1" << std::endl; 
        MinTransVel = 0.1;
    }

    if (max_vel == 0) { 
        std::cerr << "WARNING: Max Velocity must be greater than 0. Setting min vel to 0.5" << std::endl; 
        MaxTransVel = 0.5;
    }
}


void DynamicWindowApproach::Set_RotationalVelocityLimits(float min_vel, float max_vel, float vel_interval) {

    MinRotVel = min_vel;
    MaxRotVel = max_vel;
    RotVelInterval = vel_interval;

    if (min_vel == 0) { 
        std::cerr << "WARNING: Min Velocity must be greater than 0. Setting min vel to 0.1" << std::endl; 
        MinRotVel = 0.1;
    }

    if (max_vel == 0) { 
        std::cerr << "WARNING: Max Velocity must be greater than 0. Setting min vel to 0.5" << std::endl; 
        MaxRotVel = 0.5;
    }
}

void DynamicWindowApproach::Set_MaxAccelerations(float _trans_accel_max, float _rot_accel_max) {

    trans_accel_max = _trans_accel_max;
    rot_accel_max = _rot_accel_max;
}

void DynamicWindowApproach::Set_Goal(VectorXf goal) {

    Goal = goal;
}

VectorXf DynamicWindowApproach::Run(VectorXf robot_pos, PointCloud point_cloud, VectorXf current_vels) {

    current_trans_vel = current_vels[0];
    current_rot_vel = current_vels[1];
    RobotPos = robot_pos;
    Cloud = point_cloud;

    // Determine Closest Obstacle
    // auto start = std::chrono::high_resolution_clock::now();
    float closest_dist = std::numeric_limits<float>::max();
    float closest_idx = 0;

    for (int i = 0; i < Cloud.points.size(); i++) {
        
        float dist = std::hypot((RobotPos[0] - Cloud.points[i][0]), (RobotPos[1] - Cloud.points[i][1]));
        if (dist < closest_dist)
            closest_idx = i;
    }
    VectorXf temp(2);
    temp << Cloud.points[closest_idx][0], Cloud.points[closest_idx][1];
    ClosestObstacle = temp;
    // auto end = std::chrono::high_resolution_clock::now();
    // std::cout << "O(N) Time: " << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() << " us" << std::endl;

    // Using KD Tree is much slower for some reason*********************************
    // start = std::chrono::high_resolution_clock::now();
    // ClosestObstacle = Cloud.kd_tree.NearestNeighbor(RobotPos).data; // Get Closest Obstacle
    // end = std::chrono::high_resolution_clock::now();
    // std::cout << "O(LogN) Time: " << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() << " us" << std::endl;

    return Optimize(SearchSpace());
}