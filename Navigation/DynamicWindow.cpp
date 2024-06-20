#include "DynamicWindow.hpp"

// NOT FINISHED!!!!!!!!!!!

// Private---------------------------------------------------------------------------------------------------------------------------------

VectorXf DynamicWindowApproach::Get_ClosestObstacle() {

    float closest_dist = std::numeric_limits<float>::max();;
    float closest_idx = 0;

    for (int i = 0; i < PointCloud.size(); i++) {

        float dist = std::sqrt(std::pow((RobotPos[0] - PointCloud[i][0]), 2) + std::pow((RobotPos[1] - PointCloud[i][1]), 2));
        if (dist < closest_dist) {
            closest_idx = i;
        }
    }

    return PointCloud[closest_idx];
}

float DynamicWindowApproach::heading(float trans_vel, float rot_vel) {

    float goal_angle = std::atan2(Goal[1], Goal[0]);
    float robot_angle = std::atan2(RobotPos[1], RobotPos[0]);
    float goal_robot_angle = goal_angle - robot_angle;

    return M_PI - goal_robot_angle;
}

float DynamicWindowApproach::distance(float trans_vel, float rot_vel) {

    float circle_trajectory_radius = trans_vel / rot_vel; // Maybe handle case of rot_vel being 0??
    float robot_angle = std::atan2(RobotPos[1], RobotPos[0]);
    VectorXf obstacle = Get_ClosestObstacle();
    float gamma = std::atan2(obstacle[1], obstacle[0]) - robot_angle;

    return circle_trajectory_radius * gamma;
}

float DynamicWindowApproach::velocity(float trans_vel, float rot_vel) {

    float dist = std::sqrt(std::pow((RobotPos[0] - Goal[0]), 2) + std::pow((RobotPos[1] - Goal[1]), 2));
    float v_norm = std::sqrt(std::pow(trans_vel, 2) + std::pow(rot_vel, 2)); // I have no idea if this is correct. Needs research

    if (dist > dist_nearing_goal) {
        (v_norm) / MaxVel;
    }

    else {
        return 1 - ((v_norm) / MaxVel);
    }
}

float DynamicWindowApproach::ObjectiveFunction(float trans_vel, float rot_vel) {

    return smoothing * (heading_weight * heading(trans_vel, rot_vel) + 
        dist_weight * distance(trans_vel, rot_vel) + vel_weight * velocity(trans_vel, rot_vel));

    // TODO: Normalize all 3 of these components to [0, 1]
}

std::vector<Velocities> DynamicWindowApproach::Generate_CircularTrajectories() {

    std::vector<Velocities> circular_trajectories;
    for (float i = MinVel; i < MaxVel + 0.1; i += VelInterval) {

        Velocities v = Velocities(i, i);
        circular_trajectories.push_back(v);
    }

    return circular_trajectories;
}

std::vector<Velocities> DynamicWindowApproach::Choose_AdmissableVelocities(std::vector<Velocities> vels) {

     std::vector<Velocities> admissable_velocities;
     for (int i = 0; i < vels.size(); i++) {
        
        float trans_accel = (vels[i].trans_vel - PreviousVel.trans_vel) / time_interval; 
        float rot_accel = (vels[i].rot_vel - PreviousVel.rot_vel) / time_interval; 
        float trans_vel_limit = std::sqrt(2 * distance(vels[i].trans_vel, vels[i].rot_vel) * trans_accel);
        float rot_vel_limit = std::sqrt(2 * distance(vels[i].trans_vel, vels[i].rot_vel) * rot_accel);
        
        if (vels[i].trans_vel <= trans_vel_limit && vels[i].rot_vel <= rot_vel_limit) {
            admissable_velocities.push_back(vels[i]);
        }
        
    }

    return admissable_velocities;
}

std::vector<Velocities> DynamicWindowApproach::Apply_DynamicWindow(std::vector<Velocities> vels) {

    std::vector<Velocities> within_window;
    for (int i = 0; i < vels.size(); i++) {
        float trans_accel = (vels[i].trans_vel - PreviousVel.trans_vel) / time_interval; 
        float rot_accel = (vels[i].rot_vel - PreviousVel.rot_vel) / time_interval; 

        float trans_lower = vels[i].trans_vel - (trans_accel * time_interval);
        float trans_upper = vels[i].trans_vel + (trans_accel * time_interval);
        float rot_lower = vels[i].rot_vel - (rot_accel * time_interval);
        float rot_upper = vels[i].rot_vel + (rot_accel * time_interval);

        if (vels[i].trans_vel >= (trans_lower) && vels[i].trans_vel <= (trans_upper) 
            && vels[i].rot_vel >= (rot_lower) && vels[i].rot_vel <= (rot_upper)) {

            within_window.push_back(vels[i]);
        }

    }

    return within_window;
}

std::vector<Velocities> DynamicWindowApproach::SearchSpace() {

    return Apply_DynamicWindow(Choose_AdmissableVelocities(Generate_CircularTrajectories()));
}

VectorXf DynamicWindowApproach::Optimize(std::vector<Velocities> vels) {

    float lowest_cost = std::numeric_limits<float>::max();
    float lowest_cost_idx = -1;
    for (int i = 0; i < vels.size(); i++) {

        float cost = ObjectiveFunction(vels[i].trans_vel, vels[i].rot_vel);

        if (cost < lowest_cost) {

            lowest_cost = cost;
            lowest_cost_idx = i;
        }
    }

    VectorXf best_vel(2);
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
    dist_nearing_goal = 1;
    PreviousVel = Velocities(0.f, 0.f);
}

void DynamicWindowApproach::Set_VelocityLimits(float min_vel, float max_vel, float vel_interval) {

    MinVel = min_vel;
    MaxVel = max_vel;
    VelInterval = vel_interval;
}

void DynamicWindowApproach::Set_Goal(VectorXf goal) {

    Goal = goal;
}

VectorXf DynamicWindowApproach::Run(VectorXf robot_pos, std::vector<VectorXf> point_cloud) {

    RobotPos = robot_pos;
    PointCloud = point_cloud;
    return Optimize(SearchSpace());
}