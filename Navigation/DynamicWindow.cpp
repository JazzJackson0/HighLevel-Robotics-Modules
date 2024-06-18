#include "DynamicWindow.hpp"

// NOT FINISHED!!!!!!!!!!!

// Private---------------------------------------------------------------------------------------------------------------------------------
float DynamicWindowApproach::heading(float trans_vel, float rot_vel) {


}

float DynamicWindowApproach::distance(float trans_vel, float rot_vel) {


}

float DynamicWindowApproach::velocity(float trans_vel, float rot_vel) {


}

float DynamicWindowApproach::ObjectiveFunction(float trans_vel, float rot_vel) {

    return smoothing * (heading_weight * heading(trans_vel, rot_vel) + 
        dist_weight * distance(trans_vel, rot_vel) + vel_weight * velocity(trans_vel, rot_vel));
}

std::vector<Velocities> DynamicWindowApproach::Generate_CircularTrajectories() {

    std::vector<Velocities> circular_trajectories;
    for (int i = 0; i < 1; i++) {


    }

    return circular_trajectories;
}

std::vector<Velocities> DynamicWindowApproach::Choose_AdmissableVelocities(std::vector<Velocities> vels) {

     std::vector<Velocities> admissable_velocities;
     for (int i = 0; i < vels.size(); i++) {
        
        float trans_accel = vels[i].trans_vel; // FINISH
        float rot_accel = vels[i].rot_vel; // FINISH
        float trans_vel_limit = std::sqrt(2 * distance(vels[i].trans_vel, vels[i].rot_vel) * trans_accel);
        float rot_vel_limit = std::sqrt(2 * distance(vels[i].trans_vel, vels[i].rot_vel) * rot_accel);
        
        if (vels[i].trans_vel <= trans_vel_limit && vels[i].rot_vel <= rot_vel_limit) {
            // FINISH
        }
        
    }

    return admissable_velocities;
}

std::vector<Velocities> DynamicWindowApproach::Apply_DynamicWindow(std::vector<Velocities> vels) {

    std::vector<Velocities> within_window;
    for (int i = 0; i < vels.size(); i++) {


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
}



// Public---------------------------------------------------------------------------------------------------------------------------------

DynamicWindowApproach::DynamicWindowApproach() {}

DynamicWindowApproach::DynamicWindowApproach(float smoothing_val, float heading_w, float dist_w, float vel_w) : 
    smoothing(smoothing_val), heading_weight(heading_w), dist_weight(dist_w), vel_weight(vel_w) {


}

VectorXf DynamicWindowApproach::Run() {

    return Optimize(SearchSpace());
}