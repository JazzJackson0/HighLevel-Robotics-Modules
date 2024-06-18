#include "utils.hpp"

AngleAndAxis RotationMatrix_to_Angle(MatrixXf R) {
    
    if (R.cols() != 3)
        std::cerr << "Rotation Matrix dimensions should be 3x3. Cannot convert to angle." << std::endl;

    float angle = (float) acos((R.trace() - 1) / 2);
    VectorXf axis(3);

    // Singularity
    if (angle == 0.0) {

        // COMPLETE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    }

    // Singularity
    else if (angle == 180.0) {
        
        // Find diagonal term with highest value
        float largest = std::numeric_limits<float>::min();;
        int row = -1;

        for (int i = 0; i < R.cols(); i++) {

            if (R(i,i) > largest) {
                row = i;
                largest = R(i,i);
            }
        }

        // Calculate the axis vector
        axis = VectorXf::Zero(3);
        axis[0] = (float) sqrt((R(row, row) + 1) / 2);
        for (int j = 1; j < R.cols(); j++) {

            axis[j] = (float) R(row, row+j) / (2 * axis[0]);
        }
    }

    else {
        // Hardcoded for 3x3 matrices
        axis << (R(2,1) - R(1,2) / (2 * sin(angle))), (R(0,2) - R(2,0) / (2 * sin(angle))), (R(1,0) - R(0,1) / (2 * sin(angle)));
    }
    return std::make_pair(angle, axis);
}



MatrixXf Angle_to_RotationMatrix(AngleAndAxis a_a) {

    // Rodriguez Formula method
    // MatrixXf identity;
    // identity = MatrixXf::Identity(3, 3);

    // if (a_a.first == 0.0) {

    //     return identity;
    // }
    
    // // Convert axis angle to matrix form
    // MatrixXf A(4, 4);
    // A << 0, -a_a.second(2), a_a.second(2), a_a.second(2), 0, -a_a.second(0), -a_a.second(1), a_a.second(0), 0;

    // return identity - (sin(a_a.first) * A) + ((1 - cos(a_a.first)) * (A * A));

    MatrixXf R(2, 2);
    R << cos(a_a.first), -sin(a_a.first), sin(a_a.first), cos(a_a.first);
    return R;
}