#include "../include/camera/VisualOdometry.hpp"
/**
 * NOT FINISHED!!
 * 
 */

// Private --------------------------------------------------------------------------------------------------------------------------------------
std::pair<cv::Mat, cv::Mat> MonocularOdometry::Decompose_Essential_Matrix(const cv::Mat& E, const cv::Mat &matched_points1, 
        const cv::Mat &matched_points2) {
    
    std::vector<int> z_sums;
    std::vector<double> relative_scales;

    // 1. Decompose Essential Matrix
    cv::Mat R1, R2, t;
    cv::decomposeEssentialMat(E, R1, R2, t);
    std::vector<std::pair<cv::Mat, cv::Mat>> decomposed = {{R1, t}, {R1, -t}, {R2, t}, {R2, -t}};

    // 2. Iterate through the 4 possible [R & t] solutions
    for (const auto& solution : decomposed) {
        cv::Mat R = solution.first;
        cv::Mat t = solution.second;

        cv::Mat points3D_1, points3D_2;
        z_sums.push_back(Get_SumOfZCoords(R, t, matched_points1, matched_points2, points3D_1, points3D_2));
        relative_scales.push_back(Caulculate_RelativeScale(points3D_1, points3D_2));
    }

    // 3. Find [R & t] solution with the largest z_sum
    int best_solution_idx = std::distance(z_sums.begin(), std::max_element(z_sums.begin(), z_sums.end()));
    auto best_solution = decomposed[best_solution_idx];
    double relative_scale = relative_scales[best_solution_idx];

    // 4. Select & Scale the correct [R & t] solution
    cv::Mat R_final = best_solution.first;
    cv::Mat t_final = best_solution.second * relative_scale;

    return {R_final, t_final};

}

int MonocularOdometry::Get_SumOfZCoords(const cv::Mat &R, const cv::Mat &t, 
        const cv::Mat &matched_points1, const cv::Mat &matched_points2, cv::Mat &points3D_1, cv::Mat &points3D_2) {
    
    // 1. Create Transformation matrix
    cv::Mat T(3, 4, CV_64F);
    cv::hconcat(R, t, T); 

    // 2. Create Projection matrix for image 2
    cv::Mat P2 = K * T;

    // 3. Triangulate the 3D points for image 1 & 2
    cv::Mat homogenous3D_points1;
    cv::triangulatePoints(P_ref, P2, matched_points1, matched_points2, homogenous3D_points1);
    cv::Mat homogenous3D_points2 = T * homogenous3D_points1;

    // 4. Un-homogenize points (Convert from 4D to 3D)
    cv::convertPointsFromHomogeneous(homogenous3D_points1.t(), points3D_1);
    cv::convertPointsFromHomogeneous(homogenous3D_points2.t(), points3D_2);

    // 5. Count points with positive z in both camera views
    int pos_z_point_count1 = 0;
    int pos_z_point_count2 = 0;

    for (int i = 0; i < points3D_1.rows; i++) {
        if (points3D_1.at<cv::Vec3d>(i)[2] > 0) pos_z_point_count1++;
        if (points3D_2.at<cv::Vec3d>(i)[2] > 0) pos_z_point_count2++;
    }

    int total_positive_zs = pos_z_point_count1 + pos_z_point_count2;
    return total_positive_zs;
}

double MonocularOdometry::Caulculate_RelativeScale(cv::Mat points3D_1, cv::Mat points3D_2) {

    double total_relative_scale = 0.0;
    int num_of_valid_pairs = points3D_1.rows - 1; // # of consecutive point pairs = num of points - 1

    for (int i = 0; i < num_of_valid_pairs; i++) {
        // Euclidean distances between consecutive 3D point pairs
        double point_pair1_dist = cv::norm(points3D_1.row(i) - points3D_1.row(i + 1));
        double point_pair2_dist = cv::norm(points3D_2.row(i) - points3D_2.row(i + 1));
        total_relative_scale += point_pair1_dist / point_pair2_dist;
    }
    return total_relative_scale / num_of_valid_pairs;
}

// Public ------------------------------------------------------------------------------------------------------------------------------------------
MonocularOdometry::MonocularOdometry() {}

MonocularOdometry::MonocularOdometry(const cv::Mat& k, const cv::Mat& p_ref) : K(k), P_ref(p_ref) {

    // Feature Detector
    orb = cv::ORB::create(3000); // 3000 Max features
    
    // Feature Matcher
    int table_number = 6; // Num of hash tables used in the LSH
    int key_size = 12; // Hash key size
    int multi_probe_level = 1; // Num of probe levels for multi-probe LSH
    int checks = 50;// Num of checks for each query during search phase
    // Use Locality Sensitive Hashing (LSH) Algorithm
    cv::Ptr<cv::flann::IndexParams> index_params = cv::makePtr<cv::flann::LshIndexParams>(table_number, key_size, multi_probe_level);
    cv::Ptr<cv::flann::SearchParams> search_params = cv::makePtr<cv::flann::SearchParams>(checks);
    flann = cv::FlannBasedMatcher(index_params, search_params);

    cur_pose = Eigen::MatrixXf::Identity(4, 4);
}

cv::Mat MonocularOdometry::Get_Pose(const cv::Mat &matched_points1, const cv::Mat &matched_points2) {

    // 1.0: Threshold val for RANSAC
    cv::Mat E = cv::findEssentialMat(matched_points1, matched_points2, K, cv::RANSAC, 1.0, 0);
    auto [R, t] = Decompose_Essential_Matrix(E, matched_points1, matched_points2);
    cv::Mat T(3, 4, CV_64F);
    cv::hconcat(R, t, T); 

    return T;
}

std::pair<cv::Mat, cv::Mat> MonocularOdometry::Get_Matches(cv::Mat img1, cv::Mat img2) {
    
    // 1. Find keypoints and descriptors
    std::vector<cv::KeyPoint> keypoints1, keypoints2;
    cv::Mat descriptors1, descriptors2;
    orb->detectAndCompute(img1, cv::Mat(), keypoints1, descriptors1);
    orb->detectAndCompute(img2, cv::Mat(), keypoints2, descriptors2);

    // 2. Find matches between Images
    std::vector<std::vector<cv::DMatch>> img_matches; // Each entry holds the 2 best matches for each keypoint descriptor (N x 2)
    flann.knnMatch(descriptors1, descriptors2, img_matches, 2);

    // 3. Filter out bad matches 
    std::vector<cv::DMatch> good_kp_matches;
    for (const auto& two_keypoint_matches : img_matches) {
        
        cv::DMatch closest_keypoint_match = two_keypoint_matches[0];
        cv::DMatch second_closest_keypoint_match = two_keypoint_matches[1];
        // Lowe's Ratio Test
        if (closest_keypoint_match.distance < 0.8 * second_closest_keypoint_match.distance) {
            good_kp_matches.push_back(closest_keypoint_match);
        }
    }

    // 4. Get points from the good matches
    std::vector<cv::Point2f> matched_points1, matched_points2;
    for (const auto& good_kp_match : good_kp_matches) {
        matched_points1.push_back(keypoints1[good_kp_match.queryIdx].pt);
        matched_points2.push_back(keypoints2[good_kp_match.trainIdx].pt);
    }

    // // Test: Draw matches
    // cv::Mat img_matches;
    // cv::drawMatches(images[i], keypoints1, img1, keypoints2, good, img_matches,
    //                 cv::Scalar::all(-1), cv::Scalar::all(-1),
    //                 std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

    // cv::imshow("Matches", img_matches);
    // cv::waitKey(200);

    return {cv::Mat(matched_points1), cv::Mat(matched_points2)};
}

Eigen::MatrixXf MonocularOdometry::Run(cv::Mat frame) {

    std::vector<cv::Point2f> trajectory;
    cv::Mat gray_frame;
    cv::cvtColor(frame, gray_frame, cv::COLOR_BGR2GRAY);

    if (!prev_frame.empty()) {
        
        auto [feature_points1, feature_points2] = Get_Matches(prev_frame, gray_frame);
        cv::Mat transf = Get_Pose(feature_points1, feature_points2);
        Eigen::Matrix4f T;
        for (int row = 0; row < 4; ++row) {
            for (int col = 0; col < 4; ++col) {
                T(row, col) = transf.at<float>(row, col);
            }
        }
        cur_pose = cur_pose * T.inverse();  // Update current pose

        // // Testing. draw the estimated trajectory on the frame
        // cv::Point2f position(cur_pose(0, 3), cur_pose(2, 3));
        // trajectory.push_back(position);
        // for (size_t i = 1; i < trajectory.size(); ++i) {
        //     cv::line(frame, trajectory[i - 1], trajectory[i], cv::Scalar(0, 255, 0), 2);
        // }
    }

    // // Testing. Display the resulting frame
    // cv::imshow("Visual Odometry", frame);

    prev_frame = gray_frame.clone();

    // Optionally save trajectory data to file or visualize it

    return cur_pose;
}




































// Private ---------------------------------------------------------------------
std::vector<cv::KeyPoint> StereoOdometry::Get_TileKeypoints(const cv::Mat &img_tile, int x, int y) {

    std::vector<cv::KeyPoint> keypoints;
    fastFeatures->detect(img_tile, keypoints);

    // Convert 'image tile' coord system to 'full image' coord system
    for (auto& coordinate : keypoints) {
        coordinate.pt.x += x;
        coordinate.pt.y += y;
    }

    // Get the 10 best keypoints (based on response value)
    if (keypoints.size() > 10) {
        std::sort(keypoints.begin(), keypoints.end(), 
            [](const cv::KeyPoint &a, const cv::KeyPoint& b) {
                return a.response > b.response;
            }
        );
        keypoints.resize(10);
    }

    return keypoints;
}


std::pair<std::vector<cv::Point2f>, std::vector<cv::Point2f>> StereoOdometry::Track_Keypoints(const cv::Mat &img1, const cv::Mat &img2, 
        const std::vector<cv::KeyPoint> &keypoints1, 
        float max_error) {

    std::vector<cv::Point2f> tracked_points1, tracked_points2, valid_tracked_points1, valid_tracked_points2;
    std::vector<uchar> status;
    std::vector<float> err;
    int height = img1.rows;
    int width = img1.cols;
    
    // Convert keypoints into a vector of points
    cv::KeyPoint::convert(keypoints1, tracked_points1);

    // Track Features (Optical Flow)
    cv::calcOpticalFlowPyrLK(img1, img2, tracked_points1, tracked_points2, status, err, lkWinSize, lkMaxLevel, lkCriteria, lkFlags);

    // Filter out unsuccessfully tracked points
    for (size_t i = 0; i < status.size(); ++i) {
        
        if (status[i] && err[i] < max_error && (tracked_points2[i].y < height && tracked_points2[i].x < width)) {
            valid_tracked_points1.push_back(tracked_points1[i]);
            valid_tracked_points2.push_back(tracked_points2[i]);
        }
    }

    return std::make_pair(valid_tracked_points1, valid_tracked_points2);
}


std::pair<std::vector<cv::Point2f>, std::vector<cv::Point2f>> StereoOdometry::Get_LeftImageKeypoints(const cv::Mat& img1, 
    const cv::Mat& img2, int tile_height, int tile_width) {

    std::vector<cv::KeyPoint> image1_keypoints;

    // Get image height and width
    int height = img1.rows;
    int width = img1.cols;

    // Get keypoints for each tile
    for (int y = 0; y < height; y += tile_height) {
        
        for (int x = 0; x < width; x += tile_width) {
            
            // Extract region (image tile) from image
            cv::Rect roi(x, y, tile_width, tile_height);
            cv::Mat img1_tile = img1(roi); 
            std::vector<cv::KeyPoint> tile_keypoints = Get_TileKeypoints(img1_tile, x, y);
            // Accumulate keypoints from all tiles
            image1_keypoints.insert(image1_keypoints.end(), tile_keypoints.begin(), tile_keypoints.end()); 
        }
    }
    return Track_Keypoints(img1, img2, image1_keypoints);
}



cv::Mat StereoOdometry::Get_DisparityMap(const cv::Mat &feature_points, const cv::Mat &l_r_disparities, cv::Mat &mask) {

    cv::Mat l_r_disparity_map;
    l_r_disparities.copyTo(l_r_disparity_map);

    cv::Mat feature_point_coords = feature_points.clone();
    feature_point_coords.convertTo(feature_point_coords, CV_32S); // Convert from float coords to 32Bit Signed integer coords

    // For each row in N x 1 Matrix
    for (int i = 0; i < feature_point_coords.rows; ++i) {
        
        int x = feature_point_coords.at<cv::Vec2i>(i, 0)[0];
        int y = feature_point_coords.at<cv::Vec2i>(i, 0)[1];
        
        // If valid disparity
        if ((x >= 0 && x < l_r_disparities.cols) && (y >= 0 && y < l_r_disparities.rows)) {
            float disparity = l_r_disparities.at<float>(y, x);
            l_r_disparity_map.at<float>(i) = disparity;
        }
    }

    // mask_temp: Each matrix element is true (255) only if both conditions are satisfied, else false (0)
    cv::Mat mask_temp = (l_r_disparity_map > min_l_r_disparity) & (l_r_disparity_map < max_l_r_disparity);
    mask_temp.convertTo(mask, CV_8U);
    return l_r_disparity_map;
    
}


std::tuple<cv::Mat, cv::Mat, cv::Mat, cv::Mat> StereoOdometry::Get_RightImageKeypoints(const cv::Mat& l_img2, const cv::Mat& r_img2, 
    cv::Mat feature_points1, cv::Mat feature_points2) {

    std::vector<cv::Point2f> l_feature_points1, l_feature_points2;
    std::vector<float> l_disparities1, l_disparities2;
    cv::Mat mask1, mask2;

    // 1. Calculate Disparity between Left image 2 and Right image 2
    cv::Mat disp;
    disparity->compute(l_img2, r_img2, disp);
    disp.convertTo(disp, CV_32F);
    disparities.push_back(disp / 16.0f); // Normalize disparity
    int idx = disparities.size() - 1;
    cv::Mat l_r_disparities1 = disparities[idx - 1];
    cv::Mat l_r_disparities2 = disparities[idx];

    // 2. Get Disparity Maps & Mask
    cv::Mat disparity_map1 = Get_DisparityMap(feature_points1, l_r_disparities1, mask1);
    cv::Mat disparity_map2 = Get_DisparityMap(feature_points2, l_r_disparities2, mask2);
    cv::Mat in_bounds_mask = mask1 & mask2;

    // 3. [Get Left Images' Feature Points] Filter out all feature points & disparities outside of bounds
    for (int i = 0; i < in_bounds_mask.rows; ++i) {
        
        if (in_bounds_mask.at<uchar>(i)) {
            
            l_feature_points1.push_back(feature_points1.at<cv::Point2f>(i));
            l_feature_points2.push_back(feature_points2.at<cv::Point2f>(i));
            l_disparities1.push_back(disparity_map1.at<float>(i));
            l_disparities2.push_back(disparity_map2.at<float>(i));
        }
    }

    // Conversions
    cv::Mat l_feature_points1_mat(l_feature_points1.size(), 1, CV_32FC2, l_feature_points1.data());
    cv::Mat l_feature_points2_mat(l_feature_points2.size(), 1, CV_32FC2, l_feature_points2.data());
    cv::Mat l_disparities1_mat(l_disparities1.size(), 1, CV_32F, l_disparities1.data());
    cv::Mat l_disparities2_mat(l_disparities2.size(), 1, CV_32F, l_disparities2.data());

    // 4. [Get Right Images' Feature Points]
    cv::Mat r_feature_points1_mat = l_feature_points1_mat.clone();
    cv::Mat r_feature_points2_mat = l_feature_points2_mat.clone();
    for (int i = 0; i < r_feature_points1_mat.rows; ++i) {
        
        r_feature_points1_mat.at<cv::Point2f>(i).x -= l_disparities1_mat.at<float>(i);
        r_feature_points2_mat.at<cv::Point2f>(i).x -= l_disparities2_mat.at<float>(i);
    }

    return std::make_tuple(l_feature_points1_mat, r_feature_points1_mat, l_feature_points2_mat, r_feature_points2_mat);
}



std::pair<cv::Mat, cv::Mat> StereoOdometry::Triangulate_3DPoints(cv::Mat l_feature_points1, cv::Mat r_feature_points1, 
        cv::Mat l_feature_points2, cv::Mat r_feature_points2) {

    cv::Mat homogenous_points1, homogenous_points2, points3D_1, points3D_2;

    cv::triangulatePoints(P_ref_left, P_ref_right, l_feature_points1, r_feature_points1, homogenous_points1);
    cv::triangulatePoints(P_ref_left, P_ref_right, l_feature_points2, r_feature_points2, homogenous_points2);

    // Un-homogenize points (Convert from 4D to 3D)
    cv::convertPointsFromHomogeneous(homogenous_points1.t(), points3D_1);
    cv::convertPointsFromHomogeneous(homogenous_points2.t(), points3D_2);

    return {points3D_1, points3D_2};
}

    

StereoOdometry::StereoReprojectionResidual::StereoReprojectionResidual(const cv::Mat& feature_points1, const cv::Mat& feature_points2, const cv::Mat& points3D_1, 
    const cv::Mat& points3D_2, const cv::Mat& p_ref_left_) : feature_points1_(feature_points1), feature_points2_(feature_points2), 
    points3D_1_(points3D_1), points3D_2_(points3D_2), P_ref_left_(p_ref_left_) {}


// Residual Block
template <typename T>
bool StereoOdometry::StereoReprojectionResidual::operator()(const T* const parameters, T* residuals_to_minimize) const {
    
    // if (std::is_same<T, int>::value) {

    // }
    // 1. Get the Rotation & Translation
    // cv::Mat r = parameters(cv::Range(0, 3)).clone();
    // cv::Mat t = parameters(cv::Range(3, 6)).clone();

    // cv::Mat r(3, 1, CV_64F), t(3, 1, CV_64F);
    // for (int i = 0; i < 3; ++i) {
    //     r.at<double>(i) = static_cast<double>(parameters[i].a);
    //     t.at<double>(i) = static_cast<double>(parameters[i + 3].a);
    // }
    // cv::Mat R;
    // cv::Rodrigues(r, R);

    // // 2. Create Transformation matrix
    // cv::Mat Transformation(3, 4, CV_64F);
    // cv::hconcat(R, t, Transformation); 

    // // 3. Homogenize the 3D points
    // cv::Mat homogeneous_points1, homogeneous_points2;
    // cv::Mat ones = cv::Mat::ones(points3D_1_.rows, 1, CV_64F);
    // cv::hconcat(points3D_1_, ones, homogeneous_points1);
    // cv::hconcat(points3D_2_, ones, homogeneous_points2);

    // // 4. Re-Project 3D points from [image 2] to 2D plane of [image 1]
    // cv::Mat feature_points1_prediction;
    // cv::Mat forward_projector = P_ref_left_ * Transformation; // Forward projection matrix
    // cv::Mat points3D_2_forward_projector = homogeneous_points2 * forward_projector.t();
    // cv::normalize(points3D_2_forward_projector.col(2), points3D_2_forward_projector);
    // feature_points1_prediction = points3D_2_forward_projector(cv::Range::all(), cv::Range(0, 2)).t();

    // // 5. Re-Project 3D points from [image 1] to 2D plane of [image 2]
    // cv::Mat feature_points2_prediction;
    // cv::Mat backward_projector = P_ref_left_ * Transformation.inv(); // Backward projection matrix
    // cv::Mat points3D_1_backward_projector = homogeneous_points1 * backward_projector.t();
    // cv::normalize(points3D_1_backward_projector.col(2), points3D_1_backward_projector);
    // feature_points2_prediction = points3D_1_backward_projector(cv::Range::all(), cv::Range(0, 2)).t();

    // // 6. Compute Reprojection Errors (residuals)
    // cv::Mat residuals_1 = feature_points1_prediction - feature_points1_.t();
    // cv::Mat residuals_2 = feature_points2_prediction - feature_points2_.t();
    // cv::Mat residuals;
    // cv::vconcat(residuals_1, residuals_2, residuals);
    // residuals = residuals.reshape(1, residuals.total());  // Flatten to a single vector

    // // Convert to array format for Ceres solver 
    // for (int i = 0; i < residuals.total(); ++i) {

    //     // Residuals to Minimize
    //     residuals_to_minimize[i] = static_cast<T>(residuals.at<double>(i));
    // }

    return true;
}


cv::Mat StereoOdometry::Get_EstimatedPose(const std::vector<cv::Point2f>& feature_points1, const std::vector<cv::Point2f>& feature_points2,
    const std::vector<cv::Point3f>& points3D_1, const std::vector<cv::Point3f>& points3D_2, int max_iter) {

    int early_termination = 0;
    const int early_termination_threshold = 5;
    double min_reprojection_error = std::numeric_limits<double>::infinity();
    double best_pose[6] = {0.0};

    std::random_device rd;
    std::mt19937 rng(rd());

    for (int i = 0; i < max_iter; i++) {
        
        // Random shuffle of feature points
        std::vector<int> shuffled_pt_indexes(feature_points1.size());
        std::iota(shuffled_pt_indexes.begin(), shuffled_pt_indexes.end(), 0); // Populate indexes
        std::shuffle(shuffled_pt_indexes.begin(), shuffled_pt_indexes.end(), rng); // Shuffle indexes

        // Select 6 random points
        std::vector<cv::Point2f> sample_feature_points1(6), sample_feature_points2(6);
        std::vector<cv::Point3f> sample_3Dpoints_1(6), sample_3Dpoints_2(6);
        
        for (int j = 0; j < 6; ++j) {
            
            int idx = shuffled_pt_indexes[j];
            sample_feature_points1[j] = feature_points1[idx];
            sample_feature_points2[j] = feature_points2[idx];
            sample_3Dpoints_1[j] = points3D_1[idx];
            sample_3Dpoints_2[j] = points3D_2[idx];
        }

        // Define initial guess (3 for rotation (Rodrigues), 3 for translation)
        double pose[6] = {0.0}; 

        // Build Problem-------------------------------------------------------------------
        /**
         * @brief GOAL: Find the best transformation (rotation and translation) 'best_pose' 
         *      that minimizes the reprojection error between 3D points (points3D_1 & points3D_2) and 2D points (feature_points1 & feature_points2)
         */
        ceres::Problem problem;
        ceres::Solver::Options options;
        options.max_num_iterations = 200;
        options.minimizer_progress_to_stdout = false;

        StereoReprojectionResidual *residual = new StereoReprojectionResidual(cv::Mat(sample_feature_points1), cv::Mat(sample_feature_points2), 
            cv::Mat(sample_3Dpoints_1), cv::Mat(sample_3Dpoints_2), P_ref_left);
        /**
         * @brief Compute the derivatives of the residual with respect to the parameters 'pose'
         * StereoReprojectionResidual: Type of the cost functor or residual block. Defines the residual calculation.
         * ceres::DYNAMIC : Number of residuals (output) in cost function is dynamic, can change and is determined at runtime.
         * 6: Number of parameters (input) for the cost function. (3 for rotation and 3 for translation))
         * feature_points1.rows * 2: Specifies the number of residuals, which is dynamic and depends on the number of 2D feature points.
         */
        ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<StereoReprojectionResidual, ceres::DYNAMIC, 6>(residual, 
            cv::Mat(feature_points1).rows * 2);
        problem.AddResidualBlock(cost_function, nullptr, pose);
        
        // Solve 
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        // Extract reprojection error
        double reprojection_error = summary.final_cost;

        // Check if this is the best solution
        if (reprojection_error < min_reprojection_error) {
            min_reprojection_error = reprojection_error;
            std::copy(pose, pose + 6, best_pose);
            early_termination = 0;
        } 
        else { early_termination++; }

        // Stop early if no improvement
        if (early_termination >= early_termination_threshold) { break; }

    }

    cv::Mat rvec(3, 1, CV_64F, &best_pose[0]); 
    cv::Mat t(3, 1, CV_64F, &best_pose[3]);
    cv::Mat R;
    cv::Mat T(3, 4, CV_64F);
    cv::Rodrigues(rvec, R);
    cv::hconcat(R, t, T); 
    return T;
}



// Public ---------------------------------------------------------------------
StereoOdometry::StereoOdometry() {}


StereoOdometry::StereoOdometry(const cv::Mat& k, const cv::Mat& p_ref_left, const cv::Mat& p_ref_right) :
    K(k), P_ref_left(p_ref_left), P_ref_right(p_ref_right) {
    int block = 11;
    int P1 = block * block * 8;
    int P2 = block * block * 32;

    disparity = cv::StereoSGBM::create(0, 32, block, P1, P2);
    disparities.push_back(cv::Mat());  // Initialize with an empty disparity vector

    fastFeatures = cv::FastFeatureDetector::create();

    // Optical Flow Parameters
    lkWinSize.width = 15;
    lkWinSize.height = 15;
    lkFlags = cv::MOTION_AFFINE;
    lkMaxLevel = 3;
    lkCriteria = cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 50, 0.03);

    cur_pose = Eigen::MatrixXf::Identity(4, 4);
}


cv::Mat StereoOdometry::Get_Pose(const cv::Mat &l_img1, const cv::Mat &l_img2, const cv::Mat &r_img1, const cv::Mat &r_img2) {

    auto [l_feature_points1, l_feature_points2] = Get_LeftImageKeypoints(l_img1, l_img2, 10, 20);
    auto [left1, right1, left2, right2] = Get_RightImageKeypoints(l_img2, r_img2, cv::Mat(l_feature_points1), cv::Mat(l_feature_points2));
    auto [points3D_1, points3D_2] = Triangulate_3DPoints(left1, right1, left2, right2);

    return Get_EstimatedPose(l_feature_points1, l_feature_points2, points3D_1, points3D_2);
}


Eigen::MatrixXf StereoOdometry::Run(cv::Mat left_frame, cv::Mat right_frame) {

    // Initialize variables for tracking the camera's trajectory
    std::vector<cv::Point2f> trajectory;

    // Convert to grayscale as required by VisualOdometry
    cv::Mat gray_left, gray_right;
    cv::cvtColor(left_frame, gray_left, cv::COLOR_BGR2GRAY);
    cv::cvtColor(right_frame, gray_right, cv::COLOR_BGR2GRAY);

    if (!prev_frame_left.empty() && !prev_frame_right.empty()) {

        // Estimate pose
        cv::Mat transf = Get_Pose(prev_frame_left, gray_left, prev_frame_right, gray_right);
        Eigen::Matrix4f T;
        for (int row = 0; row < 4; ++row) {
            for (int col = 0; col < 4; ++col) {
                T(row, col) = transf.at<float>(row, col);
            }
        }
        cur_pose = cur_pose * T.inverse();  // Update current pose

        // // Testing. draw the estimated trajectory on the frame
        // cv::Point2f position(cur_pose(0, 3), cur_pose(2, 3));
        // trajectory.push_back(position);
        // for (size_t i = 1; i < trajectory.size(); ++i) {
        //     cv::line(frame_left, trajectory[i - 1], trajectory[i], cv::Scalar(0, 255, 0), 2);
        // }
    }

    // // Testing. Display the resulting frame
    // cv::imshow("Visual Odometry (Left)", frame_left);

    // Update prev frames
    prev_frame_left = gray_left.clone();
    prev_frame_right = gray_right.clone();

    // Optionally save trajectory data to file or visualize it

    return cur_pose;
}




