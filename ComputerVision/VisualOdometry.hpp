#pragma once
#include <iostream>
#include <ceres/ceres.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <vector>
#include <utility>  // For std::pair
#include <tuple>    // For std::tuple
#include <algorithm>
#include <random>


//--------------------------------------------------------------------------------------------------------------------------------------
class MonocularOdometry {

    private:
        const cv::Mat K;
        const cv::Mat P_ref;
        cv::Ptr<cv::ORB> orb;
        cv::FlannBasedMatcher flann;
        cv::Mat prev_frame;
        Eigen::MatrixXf cur_pose;

        /**
         * @brief 
         * 
         * @param E Essential Matrix
         * @param matched_points1 
         * @param matched_points2 
         * @return std::pair<cv::Mat, cv::Mat> 
         */
        std::pair<cv::Mat, cv::Mat> Decompose_Essential_Matrix(const cv::Mat& E, const cv::Mat &matched_points1, const cv::Mat &matched_points2);

        /**
         * @brief Calculate and return the number of positive z values between the 2 point sets.
         *      Why: Only the points with positive z coordinates (in the camera coordinate system) 
         *      are in front of the camera and thus are valid 3D points.
         *      Ensures that the reconstructed 3D points are primarily in front of both cameras.
         * 
         * @param R 
         * @param t 
         * @param matched_points1 
         * @param points3D_1 
         * @param points3D_2
         * @return int 
         */
        int Get_SumOfZCoords(const cv::Mat &R, const cv::Mat &t, const cv::Mat &matched_points1, const cv::Mat &matched_points2,
            cv::Mat &points3D_1, cv::Mat &points3D_2);

        /**
         * @brief Calculate relative scale between points in both images
            (Mitigates 'scale ambiguity' that comes with Monocular cameras)
         * 
         * @param points3D_1 
         * @param points3D_2 
         * @return double 
         */
        double Caulculate_RelativeScale(cv::Mat points3D_1, cv::Mat points3D_2);

    public:

        MonocularOdometry();

        MonocularOdometry(const cv::Mat& k, const cv::Mat& p_ref);

        /**
         * @brief Return the current pose as a transformation matrix
         * 
         * @param matched_points1 
         * @param matched_points2 
         * @return cv::Mat 
         */
        cv::Mat Get_Pose(const cv::Mat &matched_points1, const cv::Mat &matched_points2);

        /**
         * @brief 
         * 
         * @param img1 
         * @param img2 
         * @return std::pair<cv::Mat, cv::Mat> 
         */
        std::pair<cv::Mat, cv::Mat> Get_Matches(cv::Mat img1, cv::Mat img2);

        /**
         * @brief 
         * 
         * @param frame 
         * @return int 
         */
        Eigen::MatrixXf Run(cv::Mat frame);
};
















//--------------------------------------------------------------------------------------------------------------------------------------
class StereoOdometry {

    private:
        const cv::Mat K;
        const cv::Mat P_ref_left;
        const cv::Mat P_ref_right;
        float min_l_r_disparity;
        float max_l_r_disparity;
        std::vector<cv::Mat> disparities;
        cv::Ptr<cv::StereoSGBM> disparity;
        cv::Ptr<cv::FastFeatureDetector> fastFeatures;

        // Optical Flow Parameters
        cv::TermCriteria lkCriteria;
        cv::Size lkWinSize;
        int lkMaxLevel;
        int lkFlags;

        cv::Mat prev_frame_left, prev_frame_right;
        Eigen::MatrixXf cur_pose;
        
        /**
         * @brief 
         * 
         */
        struct StereoReprojectionResidual {
            
            private:
                const cv::Mat feature_points1_;
                const cv::Mat feature_points2_;
                const cv::Mat points3D_1_;
                const cv::Mat points3D_2_;
                const cv::Mat P_ref_left_;

            public:
            
            /**
             * @brief Construct a new Stereo Reprojection Residual object
             * 
             * @param feature_points1 
             * @param feature_points2 
             * @param points3D_1 
             * @param points3D_2 
             * @param p_ref_left 
             */
            StereoReprojectionResidual(const cv::Mat& feature_points1, const cv::Mat& feature_points2, const cv::Mat& points3D_1, 
                const cv::Mat& points3D_2, const cv::Mat& p_ref_left);

            /**
             * @brief 
             * 
             * @tparam T 
             * @param parameters
             * @param residuals_to_minimize
             * @return true 
             * @return false 
             */
            template <typename T>
            bool operator()(const T* const parameters, T* residuals_to_minimize) const;

            
        };

        /**
         * @brief 
         * 
         * @param img1 
         * @param img2 
         * @param keypoints1 
         * @param max_error 
         * @return std::pair<cstd::vector<cv::Point2f>, std::vector<cv::Point2f>> 
         */
        std::pair<std::vector<cv::Point2f>, std::vector<cv::Point2f>> Track_Keypoints(const cv::Mat &img1, const cv::Mat &img2, 
            const std::vector<cv::KeyPoint> &keypoints1, float max_error = 4.0f);

        /**
         * @brief 
         * 
         * @param img_tile 
         * @param x 
         * @param y 
         * @return std::vector<cv::KeyPoint> 
         */
        std::vector<cv::KeyPoint> Get_TileKeypoints(const cv::Mat &img_tile, int x, int y);

        /**
         * @brief 
         * 
         * @param img1 
         * @param img2 
         * @param tile_height 
         * @param tile_width 
         * @return std::pair<cstd::vector<cv::Point2f>, std::vector<cv::Point2f>> 
         */
        std::pair<std::vector<cv::Point2f>, std::vector<cv::Point2f>> Get_LeftImageKeypoints(const cv::Mat &img1, const cv::Mat &img2, 
            int tile_height, int tile_width);

        /**
         * @brief 
         * 
         */
        cv::Mat Get_DisparityMap(const cv::Mat &feature_points, const cv::Mat &l_r_disparities, cv::Mat &mask);
        
        /**
         * @brief Calculate the keypoints in the right images using the disparities between them and the left images' keypoints
         * 
         * @param l_img2 
         * @param r_img2 
         * @param feature_points1 
         * @param feature_points2 
         * @return std::tuple<cv::Mat, cv::Mat, cv::Mat, cv::Mat> 
         */
        std::tuple<cv::Mat, cv::Mat, cv::Mat, cv::Mat> Get_RightImageKeypoints(const cv::Mat& l_img2, const cv::Mat& r_img2, 
            cv::Mat feature_points1, cv::Mat feature_points2);

        /**
         * @brief 
         * 
         * @param l_feature_points1 
         * @param r_feature_points1 
         * @param l_feature_points2 
         * @param r_feature_points2 
         * @return std::pair<cv::Mat, cv::Mat> 
         */
        std::pair<cv::Mat, cv::Mat> Triangulate_3DPoints(cv::Mat l_feature_points1, cv::Mat r_feature_points1, 
            cv::Mat l_feature_points2, cv::Mat r_feature_points2);
        
        /**
         * @brief 
         * 
         * @param feature_points1 
         * @param feature_points2 
         * @param points3D_1 
         * @param points3D_2 
         * @param max_iter 
         * @return cv::Mat 
         */
        cv::Mat Get_EstimatedPose(const std::vector<cv::Point2f>& feature_points1, const std::vector<cv::Point2f>& feature_points2,
            const std::vector<cv::Point3f>& points3D_1, const std::vector<cv::Point3f>& points3D_2,
            int max_iter = 100);

    public:

        StereoOdometry();

        /**
         * @brief Construct a new Stereo Odometry object
         * 
         * @param k 
         * @param p_ref_left 
         * @param p_ref_right 
         */
        StereoOdometry(const cv::Mat& k, const cv::Mat& p_ref_left, const cv::Mat& p_ref_right);

        /**
         * @brief 
         * 
         * @param l_img1 
         * @param l_img2 
         * @param r_img1 
         * @param r_img2 
         * @return cv::Mat 
         */
        cv::Mat Get_Pose(const cv::Mat &l_img1, const cv::Mat &l_img2, const cv::Mat &r_img1, const cv::Mat &r_img2);
        
        /**
         * @brief 
         * 
         * @param left_frame 
         * @param right_frame 
         * @return Eigen::MatrixXf 
         */
        Eigen::MatrixXf Run(cv::Mat left_frame, cv::Mat right_frame);
};