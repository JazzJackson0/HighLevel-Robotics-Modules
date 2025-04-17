#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <vector>
#include <utility>  // For std::pair
#include <tuple>    // For std::tuple
#include <string>
#include <iostream>
#include <glob.h>

class CameraCalibrator {

    private:
        int cols; // Num of internal corners (intersections) along width of checkerboard. [corresponds to num of columns of internal corners]
        int rows; // Num of internal corners along height of checkerboard. [corresponds to num of rows of internal corners]
        cv::Mat grid_3D; // grid representing the chessboard's internal corners
        std::vector<cv::Mat> objpoints_3D; // 3d points in real world space [Stores grid_3D objects for each image]
        std::vector<std::vector<cv::Point2f>> imgpoints_2D; // 2d points in image plane [Stores 2D image points for each image]
        cv::Mat cameraMatrix_K; // The intrinsic parameters (focal lengths, principal point, and skew).
        cv::Mat dist_coeffs; // The distortion coefficients (radial and tangential distortions).
        std::vector<cv::Mat> rvecs; // Rotation vectors for each image.
        std::vector<cv::Mat> tvecs; // Translation vectors for each image.
        cv::TermCriteria criteria; // termination criteria for the 'cv.cornerSubPix()' function
        float size_of_chessboard_squares_mm;
        cv::VideoCapture cap;
        std::vector<cv::Mat> images;


        /**
         * @brief 
         * 
         * @param path 
         */
        void loadImages(const std::string& path);


    public:

        /**
         * @brief Construct a new Camera Callibrator object
         * 
         */
        CameraCalibrator();

        /**
         * @brief Construct a new Camera Calibrator object
         * 
         * @param _cols 
         * @param _rows 
         * @param _size_of_chessboard_squares_mm 
         */
        CameraCalibrator(int _cols, int _rows, float _size_of_chessboard_squares_mm);

        /**
         * @brief 
         * 
         * @param camera_id 
         * @param num_of_frames Number of image frames to capture
         */
        void captureImages(int camera_id, int num_of_frames);


        /**
         * @brief Determine internal (intrinsic) and external (extrinsic) parameters of camera. 
         * 
         * @param images_path 
         * @return std::tuple<cv::Mat, std::vector<cv::Mat>, std::vector<cv::Mat>> K Matrix, rvecs, tvecs
         */
        std::tuple<cv::Mat, std::vector<cv::Mat>, std::vector<cv::Mat>> calibrateCamera(const std::string& images_path);


        /**
         * @brief Provides a quantitative measure of how well the camera calibration parameters fit the actual data. 
         *  (A lower reprojection error indicates a more accurate calibration).
         * 
         * @return double 
         */
        double diagnoseCalibration();

        // TODO:
        /**
         * @brief Refine the camera parameters
         * 
         * 
         * Use an optimization technique to refine the camera parameters by minimizing the re-projection error, 
         * which measures the difference between the observed image points and the projected points.
         * 
         * @return std::tuple<cv::Mat, std::vector<cv::Mat>, std::vector<cv::Mat>> K Matrix, rvecs, tvecs
         */
        std::tuple<cv::Mat, std::vector<cv::Mat>, std::vector<cv::Mat>> refineParameters();


};


