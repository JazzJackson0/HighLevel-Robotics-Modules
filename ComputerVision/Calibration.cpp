#include "../include/camera/Calibration.hpp"
/**
 * NOT FINISHED!!
 * 
 */

// Private-----------------------------------------------------------------------------------------------------------------------------------
void CameraCalibrator::loadImages(const std::string& path) {
    
    glob_t glob_result;
    glob((path + "/*.png").c_str(), 0, nullptr, &glob_result);
    for (size_t i = 0; i < glob_result.gl_pathc; ++i) {
        cv::Mat img = cv::imread(glob_result.gl_pathv[i]);
        images.push_back(img);
    }
    globfree(&glob_result);


    for (const auto& img : images) {
        cv::Mat gray;
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

        std::vector<cv::Point2f> corners_2D;
        if (cv::findChessboardCorners(gray, cv::Size(cols, rows), corners_2D)) {
            
            objpoints_3D.push_back(grid_3D);
            /**
             * @brief Refines the corner positions detected by cv2.findChessboardCorners to subpixel accuracy, 
             *       improving the precision of the corner detection. 
             */
            cv::cornerSubPix(gray, corners_2D, cv::Size(11, 11), cv::Size(-1, -1), criteria);
            imgpoints_2D.push_back(corners_2D);

            // // Draw and display the corners
            // cv::drawChessboardCorners(img, cv::Size(cols, rows), corners, found);
            // cv::imshow("img", img);
            // cv::waitKey(1000);
        }
    }

    cv::destroyAllWindows();
}


// Public-----------------------------------------------------------------------------------------------------------------------------------
CameraCalibrator::CameraCalibrator() {}


CameraCalibrator::CameraCalibrator(int _cols, int _rows, float _size_of_chessboard_squares_mm) : cols(_cols), rows(_rows),
        size_of_chessboard_squares_mm(_size_of_chessboard_squares_mm), criteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.001) {
    
    // Create zero-initialized array for 3D checkerboard grid points
    grid_3D = cv::Mat(rows * cols, 3, CV_32F, cv::Scalar(0));
    
    /**
     * @brief Each square on the checkerboard is 1 UNIT in size (e.g., 1 cm or 1 mm)
     * Each element in grid represents the position of a corner in real-world units 
     * (e.g., millimeters or centimeters) on the calibration pattern. 
     */
    for (int i_units = 0; i_units < rows; ++i_units) {
        
        for (int j_units = 0; j_units < cols; ++j_units) {
            
            grid_3D.at<float>(i_units * cols + j_units, 0) = j_units;
            grid_3D.at<float>(i_units * cols + j_units, 1) = i_units;
        }
    }

    // Scale the checkerboard grid points
    // A single unit is now (1 * size_of_chessboard_squares_mm)
    grid_3D *= size_of_chessboard_squares_mm;
}



void CameraCalibrator::captureImages(int camera_id, int num_of_frames) {
    
    // Initialize video capture
    cap.open(camera_id);
    if (!cap.isOpened()) {
        std::cerr << "Error: Unable to open the camera." << std::endl;
        exit(EXIT_FAILURE);
    }

    cv::Mat img;
    // Capture a set number of frames for calibration
    for (int i = 0; i < num_of_frames; ++i) { 
        
        cap >> img;
        if (img.empty()) {
            std::cerr << "Error: Captured empty frame." << std::endl;
            continue;
        }

        std::ostringstream oss;
        oss << "image_" << i << ".png";
        std::string filename = oss.str();

        if (!cv::imwrite(filename, img)) {
            std::cerr << "Error: Could not save image as " << filename << std::endl;
            return;
        }
    }
    
    cap.release();
    cv::destroyAllWindows();
}



std::tuple<cv::Mat, std::vector<cv::Mat>, std::vector<cv::Mat>> CameraCalibrator::calibrateCamera(const std::string& images_path) {
    
    loadImages(images_path);

    if (objpoints_3D.empty() || imgpoints_2D.empty()) {
        std::cerr << "Error: No object points or image points collected." << std::endl;
        //return;
    }

    // Calibrate
    cv::Size frameSize = images[0].size();  // Assuming the size of the image from the first capture
    cv::calibrateCamera(objpoints_3D, imgpoints_2D, frameSize, cameraMatrix_K, dist_coeffs, rvecs, tvecs);

    // Remove Distortion to clean K, {R,...}, {t,...}, etc...
    cv::Mat newCameraMatrix_K;
    cv::Rect roi;
    newCameraMatrix_K = cv::getOptimalNewCameraMatrix(cameraMatrix_K, dist_coeffs, frameSize, 1, frameSize, &roi);
    cameraMatrix_K = newCameraMatrix_K;
    
    // Save calibration results
    cv::FileStorage fs("calibration.yml", cv::FileStorage::WRITE);
    fs << "cameraMatrix" << cameraMatrix_K;
    fs << "distCoeffs" << dist_coeffs;
    fs.release();

    return {cameraMatrix_K, rvecs, tvecs};
}


double CameraCalibrator::diagnoseCalibration() {
    
    // Calculate Reprojection Error
    double meanError = 0;
    for (size_t i = 0; i < objpoints_3D.size(); ++i) {
        
        std::vector<cv::Point2f> imgpoints2_2D;
        cv::projectPoints(objpoints_3D[i], rvecs[i], tvecs[i], cameraMatrix_K, dist_coeffs, imgpoints2_2D);
        cv::Mat imgpointsMat = cv::Mat(imgpoints_2D[i]);
        cv::Mat imgpoints2Mat = cv::Mat(imgpoints2_2D);

        double error = cv::norm(imgpointsMat, imgpoints2Mat, cv::NORM_L2) / imgpoints2_2D.size();
        meanError += error;
    }

    meanError /= objpoints_3D.size();
    return meanError;
}



std::tuple<cv::Mat, std::vector<cv::Mat>, std::vector<cv::Mat>> CameraCalibrator::refineParameters() {

    // TODO: 
}