#pragma once
#include <iostream>
#include <cmath>
#include <vector>
#include </usr/include/eigen3/Eigen/Dense>
#include </usr/include/eigen3/Eigen/src/Core/Matrix.h>

#include "utils.hpp"

using std::pair;
using std::vector;
using namespace Eigen;

struct Point {

    float x;
    float y;
    float angle;
    int err;
};

struct GeneralFormLine {

    double a;
    double b;
    double c;
    int err;
};

struct SlopeInterceptLine{

    double m;
    double b;
    int err;
};

struct Landmark {

    int id;
    Point position;
    float range; // Range from robot
    float bearing; // Bearing relative to robot
    std::vector<Point> points;
    GeneralFormLine line;
    int err;
};

struct LineSegment {

    GeneralFormLine line_fit;
    std::vector<Point> points;
    std::vector<Point> predicted_points;
    int start_idx;
    int end_idx;
    std::vector<Point> endpoints;
    int err;
};


class FeatureExtractor {

    private:
        int MinSeedSegNum; // 
        int MinLineSegNum; // 
        float MinLineSegLen; // 
        int SeedSegWindowSize;
        float Delta; // Distance (in meters) threshold from point position to predicted point position
        float Epsillon; // Distance (in meters) threshold from every potential segment point to the fitting line
        std::vector<Point> LaserPoints;
        std::vector<Point> SeedSeg;
        //vector<Point> PredictedPoints;
        Point RobotPos; // Current Robot Position
        int LandmarkIDs;
        float GapValue; // Acceptable distance between points
        std::vector<Landmark> NewLandmarks;
        std::vector<Landmark> AllLandmarks;
        int breakpoint_idx;
        

        /**
         * @brief Calculate euclidean distance between two points
         * 
         * @param point_a 
         * @param point_b 
         * @return float 
         */
        float Get_EuclideanDistance(Point point_a, Point point_b);

        /**
         * @brief Calculate distance between point and line
         * 
         * @param point 
         * @param general_line 
         * @return float 
         */
        float Get_Point2LineDistance(Point point, GeneralFormLine general_line);

        /**
         * @brief Get two points in a given line
         * 
         * @param x1 The x value used to calculate the first point
         * @param x2 The x value used to calculate the second point
         * @param slope_line The line to get the points from
         * @return vector<Point> The two points
         */
        std::vector<Point> Get_2PointsFromLine(int x1, int x2, SlopeInterceptLine slope_line);

        /**
         * @brief Converts Line from Slope-Intercept Form to General Form
         * 
         * @param slope_line line in slope-intercept form
         * @return GeneralFormLine - line in general form
         */
        GeneralFormLine SlopeInt2General(SlopeInterceptLine slope_line);


        /**
         * @brief Converts Line from General Form to Slope-Intercept Form
         * 
         * @param general_line line in general form
         * @return SlopeInterceptLine - line in slope-intercept form
         */
        SlopeInterceptLine General2SlopeInt(GeneralFormLine general_line);

        /**
         * @brief Calculate the intersection between 2 lines
         * 
         * @param general_line_1 line 1 (in general form)
         * @param general_line_2 line 2 (in general form)
         * @return Point - Intersection point
         */
        Point Get_Intersection(GeneralFormLine general_line_1, GeneralFormLine general_line_2);

        /**
         * @brief Get Position Coordinate from Angle & Distance Information
         * 
         * @param dist distance measurement
         * @param angle angle measurement
         * @return Point 
         */
        Point AD2Position(float dist, float angle);

        /**
         * @brief Transform scan from array of range & bearing values to array of position coordinates
         * 
         * @param scan point cloud
         * @return vector<Point> 
         */
        std::vector<Point> TransformScan(PointCloud scan);

        /**
         * @brief Create a Linear Model given two points
         * 
         * @param point_1 
         * @param point_2 
         * @return SlopeInterceptLine 
         */
        SlopeInterceptLine CreateLinearModel(Point point_1, Point point_2);


        /**
         * @brief Fit line to set of points using Orthogonal Distance Regression (i.e. Orthogonal line fitting)
         * 
         * @param laser_points Set of points
         * @return GeneralFormLine 
         */
        GeneralFormLine ODRFit(std::vector<Point> laser_points);

        /**
         * @brief Calculate the predicted position of a given point by calculating the intersection 
         *          between its (range & bearing) and the fitted line.
         * 
         * @param fitted_line The line parameters for the fitted line (e.g. ODR) that will intersect with 
         *              the calculated beam line in order to get the predicted point.
         * @param point_in_scan The geven point from the scan
         * @return Point - The predicted point
         */
        Point Get_PointPrediction(GeneralFormLine fitted_line, Point point_in_scan);


        /**
         * @brief Calculate the endpoints for a seed segment
         * 
         * @param line The fitted line of the segment
         * @param point_a The outermost point on end a
         * @param point_b The outermost point on end b
         * @return vector<Point> The two endpoints
         */
        std::vector<Point> Get_Endpoints(GeneralFormLine line, Point point_a, Point point_b);
        
        
        /**
         * @brief Calculate the Orthogonal Projection of a given point to a given line
         * 
         * @param slope_line 
         * @param data_point 
         * @return Point 
         */
        Point OrthogProjectPoint2Line(SlopeInterceptLine slope_line, Point data_point);


        /**
         * @brief Compare the landmarks obtained from the curent scan to all previously saved landmarks.
         *       Determine if the new landmarks are ones previously seen (upon which you update the old
         *      version of it with the new one) or new (upon which you add that landmark to the saved ones).
         * 
         */
        void CheckOverlap();


        /**
         * @brief Detect seed segment from current set of points
         * 
         * @return SeedSegment 
         */
        LineSegment DetectSeedSegment();


        /**
         * @brief Grow the given seed segment
         * 
         * @param seed_seg 
         * @return LineSegment 
         */
        LineSegment GrowSeedSegment(LineSegment seed_seg);


        /**
         * @brief Validates the given feature by outputting it as a landmark, provided
         *          it passes validation requirements
         * 
         * @param feature line segment to validate as a landmark
         * @return Landmark Unvalidated landmarks will have an error value of 1
         */
        Landmark ValidationGate(LineSegment feature);


        /**
         * @brief Reset all global variables needed to find new landmarks 
         * 
         */
        void reset();


    public:

        // Default construtor
        FeatureExtractor();

        /**
         * @brief Construct a new Feature Extractor object
         * 
         * @param delta Distance threshold from point position to predicted point position
         * @param epsillon Distance threshold from every potential segment point to the fitting line
         * @param gap_value Acceptable distance between points
         * @param min_seed_seg_num 
         */
        FeatureExtractor(float delta, float epsillon, float gap_value, int min_seed_seg_num);

        /**
         * @brief Runs the Feature Extraction Algorithm, pulling all Landmarks out of the current scan
         * 
         * @param current_scan 
         * @param current_pose
         */
        std::vector<Landmark> LandmarksFromScan(PointCloud current_scan, VectorXf current_pose);


        /**
         * @brief Set new delta value
         * 
         * @param delta Distance threshold from point position to predicted point position
         */
        void Set_Delta(float delta);


        /**
         * @brief Set new epsillon value
         * 
         * @param epsillon  Distance threshold from every potential segment point to the fitting line
         */
        void Set_Epsillon(float epsillon);


        /**
         * @brief 
         * 
         * @param gap_val 
         */
        void Set_GapValue(float gap_val);


        /**
         * @brief 
         * 
         * @param min_seed_seg_num 
         */
        void Set_MinSeedSegNum(int min_seed_seg_num);


        /**
         * @brief Set minimum line segment length threshold
         * 
         * @param min_line_seg_len 
         */
        void Set_MinLineSegLen(float min_line_seg_len);

};


