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
    vector<Point> points;
    GeneralFormLine line;
    int err;
};

struct LineSegment {

    GeneralFormLine line_fit;
    vector<Point> points;
    int start_idx;
    int end_idx;
    vector<Point> endpoints;
    int err;
};


class FeatureExtractor {

    private:
        int MinSeedSegNum;
        int MinLineSegNum;
        float MinLineSegLen;
        int CurrentPointsInLineSeg;
        float Delta;
        float Epsillon;
        vector<Point> LaserPoints;
        //vector<Point> PredictedPoints;
        Point RobotPos; // Current Robot Position
        int LandmarkIDs;
        float GapValue;
        vector<Landmark> NewLandmarks;
        vector<Landmark> AllLandmarks;
        int breakpoint_idx;
        

        /**
         * @brief 
         * 
         * @param point_a 
         * @param point_b 
         * @return float 
         */
        float Get_EuclideanDistance(Point point_a, Point point_b);

        /**
         * @brief 
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
        vector<Point> Get_2PointsFromLine(int x1, int x2, SlopeInterceptLine slope_line);

        /**
         * @brief Converts Line from Slope-Intercept Form to General Form
         * 
         * @param slope_line 
         * @return GeneralFormLine 
         */
        GeneralFormLine SlopeInt2General(SlopeInterceptLine slope_line);


        /**
         * @brief Converts Line from General Form to Slope-Intercept Form
         * 
         * @param general_line 
         * @return SlopeInterceptLine 
         */
        SlopeInterceptLine General2SlopeInt(GeneralFormLine general_line);

        /**
         * @brief Calculate the intersection between 2 lines
         * 
         * @param general_line_1 
         * @param general_line_2 
         * @return Point 
         */
        Point Get_Intersection(GeneralFormLine general_line_1, GeneralFormLine general_line_2);

        /**
         * @brief Get Position Coordinate from Angle & Distance Information
         * 
         * @param dist 
         * @param angle 
         * @return Point 
         */
        Point AD2Position(float dist, float angle);

        /**
         * @brief Transform scan from array of range & bearing values to array of position coordinates
         * 
         * @param scan 
         * @return vector<Point> 
         */
        vector<Point> TransformScan(vector<VectorXf> scan);

        /**
         * @brief Create a Linear Model given two points
         * 
         * @param point_1 
         * @param point_2 
         * @return SlopeInterceptLine 
         */
        SlopeInterceptLine CreateLinearModel(Point point_1, Point point_2);


        /**
         * @brief 
         * 
         * @param laser_points 
         * @return GeneralFormLine 
         */
        GeneralFormLine ODRFit(vector<Point> laser_points);

        /**
         * @brief 
         * 
         * @param fitted_line The line parameters for the fitted line (e.g. ODR) that will intersect with 
         *              the calculated beam line in order to get the predicted point.
         * @param point_in_scan The geven point from the scan
         * @return Point 
         */
        Point Get_PointPrediction(GeneralFormLine fitted_line, Point point_in_scan);


        /**
         * @brief 
         * 
         * @param line 
         * @param point_a 
         * @param point_b 
         * @return vector<Point> 
         */
        vector<Point> Get_Endpoints(GeneralFormLine line, Point point_a, Point point_b);
        
        /**
         * @brief 
         * 
         * @param feature 
         * @return Landmark 
         */
        Landmark ValidationGate(LineSegment feature);
        
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
         * @brief 
         * 
         */
        void reinit();


    public:

        /**
         * @brief Construct a new Feature Extractor object
         * 
         * @param delta 
         * @param epsillon
         * @param min_seed_seg_num 
         */
        FeatureExtractor(float delta, float epsillon, int min_seed_seg_num);

        /**
         * @brief Runs the Feature Extraction Algorithm, pulling all Landmarks out of the current scan
         * 
         * @param current_scan 
         */
        vector<Landmark> LandmarksFromScan(vector<VectorXf> current_scan);


        /**
         * @brief 
         * 
         * @param num_of_points 
         * @return SeedSegment 
         */
        LineSegment DetectSeedSegment(int num_of_points);


        /**
         * @brief 
         * 
         * @param seed_seg 
         * @return LineSegment 
         */
        LineSegment GrowSeedSegment(LineSegment seed_seg);


        /**
         * @brief 
         * 
         * @param delta 
         */
        void Set_Delta(float delta);


        /**
         * @brief 
         * 
         * @param epsillon 
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
         * @brief 
         * 
         * @param min_line_seg_num 
         */
        void Set_MinLineSegNum(int min_line_seg_num);


        /**
         * @brief 
         * 
         * @param min_line_seg_len 
         */
        void Set_MinLineSegLen(float min_line_seg_len);
};


