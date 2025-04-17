#include "FeatureExtraction.hpp"

// Private -------------------------------------------------------------------------------------------------------------------

float FeatureExtractor::Get_EuclideanDistance(Point point_a, Point point_b) {

    return hypot((point_b.x - point_a.x), (point_b.y - point_a.y));
    
}

float FeatureExtractor::Get_Point2LineDistance(Point point, GeneralFormLine general_line) {

    return abs((general_line.a * point.x) + (general_line.b * point.y) + general_line.c) / 
        hypot(general_line.a, general_line.b);
    
}

std::vector<Point> FeatureExtractor::Get_2PointsFromLine(int x1, int x2, SlopeInterceptLine slope_line) {

    std::vector<Point> points;
    Point point_a;
    Point point_b;

    point_b.x = x2;
    point_a.x = x1;
    point_a.y = (slope_line.m * x1) + slope_line.b;
    point_b.y = (slope_line.m * x2) + slope_line.b;
    points.push_back(point_a);
    points.push_back(point_b);

    return points;
}

GeneralFormLine FeatureExtractor::SlopeInt2General(SlopeInterceptLine slope_line) {

    int denominator_limit = 1000;
    double integerPart;
    double fractionPart;
    int numerator;
    int div;

    GeneralFormLine general_line;
    general_line.a = -slope_line.m;
    general_line.b = 1;
    general_line.c = -slope_line.b;

    // if (general_line.a < 0) {
    //     general_line.a *= -1;
    //     general_line.b *= -1;
    //     general_line.c *= -1;
    // }

    // // Pull out fraction part of A 
    // fractionPart = modf(general_line.a, &integerPart);

    // // Calculate Simplified Denominator for A 
    // numerator = (int)fractionPart * denominator_limit;
    // int denominator_a = denominator_limit;
    // div = gcd(numerator, denominator_limit);
    // denominator_a /= div; // (This denom calculated for the fraction part is the same as it would be for the entire number)

    // // Pull out fraction part of C 
    // fractionPart = modf(general_line.c, &integerPart);

    // // Calculate Simplified Denominator for C 
    // numerator = (int)fractionPart * denominator_limit;
    // int denominator_c = denominator_limit;
    // div = gcd(numerator, denominator_limit);
    // denominator_c /= div; // (This denom calculated for the fraction part is the same as it would be for the entire number)

    // // Calculate the LCM
    // int lcm = abs((denominator_a * denominator_c)) / gcd(denominator_a, denominator_c);

    // // Update Line Params
    // general_line.a *= lcm;
    // general_line.b *= lcm;
    // general_line.c *= lcm;

    return general_line;
}

SlopeInterceptLine FeatureExtractor::General2SlopeInt(GeneralFormLine general_line) {

    SlopeInterceptLine slope_line;
    slope_line.m = -(general_line.a / general_line.b);
    slope_line.b = -(general_line.c / general_line.b);

    return slope_line;
}


Point FeatureExtractor::Get_Intersection(GeneralFormLine general_line_1, GeneralFormLine general_line_2) {

    Point point;

    // Switched the subtraction order in the denominators so the sign of the output point match the actual point
    point.x = ((general_line_1.b * general_line_2.c) - (general_line_2.b * general_line_1.c)) / 
        ((general_line_1.a * general_line_2.b) - (general_line_2.a * general_line_1.b));

    point.y = ((general_line_2.a * general_line_1.c) - (general_line_1.a * general_line_2.c)) / 
        ((general_line_1.a * general_line_2.b) - (general_line_2.a * general_line_1.b));

    // point.y = ((general_line_1.a * point.x) - general_line_1.c) / general_line_1.b;
        
    return point;
}


Point FeatureExtractor::AD2Position(float dist, float angle) {

    Point point;
    point.x = RobotPos.x + (dist * cos(angle));
    point.y = RobotPos.y + (dist * sin(angle));
    point.angle = angle;
    return point;
}

// Get rid of this function eventually. No longer neded
std::vector<Point> FeatureExtractor::TransformScan(PointCloud scan) {

    std::vector<Point> points;

    for (int n = 0; n < scan.points.size(); n++) {

        Point p;
        p.x = scan.points[n][0];
        p.y = scan.points[n][1];
        points.push_back(p);
    }

    return points;
}


SlopeInterceptLine FeatureExtractor::CreateLinearModel(Point point_1, Point point_2) {

    SlopeInterceptLine slope_line;
    slope_line.m = (point_2.y - point_1.y) / (point_2.x - point_1.x);
    slope_line.b = point_2.y - (slope_line.m * point_2.x);

    return slope_line;
}



GeneralFormLine FeatureExtractor::ODRFit(std::vector<Point> laser_points) {

    // Test
    // std::cout << "Points:" << std::endl;
    // for (int i = 0; i < laser_points.size(); i++) {
    //     std::cout << laser_points[i].x << ", " << laser_points[i].y << std::endl;
    // }

    GeneralFormLine fit_line;
    fit_line.err = 0;
    int N = laser_points.size();
    float mX = 0.0;
    float mY = 0.0;
    float sXX = 0.0;
    float sYY = 0.0;
    float sXY = 0.0;
    fit_line.a = 0.0;
    fit_line.b = 0.0;
    fit_line.c = 0.0;

    
    for (int i = 0; i < N; i++) {
        
        // Calculate mX & mY
        mX += laser_points[i].x;
        mY += laser_points[i].y;
    }

    mX /= N;
    mY /= N;


    for (int i = 0; i < N; i++) {

        // sXX, sYY & sXY
        sXX = (laser_points[i].x - mX) * (laser_points[i].x - mX);
        sYY = (laser_points[i].y - mY) * (laser_points[i].y - mY);
        sXY = (laser_points[i].x - mX) * (laser_points[i].y - mY);
    }

    if (sXY == 0) {

        // Vertical Line
        if (sXX < sYY) {

            fit_line.a = 1;
            fit_line.b = 0;
            fit_line.c = mX;
        }

        // Horizontal Line
        else if (sXX > sYY) {

            fit_line.a = 0;
            fit_line.b = 1;
            fit_line.c = mY;
        }

        // Indefinite: sXX == sXY
        else {
            std::cerr << "ODRFit ERROR: Indefinite Line" << std::endl;
            fit_line.err = 1;
            return fit_line;
        }
            
    }

   
    else {

        // General Line Output  
        float  slope_m = sYY - sXX + sqrt(((sYY - sXX) * (sYY - sXX)) + (4 * (sXY * sXY))) / (2 * sXY);
        float intercept_b = mY - (slope_m * mX);
        // float norm_factor = (intercept_b >= 0.0? 1.0 : -1.0) * sqrt((slope_m * slope_m) + 1);

        fit_line.a = (-slope_m );
        fit_line.b = (1.0);
        fit_line.c = (-intercept_b);
        // std::cout << "General Fit Line: " << fit_line.a << ", " << fit_line.b << ", " << fit_line.c << std::endl;
        // std::cout << "Slope Fit Line: " << slope_m << ", " << intercept_b << std::endl;
        // std::cout << std::endl;
    }

    return fit_line;
}


Point FeatureExtractor::Get_PointPrediction(GeneralFormLine fitted_line, Point point_in_scan) {

    // Calculate the Beam Line
    SlopeInterceptLine line = CreateLinearModel(RobotPos, point_in_scan);
    GeneralFormLine general_beam = SlopeInt2General(line);

    return Get_Intersection(general_beam, fitted_line);
}


std::vector<Point> FeatureExtractor::Get_Endpoints(LineSegment line, Point point_a, Point point_b) {

    std::vector<Point> endpoints;
    endpoints.push_back(OrthogProjectPoint2Line(line, point_a));
    endpoints.push_back(OrthogProjectPoint2Line(line, point_b));

    return endpoints;
}

Point FeatureExtractor::OrthogProjectPoint2Line(LineSegment line, Point data_point) {

    SlopeInterceptLine slope_line = line.slope_fit_line;
    Point projected_point;
    projected_point.x = (data_point.x + (slope_line.m * data_point.y) - (slope_line.m * slope_line.b)) /
            ((slope_line.m * slope_line.m) + 1);

    projected_point.y = (slope_line.m * projected_point.x) + slope_line.b;

    return projected_point;
}


Point FeatureExtractor::ClampPointOnLine(LineSegment line, Point point) {

    // Clamp projected point to within range of seed segment.
    Point clamped_point;
    VectorXf segment_direction(2); segment_direction << line.endpoints[1].x - line.endpoints[0].x, line.endpoints[1].y - line.endpoints[0].y; 
    VectorXf endpoint1_to_projp(2); endpoint1_to_projp << point.x - line.endpoints[0].x, point.y - line.endpoints[0].y; 
    VectorXf endpoint2_to_projp(2); endpoint2_to_projp << point.x - line.endpoints[1].x, point.y - line.endpoints[1].y; 
    
    float d_dot_d = segment_direction.dot(segment_direction);
    float t1 = endpoint1_to_projp.dot(segment_direction) / d_dot_d;
    float t2 = endpoint2_to_projp.dot(segment_direction) / d_dot_d;
    float t = endpoint1_to_projp.dot(endpoint2_to_projp) / d_dot_d;
    float t_min = min(t1, t2);
    float t_max = max(t1, t2);

    // Clamp between t1 & t2
    float t_clamped = max(t_min, min(t_max, t));
    // float t_clamped = max(0, min(1, t));

    clamped_point.x = line.endpoints[0].x + t_clamped * (line.endpoints[1].x - line.endpoints[0].x);
    clamped_point.y = line.endpoints[0].y + t_clamped * (line.endpoints[1].y - line.endpoints[0].y);

    return clamped_point;
}


void FeatureExtractor::CheckOverlap() {

    float overlap_threshold = 0.5; // Just made up some random number
    bool match = false;

    for (int i = 0; i < NewLandmarks.size(); i++) {

        
        for (int j = 0; j < AllLandmarks.size(); j++) {

            float dist = Get_EuclideanDistance(NewLandmarks[i].position, AllLandmarks[j].position);

            if (dist < overlap_threshold) {

                // Update old landmark with new one
                AllLandmarks[j] = NewLandmarks[i];
                match = true;
                break;
            }
        }

        // Landmark never seen before, so add it to list of landmarks
        if (!match) {
            
            AllLandmarks.push_back(NewLandmarks[i]);
        }

        else {
            match = false;
        }
    }
}

LineSegment FeatureExtractor::DetectSeedSegment() {
    
    LineSegment seed_seg;
    seed_seg.err = 0;

    // Slide seed segment window
    for (int i = breakpoint_idx; i < (LaserPoints.size() - MinSeedSegNum); i++) {
        bool flag = true;

        seed_seg.start_idx = i;
        seed_seg.end_idx = i + SeedSegWindowSize;

        // Update the sliding window
        std::vector<Point> sliding_window;
        for (int pos = seed_seg.start_idx; pos < seed_seg.end_idx; pos++) {
            sliding_window.push_back(LaserPoints[pos]);
        }
        seed_seg.points = sliding_window;
        seed_seg.general_fit_line = ODRFit(seed_seg.points);
        
        // Check if all points within window satisfy as a seed segment.
        for (int k = i; k < seed_seg.end_idx; k++) {
            
            Point predicted = Get_PointPrediction(seed_seg.general_fit_line, LaserPoints[k]);
            float dist1 = Get_EuclideanDistance(predicted, LaserPoints[k]); 
            // std::cout << "Distance btw Point & Predicted: " << dist1 << " Delta: " << Delta << std::endl;  

            // Distance btw Point & Predicted Point        
            if (dist1 > Delta) {
                flag = false;
                break;
            }

            float dist2 = Get_Point2LineDistance(LaserPoints[k], seed_seg.general_fit_line);

            // Distance btw Point & Fit Line:
            if (dist2 > Epsillon) {
                flag = false;
                break;
            }  
        }

        if (flag) {
            // std::cout << "Seed Seg Size (During Detection): " << seed_seg.points.size() << std::endl;
            return seed_seg;
        }
    }

    seed_seg.err = 1;
    // std::cout << "Error Segment Returned (During Detection): " << seed_seg.points.size() << std::endl;
    return seed_seg;
}


LineSegment FeatureExtractor::GrowSeedSegment(LineSegment seed_seg) {

    // std::cout << "Growing Seed Segment---------------------------------------------" << std::endl;
    int beginning_point_index = max(breakpoint_idx, seed_seg.start_idx - 1);
    int final_point_index = min(seed_seg.end_idx + 1, LaserPoints.size() - 1);
    breakpoint_idx = min(final_point_index + 1, LaserPoints.size());
    GeneralFormLine refit;
    LineSegment error;
    error.err = 0;
    // No Seed Segment Detected. Cannot Grow
    if (seed_seg.err == 1) {
        error.err = 1;
        return error;
    }

    // Grow Right
    while (Get_Point2LineDistance(LaserPoints[final_point_index], seed_seg.general_fit_line) < Epsillon) {

        if (final_point_index >= LaserPoints.size()) 
            break;
        
        // Check for doors, windows and other other gaps
        if (Get_EuclideanDistance(LaserPoints[final_point_index], LaserPoints[final_point_index - 1]) > GapValue) {
            break;
        }

        // Refit line w/ new point Pf
        else {
            seed_seg.points.push_back(LaserPoints[final_point_index]);
            seed_seg.general_fit_line = ODRFit(seed_seg.points);
        }
        final_point_index++;
  
    }

    final_point_index--;
    
    // Grow Left
    while (Get_Point2LineDistance(LaserPoints[beginning_point_index], seed_seg.general_fit_line) < Epsillon) {

        if (beginning_point_index < 0)
            break;

        // Check for doors, windows and other other gaps
        if (Get_EuclideanDistance(LaserPoints[beginning_point_index], LaserPoints[beginning_point_index + 1]) > GapValue) {
            break;
        }

        // Refit line w/ new point Pb
        else {

            // Create subset from Pb to Pf
            std::vector<Point> subset;
            for (int i = beginning_point_index; i <= final_point_index; i++)
                subset.push_back(LaserPoints[i]);

            refit = ODRFit(subset);
            seed_seg.points = subset;
            seed_seg.general_fit_line = refit;
        }
        beginning_point_index--;
        
        
    }
    beginning_point_index++;
    
    
    seed_seg.start_idx = 0;
    seed_seg.end_idx = seed_seg.points.size() - 1;

    // Validate Seed Segment
    int line_seg_point_num = seed_seg.points.size();
    float line_seg_len = Get_EuclideanDistance(seed_seg.points[seed_seg.start_idx], seed_seg.points[seed_seg.end_idx]);

    // std::cout << "Line Seg Point Num: " << line_seg_point_num << " >= Min Seed Seg Num: " << MinSeedSegNum << "?" << std::endl;
    // std::cout << "Line Seg Len: " << line_seg_len << " >= Min Line Seg Len: " << MinLineSegLen << "?" << std::endl;
    if (line_seg_point_num >= MinSeedSegNum && line_seg_len >= MinLineSegLen) {

        seed_seg.slope_fit_line = General2SlopeInt(seed_seg.general_fit_line);
        seed_seg.endpoints = Get_Endpoints(seed_seg, seed_seg.points[seed_seg.start_idx], seed_seg.points[seed_seg.end_idx]);     
        // std::cout << "Endpoints: (" << seed_seg.endpoints[0].x << ", " <<  seed_seg.endpoints[0].y << ") (" 
        //     << seed_seg.endpoints[1].x << ", " <<  seed_seg.endpoints[1].y << ")" << std::endl;
        return seed_seg;
    }

    error.err = 1;
    // std::cout << "About to return Error Segment (After Growth)" << std::endl;
    return error;
}



Landmark FeatureExtractor::ValidationGate(LineSegment feature) {

    Landmark validated;
    validated.err = 0;

    if (feature.err == 1) {
        validated.err = 1;
        return validated;
    }
    
     
    // If Validated
    if (1 /*Nothing to Validate right now*/) {
        validated.line_seg = feature;
        return validated;
    }
    
    validated.err = 1;
    return validated;
}


void FeatureExtractor::reset() {

    AllLandmarks.clear();
    NewLandmarks.clear();
    LaserPoints.clear();
    SeedSegWindowSize = 10;
    MinLineSegLen = 0.001; // m (Very small length for test case)
    breakpoint_idx = 0;
}


// Public---------------------------------------------------------------------------------------------------------------------------------------
FeatureExtractor::FeatureExtractor(){

};


FeatureExtractor::FeatureExtractor(float delta, float epsillon, float gap_value, int min_seed_seg_num)
    : Delta(delta), Epsillon(epsillon), GapValue(gap_value), MinSeedSegNum(min_seed_seg_num) {

    LandmarkIDs = 0;
    SeedSegWindowSize = 10; 
    MinLineSegLen = 0.001; // m (Very small length for test case)
    breakpoint_idx = 0;
}


std::vector<Landmark> FeatureExtractor::LandmarksFromScan(PointCloud current_scan, VectorXf current_pose) {
    
    if (current_scan.points.size() == 0) {
        return AllLandmarks;
    }

    reset();
    LaserPoints = TransformScan(current_scan);
    LineSegment line_seg;
    Landmark landmark;
    RobotPos.x = current_pose[0];
    RobotPos.y = current_pose[1];
    RobotPos.angle = current_pose[2];

    while (breakpoint_idx < (LaserPoints.size() - MinSeedSegNum)) {
        
        landmark = ValidationGate(GrowSeedSegment(DetectSeedSegment()));

        // If landmark is valid
        if (landmark.err == 0) {

            // std::cout << "Printing Landmark Points" << std::endl;
            // for (int i = 0; i < landmark.line_seg.points.size(); i++) {
            //     std::cout << landmark.line_seg.points[i].x << ", " << landmark.line_seg.points[i].y << std::endl;
            // }
            // std::cout << "Landmark General Line: " << landmark.line_seg.general_fit_line.a << ", " 
            //     << landmark.line_seg.general_fit_line.b << ", " << landmark.line_seg.general_fit_line.c << std::endl;
            // std::cout << "Landmark Slope Line: " << landmark.line_seg.slope_fit_line.m << ", " << landmark.line_seg.slope_fit_line.b << std::endl;
            
            landmark.position = ClampPointOnLine(landmark.line_seg, OrthogProjectPoint2Line(landmark.line_seg, RobotPos));
            landmark.range = Get_EuclideanDistance(RobotPos, landmark.position);
            landmark.bearing = atan2(landmark.position.y, landmark.position.x) - RobotPos.angle;
            NewLandmarks.push_back(landmark);
            
            // std::cout << "Landmark Position: " << landmark.position.x << ", " << landmark.position.y << std::endl;
            // std::cout << std::endl;
        }
    }

    CheckOverlap();
   
    return AllLandmarks;
}



void FeatureExtractor::Set_Delta(float delta) {

    Delta = delta;
}


void FeatureExtractor::Set_Epsillon(float epsillon) {

    Epsillon = epsillon;
}


void FeatureExtractor::Set_GapValue(float gap_val) {

    GapValue = gap_val;
}


void FeatureExtractor::Set_MinSeedSegNum(int min_seed_seg_num) {

    MinSeedSegNum = min_seed_seg_num;
}


void FeatureExtractor::Set_MinLineSegLen(float min_line_seg_len) {

    MinLineSegLen = min_line_seg_len;
}




/*
 * 			TO-DO
 * 			-----
 *  - 
 *
 *  - 
 *  
 *  - 
 *  */

