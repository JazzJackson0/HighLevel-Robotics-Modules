#include "FeatureExtraction.hpp"

// Private -------------------------------------------------------------------------------------------------------------------

float FeatureExtractor::Get_EuclideanDistance(Point point_a, Point point_b) {

    return sqrt(pow((point_b.x - point_a.x), 2) + pow((point_b.y - point_a.y), 2));
}

float FeatureExtractor::Get_Point2LineDistance(Point point, GeneralFormLine general_line) {

    return abs((general_line.a * point.x) + (general_line.b * point.y) + general_line.c) / 
        sqrt((general_line.a * general_line.a) + general_line.b * general_line.b);

}

std::vector<Point> FeatureExtractor::Get_2PointsFromLine(int x1, int x2, SlopeInterceptLine slope_line) {

    std::vector<Point> points;
    Point point_a;
    Point point_b;

    point_a.x = x1;
    point_a.y = (slope_line.m * x1) + slope_line.b;
    points.push_back(point_a);

    point_b.x = x2;
    point_b.y = (slope_line.m * x2) + slope_line.b;
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

    if (general_line.a < 0) {
        general_line.a *= -1;
        general_line.b *= -1;
        general_line.c *= -1;
    }

    // Pull out fraction part of A 
    fractionPart = modf(general_line.a, &integerPart);

    // Calculate Simplified Denominator for A 
    numerator = (int)fractionPart * denominator_limit;
    int denominator_a = denominator_limit;
    div = gcd(numerator, denominator_limit);
    denominator_a /= div; // (This denom calculated for the fraction part is the same as it would be for the entire number)

    // Pull out fraction part of C 
    fractionPart = modf(general_line.c, &integerPart);

    // Calculate Simplified Denominator for C 
    numerator = (int)fractionPart * denominator_limit;
    int denominator_c = denominator_limit;
    div = gcd(numerator, denominator_limit);
    denominator_c /= div; // (This denom calculated for the fraction part is the same as it would be for the entire number)

    // Calculate the LCM
    int lcm = abs((denominator_a * denominator_c)) / gcd(denominator_a, denominator_c);

    // Update Line Params
    general_line.a *= lcm;
    general_line.b *= lcm;
    general_line.c *= lcm;

    return general_line;
}

SlopeInterceptLine FeatureExtractor::General2SlopeInt(GeneralFormLine general_line) {

    SlopeInterceptLine slope_line;
    slope_line.m = -(general_line.a / general_line.b);
    slope_line.b = -(general_line.a / general_line.b);

    return slope_line;
}


Point FeatureExtractor::Get_Intersection(GeneralFormLine general_line_1, GeneralFormLine general_line_2) {

    Point point;

    // Switched the subtraction order in the denominators so the sign of the output point match the actual point
    point.x = ((general_line_1.b * general_line_2.c) - (general_line_2.b * general_line_1.c)) / 
        ((general_line_2.a * general_line_1.b) - (general_line_1.a * general_line_2.b));
    point.y = ((general_line_2.a * general_line_1.c) - (general_line_1.a * general_line_2.c)) / 
        ((general_line_2.a * general_line_1.b) - (general_line_1.a * general_line_2.b));
        
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
            std::cerr << "ODRFit Error: Indefinite Line" << std::endl;
            fit_line.err = 1;
            return fit_line;
        }
            
    }

   
    else {

        // General Line Output
        
        float  slope_m = sYY - sXX + sqrt(pow((sYY - sXX), 2) + (4 * (sXY * sXY))) / (2 * sXY);
        float intercept_b = mY - (slope_m * mX);
        float norm_factor = (intercept_b >= 0.0? 1.0 : -1.0) * sqrt((slope_m * slope_m) + 1);

        fit_line.a = (-slope_m / norm_factor);
        fit_line.b = (1.0 / norm_factor);
        fit_line.c = (intercept_b / norm_factor);
    }


    return fit_line;
}


Point FeatureExtractor::Get_PointPrediction(GeneralFormLine fitted_line, Point point_in_scan) {

    // Calculate the Beam Line
    SlopeInterceptLine line = CreateLinearModel(point_in_scan, RobotPos);
    GeneralFormLine general_beam = SlopeInt2General(line);

    return Get_Intersection(fitted_line, general_beam);
}


std::vector<Point> FeatureExtractor::Get_Endpoints(GeneralFormLine line, Point point_a, Point point_b) {

    std::vector<Point> endpoints;
    endpoints.push_back(OrthogProjectPoint2Line(General2SlopeInt(line), point_a));
    endpoints.push_back(OrthogProjectPoint2Line(General2SlopeInt(line), point_b));

    return endpoints;
}

Point FeatureExtractor::OrthogProjectPoint2Line(SlopeInterceptLine slope_line, Point data_point) {

    
    Point intersect_point;
    intersect_point.x = (data_point.x+ (slope_line.m*data_point.y) - (slope_line.m*slope_line.b)) /
            ((slope_line.m * slope_line.m) + 1);

    intersect_point.y = (slope_line.m * intersect_point.x) + slope_line.b;

    return intersect_point;
}


void FeatureExtractor::CheckOverlap() {

    float overlap_threshold = 0.5; // Just made up some bullshit number
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

    // std::cout << "Num of Points: " << num_of_points << std::endl;
    // std::cout << "Min Seed Seg Num: " << MinSeedSegNum << std::endl;
    // std::cout << "Detecting Seed Segment----------------------------------------------" << std::endl;
    
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
        seed_seg.line_fit = ODRFit(seed_seg.points);
        //std::cout << "Fit Line (During Detection): " << seed_seg.line_fit.a << "A + " << seed_seg.line_fit.b << "B + " << seed_seg.line_fit.c << "C" << std::endl;

        // Validate the seed segment window
        for (int k = i; k < seed_seg.end_idx; k++) {

            Point predicted = Get_PointPrediction(seed_seg.line_fit, LaserPoints[k]);
            // std::cout << "Laser Point: (" << LaserPoints[k].x << ", " << LaserPoints[k].y << ") Predicted Point: (" 
            //     << predicted.x << ", " << predicted.y << ")" << std::endl;
            float dist1 = Get_EuclideanDistance(predicted, LaserPoints[k]); 
            //std::cout << "Distance btw Point & Predicted: " << dist1 << " Delta: " << Delta << std::endl;           
            if (dist1 > Delta) {
                flag = false;
                break;
            }

            float dist2 = Get_Point2LineDistance(LaserPoints[k], seed_seg.line_fit);
            //std::cout << "Distance btw Point & Fit Line: " << dist2 << " Epsillon: " << Epsillon << std::endl; 
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
    std::cout << "Error Segment Returned (During Detection): " << seed_seg.points.size() << std::endl;
    return seed_seg;
}


LineSegment FeatureExtractor::GrowSeedSegment(LineSegment seed_seg) {

    // std::cout << "Growing Seed Segment---------------------------------------------" << std::endl;
    
    int beginning_point_index = max(breakpoint_idx, seed_seg.start_idx - 1);
    int final_point_index = min(seed_seg.end_idx + 1, LaserPoints.size() - 1);
    //std::cout << "P_b: " << beginning_point_index << " P_f: " << final_point_index << std::endl;
    //std::cout << "Seed Seg Start: " << seed_seg.start_idx - 1 <<  " Breakpoint: " << breakpoint_idx << std::endl;
    GeneralFormLine refit;
    LineSegment error;
    error.err = 0;

    if (seed_seg.err == 1) {
        std::cout << "No Seed Segment Detected. Cannot Grow" << std::endl;
        error.err = 1;
        return error;
    }

    // std::cout << "Growing Right" << std::endl;
    // Grow Right
    while (Get_Point2LineDistance(LaserPoints[final_point_index], seed_seg.line_fit) < Epsillon) {
        // std::cout << "[RIGHT] Point-2-FitLine Dist: " << Get_Point2LineDistance(LaserPoints[final_point_index], seed_seg.line_fit) << "m   e = " << Epsillon << std::endl;

        if (final_point_index >= LaserPoints.size()) 
            break;
        
        // Check for doors, windows and other other gaps
        if (Get_EuclideanDistance(LaserPoints[final_point_index], LaserPoints[final_point_index - 1]) > GapValue) {
            break;
        }

        // Refit line w/ new point Pf
        else {
            seed_seg.points.push_back(LaserPoints[final_point_index]);
            seed_seg.line_fit = ODRFit(seed_seg.points);
            //std::cout << "[RIGHT] New Fit Made. Seg Size: " << seed_seg.points.size() << std::endl;
        }
        final_point_index++;
  
    }

    final_point_index--;
    
    // std::cout << "Growing Left" << std::endl;
    // Grow Left
    while (Get_Point2LineDistance(LaserPoints[beginning_point_index], seed_seg.line_fit) < Epsillon) {
        
        // std::cout << "[LEFT] Point-2-FitLine Dist: " << Get_Point2LineDistance(LaserPoints[beginning_point_index], seed_seg.line_fit) << "m   e = " << Epsillon << std::endl;

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
            seed_seg.line_fit = refit;
            //std::cout << "[LEFT] New Fit Made. Seg Size: " << seed_seg.points.size() << std::endl;
        }
        beginning_point_index--;
        
        
    }
    beginning_point_index++;
    
    
    seed_seg.start_idx = 0;
    seed_seg.end_idx = seed_seg.points.size() - 1;

    // std::cout << "Start Index: " << seed_seg.start_idx << " End Index: " << seed_seg.end_idx << std::endl;
    // std::cout << "Start: (" << seed_seg.points[seed_seg.start_idx].x << ", " << seed_seg.points[seed_seg.start_idx].y 
    //     <<") End: (" << seed_seg.points[seed_seg.end_idx].x << ", " << seed_seg.points[seed_seg.end_idx].y << ")" << std::endl;

    // Validate Seed Segment
    int line_seg_point_num = seed_seg.points.size();
    float line_seg_len = Get_EuclideanDistance(seed_seg.points[seed_seg.start_idx], seed_seg.points[seed_seg.end_idx]);

    // std::cout << "Num Points: " << line_seg_point_num << " Seed Line Len: " << line_seg_len << std::endl;
    if (line_seg_point_num >= MinSeedSegNum && line_seg_len >= MinLineSegLen) {

        breakpoint_idx = min(final_point_index + 1, LaserPoints.size());
        // std::cout << "Seed Seg Size (After Growth): " << seed_seg.points.size() << std::endl;
        seed_seg.endpoints = Get_Endpoints(seed_seg.line_fit, seed_seg.points[seed_seg.start_idx], seed_seg.points[seed_seg.end_idx]);
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
        validated.points = feature.points;
        validated.line = feature.line_fit;
        return validated;
    }
    
    validated.err = 1;
    return validated;
}


void FeatureExtractor::reset() {

    AllLandmarks.clear();
    NewLandmarks.clear();
    LaserPoints.clear();
    //PredictedPoints.clear();
    SeedSegWindowSize = 15;
    MinLineSegLen = 0.001; // m (Very small length for test case)
    breakpoint_idx = 0;
}


// Public---------------------------------------------------------------------------------------------------------------------------------------
FeatureExtractor::FeatureExtractor(){

};


FeatureExtractor::FeatureExtractor(float delta, float epsillon, float gap_value, int min_seed_seg_num)
    : Delta(delta), Epsillon(epsillon), GapValue(gap_value), MinSeedSegNum(min_seed_seg_num) {

    LandmarkIDs = 0;
    SeedSegWindowSize = 15; 
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

        // std::cout << "\n";
        
        landmark = ValidationGate(GrowSeedSegment(DetectSeedSegment()));

        // If landmark is valid
        if (landmark.err == 0) {
            
            Point origin;
            origin.x = 0;
            origin.y = 0;
            origin.angle = 0;
            landmark.position = OrthogProjectPoint2Line(General2SlopeInt(landmark.line), origin);
            NewLandmarks.push_back(landmark);

            landmark.range = Get_EuclideanDistance(RobotPos, landmark.position);
            landmark.bearing = atan2(landmark.position.y, landmark.position.x) - RobotPos.angle;

            // std::cout << "NEW LANDMARK!!!!!!!!!!!!" << std::endl;
            // std::cout << "Position: " << landmark.position.x << ", " << landmark.position.y  << std::endl;
            // std::cout << "Line: " << landmark.line.a << "A + " << landmark.line.b << "B + " << landmark.line.c << "C" << std::endl;
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

