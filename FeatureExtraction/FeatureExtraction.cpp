#include "FeatureExtraction.hpp"



float FeatureExtractor::Get_EuclideanDistance(Point point_a, Point point_b) {

    return sqrt(pow((point_b.x - point_a.x), 2) + pow((point_b.y - point_a.y), 2));
}

float FeatureExtractor::Get_Point2LineDistance(Point point, GeneralFormLine general_line) {

    return abs((general_line.a * point.x) + (general_line.b * point.y) + general_line.c) / 
        sqrt((general_line.a * general_line.a) + general_line.b * general_line.b);

}

vector<Point> FeatureExtractor::Get_2PointsFromLine(int x1, int x2, SlopeInterceptLine slope_line) {

    vector<Point> points;
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
    int numerator = (int)fractionPart * denominator_limit;
    int denominator_a = denominator_limit;
    int div = gcd(numerator, denominator_limit);
    denominator_a /= div; // (This denom calculated for the fraction part is the same as it would be for the entire number)

    // Pull out fraction part of C 
    fractionPart = modf(general_line.c, &integerPart);

    // Calculate Simplified Denominator for C 
    int numerator = (int)fractionPart * denominator_limit;
    int denominator_c = denominator_limit;
    int div = gcd(numerator, denominator_limit);
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
    point.x = ((general_line_1.b * general_line_2.c) - (general_line_2.b * general_line_1.c)) / 
        ((general_line_1.a * general_line_2.b) - (general_line_2.a * general_line_1.b));
    point.y = ((general_line_2.a * general_line_1.c) - (general_line_1.a * general_line_2.c)) / 
        ((general_line_1.a * general_line_2.b) - (general_line_2.a * general_line_1.b));

    return point;
}


Point FeatureExtractor::AD2Position(float dist, float angle) {

    Point point;
    point.x = RobotPos.x + (dist * cos(angle));
    point.y = RobotPos.y + (dist * sin(angle));
    point.angle = angle;

    return point;
}


vector<Point> FeatureExtractor::TransformScan(vector<VectorXf> scan) {

    vector<Point> points;

    for (int n = 0; n < scan.size(); n++) {

        points.push_back(AD2Position(scan[n](0), scan[n](1)));
    }

    return points;
}


SlopeInterceptLine FeatureExtractor::CreateLinearModel(Point point_1, Point point_2) {

    SlopeInterceptLine slope_line;
    slope_line.m = (point_2.y - point_1.y) / (point_2.x - point_1.x);
    slope_line.b = point_2.y - (slope_line.m * point_2.x);

    return slope_line;
}



GeneralFormLine FeatureExtractor::ODRFit(vector<Point> laser_points) {

    GeneralFormLine fit_line;
    int N = laser_points.size();
    float mX = 0.0;
    float mY = 0.0;
    float sXX = 0.0;
    float sYY = 0.0;
    float sXY = 0.0;
    fit_line.a = 0.0;
    fit_line.b = 0.0;
    fit_line.c = 0.0;

    // Calculate mX & mY
    for (int i = 0; i < N; i++) {

        mX += laser_points[i].x / N;
        mY += laser_points[i].y / N;
    }

    // sXX, sYY & sXY
    for (int i = 0; i < N; i++) {

        sXX = (laser_points[i].x - mX) * (laser_points[i].x - mX);
        sYY = (laser_points[i].y - mY) * (laser_points[i].y - mY);
        sXY = (laser_points[i].x - mX) * (laser_points[i].y - mY);
    }

    if (sXY == 0) {

        // Vertical Line
        if (sXX < sXY) {

            fit_line.a = 1;
            fit_line.b = 0;
            fit_line.c = mX;
        }

        // Horizontal Line
        else if (sXX > sXY) {

            fit_line.a = 0;
            fit_line.b = 1;
            fit_line.c = mY;
        }

        // Indefinite: sXX == sXY
        else
            return NULL;
    }

   
    else {

        float slope = mY - (fit_line.b * mX);
        float intercept = sYY - sXX + sqrt(pow((sYY - sXX), 2) + (4 * (sXY * sXY))) / (2 * sXY);
        float norm_factor = (intercept >= 0.0? 1.0 : -1.0) * sqrt((slope * slope) + 1);

        fit_line.a = (-slope / norm_factor);
        fit_line.b = (1.0 / norm_factor);
        fit_line.c = (intercept / norm_factor);
    }


    return fit_line;
}


Point FeatureExtractor::Get_PointPrediction(GeneralFormLine fitted_line, Point point_in_scan) {

    // Calculate the Beam Line
    SlopeInterceptLine line = CreateLinearModel(point_in_scan, RobotPos);
    GeneralFormLine general_beam = SlopeInt2General(line);

    return Get_Intersection(fitted_line, general_beam);
}


vector<Point> FeatureExtractor::Get_Endpoints(GeneralFormLine line, Point point_a, Point point_b) {

    vector<Point> endpoints;
    endpoints.push_back(OrthogProjectPoint2Line(General2SlopeInt(line), point_a));
    endpoints.push_back(OrthogProjectPoint2Line(General2SlopeInt(line), point_b));

    return endpoints;
}


Landmark FeatureExtractor::ValidationGate(LineSegment feature) {

    Landmark validated;
     
    // If Validated
    if (1 /*Nothing to Validate right now*/) {
        validated.landmark_points = feature.line_points;
        validated.landmark_line = feature.line_fit;
        validated.landmark_id = LandmarkIDs++;
        return validated;
    }
    
    return NULL;
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

            float dist = Get_EuclideanDistance(NewLandmarks[i].landmark_position, AllLandmarks[j].landmark_position);

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


void FeatureExtractor::reinit() {

    AllLandmarks.clear();
    NewLandmarks.clear();
    LaserPoints.clear();
    //PredictedPoints.clear();
    CurrentPointsInLineSeg = 0;
    breakpoint_idx = 0;
}



FeatureExtractor::FeatureExtractor(float delta, float epsillon, int min_seed_seg_num) {

    LandmarkIDs = 0;
    Delta = delta;
    Epsillon = epsillon; 
    MinSeedSegNum = min_seed_seg_num;
    CurrentPointsInLineSeg = 0;
    breakpoint_idx = 0;
}


vector<Landmark> FeatureExtractor::LandmarksFromScan(vector<VectorXf> current_scan) {

    reinit();
    int num_of_points = current_scan.size();
    LaserPoints = TransformScan(current_scan);
    LineSegment line_seg;
    Landmark landmark;

    while (breakpoint_idx < (num_of_points - MinSeedSegNum)) {

        CurrentPointsInLineSeg = 0;
        landmark = ValidationGate(GrowSeedSegment(DetectSeedSegment(num_of_points)));

        Point origin;
        origin.x = 0;
        origin.y = 0;
        origin.angle = 0;
        landmark.landmark_position = OrthogProjectPoint2Line(General2SlopeInt(landmark.landmark_line), origin);
        NewLandmarks.push_back(landmark);
    }

    CheckOverlap();
   
    return AllLandmarks;
}


LineSegment FeatureExtractor::DetectSeedSegment(int num_of_points) {

    bool flag = false;
    LineSegment seed_seg;
    seed_seg.start_idx = breakpoint_idx;

    for (int i = seed_seg.start_idx; i < (num_of_points - MinSeedSegNum); i++) {

        seed_seg.end_idx = i + CurrentPointsInLineSeg;
        
        seed_seg.line_fit = ODRFit(LaserPoints);

        for (int k = i; k < seed_seg.end_idx; k++) {

            Point predicted = Get_PointPrediction(seed_seg.line_fit, LaserPoints[k]);
            float dist1 = Get_EuclideanDistance(predicted, LaserPoints[k]);            
            if (dist1 > Delta) {
                flag = false;
                break;
            }

            float dist2 = Get_Point2LineDistance(LaserPoints[k], seed_seg.line_fit);
            if (dist2 > Epsillon) {
                flag = false;
                break;
            }

            seed_seg.line_points.push_back(LaserPoints[k]);
            CurrentPointsInLineSeg++;
            //PredictedPoints.push_back(predicted);
        }

        if (flag) {
            
            return seed_seg;
        }
    }

    return NULL;
}


LineSegment FeatureExtractor::GrowSeedSegment(LineSegment seed_seg) {

    int line_seg_len = 0;
    int beginning_point_index = max(breakpoint_idx, seed_seg.start_idx - 1);
    int final_point_index = min(seed_seg.end_idx + 1, LaserPoints.size());
    GeneralFormLine refit;
    

    while (Get_Point2LineDistance(LaserPoints[final_point_index], seed_seg.line_fit) < Epsillon) {

        if (final_point_index > CurrentPointsInLineSeg) 
            break;

        // Refit Line with the new point Pf
        else {
            
            // Create subset from Pb to Pf
            vector<Point> subset;
            for (int i = beginning_point_index; i <= final_point_index; i++)
                subset.push_back(LaserPoints[i]);

            refit = ODRFit(subset);
        }
        final_point_index++;
        
        // Check for doors, windows, etc
        if (Get_EuclideanDistance(LaserPoints[final_point_index], LaserPoints[final_point_index - 1]) > GapValue)
            break;
    }

    final_point_index--;
    


    while (Get_Point2LineDistance(LaserPoints[beginning_point_index], seed_seg.line_fit) < Epsillon) {

        if (beginning_point_index < 1)
            break;

        else {

            // Create subset from Pb to Pf
            vector<Point> subset;
            for (int i = beginning_point_index; i <= final_point_index; i++)
                subset.push_back(LaserPoints[i]);

            refit = ODRFit(subset);
        }
        beginning_point_index--;
        
        if (Get_EuclideanDistance(LaserPoints[beginning_point_index], LaserPoints[beginning_point_index + 1]) > GapValue)
            break;
    }
    beginning_point_index++;


    // Create Final Line
    vector<Point> subset;
    for (int i = beginning_point_index; i <= final_point_index; i++)
        subset.push_back(LaserPoints[i]);
    
    // Turn Seed Segment into the grown Line Segment
    seed_seg.line_points = subset;
    seed_seg.line_fit = refit;
    seed_seg.start_idx = beginning_point_index;
    seed_seg.end_idx = final_point_index;

    CurrentPointsInLineSeg = subset.size();
    line_seg_len = Get_EuclideanDistance(LaserPoints[beginning_point_index], LaserPoints[final_point_index]);

    if (CurrentPointsInLineSeg <= MinLineSegNum && line_seg_len <= MinLineSegLen) {

        breakpoint_idx = min(final_point_index + 1, LaserPoints.size());
        return seed_seg;
    }

    return NULL;
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


void FeatureExtractor::Set_MinLineSegNum(int min_line_seg_num) {

    MinLineSegNum = min_line_seg_num;
}


void FeatureExtractor::Set_MinLineSegLen(float min_line_seg_len) {

    MinLineSegLen = min_line_seg_len;
}




/*
 * 			TO-DO
 * 			-----
 *  - Completely Untested. Not sure if I should return pointers or not.
 *
 *  - Test Code
 *  
 *  - 
 *  */

