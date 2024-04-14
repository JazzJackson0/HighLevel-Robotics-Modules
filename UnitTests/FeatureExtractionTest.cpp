#include <iostream>
#include <gtest/gtest.h>
#include <gmock/gmock.h> // Vscode error
#include "FeatureExtraction.hpp"

// For gmock
using ::testing::AtLeast;
using ::testing::Return;
using ::testing::_;

// THIS ACTUALLY DOES NOT NEED MOCKING. I JUST WANTED TO PLAY AROUND WITH IT
// Change it back to normal tests

class MockFeatureExtractor : public FeatureExtractor {

	public:
		// Set up a mock of the constructor
		//MockFeatureExtractor() : FeatureExtractor(1, 2, 40) {}

		// Private Functions
		// VSCODE ISSUE: THERE IS NO REAL ERROR!!!!
		MOCK_METHOD(float, Get_EuclideanDistance, (Point point_a, Point point_b), (override));
		MOCK_METHOD(float, Get_Point2LineDistance, (Point point, GeneralFormLine general_line), (override));
		MOCK_METHOD(vector<Point>, Get_2PointsFromLine, (int x1, int x2, SlopeInterceptLine slope_line), (override));
		MOCK_METHOD(GeneralFormLine, SlopeInt2General, (SlopeInterceptLine slope_line), (override));
		MOCK_METHOD(SlopeInterceptLine, General2SlopeInt, (GeneralFormLine general_line), (override));
		MOCK_METHOD(Point, Get_Intersection, (GeneralFormLine general_line_1, GeneralFormLine general_line_2), (override));
		MOCK_METHOD(Point, AD2Position, (float dist, float angle), (override));
		MOCK_METHOD(vector<Point>, TransformScan, (vector<VectorXf> scan), (override));
		MOCK_METHOD(SlopeInterceptLine, CreateLinearModel, (Point point_1, Point point_2), (override));
		MOCK_METHOD(GeneralFormLine, ODRFit, (vector<Point> laser_points), (override));
		MOCK_METHOD(Point, Get_PointPrediction, (GeneralFormLine fitted_line, Point point_in_scan), (override));
		MOCK_METHOD(vector<Point>, Get_Endpoints, (GeneralFormLine line, Point point_a, Point point_b), (override));
		MOCK_METHOD(Landmark, ValidationGate, (LineSegment feature), (override));
		MOCK_METHOD(Point, OrthogProjectPoint2Line, (SlopeInterceptLine slope_line, Point data_point), (override));
		MOCK_METHOD(void, CheckOverlap, (), (override));
		MOCK_METHOD(void, reinit, (), (override));

		// Public Functions
		// VSCODE ISSUE: THERE IS NO REAL ERROR!!!!!
		MOCK_METHOD(vector<Landmark>, LandmarksFromScan, (vector<VectorXf> current_scan), (override));
		MOCK_METHOD(LineSegment, DetectSeedSegment, (int num_of_points), (override));
		MOCK_METHOD(LineSegment, GrowSeedSegment, (LineSegment seed_seg), (override));
		MOCK_METHOD(void, Set_Delta, (float delta), (override));
		MOCK_METHOD(void, Set_Epsillon, (float epsillon), (override));
		MOCK_METHOD(void, Set_GapValue, (float gap_val), (override));
		MOCK_METHOD(void, Set_MinSeedSegNum, (int min_seed_seg_num), (override));
		MOCK_METHOD(void, Set_MinLineSegNum, (int min_line_seg_num), (override));
		MOCK_METHOD(void, Set_MinLineSegLen, (float min_line_seg_len), (override));

		// Non-Mocked Versions (To Actually Test) -> Only works for Public Functions (Which is expected)
		vector<Landmark> RealLandmarksFromScan(vector<VectorXf> current_scan) { return FeatureExtractor::LandmarksFromScan(current_scan); }
		LineSegment RealDetectSeedSegment(int num_of_points) { return FeatureExtractor::DetectSeedSegment(num_of_points); }
		LineSegment RealGrowSeedSegment(LineSegment seed_seg) { return FeatureExtractor::GrowSeedSegment(seed_seg); }
}; 

/**
 *	@brief 
 *
 * **/
TEST(FeatureExtractionTest, RealLandmarksFromScan1) {
	
	MockFeatureExtractor mfe;

	EXPECT_CALL(mfe, ValidationGate)
		.WillRepeatedly(Return());

	EXPECT_CALL(mfe, GrowSeedSegment)
		.Times(1)
		.WillRepeatedly(Return());

	vector<VectorXf> current_scan;
	vector<Landmark> result;

	EXPECT_FLOAT_EQ(25.0, mfe.RealLandmarksFromScan(current_scan)[0].bearing);
	EXPECT_FLOAT_EQ(25.0, mfe.RealLandmarksFromScan(current_scan)[0].id);
	// ...
	// ...
}


/**
 *	@brief 
 *
 * **/
TEST(FeatureExtractionTest, DetectSeedSegment1) {
	

}


/**
 *	@brief 
 *
 * **/
TEST(FeatureExtractionTest, GrowSeedSegment1) {
	

}

// Maybe Not going to test the Setters??






// class FeatureExtractionTest : public ::testing::Test {

// 	protected:

// 		FeatureExtractionTest() {

// 		}
			
// 		//~FeatureExtractionTest() {}

// };


// /**
//  *	@brief 
//  *
//  * **/
// TEST_F(FeatureExtractionTest, GrowSeedSegment1) {
	

// }


// /**
//  *	@brief 
//  *
//  * **/
// TEST_F(FeatureExtractionTest, LandmarksFromScan1) {
	

// }


// /**
//  *	@brief 
//  *
//  * **/
// TEST_F(FeatureExtractionTest, DetectSeedSegment1) {
	

// }






