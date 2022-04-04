#include <gtest/gtest.h>


class BUGTest : public ::testing::Test {

	protected:

		BUGTest() {

		}
			
		~BUGTest() {

		}
};

/**
 * @brief 
 *
 * **/
TEST(BUGTest, Constructor) {

	int val1, val2;
	bool condition;
	EXPECT_EQ(val1, val2);
	EXPECT_TRUE(condition);
	ASSERT_TRUE(condition);
}

/**
 * @brief
 *
 * **/
TEST(BUGTest, Bug0) {

}

/**
 * @brief
 *
 * **/
TEST(BUGTest, Bug1) {

}

/**
 * @brief
 *
 * **/
TEST(BUGTest, Bug2) {

}


/**
 * @brief
 *
 * **/
TEST(BUGTest, TangentBug) {

}















