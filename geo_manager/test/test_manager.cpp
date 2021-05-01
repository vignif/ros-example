#include <gtest/gtest.h>
#include <climits>
#include "geo_manager/manager.hpp"
#include <ros/init.h>
#include <ros/node_handle.h>

class ManagerFixture : public ::testing::Test
{
protected:
    ros::NodeHandle nh{};

    // Setup
    ManagerFixture()
    {
        Manager M(nh);
    }
};

int add(int a, int b)
{
    return a + b;
}

TEST(NumberCmpTest, ShouldPass)
{
    ASSERT_EQ(3, add(1, 2));
}

TEST(NumberCmpTest, ShouldFail)
{
    ASSERT_EQ(INT_MAX, add(INT_MAX, 1));
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "manager_test");

    return RUN_ALL_TESTS();
}
