#include <gtest/gtest.h>
#include <climits>
#include "geo_manager/manager.hpp"
#include <ros/init.h>
#include <ros/node_handle.h>

class ManagerFixture : public ::testing::Test
{
protected:
    ros::NodeHandle nh{};
    Manager M;
    // Setup
    ManagerFixture() : M(nh)
    {
    }
};

TEST_F(ManagerFixture, ShouldPass)
{
    ASSERT_TRUE(1 == 1);
}

TEST_F(ManagerFixture, ShouldFail)
{
    ASSERT_FALSE(1 == 0);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "manager_test");

    return RUN_ALL_TESTS();
}
