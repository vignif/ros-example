#include <gtest/gtest.h>
#include <climits>
#include "db_handler/db_handler.hpp"
#include <ros/init.h>
#include <ros/node_handle.h>
#include <log4cxx/logger.h>
#include <memory>

class DatabaseFixture : public ::testing::Test
{
protected:
    ros::NodeHandle _nh{};
    DatabaseHandler _db;

    // Setup
    DatabaseFixture() : _db(_nh) {}
};

/**
 * @brief If databasehandler is initialized correctly
 * the member has value true 
 */
TEST_F(DatabaseFixture, DBisInitializedCorrectly)
{
    ASSERT_TRUE(_db._initOK == true);
}

/**
 * @brief Add new city to the db
 */
TEST_F(DatabaseFixture, InsertCity)
{
    shared_msgs::CityInfo city;
    auto res = _db.InsertCity(city);
    ASSERT_TRUE(res);
}

/**
 * @brief Return all cities stored in the db
 */
TEST_F(DatabaseFixture, CheckCities)
{
    auto cities = _db.GetCities();
    auto res = cities.size();
    ASSERT_GE(res, 1); // there is at least one city in the db
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "Database_test");
    ros::NodeHandle nh{"~"};

    return RUN_ALL_TESTS();
}
