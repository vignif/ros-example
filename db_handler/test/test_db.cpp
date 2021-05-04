/**
 * @file test_db.cpp
 * @author Francesco Vigni (vignif@gmail.com)
 * @brief Test the basic functionalities of the db_handler
 * @version 0.4

 * 
 * @copyright Copyright (c) 2021
 * 
 */

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
    city.city_name = "Roma";
    city.postal = 00100;
    city.region_name = "Lazio";
    city.longitude = 12.4829321;
    city.latitude = 41.8933203;
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
