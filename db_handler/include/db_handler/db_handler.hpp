#ifndef DB_HANDLER_LIBRARY_H
#define DB_HANDLER_LIBRARY_H

#include <sqlite3.h>
#define SQLITE_OPEN_CREATE 0x00000004 /* Ok for sqlite3_open_v2() */
#include <iostream>
#include <memory>
#include <ros/ros.h>
#include "shared_msgs/AddCityToRegion.h"
#include "shared_msgs/CityInfo.h"

#include "std_msgs/String.h"
#include <map>
#include <tuple>
#include <ros/package.h>
#include <vector>
#include <string>

class DatabaseHandler
{
public:
    DatabaseHandler(const ros::NodeHandle &nh);
    ~DatabaseHandler();
    bool InsertCity(const shared_msgs::CityInfo &city);
    std::vector<shared_msgs::CityInfo> GetCities();

private:
    void InitDatabase();
    void InitTable();
    std::vector<shared_msgs::CityInfo> _cities;
    static int CallbackGetCities(void *NotUsed, int argc, char **argv, char **azColName);

    ros::NodeHandle _nh; /** Ros nodehandle */
    sqlite3 *_db;        /** Pointer to SQLite connection */
    char *_zErrMsg = 0;  /** Save any error messages */
    int _rc;             /** Database connection object */
};

#endif