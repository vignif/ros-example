#include <iostream>
#include <memory>
#include <ros/ros.h>
#include "mypkg/AddCityToRegion.h"
#include "std_msgs/String.h"
#include <map>
#include <tuple>
#include "mypkg/region.hpp"
#include <string>
#include <sqlite3.h>
#include <ros/package.h>
#include <vector>
#define SQLITE_OPEN_CREATE 0x00000004 /* Ok for sqlite3_open_v2() */

class Manager
{
public:
    Manager(const ros::NodeHandle &nh);
    ~Manager();
    void ShowState();

private:
    void InitDatabase();
    void InitTable();
    void LoadJson();

    bool CreateCity(mypkg::AddCityToRegion::Request &req,
                    mypkg::AddCityToRegion::Response &res);

    std::vector<std::pair<int, std::string>> _jsonEntries; /** Vector of Postal-City from json file */
    ros::NodeHandle _nh;                                   /** Ros nodehandle */
    ros::ServiceServer _service;                           /** Ros server checks if the user wants to add a new city */
    std::vector<std::pair<City, Region>> _objects;         /** Store pairs City-Region*/

    sqlite3 *_db;       /** Pointer to SQLite connection */
    char *_zErrMsg = 0; /** Save any error messages */
    int _rc;            /** Database connection object */
};