#include <iostream>
#include <memory>
#include <ros/ros.h>
#include "geo_manager/AddCityToRegion.h"
#include "geo_manager/RTCityReq.h"
#include "std_msgs/String.h"
#include <map>
#include <tuple>
#include "geo_manager/region.hpp"
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
    void GetFullInfoCities();
    void InsertCity(geo_manager::AddCityToRegion::Response &res);
    bool CreateCity(geo_manager::AddCityToRegion::Request &req,
                    geo_manager::AddCityToRegion::Response &res);
    void CreateCityRunTime(const geo_manager::RTCityReqPtr &req);

    std::vector<std::pair<int, std::string>> _jsonEntries; /** Vector of Postal-City from json file */
    ros::NodeHandle _nh;                                   /** Ros nodehandle */
    ros::ServiceClient _client;                            /** Ros Client asks to server for full info of a city */
    ros::Subscriber _subscriber;
    std::vector<std::pair<City, Region>> _objects; /** Store pairs City-Region*/

    sqlite3 *_db;       /** Pointer to SQLite connection */
    char *_zErrMsg = 0; /** Save any error messages */
    int _rc;            /** Database connection object */
};