#include <iostream>
#include <memory>
#include <ros/ros.h>
#include "geo_manager/AddCityToRegion.h"
#include "geo_manager/RTCityReq.h"
#include "std_msgs/String.h"
#include <map>
#include <tuple>
#include "geo_manager/region.hpp"
#include <ros/package.h>
#include <vector>
#include <string>

class Manager
{
public:
    Manager(const ros::NodeHandle &nh);
    ~Manager();
    void ShowState();

private:
    void LoadJson();
    void GetFullInfoCities();
    bool CreateCity(geo_manager::AddCityToRegion::Request &req,
                    geo_manager::AddCityToRegion::Response &res);
    void CreateCityRunTime(const geo_manager::RTCityReqPtr &req);

    std::vector<std::pair<int, std::string>> _jsonEntries; /** Vector of Postal-City from json file */
    ros::NodeHandle _nh;                                   /** Ros nodehandle */
    ros::ServiceClient _client;                            /** Ros Client asks to server for full info of a city */
    ros::Subscriber _subscriber;
    std::vector<std::pair<City, Region>> _objects; /** Store pairs City-Region*/
};