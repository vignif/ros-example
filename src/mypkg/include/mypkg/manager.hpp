#include <iostream>
#include <memory>
#include <ros/ros.h>
#include "mypkg/AddCityToRegion.h"
#include "std_msgs/String.h"
#include <map>

#include "mypkg/region.hpp"

class Manager
{
public:
    Manager(const ros::NodeHandle &nh);
    ~Manager(){};
    void ShowState();

private:
    bool CreateCity(mypkg::AddCityToRegion::Request &req,
                    mypkg::AddCityToRegion::Response &res);
    ros::NodeHandle _nh;
    ros::ServiceServer _service;
    std::vector<std::pair<City, Region>> _objects;
};