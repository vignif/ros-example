#include <iostream>
#include <memory>
#include <ros/ros.h>
#include "mypkg/AddCityToRegion.h"
#include "std_msgs/String.h"

class Manager
{
public:
    Manager(const ros::NodeHandle &nh);
    ~Manager(){};

private:
    bool CreateCity(mypkg::AddCityToRegion::Request &req,
                    mypkg::AddCityToRegion::Response &res);
    ros::NodeHandle _nh;
    ros::ServiceServer _service;
};