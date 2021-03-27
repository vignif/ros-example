#include "mypkg/manager.hpp"

Manager::Manager(const ros::NodeHandle &nh) : _nh(nh)
{
    ROS_INFO_STREAM("Created Manager");
    _service = _nh.advertiseService("CreateCity", &Manager::CreateCity, this);
}

bool Manager::CreateCity(mypkg::AddCityToRegion::Request &req,
                         mypkg::AddCityToRegion::Response &res)
{
    if (req.city_name.size() > 0 && req.region_name.size() > 0)
    {
        auto city = City(req.city_name);
        auto region = Region(req.region_name);

        _objects.push_back(std::make_pair(city, region));
        ShowState();
        return true;
    }

    return false;
}

void Manager::ShowState()
{
    ROS_INFO("State");
    for (auto obj : _objects)
    {
        ROS_INFO("City %s in Region %s", obj.first.GetName(), obj.second.GetName());
    }
}