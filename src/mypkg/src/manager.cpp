#include "mypkg/manager.hpp"

Manager::Manager(const ros::NodeHandle &nh) : _nh(nh)
{
    ROS_INFO_STREAM("Created Manager");
    _service = _nh.advertiseService("CreateCity", &Manager::CreateCity, this);

    auto path = ros::package::getPath("mypkg");
    auto fullpath = path + std::string("/test.db");

    ROS_DEBUG_STREAM("Database located in: " << fullpath);

    _rc = sqlite3_open(fullpath.c_str(), &_db);

    if (_rc)
    {
        ROS_ERROR("Can't open database: %s\n", sqlite3_errmsg(_db));
    }
    else
    {
        ROS_DEBUG("Opened database successfully\n");
    }
}

Manager::~Manager()
{
    ROS_DEBUG("Closing DB");
    sqlite3_close(_db);
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
    for (auto obj : _objects)
    {
        ROS_INFO("City %s in Region %s", obj.first.GetName(), obj.second.GetName());
    }
}