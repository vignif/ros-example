#include "geo_manager/manager.hpp"
#include <jsoncpp/json/json.h>
#include <fstream>

Manager::Manager(const ros::NodeHandle &nh) : _nh(nh)
{
    ROS_INFO_STREAM("Created Manager");
    _client = _nh.serviceClient<shared_msgs::AddCityToRegion>("/CreateCity");
    _subscriber = _nh.subscribe("/RTCreateCity", 1, &Manager::CreateCityRunTime, this);
    _db = std::move(std::make_unique<DatabaseHandler>(_nh));
    LoadJson();
    InitDBwithCities();
}

Manager::~Manager()
{
    _db->~DatabaseHandler();
    ROS_ERROR("Destroy Manager");
}

void Manager::CreateCityRunTime(const shared_msgs::RTCityReqPtr &req)
{
    shared_msgs::AddCityToRegion srv;
    srv.request.city_name = req->city_name;
    srv.request.postal = req->postal;
    _client.waitForExistence();
    if (_client.call(srv))
    {
        ROS_DEBUG("Insert City %s", srv.response.city.city_name.c_str());
        CreateCity(srv.response.city);
    }
    else
    {
        ROS_ERROR_STREAM("failed");
    }
}

void Manager::InitDBwithCities()
{
    for (auto i : _jsonEntries)
    {
        auto postal = i.first;
        auto name = i.second;
        shared_msgs::AddCityToRegion srv;
        srv.request.city_name = name;
        srv.request.postal = postal;
        _client.waitForExistence();

        if (_client.call(srv))
        {
            CreateCity(srv.response.city);
        }
        else
        {
            ROS_ERROR_STREAM("failed");
        }
    }
    ShowState();
}

bool Manager::CreateCity(const shared_msgs::CityInfo &city)
{
    _cities.push_back(City{city});
    if (_db->InsertCity(city))
    {
        return true;
    }
    return false;
}

void Manager::LoadJson()
{
    Json::Value root;
    Json::Reader reader;
    auto path = ros::package::getPath("geo_manager");
    std::string jsonName = "/data/cities.json";

    auto fullpath = path + jsonName;

    std::ifstream ifs(fullpath);

    if (!reader.parse(ifs, root))
    {
        ROS_ERROR("failed to parse");
    }

    Json::Value val = root["cities"];
    for (auto i : val)
    {
        auto name = i["name"].asString();
        auto postal = i["postal"].asInt();
        _jsonEntries.push_back(std::make_pair(postal, name));
    }
}

void Manager::ShowState()
{
    ROS_DEBUG("Current DB entries:");
    for (auto obj : _cities)
    {
        ROS_INFO("City %s \tin Coordinates %s", obj.GetName(), obj.GetCoordinates().c_str());
    }
}