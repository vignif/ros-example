#include "geo_manager/manager.hpp"
#include <fstream>
#include <jsoncpp/json/json.h>

Manager::Manager(const ros::NodeHandle &nh) : _nh(nh)
{
    ROS_DEBUG_STREAM("Created Manager");
    _client = _nh.serviceClient<shared_msgs::AddCityToRegion>("/CreateCity");
    _subscriber = _nh.subscribe("/RTCreateCity", 1, &Manager::CreateCityRunTime, this);
    _db = std::make_unique<DatabaseHandler>(_nh);

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
    if (!_client.call(srv))
    {
        ROS_ERROR_STREAM("Failed to call srv client");
    }

    if (srv.response.success)
    {
        CreateCity(srv.response.city);
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
        _client.waitForExistence(ros::Duration(5));

        if (_client.call(srv))
        {
            CreateCity(srv.response.city);
        }
        else
        {
            ROS_ERROR_STREAM("Failed to call srv client");
        }
    }
    ShowState();
}

bool Manager::CreateCity(const shared_msgs::CityInfo &city)
{
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
        ROS_ERROR("Failed to parse json file");
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
    for (auto obj : _db->GetCities())
    {
        _cities.push_back(City{obj});
        ROS_DEBUG("City %s \t at %s", _cities.back().GetName(), _cities.back().GetCoordinates().c_str());
    }
}