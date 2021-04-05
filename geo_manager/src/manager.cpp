#include "geo_manager/manager.hpp"
#include <jsoncpp/json/json.h>
#include <fstream>

Manager::Manager(const ros::NodeHandle &nh) : _nh(nh)
{
    ROS_INFO_STREAM("Created Manager");
    _client = _nh.serviceClient<geo_manager::AddCityToRegion>("/CreateCity");

    _subscriber = _nh.subscribe("/RTCreateCity", 1, &Manager::CreateCityRunTime, this);

    LoadJson();
    GetFullInfoCities();
}

Manager::~Manager()
{
}

void Manager::CreateCityRunTime(const geo_manager::RTCityReqPtr &req)
{
    geo_manager::AddCityToRegion srv;
    srv.request.city_name = req->city_name;
    srv.request.postal = req->postal;
    _client.waitForExistence();
    if (_client.call(srv))
    {
        ROS_DEBUG("Insert City %s", srv.response.city.city_name.c_str());
        //InsertCity(srv.response);
    }
    else
    {
        ROS_ERROR_STREAM("failed");
    }
}

void Manager::GetFullInfoCities()
{
    for (auto i : _jsonEntries)
    {
        auto postal = i.first;
        auto name = i.second;
        geo_manager::AddCityToRegion srv;
        srv.request.city_name = name;
        srv.request.postal = postal;

        _client.waitForExistence();
        if (_client.call(srv))
        {

            // InsertCity(srv.response);
        }
        else
        {
            ROS_ERROR_STREAM("failed");
        }
    }
}

bool Manager::CreateCity(geo_manager::AddCityToRegion::Request &req,
                         geo_manager::AddCityToRegion::Response &res)
{
    if (req.city_name.size() > 0 && req.postal > 0)
    {
        auto city = City(req.city_name);

        // _objects.push_back(std::make_pair(city, region));
        ShowState();
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

    std::ifstream ifs(fullpath); //open file example.json

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
    for (auto obj : _objects)
    {
        ROS_INFO("City %s in Region %s", obj.first.GetName(), obj.second.GetName());
    }
}