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
    GetFullInfoCities();
}

Manager::~Manager()
{
    // _db.~DatabaseHandler();
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
        auto city = City(srv.response.city.city_name.c_str());
        _db->InsertCity(srv.response.city);
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
        shared_msgs::AddCityToRegion srv;
        srv.request.city_name = name;
        srv.request.postal = postal;

        _client.waitForExistence();
        if (_client.call(srv))
        {

            _db->InsertCity(srv.response.city);
        }
        else
        {
            ROS_ERROR_STREAM("failed");
        }
    }
}

bool Manager::CreateCity(shared_msgs::AddCityToRegion::Request &req,
                         shared_msgs::AddCityToRegion::Response &res)
{
    if (req.city_name.size() > 0 && req.postal > 0)
    {
        auto city = City(req.city_name);
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
    for (auto obj : _objects)
    {
        ROS_INFO("City %s in Region %s", obj.first.GetName(), obj.second.GetName());
    }
}