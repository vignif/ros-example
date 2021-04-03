#include "mypkg/manager.hpp"
#include <jsoncpp/json/json.h>
#include <fstream>

Manager::Manager(const ros::NodeHandle &nh) : _nh(nh)
{
    ROS_INFO_STREAM("Created Manager");
    _client = _nh.serviceClient<mypkg::AddCityToRegion>("/CreateCity");

    _subscriber = _nh.subscribe("/RTCreateCity", 1, &Manager::CreateCityRunTime, this);

    InitDatabase();
    LoadJson();
    GetFullInfoCities();
}

Manager::~Manager()
{
    ROS_DEBUG("Closing DB");
    sqlite3_close(_db);
}

void Manager::CreateCityRunTime(const mypkg::RTCityReqPtr &req)
{
    mypkg::AddCityToRegion srv;
    srv.request.city_name = req->city_name;
    srv.request.postal = req->postal;
    _client.waitForExistence();
    if (_client.call(srv))
    {
        ROS_DEBUG("Insert City %s", srv.response.city.city_name.c_str());
        InsertCity(srv.response);
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
        mypkg::AddCityToRegion srv;
        srv.request.city_name = name;
        srv.request.postal = postal;

        _client.waitForExistence();
        if (_client.call(srv))
        {

            InsertCity(srv.response);
        }
        else
        {
            ROS_ERROR_STREAM("failed");
        }
    }
}

void Manager::InitDatabase()
{
    auto path = ros::package::getPath("mypkg");
    std::string nameDB;
    _nh.getParam("db_name", nameDB);

    auto fullpath = path + nameDB;

    ROS_DEBUG_STREAM("Database located in: " << fullpath);

    _rc = sqlite3_open(fullpath.c_str(), &_db);

    if (_rc)
    {
        ROS_ERROR("Can't open database: %s\n", sqlite3_errmsg(_db));
    }
    else
    {
        ROS_DEBUG("Opened database successfully\n");
        InitTable();
    }
}

static int callback(void *NotUsed, int argc, char **argv, char **azColName)
{
    int i;
    for (i = 0; i < argc; i++)
    {
        printf("%s = %s\n", azColName[i], argv[i] ? argv[i] : "NULL");
    }
    printf("\n");
    return 0;
}

void Manager::InsertCity(mypkg::AddCityToRegion::Response &res)
{
    auto name = res.city.city_name;
    auto postal = res.city.postal;
    auto region = res.city.region_name;
    auto latitude = res.city.latitude;
    auto longitude = res.city.longitude;

    auto sql = "INSERT INTO cities (id, name, postal, region, latitude, longitude) VALUES ("
               "NULL, "
               "'" +
               name + "', "
                      "'" +
               std::to_string(postal) + "', "
                                        "'" +
               region + "', "
                        "'" +
               std::to_string(latitude) + "', "
                                          "'" +
               std::to_string(longitude) + "');";

    /* Execute SQL statement */
    _rc = sqlite3_exec(_db, sql.c_str(), callback, 0, &_zErrMsg);

    if (_rc != SQLITE_OK)
    {
        ROS_ERROR("SQL error: %s", _zErrMsg);
        sqlite3_free(_zErrMsg);
    }
    else
    {
        ROS_DEBUG("City %s inserted in DB successfully", name.c_str());
    }
}

void Manager::InitTable()
{
    /* Create SQL statement */
    auto sql = "CREATE TABLE IF NOT EXISTS cities("
               "id INTEGER PRIMARY KEY     NOT NULL,"
               "name           TEXT    NOT NULL,"
               "postal         INT     NOT NULL,"
               "region         TEXT    NOT NULL,"
               "latitude       FLOAT,"
               "longitude      FLOAT,"
               "UNIQUE(name, postal));";

    /* Execute SQL statement */
    _rc = sqlite3_exec(_db, sql, callback, 0, &_zErrMsg);

    if (_rc != SQLITE_OK)
    {
        ROS_ERROR("SQL error: %s", _zErrMsg);
        sqlite3_free(_zErrMsg);
    }
    else
    {
        ROS_DEBUG("Table created successfully");
    }
}

bool Manager::CreateCity(mypkg::AddCityToRegion::Request &req,
                         mypkg::AddCityToRegion::Response &res)
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
    auto path = ros::package::getPath("mypkg");
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