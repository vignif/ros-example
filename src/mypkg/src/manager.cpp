#include "mypkg/manager.hpp"

Manager::Manager(const ros::NodeHandle &nh) : _nh(nh)
{
    ROS_INFO_STREAM("Created Manager");
    _service = _nh.advertiseService("CreateCity", &Manager::CreateCity, this);
    InitDatabase();
}

Manager::~Manager()
{
    ROS_DEBUG("Closing DB");
    sqlite3_close(_db);
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

void Manager::InitTable()
{
    /* Create SQL statement */
    auto sql = "CREATE TABLE IF NOT EXISTS CITIES("
               "ID INT PRIMARY KEY     NOT NULL,"
               "NAME           TEXT    NOT NULL,"
               "REGION         TEXT    NOT NULL,"
               "LATITUDE       FLOAT,"
               "LONGITUDE      FLOAT );";

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