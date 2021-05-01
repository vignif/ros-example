#include "db_handler/db_handler.hpp"

DatabaseHandler::DatabaseHandler(const ros::NodeHandle &nh) : _nh(nh)
{
    InitDatabase();
}

DatabaseHandler::~DatabaseHandler()
{
    ROS_DEBUG("Closing DB");
    sqlite3_close(_db);
}

void DatabaseHandler::InitDatabase()
{
    auto path = ros::package::getPath("db_handler");
    std::string nameDB{"/test.db"};
    if (_nh.hasParam("db_name"))
    {
        ROS_DEBUG("get db_name");
        _nh.getParam("db_name", nameDB);
    }
    else
    {
        ROS_DEBUG("set db_name");
        _nh.setParam("db_name", nameDB);
    }
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

bool DatabaseHandler::InsertCity(const shared_msgs::CityInfo &city)
{
    auto name = city.city_name;
    auto postal = city.postal;
    auto region = city.region_name;
    auto latitude = city.latitude;
    auto longitude = city.longitude;

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
        return false;
    }
    else
    {
        ROS_DEBUG("City %s inserted in DB successfully", name.c_str());
        return true;
    }
}

void DatabaseHandler::InitTable()
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
