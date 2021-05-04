#include "db_handler/db_handler.hpp"

DatabaseHandler::DatabaseHandler(const ros::NodeHandle &nh) : _nh(nh)
{
    if (InitDatabase())
    {
        _initOK = true;
    }
}

DatabaseHandler::~DatabaseHandler()
{
    ROS_DEBUG("Closing DB");
    sqlite3_close(_db);
}

bool DatabaseHandler::InitDatabase()
{
    auto path = ros::package::getPath("db_handler");
    std::string nameDB{"/test.db"}; //default db name

    if (_nh.hasParam("db_name"))
    {
        // get param if already stored in param server
        _nh.getParam("db_name", nameDB);
    }
    else
    {
        // set param in param server wish default name
        _nh.setParam("db_name", nameDB);
    }
    auto fullpath = path + nameDB;

    ROS_DEBUG_STREAM("Database located in: " << fullpath);

    _rc = sqlite3_open(fullpath.c_str(), &_db);

    if (_rc)
    {
        ROS_ERROR("Can't open database: %s\n", sqlite3_errmsg(_db));
        return false;
    }
    else
    {
        ROS_DEBUG("Opened database successfully\n");
        InitTable();
        return true;
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

bool DatabaseHandler::InitTable()
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
        return false;
    }
    else
    {
        ROS_DEBUG("Table created successfully");
        return true;
    }
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

    if (sqlite3_errcode(_db) == 19) // code 19 is unique constraint violation
    {
        ROS_WARN("City %s is already stored in the database", name.c_str());
        sqlite3_free(_zErrMsg);
        return true;
    }
    else if (_rc != SQLITE_OK)
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

int DatabaseHandler::CallbackGetCities(void *NotUsed, int argc, char **argv, char **azColName)
{

    auto *cities = static_cast<std::vector<shared_msgs::CityInfo> *>(NotUsed);

    shared_msgs::CityInfo city;
    city.city_name = argv[1] ? argv[1] : "(NULL)";
    city.postal = std::atoi(argv[2] ? argv[2] : "(NULL)");
    city.region_name = (argv[3] ? argv[3] : "(NULL)");
    city.latitude = std::stof(argv[4]);
    city.longitude = std::stof(argv[5]);
    cities->push_back(city);

    return 0;
}

std::vector<shared_msgs::CityInfo> DatabaseHandler::GetCities()
{
    auto sql = "SELECT * FROM `cities`;";

    /* Execute SQL statement */
    _rc = sqlite3_exec(_db, sql, CallbackGetCities, &_cities, &_zErrMsg);

    if (_rc != SQLITE_OK)
    {
        ROS_ERROR("SQL error: %s", _zErrMsg);
        sqlite3_free(_zErrMsg);
    }
    else
    {
        // ROS_DEBUG("Query executed correctly");
    }
    return _cities;
}
