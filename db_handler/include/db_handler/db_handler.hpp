/**
 * @file db_handler.hpp
 * @author Francesco Vigni (vignif@gmail.com)
 * @brief Custom wrapper for managing a sqlite3 database file.
 * @version 0.4

 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef DB_HANDLER_LIBRARY_H
#define DB_HANDLER_LIBRARY_H

#include <sqlite3.h>
#define SQLITE_OPEN_CREATE 0x00000004 /* Ok for sqlite3_open_v2() */
#include <iostream>
#include <memory>
#include <ros/ros.h>
#include "shared_msgs/AddCityToRegion.h"
#include "shared_msgs/CityInfo.h"

#include "std_msgs/String.h"
#include <map>
#include <tuple>
#include <ros/package.h>
#include <vector>
#include <string>

class DatabaseHandler
{
public:
    /**
     * @brief Construct a new Database Handler object
     * 
     * @param nh ros node handle
     */
    DatabaseHandler(const ros::NodeHandle &nh);

    /**
     * @brief Destroy the Database Handler object
     * close db resource
     */
    ~DatabaseHandler();

    /**
     * @brief InsertCity in database
     * 
     * @param city 
     * @return true if new city is added 
     * @return false if city is already in db or db generic error
     */
    bool InsertCity(const shared_msgs::CityInfo &city);

    /**
     * @brief Get the Cities from the database
     * Execute a select * query on the database
     * @return std::vector<shared_msgs::CityInfo> 
     */
    std::vector<shared_msgs::CityInfo> GetCities();
    bool _initOK{false}; /** Flag return true if database is initialized correctly */

private:
    /** Callback to handle sql query response */
    static int CallbackGetCities(void *refCities, int argc, char **argv, char **azColName);

    bool InitDatabase();                        /** Check if db file exists, if not create one */
    bool InitTable();                           /** Check if db table exists, if not create one with proper schema */
    std::vector<shared_msgs::CityInfo> _cities; /** Member stores the table cities when requested */
    ros::NodeHandle _nh;                        /** Ros nodehandle */
    sqlite3 *_db;                               /** Pointer to SQLite connection */
    char *_zErrMsg = 0;                         /** Save any error messages */
    int _rc;                                    /** Database connection object */
};

#endif