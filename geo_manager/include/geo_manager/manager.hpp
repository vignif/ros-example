/**
 * @file manager.hpp
 * @author Francesco Vigni (vignif@gmail.com)
 * @brief Instanciate the resources at startup and makes sure to 
 * advertise correct services in order to add more cities at run time.
 * It parses the json at start time and propagates the info to the api handler.
 * It accesses the database with a unique pointer.
 * @version 0.4

 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "shared_msgs/AddCityToRegion.h"
#include "shared_msgs/RTCityReq.h"
#include "geo_manager/region.hpp"
#include "db_handler/db_handler.hpp"
#include <vector>
#include <memory>

class Manager
{
public:
    /**
     * Constructor of Manager object
    */
    Manager(const ros::NodeHandle &nh);

    /**
     * destructor of Manager object
    */
    ~Manager();

    void ShowState();

private:
    /**
     * Parse json file at init
    */
    void LoadJson();

    /**
     * @brief Insert in db the cities present in the json file
    */
    void InitDBwithCities();

    /**
     * Wrapper of database call 
     * Creates a new object of type City 
    */
    bool CreateCity(const shared_msgs::CityInfo &city);

    /**
     * Create a city at run time and store it in the database
    */
    void CreateCityRunTime(const shared_msgs::RTCityReqPtr &req);

    std::vector<std::pair<int, std::string>> _jsonEntries; /** Vector of Postal-City from json file */
    ros::NodeHandle _nh;                                   /** Ros Nodehandle */
    ros::ServiceClient _client;                            /** Ros Client asks to server for full info of a city */
    ros::Subscriber _subscriber;                           /** Ros Subscriber for runtime requests */
    std::vector<City> _cities;                             /** Store Cities */
    std::unique_ptr<DatabaseHandler> _db;                  /** Unique pointer to the database handler */
};