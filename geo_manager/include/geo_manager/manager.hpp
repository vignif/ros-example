#include "shared_msgs/AddCityToRegion.h"
#include "shared_msgs/RTCityReq.h"
#include "geo_manager/region.hpp"
#include "db_handler/db_handler.hpp"
#include <vector>
#include <memory>

class Manager
{
public:
    Manager(const ros::NodeHandle &nh);
    ~Manager();
    void ShowState();

private:
    void LoadJson();
    void GetFullInfoCities();
    bool CreateCity(shared_msgs::AddCityToRegion::Request &req,
                    shared_msgs::AddCityToRegion::Response &res);
    void CreateCityRunTime(const shared_msgs::RTCityReqPtr &req);

    std::vector<std::pair<int, std::string>> _jsonEntries; /** Vector of Postal-City from json file */
    ros::NodeHandle _nh;                                   /** Ros nodehandle */
    ros::ServiceClient _client;                            /** Ros Client asks to server for full info of a city */
    ros::Subscriber _subscriber;
    std::vector<std::pair<City, Region>> _objects; /** Store pairs City-Region*/
    std::unique_ptr<DatabaseHandler> _db;
    // DatabaseHandler _db;
};