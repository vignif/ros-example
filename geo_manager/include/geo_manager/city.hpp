#include "geo_manager/Entity.hpp"
#include "shared_msgs/CityInfo.h"
#include <string>

class City : public Entity
{
public:
    City(const shared_msgs::CityInfo &city);
    std::string GetCoordinates();

private:
    float _lat;
    float _lon;
    std::string _postal;
    std::string _region;
};
