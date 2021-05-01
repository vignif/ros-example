#include "geo_manager/entity.hpp"
#include "shared_msgs/CityInfo.h"
#include <string>

class City : public Entity
{
public:
    /**
     * Constructor with city info signature
    */
    City(const shared_msgs::CityInfo &city);

    /**
     * Return coordinates of the city
    */
    std::string GetCoordinates();

private:
    float _lat;          /** Latitude coordinate of the city */
    float _lon;          /** Longitude coordinate of the city */
    std::string _postal; /** Postal code of the city */
    std::string _region; /** Region of the city */
};
