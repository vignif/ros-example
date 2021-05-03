#include "geo_manager/city.hpp"

City::City(const shared_msgs::CityInfo &city) : Entity(city.city_name)
{
    _lat = city.latitude;
    _lon = city.longitude;
    _postal = city.postal;
    _region = city.region_name;
}

std::string City::GetCoordinates()
{
    return std::to_string(_lat) + " " + std::to_string(_lon);
}
