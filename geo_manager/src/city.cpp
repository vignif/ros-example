#include "geo_manager/city.hpp"

City::City(const shared_msgs::CityInfo &city) : Entity(city.city_name)
{
    ROS_INFO_STREAM("Created city!: " << this->GetName());
    _lat = city.latitude;
    _lon = city.longitude;
    _postal = city.postal;
    _region = city.region_name;
}

std::string City::GetCoordinates()
{
    return std::to_string(_lat) + " " + std::to_string(_lon);
}
