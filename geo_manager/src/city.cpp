#include "geo_manager/city.hpp"

City::City(std::string name) : Entity(name)
{
    ROS_INFO_STREAM("Created city!: " << this->GetName());
}
