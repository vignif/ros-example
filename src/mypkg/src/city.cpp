#include "mypkg/city.hpp"

City::City(std::string name) : _name(name)
{
    ROS_INFO_STREAM("Created city: " << name);
}

std::string City::GetName()
{
    return this->_name;
}