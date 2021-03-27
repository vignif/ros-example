#include "mypkg/city.hpp"

City::City(std::string name) : _name(name)
{
    ROS_INFO_STREAM("Created city: " << name);
}

const char *City::GetName()
{
    return this->_name.c_str();
}