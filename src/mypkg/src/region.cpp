#include "mypkg/region.hpp"
#include "mypkg/city.hpp"

Region::Region(std::string name) : _name(name)
{
    ROS_INFO_STREAM("Created region: " << name);
};

void Region::AddCity(City city)
{
    ROS_INFO("Added city %s to region %s", city.GetName(), this->_name);
    _cities.emplace(city);
};