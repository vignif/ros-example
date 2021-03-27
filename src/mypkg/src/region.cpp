#include "mypkg/region.hpp"

Region::Region(std::string name) : _name(name)
{
    ROS_INFO_STREAM("Created region: " << name);
};

void Region::AddCity(City city)
{
    ROS_INFO("Added city %s to region %s", city.GetName().c_str(), this->_name.c_str());

    _cities.emplace(std::make_pair(_cities.size(), &city));
};