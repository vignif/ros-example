#include "mypkg/region.hpp"

Region::Region(std::string name) : _name(name)
{
    ROS_INFO_STREAM("Created region: " << name);
};

void Region::AddCity(City city)
{
    ROS_INFO("Added city %s to region %s", city.GetName(), this->_name.c_str());

    _cities.push_back(city);
};

void Region::ShowCities()
{
    for (auto city : _cities)
    {
        ROS_INFO("%s", city.GetName());
    }
};