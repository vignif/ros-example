#include "geo_manager/region.hpp"

Region::Region(std::string name) : Entity(name)
{
    ROS_INFO_STREAM("Created region: " << name);
};

void Region::AddCity(City city)
{
    ROS_INFO("Added city %s to region %s", city.GetName(), this->GetName());

    _cities.push_back(city);
};

void Region::ShowCities()
{
    for (auto city : _cities)
    {
        ROS_INFO("%s", city.GetName());
    }
};
