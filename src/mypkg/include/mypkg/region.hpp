#include "mypkg/city.hpp"
#include <iostream>
#include <memory>
#include <ros/ros.h>

class Region
{
public:
    Region(std::string name);
    void AddCity(City city);

private:
    std::string _name;
    std::unique_ptr<std::string> _region_name_ptr;
    std::map<int, std::unique_ptr<City>> _cities;
};