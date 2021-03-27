#include "mypkg/city.hpp"
#include <iostream>
#include <memory>
#include <ros/ros.h>

class Region
{
public:
    Region(std::string name);
    void AddCity(City city);
    void ShowCities();

private:
    std::string _name;
    std::vector<City> _cities;
};