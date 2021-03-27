#include "mypkg/city.hpp"
#include <iostream>
#include <memory>
#include <ros/ros.h>

class Region
{
public:
    Region(std::string name);
    ~Region(){};
    void AddCity(City city);
    void ShowCities();
    const char *GetName();

private:
    std::string _name;
    std::vector<City> _cities;
};