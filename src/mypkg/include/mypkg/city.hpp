#include <iostream>
#include <memory>
#include <ros/ros.h>

class City
{
public:
    City(std::string name);
    std::string GetName();

private:
    std::string _name;
    std::unique_ptr<std::string> _city_name_ptr;
};