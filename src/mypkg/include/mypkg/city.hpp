#include <iostream>
#include <memory>
#include <ros/ros.h>

class City
{
public:
    City(std::string name);
    const char *GetName();

private:
    std::string _name;
};