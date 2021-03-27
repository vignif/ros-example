#include <iostream>
#include <memory>
#include <ros/ros.h>

class City
{
public:
    City(std::string name);
    ~City(){};
    const char *GetName();

private:
    std::string _name;
    std::pair<float, float> _coordinates;
};