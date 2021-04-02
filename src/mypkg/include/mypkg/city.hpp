#include <iostream>
#include <memory>
#include <ros/ros.h>
#include "mypkg/Entity.hpp"

class City : public Entity
{
public:
    City(std::string name);

private:
    std::pair<float, float> _coordinates;
};
