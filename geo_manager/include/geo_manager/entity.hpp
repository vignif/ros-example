#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>
#include <vector>

class Entity
{
public:
    Entity(std::string name) : _name(name){};
    ~Entity(){};
    const char *GetName()
    {
        return this->_name.c_str();
    }

private:
    std::string _name;
};
