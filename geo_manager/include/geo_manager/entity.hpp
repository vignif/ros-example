/**
 * @file entity.hpp
 * @author your name (you@domain.com)
 * @brief Class used for returning name of an entity
 * @version 0.1
 * @date 2021-05-03
 * 
 * @copyright Copyright (c) 2021
 * 
 */

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
