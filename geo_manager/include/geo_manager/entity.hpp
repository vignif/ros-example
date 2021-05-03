/**
 * @file entity.hpp
 * @author Francesco Vigni (vignif@gmail.com)
 * @brief Class used for returning name of an entity
 * @version 0.4

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
