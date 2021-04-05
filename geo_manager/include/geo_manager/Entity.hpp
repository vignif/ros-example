#include <iostream>

class Entity
{
public:
    Entity(std::string name) : _name(name)
    {
        ROS_DEBUG("Created Entity");
    };
    ~Entity(){};
    const char *GetName()
    {
        return this->_name.c_str();
    }

private:
    std::string _name;
};
