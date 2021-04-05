#include "geo_manager/Entity.hpp"

class City : public Entity
{
public:
    City(std::string name);

private:
    std::pair<float, float> _coordinates;
};
