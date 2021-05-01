// UNUSED

#include "geo_manager/city.hpp"

class Region : public Entity
{
public:
    Region(std::string name);
    void AddCity(City city);
    void ShowCities();

private:
    std::vector<City> _cities;
};