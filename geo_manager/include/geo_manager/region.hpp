/**
 * @file region.hpp
 * @author your name (you@domain.com)
 * @brief ----UNUSED----
 * @version 0.1
 * @date 2021-05-03
 * 
 * @copyright Copyright (c) 2021
 * 
 */

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