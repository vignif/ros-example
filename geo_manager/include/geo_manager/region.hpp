/**
 * @file region.hpp
 * @author Francesco Vigni (vignif@gmail.com)
 * @brief ----UNUSED----
 * @version 0.4

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