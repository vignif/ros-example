#include <iostream>
#include "mypkg/region.hpp"

int main(int argc, char **argv)
{
    auto forli = City("Forli");
    auto emilia_romagna = Region("Emilia-Romagna");
    emilia_romagna.AddCity(City("Forli"));
    return 0;
}