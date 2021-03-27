#include <iostream>
#include "mypkg/region.hpp"
#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");
    ros::NodeHandle nh;

    auto forli = City("Forli");
    auto cesena = City("Cesena");
    auto emilia_romagna = Region("Emilia-Romagna");

    emilia_romagna.AddCity(forli);
    emilia_romagna.AddCity(cesena);

    emilia_romagna.ShowCities();
    ros::spin();
    return 0;
}