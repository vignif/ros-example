#include <ros/ros.h>
#include <stdio.h>

#include "mypkg/manager.hpp"
#include "std_msgs/String.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "CitiesInRegion");
    ros::NodeHandle nh{"~"};
    Manager m(nh);
    ros::spin();
    return 0;
}