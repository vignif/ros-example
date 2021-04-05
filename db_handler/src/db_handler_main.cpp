#include <ros/ros.h>
#include <stdio.h>

#include "db_handler/db_handler.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "DatabaseHandler");
    ros::NodeHandle nh{"~"};
    DatabaseHandler D(nh);
    ros::spin();
    return 0;
}