# ros-example
Complete example of ROS package - C++

Load coordinates of cities of a region from a json file
and forward the information to the ros diagnostic tool.

A subscriber to the topic `/city` will convert the string naming information of a city and
transmit on the topic `/diagnostics` all the relevant information of the city.

