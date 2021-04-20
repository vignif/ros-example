# The Great C++ & Python Ros Example

Catkin build status:

[![Build Actions Status](https://github.com/vignif/ros-example/workflows/Build/badge.svg)](https://github.com/vignif/ros-example/actions)

This repository wants to explore with a practical approach the paradigms in the middleware ROS (Robotic Operating System) while connecting ROS with external functionalities. It contains different packages exploring the ROS components:
- topics
- services

When the project starts a json file is parsed. The file contains entries like:
```json
{
    "cities": [
        {
            "name": "Forli",
            "postal": 47122
        },
        {
            
        }
    ]
}
```
The pairs [city-postal] are passed to the component `server.py` which is advertising a Service named `CreateCity`. Once the component `server.py` receives this information it queries an API for retrieving geo information [link](https://nominatim.openstreetmap.org) and is interpreting the response of the API. The response contains the relevant information of each city like:
- Region
- Latitude
- Longitude

These information is packed into the service response and the node `geo_manager` is unpacking the data and writing in the database with the schema [id, name, postal, region, latitude, longitude]. The id is autoincremental. A constraint on uniqueness is considering unique the tuple [name, postal].

Once this is done, the database can still be filled up with information of more cities. In order to do so, a ros node or simply the user can publish a proper topic with the pair [city-postal] like:

```
rostopic pub /RTCreateCity geo_manager/RTCityReq "city_name: 'New York'
postal: 10001"
```

## Outside ROS
The project also contains:
- the data management of an SQL database using [SQLite3](https://www.sqlite.org/)
- A JSON parser implemented using a cpp library [json](https://github.com/open-source-parsers/jsoncpp)
- Interface to a remote API [nominatim](https://nominatim.openstreetmap.org)

## How to install
Given your catkin workspace `~/ros_ws`, clone the current repository with:

```
git clone git@github.com:vignif/ros-example.git src
```

Check that your system has all the dependencies needed for the project with:

```
rosdep install --from-paths src --ignore-src -r -y
```

Build the project with catkin using:

```
catkin_make
```

## How to run

Now that everything is built on your machine, you need to source the binaries with:

in case you are using bash
```
source ~/ros_ws/devel/setup.bash
```

in case you are using zsh
```
source ~/ros_ws/devel/setup.zsh
```

Now you can fire the two main nodes with:
```
roslaunch geo_manager geo_manager.launch
```

The json file located in `geo_manager/data/cities.json` is parsed, the information is transmitted to the API which is returning the data [region, latitude, longitude] of the cities listed in the file.
This information is then written in a database [created at runtime] named `test.db`.
If you want to change the name of your database you can modify the entry of the `geo_manager.launch` file containing the db name.
The schema of the table `cities` is fixed and is managed with the object `Manager`.

## Unit Tests
Run the unit tests using the [Catkin Command Line Tools](http://catkin-tools.readthedocs.io/en/latest/index.html#)

```catkin build geo_manager --no-deps --verbose --catkin-make-args run_tests```

## Versioning

We use [SemVer](http://semver.org/) for versioning. For the versions available, see the [tags on this repository](https://github.com/vignif/ros-example/tags). 


## License

[![License](http://img.shields.io/:license-mit-blue.svg?style=flat-square)](http://badges.mit-license.org)

**[MIT license](http://opensource.org/licenses/mit-license.php)**
- Copyright 2021 © Francesco Vigni