    """Wrapper for handling API requests and responses
    It advertise a ros topic and queries the API with proper information.
    The API response is handled and return as ros service response.
    """
#!/usr/bin/env python

from __future__ import print_function

from shared_msgs.srv import AddCityToRegion, AddCityToRegionResponse
import rospy
import requests
import json
from shared_msgs.msg import CityInfo


class Server:
    """Server
    Ros server class to manage API communications
    """
    def __init__(self):
        self.ros_init()

    """ros_init
    Initialize ros related structure at object creation
    """
    def ros_init(self):
        rospy.init_node("connect_service", log_level=rospy.INFO)
        s = rospy.Service("CreateCity", AddCityToRegion, self.handle_response)

        print("Ready to receive new city query")

    """handle_response
    It receives the city information on a ros service request and 
    execute the API call accordingly.
    If the API doesn't responde a warning is raised.
    If the API is not able to respond (city name is spelled wrong), a warning is raised.
    Other exceptions will return errors.
    The API response is stored in a proper structure and returned as a service response.
    """
    def handle_response(self, req):
        if req.city_name != "" or req.postal != 0:
            ploads = {
                "city": req.city_name,
                "postalcode": req.postal,
                "format": "json",
                "limit": 1,
            }
            response = AddCityToRegionResponse()
            try:
                r = requests.get(
                    "https://nominatim.openstreetmap.org/search?",
                    params=ploads,
                )
                json_res = r.json()
                lat = json_res[0]["lat"]
                lon = json_res[0]["lon"]
                region = json_res[0]["display_name"]
                # rospy.logdebug(region)
                response.success = True
                response.city.city_name = req.city_name
                response.city.postal = req.postal
                response.city.region_name = str(region)
                response.city.latitude = float(lat)
                response.city.longitude = float(lon)
            except requests.ConnectionError as co_e:
                rospy.logwarn("API is not available")
                response.success = False
            except IndexError as id_e:
                rospy.logwarn(
                    "Api can't find info for city %s! Did you spell it correctly?",
                    req.city_name,
                )
                response.success = False
            except Exception as e:
                rospy.logerr("Failed to retrieve info of the city! %s", e)
                response.success = False
        else:
            rospy.logerr("Service content can't be empty!")
            response.success = False
        return response


if __name__ == "__main__":
    Server()
    rospy.spin()