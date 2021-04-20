#!/usr/bin/env python

from __future__ import print_function

from geo_manager.srv import AddCityToRegion, AddCityToRegionResponse
import rospy
import requests
import json
from geo_manager.msg import CityInfo


class Server:
    def __init__(self):

        self.connect_service()

    def connect_service(self):
        rospy.init_node("connect_service", log_level=rospy.DEBUG)
        s = rospy.Service("CreateCity", AddCityToRegion, self.handle_response)

        print("Ready to receive new city query")

    def handle_response(self, req):
        if req.city_name != "" or req.postal != 0:
            ploads = {
                "city": req.city_name,
                "postalcode": req.postal,
                "format": "json",
                "limit": 1,
            }
            try:
                r = requests.get(
                    "https://nominatim.openstreetmap.org/search?",
                    params=ploads,
                )
                json_res = r.json()
                lat = json_res[0]["lat"]
                lon = json_res[0]["lon"]
                region = json_res[0]["display_name"]
                rospy.logdebug(region)

                response = AddCityToRegionResponse()
                response.success = True
                response.city.city_name = req.city_name
                response.city.postal = req.postal
                response.city.region_name = str(region)
                response.city.latitude = float(lat)
                response.city.longitude = float(lon)
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