#!/usr/bin/env python

from __future__ import print_function

from mypkg.srv import AddCityToRegion, AddCityToRegionResponse
import rospy
import requests
import json
from mypkg.msg import CityInfo


class Server:
    def __init__(self):
        self.pub = rospy.Publisher("CityInfo", CityInfo, queue_size=10)
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
            r = requests.get(
                "https://nominatim.openstreetmap.org/search?",
                params=ploads,
            )
            json_res = r.json()
            lat = json_res[0]["lat"]
            lon = json_res[0]["lon"]
            region = json_res[0]["display_name"]
            rospy.logdebug(lat)

            msg = CityInfo()

            msg.city_name = req.city_name
            msg.region_name = str(region)
            msg.latitude = float(lat)
            msg.longitude = float(lon)
            self.pub.publish(msg)

            res = True
        else:
            rospy.logerr("Service content can't be empty!")
            res = False
        return AddCityToRegionResponse(res)


if __name__ == "__main__":
    Server()
    rospy.spin()