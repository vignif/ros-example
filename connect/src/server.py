#!/usr/bin/env python

from __future__ import print_function

from mypkg.srv import AddCityToRegion, AddCityToRegionResponse
import rospy
import requests


def handle_response(req):
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
        rospy.logdebug(r.text)
        res = True
    else:
        rospy.logerr("Service content can't be empty!")
        res = False
    return AddCityToRegionResponse(res)


def connect_service():
    rospy.init_node("connect_service")
    s = rospy.Service("CreateCity", AddCityToRegion, handle_response)
    print("Ready to receive new city query")
    rospy.spin()


if __name__ == "__main__":
    connect_service()