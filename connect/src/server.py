#!/usr/bin/env python

from __future__ import print_function

from mypkg.srv import AddCityToRegion, AddCityToRegionResponse
import rospy


def handle_response(req):
    res = True
    return AddCityToRegionResponse(res)


def connect_service():
    rospy.init_node("connect_service")
    s = rospy.Service("CreateCity", AddCityToRegion, handle_response)
    print("Ready to receive new city query")
    rospy.spin()


if __name__ == "__main__":
    connect_service()