#!/usr/bin/env python

import plotly.graph_objects as go
import pandas as pd
import sqlite3
import os
import rospkg
from std_msgs.msg import Empty
import rospy


class Show:
    def __init__(self):
        rospy.init_node("Show")
        self.df = None
        self.sub = rospy.Subscriber("render_cities", Empty, self.get_cities)

    def get_cities(self, data):
        try:
            rospack = rospkg.RosPack()
            path = rospack.get_path("db_handler")
            db_path = path + "/test.db"
            con = sqlite3.connect(db_path)
            sql = "SELECT * FROM cities"
            self.df = pd.read_sql_query(sql, con)

        except Exception as e:
            rospy.logerr("Can't access database")
        self.render()

    def render(self):
        fig = go.Figure(
            data=go.Scattergeo(
                lon=self.df["longitude"],
                lat=self.df["latitude"],
                text=self.df["name"],
                marker=dict(color="red", size=10),
            ),
        )

        fig.update_layout(
            title='City in database of ros example<br>(Hover for city names)<br> <a href="https://github.com/vignif/ros-example">link</a>',
            geo_scope="world",
        )
        fig.update_geos(projection_type="natural earth")
        fig.show()


if __name__ == "__main__":
    Show()
    rospy.spin()
