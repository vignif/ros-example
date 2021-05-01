import plotly.graph_objects as go
import pandas as pd
import sqlite3
import os
import rospy
import rospkg


try:
    rospack = rospkg.RosPack()
    path = rospack.get_path("db_handler")
    db_path = path + "/test.db"
    con = sqlite3.connect(db_path)
    sql = "SELECT * FROM cities"
    df = pd.read_sql_query(sql, con)

except Exception as e:
    rospy.logerr("Can't access database")

fig = go.Figure(
    data=go.Scattergeo(
        lon=df["longitude"],
        lat=df["latitude"],
        text=df["name"],
        mode="markers",
    )
)

fig.update_layout(
    title='City in database of ros example<br>(Hover for city names)<br> <a href="https://github.com/vignif/ros-example">link</a>',
    geo_scope="europe",
)
fig.show()