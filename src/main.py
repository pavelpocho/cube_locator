#! /usr/bin/env python

import rospy
import math
from cube_spotter.msg import cubeArray
from cube_spotter.msg import cubeData

from robotics_cswk_kin.srv import FKinMsg, FKinMsgRequest
from cube_locator.msg import RealCubeArray, RealCube
from geometry_msgs.msg import Point

rospy.init_node('cube_locator')

i = 0

def cube_handler(msg):
    # global i
    # i += 1
    # if i < 100:
    #     return

    # i = 0

    rca = RealCubeArray()

    for data in msg.cubes:
        color = data.cube_colour
        f = data.normalisedCoordinateX
        w = 640
        g = data.normalisedCoordinateY
        h = 480
        fwd_kin_rq = FKinMsgRequest()
        res = fwd_kin_proxy(fwd_kin_rq)
        camera_r = max(0.000001, math.sqrt(res.position.x**2 + res.position.y**2))
        ee_a_angle = math.asin(res.position.y / camera_r)
        target_angle = res.angle.data
        fov = 40.0 / 180.0 * math.pi
        fov_y = 120.0 / 180.0 * math.pi

        beta = (math.pi / 2 - target_angle) - (g - 0.5) * fov
        alpha = ee_a_angle + math.asin((f - 0.5) * 2) / 2 * fov_y

        print(alpha)

        z = res.position.z + 0.06
        r = z * math.tan(beta)
        r_z = z * math.tan(alpha)
        if f - 0.5 < 0:
            r_z = -r_z
        y = r_z * math.sin(alpha) + res.position.y
        print(y)
        x = r * math.cos(alpha) + res.position.x

        rc = RealCube()
        rc.position = Point()
        rc.color.data = color
        rc.position.x = x
        rc.position.y = -y
        rc.position.z = -0.04
        rca.cubes.append(rc)
        
    real_cube_pub.publish(rca)


cube_sub = rospy.Subscriber('/cubes', cubeArray, cube_handler)
real_cube_pub = rospy.Publisher('/real_cubes', RealCubeArray, queue_size=10)
fwd_kin_proxy = rospy.ServiceProxy('/fwd_kin', FKinMsg)

rospy.spin()