#! /usr/bin/env python

from cmath import pi
import rospy
import math
from cube_spotter.msg import cubeArray
from cube_spotter.msg import cubeData

from robotics_cswk_kin.srv import FKinMsg, FKinMsgRequest
from cube_locator.msg import RealCubeArray, RealCube
from geometry_msgs.msg import Point

rospy.init_node('cube_locator')

i = 0

def calc_position(img_x, img_y, alpha, beta, r, z):
    img_x_fov = 70.0 / 180.0 * pi
    img_y_fov = 70.0 / 180.0 * pi

    # Are these in the correct direction?
    alpha_dash = (img_x - 0.5) * img_x_fov
    beta_dash = (img_y - 0.5) * img_y_fov

    # Are these correctly added?
    # sin(beta + beta_dash) = z / c
    c = z / math.sin(beta + beta_dash)

    # Are these correctly added?
    # cos(beta + beta_dash) = r_dash_x / c
    r_dash_x = math.cos(beta + beta_dash) * c

    # Good direction?
    # tan(alpha_dash) = r_dash_y / c
    r_dash_y = math.tan(alpha_dash) * c

    # Each r_dash_x and r_dash_y impacts both x_dash
    # and y_dash!

    # Working with alpha here since it's already normalized
    # to the direction of r and 90 degs from it
    # are the signs here correct?
    # r_dash_x splits into two
    r_dash_x_for_x = math.cos(alpha) * r_dash_x
    r_dash_x_for_y = math.sin(alpha) * r_dash_x

    # be careful about + and - here especially for x
    # r_dash_y splits into two
    r_dash_y_for_y = math.cos(alpha) * r_dash_y
    r_dash_y_for_x = - math.sin(alpha) * r_dash_y
    
    ## Calculate x_dash and y_dash here
    x_dash = r_dash_x_for_x + r_dash_y_for_x
    y_dash = r_dash_y_for_y + r_dash_x_for_y

    x = math.cos(alpha) * r
    y = math.sin(alpha) * r
    # 0.015 here is arbitrary constant

    x_total = x + x_dash
    y_total = y + y_dash

    return [
        x_total,
        y_total,
        # math.sqrt(pow(x_total, 2) + pow(y_total, 2)), 
        # math.tan(y_total / x_total)
    ]

def cube_handler(msg):
    # global i
    # i += 1
    # if i < 100:
    #     return

    # i = 0

    rca = RealCubeArray()

    for data in msg.cubes:
        color = data.cube_colour
        img_x = data.normalisedCoordinateX
        img_y = data.normalisedCoordinateY
        res = fwd_kin_proxy(FKinMsgRequest())

        # This stuff is all calculated from the camera
        # So it makes sense to directly use the fkin values
        # Is this max here really needed?
        r = max(0.000001, math.sqrt(res.position.x**2 + res.position.y**2))
        alpha = -math.asin(res.position.y / r)
        beta = res.angle.data

        # Subtract a certain amount from the z if you
        # think it's overestimating the distance consistently
        # But think about the fact that rc.position.z below
        # is set to -0.04, so the top of the cube might just be
        # below or at 0
        z = res.position.z + 0.04

        [cube_x, cube_y] = calc_position(img_x, img_y, alpha, beta, r, z)

        rc = RealCube()
        rc.position = Point()
        rc.color.data = color
        rc.position.x = cube_x
        rc.position.y = -cube_y
        rc.position.z = -0.04
        rca.cubes.append(rc)
        
    real_cube_pub.publish(rca)


cube_sub = rospy.Subscriber('/cubes', cubeArray, cube_handler)
real_cube_pub = rospy.Publisher('/real_cubes', RealCubeArray, queue_size=10)
fwd_kin_proxy = rospy.ServiceProxy('/fwd_kin', FKinMsg)

rospy.spin()