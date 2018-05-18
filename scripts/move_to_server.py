#! /usr/bin/env python
from beginner_tutorials.srv import *
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped, PoseStamped, Point
import numpy as np
import math as m
import tf

# initializing the x,y,theta measured by C525
x_global1 = 0.0
y_global1 = 0.0
yaw1 = 0.0

# initializing the x,y,theta measured by C525
x_global2 = 0.0
y_global2 = 0.0
yaw2 = 0.0

# initializing speed
speed_new = 0.18
speed_old = 0.18
total_distance = []
max_speed = 0.95
min_speed = 0.3

# initializing turn_error
turn_error = 0

def initialize():
# initializing the main node which subscribes to the pose and pixel topics of aruco singles and publishes the command velocity
    global cmd_vel_pub
    # rospy.init_node('aruco_mapper_v2', anonymous=True)
    rospy.init_node('move_to_server_node')
    rospy.loginfo('aruco_mapper_v2 node crearted')

    rospy.Subscriber('/aruco_single1/pixel', PointStamped, xy_callback1)
    rospy.Subscriber('/aruco_single1/pose', PoseStamped, heading_callback1)
    rospy.Subscriber('/aruco_single2/pixel', PointStamped, xy_callback2)
    rospy.Subscriber('/aruco_single2/pose', PoseStamped, heading_callback2)

    rospy.loginfo('Subscribing to Marker data for moving base')

    cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    rospy.Rate(10) # 10hz

def xy_callback1(point):
    global x_global1
    global y_global1
    H = [[  -0.42956214,    0.01246246,  154.22523254],
         [  -0.02506026,    1.02581476,  -50.04769301],
         [   0.00005985,    0.00307095,    1.000     ]]

    scale1 = 1
    unscaled1 = np.dot(np.dot(H,np.array([[point.point.x],[point.point.y],[1.0]])),scale1)
    x_global1 = unscaled1[0]/unscaled1[-1]
    y_global1 = unscaled1[1]/unscaled1[-1]

    Hn2 = [[  -0.40493398,    0.01448238,  152.45101186],
           [  -0.01992332,    1.29879782,  124.13940842],
           [  -0.00003885,    0.00276678,    1.0       ]]

def heading_callback1(pose):
    global yaw1
    quaternion = (pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    roll = euler[0]
    pitch = euler[1]
    yaw1 = euler[2]


def xy_callback2(point):
    global x_global2
    global y_global2
    H = [[  -0.40493398,    0.01448238,  152.45101186],
         [  -0.01992332,    1.29879782,  124.13940842],
         [  -0.00003885,    0.00276678,    1.000     ]]

    scale2 = 1
    unscaled2 = np.dot(np.dot(H,np.array([[point.point.x],[point.point.y],[1.0]])),scale2)
    x_global2 = unscaled2[0]/unscaled2[-1]
    y_global2 = unscaled2[1]/unscaled2[-1]


def heading_callback2(pose):
    global yaw2
    quaternion = (pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    roll = euler[0]
    pitch = euler[1]
    yaw2 = euler[2]

def move_to_function(mob_pose):
    global cmd_vel_pub
    global speed_new
    global speed_old
    global total_distance
    global max_speed
    global min_speed
    global turn_error

    vx_max = 0.18

    rospy.sleep(1)
    
    cmdvel = Twist()
    cmdvel.linear.x = 0
    cmdvel.linear.y = 0
    cmdvel.linear.z = 0

    cmdvel.angular.x = 0
    cmdvel.angular.y = 0
    cmdvel.angular.z = 0

    wp_x = mob_pose.a
    wp_y = mob_pose.b

    check_done = 0

# ============= Check 1 ===========

    if (159 > y_global1) & (0 != y_global1):
        curr_x = x_global1
        curr_y = y_global1
        yaw = yaw1
    else:
        curr_x = x_global2
        curr_y = y_global2
        yaw = yaw2

    if wp_y < curr_y:
        yaw = yaw + m.pi
    else:
        yaw = yaw


    target_theta = m.atan2((wp_x - curr_x), (wp_y - curr_y))

# ============= Check 2 ===========

    if -0.5 * m.pi > target_theta:
        target_theta = target_theta + m.pi
    elif 0.5 * m.pi < target_theta:
        target_theta = target_theta - m.pi
    else:
        target_theta = target_theta

    theta_tolerance = 0.05
    distance_tolerance = 10

# ============= Main loop - Turn ===========

    while (abs(yaw - target_theta) > theta_tolerance) and (yaw < 0.04) and (yaw > -0.04):
        
        if (159 > y_global1) & (0 != y_global1):
            curr_x = x_global1
            curr_y = y_global1
            yaw = yaw1
        else:
            curr_x = x_global2
            curr_y = y_global2
            yaw = yaw2

        if -0.5 * m.pi > target_theta:
            target_theta = target_theta + m.pi
        elif 0.5 * m.pi < target_theta:
            target_theta = target_theta - m.pi
        else:
            target_theta = target_theta

        rospy.loginfo('Turning')

        # print mob_pose
        # rospy.loginfo('Target theta')
        # print target_theta

        if 0 < target_theta:
            w = 0.18   #CW
        else:
            w = -0.18   #CCW

        # w = -0.1 * (yaw - target_theta)
        # if abs(w) > 0.12:

        cmdvel.linear.x = w

        # else:
        #     cmdvel.linear.x = np.sign(w) * 0.11

        cmdvel.angular.z = 0

        print 'heading:',yaw
        print 'target_theta:',target_theta
        print "angle difference"
        print abs(yaw - target_theta)

        cmd_vel_pub.publish(cmdvel)

        rospy.sleep(0.01)

        print 'x_global1:',x_global1
        print 'y_global1:',y_global1
        print 'x_global2:',x_global2
        print 'y_global2:',y_global2

        print 'x:',curr_x
        print 'y:',curr_y

    last_distance = np.linalg.norm([wp_x - curr_x, wp_y - curr_y])
    total_distance.append(np.linalg.norm([wp_x - curr_x, wp_y - curr_y]))
    distance_percent = ((total_distance[0] - (np.linalg.norm([wp_x - curr_x, wp_y - curr_y])))/(total_distance[0]))

# ============= Check 3 ===========

    if (yaw > 0.04) or (yaw < -0.04):
    	turn_error =  1


# ============= Main loop - Go ===========

    while (np.linalg.norm([wp_x - curr_x, wp_y - curr_y]) >= distance_tolerance) and (0 == turn_error):  # 4 * np.linalg.norm([curr_x - wp_x, curr_y - wp_y]) * m.tan(0.05):

        if (159 > y_global1) & (0 != y_global1):
            curr_x = x_global1
            curr_y = y_global1
            yaw = yaw1
        else:
            curr_x = x_global2
            curr_y = y_global2
            yaw = yaw2

        if (1 != check_done):
            if (wp_y < curr_y):
                direction = -1
            else:
                direction = 1

        check_done = 1

        distance_percent = ((total_distance[0] - (np.linalg.norm([wp_x - curr_x, wp_y - curr_y])))/(total_distance[0]))

        rospy.loginfo('distance percent: %f',distance_percent)

        total_distance.append(np.linalg.norm([wp_x - curr_x, wp_y - curr_y]))

        
        if (distance_percent) <= 0.3:
            speed_new = min(speed_old + 0.0015,max_speed)
        elif (distance_percent) >= 0.8:
            speed_new = max(speed_old - 0.0015,min_speed)
            print ' \n current distance - t-10 distance : ', np.linalg.norm([wp_x - curr_x, wp_y - curr_y]) - total_distance[-10]
        else:
        	speed_new = speed_old

        cmdvel.linear.x = 0
        cmdvel.angular.z =  direction * speed_new

        rospy.loginfo('Going forward')

        cmd_vel_pub.publish(cmdvel)

        rospy.sleep(0.01)

        speed_old = speed_new

        last_distance = np.linalg.norm([wp_x - curr_x, wp_y - curr_y])

        print 'x:',curr_x
        print 'y:',curr_y

        print 'distance difference:',np.linalg.norm([wp_x - curr_x, wp_y - curr_y])

    return move_toResponse("Reached")


def move_to_server():

    # rospy.init_node('move_to_server_node')
    s = rospy.Service('move_to_service', move_to, move_to_function)
    print "Waiting for coordinates..."
    rospy.spin()


if __name__ == "__main__":
    initialize()
    move_to_server()