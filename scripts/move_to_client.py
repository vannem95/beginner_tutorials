#!/usr/bin/env python
import sys
import rospy
from beginner_tutorials.srv import *

def move_to_client(x, y):
    rospy.wait_for_service('move_to_service')
    try:
        add_two_ints = rospy.ServiceProxy('move_to_service', move_to)
        resp1 = add_two_ints(x, y)
        return resp1.sum
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 3:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
    else:
        print usage()
        sys.exit(1)
    print "Going to %s,%s"%(x, y)
    print "%s %s,%s"%(move_to_client(x, y),x,y)
