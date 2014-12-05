#!/usr/bin/env python

# import cherrypy
# import dowser
# cherrypy.config.update({
#     'environment': 'embedded',
#     'server.socket_port': 8088,
# })
# cherrypy.tree.mount(dowser.Root(), '/dowser')
# cherrypy.engine.start()

import gc

import rospy
from std_msgs.msg import String


SUBTOPIC = '/brain/rstatus'
# SUBTOPIC = '/chatter'
MSGTYPE = String


def do_sub():
    s = rospy.Subscriber(SUBTOPIC, MSGTYPE, lambda m: m)
    rospy.sleep(1.0)
    s.unregister()


def get_counts(ros_only=True):
    # from dowser code
    gc.collect()
    typecounts = {}
    for obj in gc.get_objects():
        objtype = type(obj)
        if objtype in typecounts:
            typecounts[objtype] += 1
        else:
            typecounts[objtype] = 1
    if not ros_only:
        return typecounts
    rostypes = {k:v for k,v in typecounts.iteritems() if 'ros' in str(k)}
    print("Garbage collect counts: {}".format(rostypes))
    return rostypes


if __name__ == '__main__':
    rospy.init_node('test_rospy_multisub_memory')
    i=0
    get_counts()
    while not rospy.is_shutdown():
        i += 1
        print "Running test {}..".format(i),
        do_sub()
        get_counts()
