#!/usr/bin/env PYTHON

from __future__ import print_function

import gc
import sys
from threading import Lock

import rospy
from rospy.topics import Subscriber
from std_msgs.msg import String

from baxter_core_msgs.msg import EndpointState

# SUBTOPIC = 'test_multisub_memleak'
#SUBTOPIC = '/brain/rstatus'
#MSGTYPE = String
SUBTOPIC = '/robot/limb/left/endpoint_state'
MSGTYPE = EndpointState

TIMEOUT = 10.0 #seconds

_last_callback = None
def callback(data):
    global _last_callback
    print("message received", data)
    #print("message received", data.data)
    _last_callback = data


if 0:
    s = rospy.Subscriber(topic, t_type)
    s.unregister()



def _callback_fn(data, i_sub):
    rospy.loginfo("Got callback for {}".format(i_sub))


_test_data = None
def _cb_global(data):
    global _test_data
    #_test_data = data.data
    _test_data = data
    print("Data received.", end='')

def test_one_sub(i_test=0, *args, **kwargs):
    global _test_data
#     got_data = None

    print("{}: Starting Subscriber.".format(i_test), end='')
    s = rospy.Subscriber(SUBTOPIC, MSGTYPE, _cb_global)
    while _test_data is None and not rospy.is_shutdown():
#         print('.', end='')
        rospy.sleep(0.01)
    s.unregister()
#     print(" |   Subscriber Unregistered. ", end='')
    _test_data = None
    print("Test finished.{}".format(i_test))
#     sys.stdout.flush()

def log_line():
    ros_counts = get_counts()
    print("Garbage collect counts: {}".format(ros_counts))

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
    return rostypes


def test_open_multi(multi_subs=50, sleep_preunsub=20.0, *args, **kwargs):
    subs = list()
    for i_sub in xrange(multi_subs):
        rospy.loginfo("- Adding Subscriber #{} of {}".format(
            i_sub, multi_subs
        ))
        subs.append(rospy.Subscriber(SUBTOPIC, MSGTYPE, _callback_fn,
                                     callback_args=i_sub))
    print("=== ALL SUBSCRIBERS ADDED ===")

    if sleep_preunsub:
        print("== Sleeping for {} seconds ==".format(sleep_preunsub))
        rospy.sleep(sleep_preunsub)

    for i, x_sub in enumerate(subs):
        rospy.loginfo("- Unregistering Subscriber #{} of {}".format(
            i, multi_subs
        ))
        x_sub.unregister()
#         sub[i] = None


def run_multi_test(test_fn, multi_test=20, do_print=True, log_fn=None,
                   *args, **kwargs):
    hbr1 = "="*50
    hbr2 = '-'*50
    i_test = 1

    rospy.loginfo("STARTING TEST...")
    while i_test <= multi_test or multi_test <0 and not rospy.is_shutdown():
        if do_print:
            print((hbr1 + "\nRunning Test Function {}: Run {} of {}...\n\n"
                   ).format(test_fn.__name__, i_test, multi_test))

        test_fn(i_test=i_test, *args, **kwargs)
        if log_fn:
            log_fn()

        if do_print:
            print(("\n Finished Test Function {}: Run {} of {}\n" + hbr1
                   ).format(test_fn.__name__, i_test, multi_test))
        i_test += 1

    rospy.loginfo("FINISHED TEST.")


if __name__ == '__main__':
    rospy.init_node('test_rospy_multisub_memory',
#                     log_level=rospy.DEBUG,
    )

    run_multi_test(test_one_sub, multi_test=-1, do_print=False,
                   log_fn=log_line)

#     run_multi_test(test_open_multi, multi_test=50,
#                    multi_subs=1, sleep_preunsub=0.5)

#     test_open_multi(multi_subs=50, sleep_preunsub=5.0)

