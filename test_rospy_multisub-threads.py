#!/usr/bin/env PYTHON
from signal import signal, SIGINT, SIG_DFL
from __builtin__ import xrange

def add_dowser():
    import cherrypy
    import dowser
    cherrypy.config.update({
        'environment': 'embedded',
        'server.socket_port': 8090,
    })
    cherrypy.tree.mount(dowser.Root(), '/dowser')
    cherrypy.engine.start()

import gc

import rospy
from std_msgs.msg import String

from baxter_core_msgs.msg import EndpointState

import random
# from multiprocessing.process import Process
import threading
import sys

#SUBTOPIC = '/brain/rstatus'
# SUBTOPIC = '/chatter'
#MSGTYPE = String
SUBTOPIC = '/robot/limb/right/endpoint_state'
MSGTYPE = EndpointState


def do_sub():
    s = rospy.Subscriber(SUBTOPIC, MSGTYPE, lambda m: m)
    rospy.sleep(0.5)
    s.unregister()


def get_counts(ros_only=True, do_print=True):
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
    if do_print:
        print("Garbage collect counts: {}".format(rostypes))
        sub_counts = sorted(
            [(k.__name__,v) for k,v in rostypes.iteritems() if 'Sub' in str(k)],
            reverse=True
        )
        print("SUMMARY: {}".format(sub_counts))
    return rostypes


_impl_lock = threading.Lock()
_impl_count = 0
def add_impl(count=1):
    with _impl_lock:
        _impl_count += count

def get_extra_impls():
    with _impl_lock:
        r_impls = _impl_count
    return r_impls

def check_for_extras(l_objects):
    sub_type = rospy.topics._SubscriberImpl
    if sub_type in l_objects and l_objects[sub_type] > 1:
        add_impl(l_objects[sub_type])
        return l_objects[sub_type]
    return 0


def run_test(loops=-1, pre_sleep=0.0, thread_id=0):
    if pre_sleep:
        rospy.sleep(pre_sleep)
    prestr = "####{:>5}## ".format(thread_id) if thread_id else "## "
#     get_counts()
    i = 0
    while not rospy.is_shutdown() and (loops<0 or i<loops):
        i += 1
#         print "{}Running test {}.. (Pre-Sub)".format(prestr, i)
        do_sub()
    if 1:
        rostypes = get_counts(do_print=False)
        print(("{}Running test {:>3}.."
               " Garbage collect counts: {}").format(prestr, i, rostypes))
        sub_counts = sorted(
            [(k.__name__,v) for k,v in rostypes.iteritems() if 'Sub' in str(k)],
            reverse=True
        )
        print("{}SUMMARY test {:>3}: {}".format(prestr, i, sub_counts))
        check_for_extras(rostypes)


def threaded_run_tests(num_threads=100, num_loops=6):
    """Run test fn in multiple threads"""
    test_threads = list()
    for p_i in xrange(1, num_threads+1):
        pre_sleep = float(p_i)*0.0 + random.uniform(0.01, 10.0)
        test_threads.append(
            threading.Thread(target=run_test, args=(num_loops, pre_sleep, p_i))
        )
    # Start tests
    for p in test_threads:
        p.start()
    print(("="*20 + "\n") * 5)
    print("All tests started...")
    rospy.sleep(7)
    # Join all tests back together
    for p in test_threads:
        p.join()
#         print("Done: {}".format(not p.is_alive()))
        if p.is_alive():
            print("NOT Done!")


def main():
    add_dowser()
    rospy.init_node('test_rospy_multisub_memory')
    signal(SIGINT, SIG_DFL)

    # Single thread test
#     run_test()
#     sys.exit()

    # Multi-Thread test
#     threaded_run_tests(num_threads=100, num_loops=10)
#     sys.exit()
    for i in xrange(100):
        threaded_run_tests(num_threads=100, num_loops=10)
        print(" {}".format(i) * 40)
        get_counts()
        print("Tally of Extra _SubscriberImpl's: {}".format(get_extra_impls()))
        print("Sleeping...")
        rospy.sleep(12)
        if rospy.is_shutdown():
            break
    get_counts()
    print("Final Tally of Extra _SubscriberImpl's: {}".format(get_extra_impls()))
    print("TESTs alllll done ... i hope.")
    sys.exit()

if __name__ == '__main__':
    sys.exit(main())
