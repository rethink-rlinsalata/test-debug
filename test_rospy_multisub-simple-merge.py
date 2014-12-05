#!/usr/bin/env PYTHON

import gc

import rospy
from std_msgs.msg import String


# SUBTOPIC = 'test_multisub_memleak'
SUBTOPIC = '/brain/rstatus'
MSGTYPE = String


_test_data = None
def _cb_global(data):
    global _test_data
    _test_data = data.data
    print("Data received.", end='')

def test_one_sub(i_test=0, *args, **kwargs):
    global _test_data

    print("{}: Starting Subscriber.".format(i_test), end='')
    s = rospy.Subscriber(SUBTOPIC, MSGTYPE, _cb_global)
    while _test_data is None and not rospy.is_shutdown():
        rospy.sleep(0.01)
    s.unregister()
    _test_data = None
    print("Test finished.{}".format(i_test))


def do_sub():

    def _cb(data):
        print ".",
        pass

#     print("{}: Starting Subscriber.".format(i_test), end='')
    s = rospy.Subscriber(SUBTOPIC, MSGTYPE, _cb)
    rospy.sleep(1.0)
    s.unregister()
#     print("Test finished.{}".format(i_test))


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
                   log_fn=get_counts)

#     run_multi_test(test_open_multi, multi_test=50,
#                    multi_subs=1, sleep_preunsub=0.5)

#     test_open_multi(multi_subs=50, sleep_preunsub=5.0)

