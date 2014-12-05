#!/usr/bin/env python

'''
Direct websocket test for rosbridge_websocket.
'''

from ws4py.client.threadedclient import WebSocketClient
import threading
import json

from time import sleep
from multiprocessing.process import Process
import random

import pprint


ROSBRIDGE_HOST = "ws://{robot}:9090"
SUBTOPIC = '/robot/limb/left/endpoint_state'
MSGTYPE = 'baxter_core_msgs/EndpointState'


class RosBridgeClient(WebSocketClient):

    def __init__(self, webSocket):
        super(RosBridgeClient, self).__init__(webSocket)
        self.currentTopicID = 1
        self.subscriptions  = {}
        self.subLock        = threading.Lock()

    def subscribe(self, topic, messageType, callback):
        topID = self.currentTopicID
        self.currentTopicID = self.currentTopicID + 1
        msg = {'op': 'subscribe', 'topic': topic,
               'type': messageType, 'id': topID, 'throttle_rate': 5}
        self.send(json.dumps(msg))
        with self.subLock:
            self.subscriptions[topID] = {'callback': callback, 'topic': topic}
        return topID

    def unsubscribe(self, subID):
        msg = {'op': 'unsubscribe', 'id': subID,
               'topic': self.subscriptions[subID]['topic']}
        with self.subLock:
            del self.subscriptions[subID]
        self.send(json.dumps(msg))

    """ IT'S ALL ABOUT THIS FUNCTION!!"""
    def received_message(self, m):
        data = json.loads(m.data)
        if data['op'] == 'publish':
            r_topic = data['topic']
            r_msg = json.dumps(data['msg'])

            with self.subLock:
                for sub in self.subscriptions.itervalues():
                    if r_topic == sub['topic']:
                        sub['callback'](r_msg)

# class MessageGetter():
# 
#     def __init__(self, rc, topic, messageType):
#         self.rosClient = rc
#         self.topic = topic
#         self.messageType = messageType
# 
#         self.message = ""
#         self.messageQueue = []
#         self.hasResponse = False
#         self.subID = ""
#         self.topicArrived = threading.Event()
#         self.numRecieved = 0
# 
#     def GetMessage(self, timeout=2.0):
#         self.topicArrived.clear()
#         self.subID = self.rosClient.subscribe(
#             self.topic, self.messageType, self.MessageRecieved)
#         self.topicArrived.wait(timeout)
#         self.rosClient.unsubscribe(self.subID)
#         return self.hasResponse
# 
#      def MessageRecieved(self,data):
#         self.message = data
#         self.hasResponse = True
#         self.topicArrived.set()
#         self.numRecieved = self.numRecieved + 1
#         self.messageQueue.append(data)


def test_1_2_base():
    ws = RosBridgeClient(ROSBRIDGE_HOST)
    ws.connect()
    mg = MessageGetter(ws, SUBTOPIC, MSGTYPE)
    mg.GetMessage()
    ws.close()

class SimpleWSClient(WebSocketClient):
    def __init__(self, ws_url, msg_cb=None):
        super(SimpleWSClient, self).__init__(ws_url)
        self.msg_cb = msg_cb

    def received_message(self, m):
        data = json.loads(m.data)
        if data['op'] == 'publish':
            if self.msg_cb:
                self.msg_cb(data)

            r_topic = data['topic']
            r_msg = json.dumps(data['msg'])

            self.subLock.acquire(True)
            for sub in self.subscriptions.itervalues():
                if r_topic == sub['topic']:
                    sub['callback'](r_msg)
            self.subLock.release()

def w_test_1_2():
    # test 'globals'
    subLock = threading.Lock()
    subscriptions = dict()
    def received_message(m):
        data = json.loads(m.data)
        if data['op'] == 'publish':
            r_topic = data['topic']
            r_msg = json.dumps(data['msg'])
            print data
            print data.keys()

            with subLock:
                for sub in subscriptions.itervalues():
                    if r_topic == sub['topic']:
                        sub['callback'](r_msg)

    w_type = 2
    if w_type==1:
        ws = WebSocketClient(ROSBRIDGE_HOST)
    elif w_type==2:
        ws = WebSocketClient(ROSBRIDGE_HOST)
        ws.received_message = received_message
#     elif w_type==3:
#         ws = WebSocketClient(ROSBRIDGE_HOST)
#         def received_message(self, m):
#             data = json.loads(m.data)
#             if data['op'] == 'publish':
#                 r_topic = data['topic']
#                 r_msg = json.dumps(data['msg'])
# 
#                 self.subLock.acquire(True)
#                 for sub in self.subscriptions.itervalues():
#                     if r_topic == sub['topic']:
#                         sub['callback'](r_msg)
#                 self.subLock.release()
# #         ws.received_message = received_message
    else:
        ws = RosBridgeClient(ROSBRIDGE_HOST)



    ws.connect()

    subLock = threading.Lock()
    subscriptions = dict()
    current_subID = 1
    # RUN: One Topic (per topic code)
    topicArrived = threading.Event()
    topicArrived.clear()
    def genf(i_topic=0):
        return lambda data: topicArrived.set()

#     subID = ws.subscribe(SUBTOPIC, MSGTYPE, genf())
    def subscribe(current_subID, topic, messageType, callback):
        subID = current_subID
        current_subID += 1
        msg = {'op': 'subscribe', 'topic': SUBTOPIC,
               'type': MSGTYPE, 'id': subID, 'throttle_rate': 5}
        ws.send(json.dumps(msg))
        with subLock:
            subscriptions[subID] = {'callback': callback, 'topic': SUBTOPIC}
        return subID
    subID = subscribe(current_subID, SUBTOPIC, MSGTYPE, genf())

    got_msg = topicArrived.wait(2.0)

#     ws.unsubscribe(subID)
    def unsubscribe(subID):
        msg = {'op': 'unsubscribe', 'id': subID, 'topic': SUBTOPIC}
        with subLock:
            del subscriptions[subID]
        ws.send(json.dumps(msg))
    unsubscribe(subID)



    ws.close()


def hack_test_4_1_2(n_topics=10):
    # test 'globals'
    subLock = threading.Lock()
    subscriptions = dict()
    def received_message(m):
        data = json.loads(m.data)
        if data['op'] == 'publish':
            r_topic = data['topic']
            r_msg = json.dumps(data['msg'])
#             print data
#             print data.keys()

            with subLock:
                for sub in subscriptions.itervalues():
                    if r_topic == sub['topic']:
                        sub['callback'](r_msg)

    w_type = 2
    if w_type==1:
        ws = WebSocketClient(ROSBRIDGE_HOST)
    elif w_type==2:
        ws = WebSocketClient(ROSBRIDGE_HOST)
        ws.received_message = received_message
#     elif w_type==3:
#         ws = WebSocketClient(ROSBRIDGE_HOST)
#         def received_message(self, m):
#             data = json.loads(m.data)
#             if data['op'] == 'publish':
#                 r_topic = data['topic']
#                 r_msg = json.dumps(data['msg'])
# 
#                 self.subLock.acquire(True)
#                 for sub in self.subscriptions.itervalues():
#                     if r_topic == sub['topic']:
#                         sub['callback'](r_msg)
#                 self.subLock.release()
# #         ws.received_message = received_message
    else:
        ws = RosBridgeClient(ROSBRIDGE_HOST)


    ws.connect()

#     subLock = threading.Lock()
#     subscriptions = dict()
    current_subID = 1
    for i_top in xrange(n_topics):
        # RUN: One Topic (per topic code)
        topicArrived = threading.Event()
        topicArrived.clear()
        def genf(i_topic=0):
            return lambda data: topicArrived.set()

    #     subID = ws.subscribe(SUBTOPIC, MSGTYPE, genf())
        def subscribe(current_subID, topic, messageType, callback):
            subID = current_subID
            current_subID += 1
            msg = {'op': 'subscribe', 'topic': SUBTOPIC,
                   'type': MSGTYPE, 'id': subID, 'throttle_rate': 5}
            ws.send(json.dumps(msg))
            with subLock:
                subscriptions[subID] = {'callback': callback, 'topic': SUBTOPIC}
            return subID
        subID = subscribe(current_subID, SUBTOPIC, MSGTYPE, genf(i_top))

        got_msg = topicArrived.wait(2.0)

    #     ws.unsubscribe(subID)
        def unsubscribe(subID):
            msg = {'op': 'unsubscribe', 'id': subID, 'topic': SUBTOPIC}
            with subLock:
                del subscriptions[subID]
            ws.send(json.dumps(msg))
        unsubscribe(subID)

    ws.close()



def skim_test_4_1_2(n_topics=10):
    # test 'globals'
    subLock = threading.Lock()
    subscriptions = dict()
    def received_message(m):
        data = json.loads(m.data)
        if data['op'] == 'publish':
            r_topic = data['topic']
            r_msg = json.dumps(data['msg'])

            with subLock:
                for sub in subscriptions.itervalues():
                    if r_topic == sub['topic']:
                        sub['callback'](r_msg)

    ws = WebSocketClient(ROSBRIDGE_HOST)
    ws.received_message = received_message

    ws.connect()

    current_subID = 1
    for i_top in xrange(n_topics):
        # RUN: One Topic (per topic code)
        topicArrived = threading.Event()
        topicArrived.clear()
        def genf(i_topic=0):
            return lambda data: topicArrived.set()

        # Subscribe
        subID = current_subID
        current_subID += 1
        msg = {'op': 'subscribe', 'topic': SUBTOPIC,
               'type': MSGTYPE, 'id': subID, 'throttle_rate': 5}
        ws.send(json.dumps(msg))
        with subLock:
            subscriptions[subID] = {'callback': genf(i_top),
                                    'topic': SUBTOPIC}

        # Wait for Msg
        got_msg = topicArrived.wait(2.0)

        # Unsubscribe
        msg = {'op': 'unsubscribe', 'id': subID, 'topic': SUBTOPIC}
        with subLock:
            del subscriptions[subID]
        ws.send(json.dumps(msg))
        sleep(0.01)  # 0.1 doesn't seem to cause the problem (after 20hr)...

    ws.close()



def test_1_2():
    if 1:
        ws = WebSocketClient(ROSBRIDGE_HOST)
    else:
        ws = RosBridgeClient(ROSBRIDGE_HOST)
    ws.connect()

    # RUN: One Topic 
    topicArrived = threading.Event()
    topicArrived.clear()
    def genf(i_topic=0):
        return lambda data: topicArrived.set()
    subID = ws.subscribe(SUBTOPIC, MSGTYPE, genf())
    got_msg = topicArrived.wait(2.0)
    ws.unsubscribe(subID)


    ws.close()


def test_4_1(n_topics=10):
    if 1:
        ws = WebSocketClient(ROSBRIDGE_HOST)
    else:
        ws = RosBridgeClient(ROSBRIDGE_HOST)
    ws.connect()

    topicArrived = threading.Event()
    for i_top in xrange(n_topics):
        topicArrived.clear()
        def genf(i_topic=0):
            return lambda data: topicArrived.set()

        subID = ws.subscribe(SUBTOPIC, MSGTYPE, genf(i_top))
        got_msg = topicArrived.wait(2.0)
        ws.unsubscribe(subID)

    ws.close()





def run_tests(test_fn, num_counts=50, p_tester=0, p_sleep=0):
    tester_id = p_tester
    test_id = test_fn.func_name
    test_iter = 1
    print("[P{}]: Start testing rosbridge: TEST(s) {}".format(
        tester_id, test_id))
    if p_sleep:
        print("Pre-Sleeping for {}s...".format(p_sleep))
        sleep(p_sleep)
#         print("Sleep done. START RUN")

    print("[P{}T{}]: =======    BEGINNING TEST-CASE {}   ========".format(
        tester_id, test_id, tester_id))
    print("[P{}T{}]:   - Using {} Topics on 1 socket.".format(
        tester_id, test_id, num_counts))

#     print("---  Start TEST {}-{}  ---".format(n_test, p_i))
    test_fn(num_counts)
#     print("---  Finish TEST {}-{}  ---".format(n_test, p_i))

    print("[P{}T{}]: =======       END OF TEST-CASE {}   ========".format(
        tester_id, test_id, tester_id))

def run_thread_tests(test_fn):
    num_proc = 100
    num_iter = 6
    my_p = [Process(target=run_tests, args=(test_fn, num_iter, p_i,
                                            (float(p_i)*0.0 + random.uniform(0.01, 5.0))))
            for p_i in xrange(num_proc)]

    for p in my_p:
        p.start()
    print("================================\n" * 5)
    p_done = [0] * len(my_p)
    for (i, p) in enumerate(my_p):
        p.join()
        p_done[i] = 1 if p.is_alive() else 0
    if any(p_done):
        sleep(1)
        for (i, p) in enumerate(my_p):
            p.join(2)
            p_done[i] = p.is_alive()
        print("(Not) Done's:\n    {}".format(p_done))
        if any(p_done):
            print(" THEY DIDN'T all FINISH! ")
    print("TESTs alllll done.")


if __name__ == '__main__':
    global ROSBRIDGE_HOST
    robot = 'pv1106'
    robot = 'juneau'
    ROSBRIDGE_HOST = "ws://{robot}:9090".format(robot=robot)

    print("Testing!")
#     w_test_1_2()
#     hack_test_4_1_2()



    run_thread_tests(skim_test_4_1_2)



    print("Tested!")





