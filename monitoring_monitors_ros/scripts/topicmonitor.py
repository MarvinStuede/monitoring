#!/usr/bin/env python
'''
   Copyright (c) 2018, University of Hannover
                       Institute for Systems Engineering - RTS
                       Professor Bernardo Wagner
 
   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions
   are met:
 
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above
      copyright notice, this list of conditions and the following
      disclaimer in the documentation and/or other materials provided
      with the distribution.
    * Neither the name of Willow Garage, Inc. nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.
 
   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
   COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.
''' 
import time
from time import sleep

import rospy
from rospy.msg import AnyMsg
from rostopic import *
from monitoring_msgs.msg import *
from monitoring_core.monitor import Monitor
from monitoring_core.monitor import AggregationStrategies

class TopicObserver:
    def __init__(self, name, timeout):
        self.name = name
        self.timeout = timeout
        self.msg_missing = False
        self.oldness = 0.0
        self.last_recieved = rospy.Time.now()
        self._sub = rospy.Subscriber(name, AnyMsg, self.any_cb)
        # check the oldness with 4 times frequency of the timeout
        self._timer = rospy.Timer(rospy.Duration(timeout / 4.0), self.timer_cb)

    def any_cb(self, msg):
        self.last_recieved = rospy.Time.now()

    def timer_cb(self, tm):
        self.oldness = (rospy.Time.now() - self.last_recieved).to_sec()
        if (self.oldness > self.timeout):
            self.msg_missing = True
        else:
            self.msg_missing = False

def topicmonitor():
    rospy.init_node("topicmonitor")
    try:
        topics = rospy.get_param(rospy.get_name() + '/topics')
        frequency = rospy.get_param(rospy.get_name() + '/frequency')
        timeout = rospy.get_param(rospy.get_name() + '/timeout') # double
    except KeyError:
        print "value not set"
        quit()

    rate = rospy.Rate(frequency)
    monitor = Monitor("topic_monitor")
    last_xvalues_for_calc = 10
    hz_monitors = {}

    topic_obs = []
    for topic in topics:
        topic_obs.append(TopicObserver(topic["name"],timeout))

    for entry in topics:
        rthz = ROSTopicHz(-1, filter_expr=None)
        print entry['name']
        sub = rospy.Subscriber(entry['name'], rospy.AnyMsg, rthz.callback_hz)
        hz_monitors[entry['name']] = rthz

    start_timestamp = time.time()
    while not rospy.is_shutdown():  # main loop
        for tobs_obj in topic_obs:
             if tobs_obj.msg_missing:
                 # no new msg recieved
                 value = "Topic " + tobs_obj.name + " sends no data"
                 rospy.logwarn("no new messages: %s", value)
                 monitor.addValue(tobs_obj.name, value, "", 1.0, 1)
                 # TODO add tobs_obj.oldness if needed
             else:
                 # everything okay
                 monitor.addValue(tobs_obj.name, "okay", "", 0.0, 1)

        for entry in topics:

            n_ = len(hz_monitors[entry['name']].times)
            if n_ < last_xvalues_for_calc:
# TODO was ist wenn keine miS gesendet werden. Dann wird nichts ueberprueft und somit keine
# Fehlermeldung rausgegeben!!! && Wenn bereits 10 narchichten da, aber keine neuen mehr
# ankommen?!?
                if time.time()-start_timestamp <= 10: #nach 10 sekunden wird durchgeschaltet
                    continue
           #the or condition was added to get the actual state of hz not only if the list is empty


            #or (hz_monitors[entry['name']].msg_tn == hz_monitors[entry['name']].last_printed_tn)
            if (n_ == 0) :  # no mis received, division by 0 catching
                rospy.logwarn("no new miS for topic %s", entry['name'])
                value = "Topic " + entry['name'] + " sends no data"
                monitor.addValue(entry['name'], value, "", 1.0, 1)
                continue
# TODO this is mean overall time. Needs mean over since last seconds
            mean = sum(hz_monitors[entry['name']].times[-last_xvalues_for_calc:]) / last_xvalues_for_calc
            freq = 1. / mean if mean > 0. else 0
            #0.5 was actually 0.1 but the intervall can be bigger
            #if entry['frequency'] - 0.5*freq > freq:
            if 0.4*entry['frequency'] > freq:
                rospy.logwarn("Frequency of %s is too low: Expected: %f Actual: %f", entry['name'], entry['frequency'], freq)
                value = "Topic " + entry['name'] + " is on " + str(freq) + "Hz, Expected: " +  str(entry['frequency'])
                monitor.addValue(entry['name'], value, "", 1.0, 1)


        rate.sleep()


if __name__ == '__main__':
    try:
        topicmonitor()
    except rospy.ROSInterruptException:
        pass
