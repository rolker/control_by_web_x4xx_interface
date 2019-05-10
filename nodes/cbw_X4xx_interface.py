#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from marine_msgs.msg import Helm
from std_msgs.msg import String

helm_publisher = None
helm_mode_publisher = None
state = 'standby'

def joystickCallback(msg):
    global state
    
    state_request = None
    if msg.buttons[0]:
        state_request = 'manual'
    if msg.buttons[1]:
        state_request = 'autonomous'
    if msg.buttons[2]:
        state_request = 'standby'
    if state_request is not None and state_request != state:
        helm_mode_publisher.publish('helm_mode '+state_request)
        state = state_request
    
    if state == 'manual':
        helm = Helm()
        helm.header.stamp = rospy.Time.now()
        helm.throttle = msg.axes[1]
        helm.rudder = -msg.axes[3]
        helm_publisher.publish(helm)
    
if __name__ == '__main__':
    rospy.init_node('joy_to_helm')
    helm_publisher = rospy.Publisher('/udp/helm', Helm, queue_size=10)
    helm_mode_publisher = rospy.Publisher('/send_command', String, queue_size=10)
    joy_subscriber = rospy.Subscriber('/joy', Joy, joystickCallback)
    rospy.spin()
    
