#!/usr/bin/env python

"""
Poll the status of a Control By Web X4xx series frame and publish the information as 
Robot Operating System (ROS) topics.  Currently, we publish one topic for information
common to the frame and then one per relay/current sensor module.

Currently this software requires specific configuration on the X4xx to work.  The X4xx
offers the ability to name the relays and current sensors.  For this software to properly
understand the configuration the following conventions must be applied

- no special characters in the name (I.E. no '()%$^'
- the names for relays MUST END IN 'Relay' CASE SENSITITVE
- the names for current sensors MUST END in 'Amps' CASE SENSITIVE

To get the status we poll the X4xx:

http://<ip address>/customState.xml

The X4xx responds with something like this:

<datavalues>
    <vin>22.7</vin>
    <register1>0</register1>
    <posmvAmps>1.433</posmvAmps>
    <posmvRelay>1</posmvRelay>
    <temperature>18.61</temperature>
    <humidity>43.22</humidity>
    <em2040Relay>1</em2040Relay>
    <stormRelay>1</stormRelay>
    <mystiqueRelay>1</mystiqueRelay>
    <jet212voltRelay>1</jet212voltRelay>
    <uhfEstopOverrideRelay>0</uhfEstopOverrideRelay>
    <aftSecondaryBilgeRelay>0</aftSecondaryBilgeRelay>
    <aisRelay>0</aisRelay>
    <vlp16Relay>1</vlp16Relay>
    <jet2PwrButtonRelay>0</jet2PwrButtonRelay>
    <stormAmps>1.64</stormAmps>
    <em2040Amps>5.37</em2040Amps>
    <mystiqueAmps>0.67</mystiqueAmps>
    <jet2Amps>-0.06</jet2Amps>
    <jet1Amps>1.35</jet1Amps>
    <utcTime>1557586418</utcTime>
    <timezoneOffset>0</timezoneOffset>
    <serialNumber>00:0C:C8:04:41:8C</serialNumber>
</datavalues>


This application parses the response and forwards it to ROS like so:

/control_by_web_status
/control_by_web_status/aftSecondaryBilgeRelay
/control_by_web_status/em2040
/control_by_web_status/jet1Amps
/control_by_web_status/jet212voltRelay
/control_by_web_status/jet2Amps
/control_by_web_status/jet2PwrButtonRelay
/control_by_web_status/mbrRelay
/control_by_web_status/mystique
/control_by_web_status/posmv
/control_by_web_status/storm
/control_by_web_status/uhfEstopOverrideRelay
/control_by_web_status/vlp16Relay


With a 5Hz update rate this node appears to consume approximately 390.07B/s

You can set the poll rate below (hard coded constant max_poll_rate_hz) which
appears to work as expected.


Instructions to run locally:
roscore in one tab
rosrun control_by_web_x4xx_interface cbw_x4xx_node.py
"""

import rospy                        # ROS python features
import requests                     # for handling http requests
import xml.etree.ElementTree as ET  # for parsing XML response
import math                         # for checking for NaNs                       
from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue


# ROS messages
from control_by_web_x4xx_interface.msg import X4xx_status
from control_by_web_x4xx_interface.msg import X4xx_relay_current_status


# the status URL of the X4xx
x4xx_url = 'http://192.168.100.212/customState.xml'

# this is the topic we publish the common status, its also the 'parent' topic
# for our module status
topic_root = 'control_by_web_status'

# this is a monotonically increasing value incremented each time we publish
# our common status message
sequence_number = 0

# a dictionary of module publishers
# the keys are the subtopic names and the values are the ROS publishers
module_publishers = dict()

# the maximum rate we should poll the X4xx for status
# NOTE: currently a command will issue its own poll so the overall poll rate can exceed this value
max_poll_rate_hz = 5

# the queque size for ROS publisher objects, select 1 because we only care about the latest message
publisher_queue_size = 1


def grab_X4xx_status():
    # make the request
    try:
        rospy.logdebug("polling X4xx at '%s'" % x4xx_url)
        response = requests.get(x4xx_url)
    except requests.exceptions.RequestException as e:
        rospy.logwarn_throttle(60.0, "error contacting Control By Web X4xx at '%s' : %s" % (x4xx_url, e))
        return None
    
    if response.status_code == 200:
        rospy.logdebug("successfully polled X4xx status")
        
        # parse the response XML
        root = ET.fromstring(response.content)

        relays = dict()
        current_sensors = dict()
        other = dict()
        
        # step through the (flat) parsed response
        # use the tag name to assign the correct type (bool for relays, float for current)
        try:
            for element in root:
                if element.tag.endswith('Relay'):
                    # relays
                    if element.text == '0':
                        relays[element.tag] = False
                    elif element.text == '1':
                        relays[element.tag] = True
                
                elif element.tag.endswith('Amps'):
                    # current sensors
                    current_sensors[element.tag] = float(element.text)
                    
                else:
                    # all other values
                    other[element.tag] = element.text
                    
            return relays, current_sensors, other
        except ValueError:
            rospy.logwarn_throttle(60.0, "unable to decode data from Control By Web X4xx at '%s'" % x4xx_url)
            return None
    
    else:
        rospy.logwarn_throttle(60.0, "unable to poll status of Control By Web X4xx at '%s'" % x4xx_url)
        return None


def bool_to_enum(value):
    if value:
        return X4xx_relay_current_status.RELAY_CLOSED
    else:
        return X4xx_relay_current_status.RELAY_OPEN


def associate_relay_with_current(relays, current_sensors):
    """
    This function takes a dictionary of relays and another of current sensors
    The goal is to associate the relays with their current sensors
    By convention the relay names end with 'Relay', the current sensors with 'Amps'
    
    We output a combined dictionary where each value is a dictionary with a 'relay_status'
    and 'current_amps' key.  
    
    For relays with no current sensor we set 'current_amps' to NaN 
    For current sensors with no relay we set 'relay_status' to 2 (0 is for open, 1 for closed) 
    """
    all_values = dict()
    
    # since we know there are more relays than current sensors loop through the relays
    # looking for matches in the current_sensors.  If we find a match remove that value from
    # the analogInput dict so that whatever remains there are current sensors with no
    # associated relay
    for relay_name in relays:
        base_name = relay_name.replace('Relay', '')  # E.G. posmvRelay is now posmv
        amp_name  = base_name + 'Amps'
        
        if amp_name in current_sensors:
            # we have a match
            all_values[base_name] = {'relay_status' : bool_to_enum(relays[relay_name]), 
                                     'current_amps' : current_sensors[amp_name]}
            
            # since we found a match, remove this entry from the current_sensors dict
            # this lets us assume the remaining values in current_sensors are current sensors without a relay
            del current_sensors[amp_name]
        else:
            # relay without an associated current sensor
            all_values[relay_name] = {'relay_status' : bool_to_enum(relays[relay_name]), 
                                      'current_amps' : float('nan')}
    
    # now loop through the remaining current_sensors
    for analog_name in current_sensors:
        all_values[analog_name] = {'relay_status' : X4xx_relay_current_status.RELAY_NA, 'current_amps' : current_sensors[analog_name]}
        
    return all_values

    

def send_X4xx_cmd(msg):
    rospy.loginfo("received cbw_x4xx_cmd: %s", msg)
    

def publish_common_values(publisher, diag_pub, values):
    """
    Here we publish the common values
    """
    status = X4xx_status()
    
    # header
    global sequence_number
    
    # only increment the sequence number here to reflect the fact that frame and module information came from
    # the same status poll
    sequence_number = sequence_number + 1
    status.header.seq      = sequence_number
    status.header.frame_id = "ROS control_by_web_x4xx"
    status.header.stamp    = rospy.Time.now()

    # establish diag msg
    ds = DiagnosticStatus()
    diag_array = DiagnosticArray()
    diag_array.header.stamp = rospy.Time.now()
    ds.name = 'cbw'

    # frame values
    if 'vin' in values:
        status.voltage_input = float(values['vin'])
        ds.values.append(KeyValue('vin', values['vin']))
    else:
        status.voltage_input = float('nan')
    
    if 'serialNumber' in values:
        status.serial_number = values['serialNumber']
        ds.hardware_id = values['serialNumber']
    else:
        status.serial_number = "unknown"
    
    if 'timezoneOffset' in values:
        status.timezone_offset = int(values['timezoneOffset'])
        ds.values.append(KeyValue('timezoneOffset', values['timezoneOffset']))
    else:
        status.timezone_offset = 99

    if 'payloadBoxTemp' in values:
        status.temperature_payload_box = float(values['payloadBoxTemp'])
        ds.values.append(KeyValue('payloadBoxTemp', values['payloadBoxTemp']))
    else:
        status.temperature_payload_box = float('nan')

    if 'payloadBoxHumidity' in values:
        status.humidity_payload_box = float(values['payloadBoxHumidity'])
        ds.values.append(KeyValue('payloadBoxHumidity', values['payloadBoxHumidity']))
    else:
        status.humidity_payload_box = float('nan')

    if 'cbwDistroBox' in values:
        status.temperature_cbw_box = float(values['cbwDistroBox'])
        ds.values.append(KeyValue('dristroboxTemp', values['cbwDistroBox']))
    else:
        status.temperature_cbw_box = float('nan')

    # and finally publish!
    rospy.logdebug("publishing common status")
    publisher.publish(status)
    
    # publish diag
    diag_array.status.append(ds)
    diag_pub.publish(diag_array)

def publish_module_values(all_modules):
    """ 
    This function publishes the individual module status 
    Currently only relays and current sensors) 
    """
    
    # these are the assumptions we make
    assert type(all_modules) is dict
    for module in all_modules:
        assert 'relay_status' in all_modules[module]
        assert 'current_amps' in all_modules[module]
    
    global module_publishers  # might have to only do this when we make a new one ...
    
    for module in all_modules:
        # first we have to determine if we have a publisher for this module yet
        if module not in module_publishers:
            module_topic = topic_root + '/' + module
            module_publishers[module] = rospy.Publisher(module_topic, X4xx_relay_current_status, queue_size=publisher_queue_size)
            rospy.loginfo("publishing '%s' module status on topic: %s" % (module, module_topic))
        assert module in module_publishers
        
        # at this point we have a publisher so prepare the message
        module_status              = X4xx_relay_current_status()
        module_status.header.seq          = sequence_number
        module_status.header.frame_id     = "ROS control_by_web_x4xx"
        module_status.header.stamp        = rospy.Time.now()
        
        module_status.name         = module        
        module_status.relay_status = all_modules[module]['relay_status']
        module_status.current_amps = all_modules[module]['current_amps']
        
        module_publishers[module].publish(module_status)
        

def cbw_x4xx_node():
    """
    This is the main function for the ROS node behavior.
    
    We setup our publishers then enter a loop that polls the status which is rate limited
    via a rospy.Rate object.
    """
    rospy.loginfo("initializing control_by_web_x4xx_interface node")
    rospy.init_node('control_by_web_x4xx_interface', anonymous=True)
    
    # setup the common status publisher
    common_pub = rospy.Publisher(topic_root, X4xx_status, queue_size=publisher_queue_size)
    rospy.loginfo("publishing overall status on topic: %s" % topic_root)
    
    #setup diagnostic publisher
    diagnostic_pub = rospy.Publisher('/diagnostics',DiagnosticArray,queue_size=10)    

    # here we can limit the rate we poll the X4xx
    status_rate = rospy.Rate(max_poll_rate_hz)
    
    rospy.loginfo("entering control_by_web_x4xx_status main loop")    
    while not rospy.is_shutdown():
        # first thing is to poll the X4xx, this gives the status for the frame and all modules
        status = grab_X4xx_status()
        if status is not None:
            relays, current_sensors, other = status
                
            # parse through the response to associate relays with their current sensors
            all_modules = associate_relay_with_current(relays, current_sensors)
        
            # now that we have all our information organized, publish the common values
            publish_common_values(common_pub, diagnostic_pub, other)
                            
            # now publish the module values
            publish_module_values(all_modules)

        # sleep to control the polling rate
        status_rate.sleep()
        
    
if __name__ == '__main__':
    # might want to do this a little better with commandline args, etc
    rospy.loginfo("calling main of cbw_x4xx_node.py")
    try:
        cbw_x4xx_node()
    except rospy.ROSInterruptException:
        pass
