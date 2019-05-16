
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
