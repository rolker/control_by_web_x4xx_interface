#!/usr/bin/env python

"""

The X4xx responds to the state.xml request with something like this:

<datavalues>
    <vin1>23.6</vin1>
    <register1>0</register1>
    <analogInput1>1.416</analogInput1>
    <relay1>1</relay1>
    <oneWireSensor1>20.78</oneWireSensor1>
    <oneWireSensor2>44.90</oneWireSensor2>
    <relay2>0</relay2>
    <relay3>1</relay3>
    <relay4>1</relay4>
    <relay5>1</relay5>
    <relay6>0</relay6>
    <relay7>0</relay7>
    <relay8>1</relay8>
    <relay9>0</relay9>
    <relay10>0</relay10>
    <analogInput3>0.88</analogInput3>
    <analogInput2>0.03</analogInput2>
    <analogInput4>0.57</analogInput4>
    <analogInput5>-0.05</analogInput5>
    <analogInput6>0.07</analogInput6>
    <utcTime>1557441793</utcTime>
    <timezoneOffset>0</timezoneOffset>
    <serialNumber>00:0C:C8:04:41:8C</serialNumber>
</datavalues>

"""


import requests                     # for handling http requests
import xml.etree.ElementTree as ET  # for parsing XML response


def poll_X4xx_status():
    # make the request
    response = requests.get('http://192.168.100.212/state.xml')
    if response.status_code == 200:
        
        # parse the response XML
        root = ET.fromstring(response.content)

        relays = dict()
        analogInputs = dict()
        
        # step through the (flat) parsed response
        # use the tag name to assign the correct type (bool for relays, float for current)
        for element in root:
            if element.tag.endswith('Relay'):
                if element.text == '0':
                    relays[element.tag] = False
                elif element.text == '1':
                    relays[element.tag] = True
            
            elif element.tag.endswith('Amps'):
                analogInputs[element.tag] = float(element.text)

        return relays, analogInputs


def command_X4xx_relay(msg):
    
    


if __name__ == '__main__':
    rospy.init_node('joy_to_helm')
