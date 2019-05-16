#!/usr/bin/env python

import math

def make_test_dataset():
    relays = dict()
    analogInputs = dict()
    
    relays['posmvRelay'] = False
    relays['em2040Relay'] = True
    relays['stormRelay'] = False
    relays['jet2PwrButtonRelay'] = True
    relays['aisRelay'] = False

    analogInputs['posmvAmps'] = 1.1
    analogInputs['em2040Amps'] = 2.1
    analogInputs['jet2Amps'] = 1.3
    
    return relays, analogInputs


def bool_to_enum(value):
    if value:
        return 1   # TODO: use msg constants
    else:
        return 0


def associate_relay_with_current(relays, analogInputs):
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
    # looking for matches in the analogInputs.  If we find a match remove that value from
    # the analogInput dict so that whatever remains there are current sensors with no
    # associated relay
    for relay_name in relays:
        base_name = relay_name.replace('Relay', '')  # E.G. posmvRelay is now posmv
        amp_name  = base_name + 'Amps'
        
        if amp_name in analogInputs:
            # we have a match
            all_values[base_name] = {'relay_status' : bool_to_enum(relays[relay_name]), 
                                     'current_amps' : analogInputs[amp_name]}
            
            # since we found a match, remove this entry from the analogInputs dict
            # this lets us assume the remaining values in analogInputs are current sensors without a relay
            del analogInputs[amp_name]
        else:
            # relay without an associated current sensor
            all_values[relay_name] = {'relay_status' : bool_to_enum(relays[relay_name]), 
                                      'current_amps' : float('nan')}
    
    # now loop through the remaining analogInputs
    # TODO: replace '2' below with constant defined in message
    for analog_name in analogInputs:
        all_values[analog_name] = {'relay_status' : 2, 'current_amps' : analogInputs[analog_name]}
        
    return all_values



relays, analogs = make_test_dataset()
all_values = associate_relay_with_current(relays, analogs)
print all_values


for val in all_values:
    assert 'relay_status' in all_values[val]
    assert 'current_amps' in all_values[val]

for val in all_values:
    assert all_values[val]['relay_status'] in [0,1,2], "%s has an invalid relay_status %d" % (val, all_values[val]['relay_status'])

for val in all_values:
    assert type(all_values[val]['current_amps']) is float

assert 'posmv' in all_values
assert 'posmvRelay' not in all_values
assert 'posmvAmps' not in all_values
assert all_values['posmv']['relay_status'] == 0
assert all_values['posmv']['current_amps'] - 1.1 < 0.0001

assert 'em2040' in all_values
assert 'em2040Relay' not in all_values
assert 'em2040Amps' not in all_values
assert all_values['em2040']['relay_status'] == 1
assert all_values['em2040']['current_amps'] - 2.1 < 0.0001

assert 'stormRelay' in all_values
assert 'storm' not in all_values
assert all_values['stormRelay']['relay_status'] == 0
assert math.isnan(all_values['stormRelay']['current_amps'])

assert 'jet2PwrButtonRelay' in all_values
assert 'jet2PwrButton' not in all_values
assert all_values['jet2PwrButtonRelay']['relay_status'] == 1, "should be 1, is %d" % all_values['jet2PwrButtonRelay']['relay_status']
assert math.isnan(all_values['jet2PwrButtonRelay']['current_amps'])

assert 'aisRelay' in all_values
assert 'ais' not in all_values
assert all_values['aisRelay']['relay_status'] == 0
assert math.isnan(all_values['aisRelay']['current_amps'])

assert 'jet2Amps' in all_values
assert 'jet2' not in all_values
assert all_values['jet2Amps']['relay_status'] == 2
assert all_values['jet2Amps']['current_amps'] - 1.3 < 0.00001

