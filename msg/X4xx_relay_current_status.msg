# X4xx_relay_and_current_status message combines the status of a Control By Web X4xx series relay and current sensing module
# these modules are separate but have been wired in a 1 to 1 fashion on the ASV Ben.

Header  header

string  name              # as displayed via customState.xml, _ or spaces are replaced with camel case
uint8   relay_status      # indicates if the relay is open, closed or not applicable
float32 current_amps      # if no current sensor is associated with relay the value will be NaN

# these constants should be the only vaules you find in 'relay_status'
uint8   RELAY_OPEN   = 0  # indicates this relay is open / powered off
uint8   RELAY_CLOSED = 1  # indicates this relay is closed / powered on
uint8   RELAY_NA     = 2  # indicates there is no relay
