# X4xx_status message contains the information common to a Control By Web X4XX series device
# separate messages are used to communicate individual module status - relay, current sensor, etc 

Header  header

float32 voltage_inputWAT
string  serial_number
uint8   timezone_offset
float32 temperature      # units are configured in device setup   
float32 humidity         # I assume relative
