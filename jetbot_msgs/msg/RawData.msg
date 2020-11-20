# Data packet sent from the JetBot firmware, according to the JetBot protocol v1
# sent by the jetbot in response to a Data Request.

uint8 which   # which data element this represents (e.g. Battery A Voltage, Firmware Version)
byte[2] value # raw binary data sent by the jetbot. How to decode this data depends on the 'which' field