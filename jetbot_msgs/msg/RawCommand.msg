# Command sent to the JetBot firmware, according to the JetBot protocol v1
# Sent in conjunction with a RawMotorCommand in order to form a complete request

uint8 verb # Action that the jetbot is to perform in response to this request
uint8 arg