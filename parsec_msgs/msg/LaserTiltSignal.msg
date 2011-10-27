# Message to signal that we either finished half a period or a full
# tilting period.

uint8 DIRECTION_DOWN = 0
uint8 DIRECTION_UP = 1

Header header

# 1 => joint reached top position
# 0 => joint reached bottom position
uint8 signal
