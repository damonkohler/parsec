# This message contains colors for the 10 LEDs, indices 0--4 are the rear LEDs,
# 5--9 the front LEDs. Alpha value 'false' does not set a color.

# std_msgs/ColorRGBA[10] would use 320 bytes, this uses 40 bytes.
uint8[10] r
uint8[10] g
uint8[10] b
bool[10] a

