# GPS position in WGS84 coordinates.
# the field 'timestamp' is for the position & velocity (microseconds)
uint64 timestamp		# time since system start (microseconds)

int32 lat			# Latitude in 1E-7 degrees
int32 lon			# Longitude in 1E-7 degrees

uint8 fix_type # 0-1: no fix, 2: 2D fix, 3: 3D fix, 4: RTCM code differential, 5: Real-Time Kinematic, float, 6: Real-Time Kinematic, fixed, 8: Extrapolated. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.

float32 hdop			# Horizontal dilution of precision

uint8 satellites_used		# Number of satellites used
