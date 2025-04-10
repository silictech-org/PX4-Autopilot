uint64 timestamp	# time since system start (microseconds)

# fov in degree
# distance in cm

uint8 RADAR_MR72 = 1
uint8 LIDAR_FSS20 = 2

uint8 NUM_SECTORS_MR72 = 8
uint8 NUM_SECTORS_S20 = 36

uint8 sensor_type
uint16 fov
uint8 sectors
uint16[36] distance

uint16 min_distance # Minimum distance the sensor can measure in centimeters.
uint16 max_distance # Maximum distance the sensor can measure in centimeters.
