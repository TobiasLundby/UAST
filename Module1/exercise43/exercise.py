import sys
from nmea_parser_class import NMEA_data_parser
# Main
# Flight data
data = NMEA_data_parser(sys.argv[1])		# Instantiate parser object
data.parse()					# Parse the data file
data.plot_altitude()				# Plot the altitude
data.plot_tracked_satellites()			# Plot tracked satellites
data.generate_track_file('drone_track.kml','Drone track','This is the track of the drone',1)

# 24 hour static data
data24 = NMEA_data_parser(sys.argv[2])		# Instantiate parser object
data24.parse()
data24.generate_GNSS_accuracy_file('GNSS_static_accuracy.kml','Static GNSS accuracy','The colors mark GNSS accuracy',1) # Generate accuracy file
