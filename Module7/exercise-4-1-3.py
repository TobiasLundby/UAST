import sys
from nmea_parser_class import NMEA_data_parser
# Main
# Flight data
data = NMEA_data_parser(sys.argv[1])		# Instantiate parser object
data.parse()					# Parse the data file

# Generate KML file for Google Maps. Simple viewer on http://www.gpsvisualizer.com/
data.generate_track_file('drone_track.kml','Drone track','This is the track of the drone',1)

# The NMEA data file does not contain heading information as specified in the exercise
combinedData = [] # Structured as (rowwise) [longtitude, latitude, altitude]
for i in range(len(data.longtitude)):
    combinedData.append([data.longtitude[i], data.latitude[i], data.altitude[i]])

# Print first 10 values to test functionality
for i in range(10):
    print combinedData[i]
