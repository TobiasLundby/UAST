import sys
from route_plan_class import Route_Plan  # Import NMEA parser

nmea_file   = sys.argv[1]
aq_log_file = sys.argv[2]

Route_Plan_obj = Route_Plan(nmea_file, aq_log_file)

data = Route_Plan_obj.get_pos_4DOF_nmea()

#Route_Plan_obj.print_aq_info()

# Print first 10 values to test functionality
for i in range(10):
    print data[i]

data_full = Route_Plan_obj.get_nmea_data()
# Generate KML file for Google Maps. Simpsle viewer on http://www.gpsvisualizer.com/
data_full.generate_track_file('drone_track.kml','Drone track','This is the track of the drone',1)
# wierdly there is no need for including the drone_track, but I think it is a dependency trhough route_plan_class->nmea_parser_class->exportkml
