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
