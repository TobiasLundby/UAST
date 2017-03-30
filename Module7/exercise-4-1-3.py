from PlanCleaner import PlanCleaner

# Init cleaner (loads log file)
a = PlanCleaner("021-AQL.LOG")
# Clean the path
a.cleanPath(0.5) # Parameter = max error
# Write the short list to file
a.write_list3D("test.txt")
