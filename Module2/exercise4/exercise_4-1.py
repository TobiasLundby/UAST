import sys
from logfile_parser import logfile_parser
import matplotlib.pyplot as plt # For plotting, install as: sudo apt-get install python-matplotlib
import numpy as np

def get_SOC(voltage):
   #s_func = 1/1+np.exp(-voltage)
   max_voltage = 4.15
   min_voltage = 3.19
   diff = max_voltage-min_voltage
   SOC = (voltage-3.19)/diff
   #print diff
   return SOC

# Main
# Flight data
data = logfile_parser(sys.argv[1],"\t")		# Instantiate parser object
time = data.get_column(4)
voltage = data.get_column(11)
time_int = []

for i in range(0,len(time)):
   time[i]=time[i].split('.')[0]
   #print time[i][0:2], time[i][2:4], time[i][4:6]
   hours = int(time[i][0:2])
   minutes = int(time[i][2:4])
   seconds = int(time[i][4:6])
   time_int.append(hours*3600+minutes*60+seconds)

# Plot graph
fig1 = plt.figure()
fig1.canvas.set_window_title('Voltage vs. time')
plt.plot(time_int,voltage)
plt.xlabel('Time')
plt.ylabel(' Voltage [V]')
plt.title('Voltage vs. time')
plt.show()

# Test for SOC func
#print get_SOC(4.15),get_SOC(3.7),get_SOC(3.19) 

SOC = []
voltage_float = []
for i in range(0,len(voltage)):
   SOC.append(get_SOC(float(voltage[i])))
   voltage_float.append(float(voltage[i]))


# Method with built-in function
p = np.poly1d(np.polyfit(voltage_float,SOC,1))
print p(3.89)

# Method without built-in function
#m,b = np.polyfit(voltage_float,SOC,1)
#print m,b

# Generate points for plotting the fit
xp=np.linspace(3.19,4.15,100)


fig2 = plt.figure()
fig2.canvas.set_window_title('SOC vs. voltage')
plt.plot(voltage,SOC,'.',xp,p(xp),'-')
plt.xlabel('Voltage [V]')
plt.ylabel('SOC')
plt.title('Voltage vs. time')
plt.show()

