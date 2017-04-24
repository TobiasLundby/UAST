#!/usr/bin/python
#/****************************************************************************
# nmea_parser
# Copyright (c) 2017, Mathias Hoejgaard Egeberg <mhoejgaard@hotmail.com>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#****************************************************************************/
import fileinput
import matplotlib.pyplot as plt # For plotting, install as: sudo apt-get install python-matplotlib
from exportkml import kmlclass
class NMEA_data_parser:
   ### Class variables ###
   # GGA field indices based on http://www.gpsinformation.org/dale/nmea.htm#GGA
   message_type = 0
   GGA_fix_time = 1
   GGA_lat = 2
   GGA_NS = 3
   GGA_lon = 4
   GGA_EW = 5
   GGA_fix_quality = 6
   GGA_num_sat = 7
   GGA_horiz_delut = 8
   GGA_alt = 9
   GGA_alt_unit = 10
   GGA_height_geoid = 11
   GGA_height_unit = 12
   GGA_time_since_update = 13
   GGA_DPGS_ID = 14
   GGA_checksum = 15

   # GSV field indices
   GSA_selection = 1
   GSA_fix_type = 2
   GSA_PRN1 = 3
   # 4-16 PRN2-14
   GSA_PDOP = 15
   GSA_HDOP = 16
   GSA_VDOP = 17
   GSA_checksum = 18


   #### Constructors ###
   def __init__(self):
      return

   def __init__(self,filename):
      # Create lists for needed information
      self.filename = filename
      self.time = []
      self.altitude = []
      self.satellites_tracked = []
      self.latitude = []
      self.longtitude = []
      self.GNSS_quality = []
      self.PDOP = []
      self.HDOP = []
      self.VDOP = []
      return

   ### Methods ###
   def parse(self):
      print 'Parsing file:',self.filename
      for line in fileinput.input(self.filename):		# Parse all lines
         line_elements = line.split(",")			# Split in elements
         if line_elements[self.message_type] == '$GPGGA':	# For GGA messages, do:
            self.time.append(line_elements[self.GGA_fix_time])		# Store message time
            self.altitude.append(line_elements[self.GGA_alt])          # Store altitude
            self.satellites_tracked.append(line_elements[self.GGA_num_sat]) # Store tracked satellites
            if(len(line_elements[self.GGA_lat])>=6):
               self.latitude.append(line_elements[self.GGA_lat]) 		# Store latitude
            else:
               self.latitude.append('x')
            if(len(line_elements[self.GGA_lat])>=6):
               self.longtitude.append(line_elements[self.GGA_lon])	# Store longtitude
            else:
               self.longtitude.append('x')
            self.GNSS_quality.append(line_elements[self.GGA_fix_quality]) # Store GNSS fix quality
         elif line_elements[self.message_type] == '$GPGSA':	# For GSA messages do:
            self.PDOP.append(line_elements[self.GSA_PDOP]) # Store position delution
      return

   # Plot altitude vs. time
   def plot_altitude(self):
      print 'Generating altitude vs. time plot'
      if len(self.altitude) == 0:
         print 'No altitude information'
      else:
         fig1 = plt.figure()
         fig1.canvas.set_window_title('Altitude vs. time')
         plt.plot(self.time,self.altitude)
         plt.xlabel('Time')
         plt.ylabel('Altitude [m]')
         plt.title('Altitude vs. time')
         plt.show()
      return

   # Plot tracked satellites vs. time
   def plot_tracked_satellites(self):
      print 'Generating tracked satellites vs. time plot'
      if len(self.satellites_tracked) == 0:
         print 'No altitude information'
      else:
         fig2 = plt.figure()
         fig2.canvas.set_window_title('Tracked satellites vs. time')
         plt.plot(self.time, self.satellites_tracked)
         plt.ylim([5,15])
         plt.xlabel('Time')
         plt.ylabel(' # tracked satellites')
         plt.title('Tracked satellites vs. time')
         plt.show()
      return

   # Convert from DM.m.... to D.d Formula from: http://www.directionsmag.com/site/latlong-converter/
   def degree_minutes_to_degree(self,degree_minutes):
      splitted = degree_minutes.split('.') # split before and after dot
      length = len(splitted[0]) # get length of the first part
      degree = splitted[0][0:length-2] # Degrees are anything but the last two digits
      minutes = splitted[0][length-2:length]+'.'+splitted[1] # combine the minutes
      minutes_in_deg = float(minutes)/float(60) # .d = M.m/60
      degree = float(degree) + float(minutes_in_deg) # D.d = D + .d
      return degree

   # Generate KML-file with drone track.
   def generate_track_file(self,filename,name,description,size):
      if len(self.latitude) == 0:
         print 'No track points'
      else:
         track = kmlclass()
         track.begin(filename,name,description,size)
         track.trksegbegin('Segname','Segdesc','yellow','absolute')
         for i in range (0,len(self.latitude)):
         #for i in range (0,1):
            if (self.latitude[i] != 'x' and self.longtitude[i] != 'x'):
               track.trkpt(self.degree_minutes_to_degree(self.latitude[i]),self.degree_minutes_to_degree(self.longtitude[i]),float(self.altitude[i]))
         track.trksegend()
         track.end()
      return

   # Generate KML-file with drone track colored based on PDOP (GNSS accuracy)
   def generate_GNSS_accuracy_file(self,filename,name,description,size):
      if len(self.latitude) == 0:
         print 'No track points'
      else:
         track = kmlclass()  # Instantiate object
         track.begin(filename,name,description,size) # Create the file
         prev_color = '' # For decision of new segment
         first = True
         for i in range(0,len(self.latitude)): # For all pairs of GGA and GSA messages
            if(self.latitude[i] != 'x' and self.longtitude[i] != 'x'):
               # Deside color and description based on PDOP value
               if float(self.PDOP[i]) < 2:
                  color = 'green'
                  seg_name = 'level1'
                  seg_desc = 'PDOP below 2'
               elif float(self.PDOP[i]) < 3:
                  color = 'blue'
                  seg_name = 'level2'
                  seg_desc = 'PDOP below 3'
               elif float(self.PDOP[i]) < 4:
                  color = 'yellow'
                  seg_name = 'level3'
                  seg_desc = 'PDOP below 4'
               elif float(self.PDOP[i]) < 5:
                  color = 'cyan'
                  seg_name = 'level4'
                  seg_desc = 'PDOP below 5'
               elif float(self.PDOP[i]) < 6:
                  color = 'grey'
                  seg_name = 'level5'
                  seg_desc = 'PDOP below 6'
               else:
                  color = 'red'
                  seg_name = 'level6'
                  seg_desc = 'PDOP above 6'
               if(color != prev_color):	# We only need to create a new segment, if color changes
                  if(first != True):
                     track.trksegend() # End segment before starting a new one (if not the first)
                  else:
                     first = False # Bookkeeping
                  track.trksegbegin(seg_name,seg_desc,color,'absolute') # Start new segment (color)
               track.trkpt(self.degree_minutes_to_degree(self.latitude[i]),self.degree_minutes_to_degree(self.longtitude[i]),float(self.altitude[i])) # Add point to segment
               prev_color = color # Store color for comparison next time
         track.trksegend() # End the last segment
         track.end() # Close the file
      return
