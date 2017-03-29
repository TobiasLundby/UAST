#!/usr/bin/python
#/****************************************************************************
# route_plan_class
# Copyright (c) 2016, Tobias Lundby <tobiaslundby@gmail.com>
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
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL HENRIK EGEMOSE SCHMIDT BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#****************************************************************************/
from nmea_parser_class import NMEA_data_parser  # Import NMEA parser
from aqlogreader.AQLogReader import aqLogReader # Import AQLogReader, note that the loaction of the file is in a subfolder
from math import pi, sin, cos, atan2       # Import math functionality
import matplotlib.pyplot as plt
import numpy as np

class Route_Plan:
    #### Constructors ###
    def __init__(self):
        return

    def __init__(self,filename_nmea,filename_aq_log):
        # Load flight data from NMEA file using the NMEA parser
        self.data_nmea = NMEA_data_parser(filename_nmea)              # Instantiate parser object
        self.data_nmea.parse()				                    # Parse the data file
        self.data_aq = aqLogReader(filename_aq_log)               # Instantiate object


    ### Methods ###
    def get_pos_4DOF_nmea(self):
        # The NMEA data file does not contain heading information as specified in the exercise, therefore calculate based on 2 points.
        combinedData = [] # Structured as (rowwise) [longtitude, latitude, altitude, heading]
        for i in range(len(self.data_nmea.longtitude)):
            if i >= 1:
                #convert nmea to deg; NMEA format dddmm.mmmm
                tmp_lon_1_ddd = self.data_nmea.longtitude[i-1][1:3]
                tmp_lon_1_mmmmmm = self.data_nmea.longtitude[i-1][3:]
                tmp_lon_1 = float(tmp_lon_1_ddd) + (float(tmp_lon_1_mmmmmm)/60)
                tmp_lon_2_ddd = self.data_nmea.longtitude[i][1:3]
                tmp_lon_2_mmmmmm = self.data_nmea.longtitude[i][3:]
                tmp_lon_2 = float(tmp_lon_2_ddd) + (float(tmp_lon_2_mmmmmm)/60)

                tmp_lat_1_ddd = self.data_nmea.latitude[i-1][1:3]
                tmp_lat_1_mmmmmm = self.data_nmea.latitude[i-1][3:]
                tmp_lat_1 = float(tmp_lat_1_ddd) + (float(tmp_lat_1_mmmmmm)/60)
                tmp_lat_2_ddd = self.data_nmea.latitude[i-1][1:3]
                tmp_lat_2_mmmmmm = self.data_nmea.latitude[i-1][3:]
                tmp_lat_2 = float(tmp_lat_2_ddd) + (float(tmp_lat_2_mmmmmm)/60)

                #convert to rad
                tmp_lon_rad1 = (pi/180)*tmp_lon_1 #lambda1
                tmp_lon_rad2 = (pi/180)*tmp_lon_2 #lambda2
                tmp_lat_rad1 = (pi/180)*tmp_lat_1 #phi1
                tmp_lat_rad2 = (pi/180)*tmp_lat_2 #phi1

                #calculate bearing/heading (initial in regards to the points)
                y = sin(tmp_lon_rad2-tmp_lon_rad1) * cos(tmp_lat_rad2)
                x = cos(tmp_lat_rad1)*sin(tmp_lat_rad2) - sin(tmp_lat_rad1)*cos(tmp_lat_rad2)*cos(tmp_lon_rad2-tmp_lon_rad1)
                brng = atan2(y, x) % 2*pi # initial bearing/heading
                combinedData.append([self.data_nmea.longtitude[i], self.data_nmea.latitude[i], self.data_nmea.altitude[i], brng])
            else:
                combinedData.append([self.data_nmea.longtitude[i], self.data_nmea.latitude[i], self.data_nmea.altitude[i], 0]) # first time only point information so no bearing yet.
        return combinedData

    def get_nmea_data(self):
        return self.data_nmea
    def get_aq_data(self):
        return self.data_aq
    def print_aq_info(self):
        print self.data_aq.printChannelNames()
        return
