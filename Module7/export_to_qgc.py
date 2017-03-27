#!/usr/bin/python
#/****************************************************************************
# export_to_qgc
# Copyright (c) 2016, Mathias Hoejgaard Egeberg <mhoejgaard@hotmail.com>
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
"""
This class is export waypoints to qgroundcontrol_aq file format.

The functions do not check for errors.

    export_to_qgc(self,filename)
        Initialize class and set waypoint file.

    set_wpt_coordinate_frame(self,frame)
        Set coordinate frame of waypoints.
        # 0 = abs 3 = relative

    set_takeoff_coordinate_frame(self,frame)
        Set coordinate frame of take off.
        # 0 = abs 3 = relative

    set_yaw_angle(self,angle)
        Set static yaw angle.

    set_max_vertical_speed(self,speed)
        Set maximal vertical speed.

    set_accuracy_radius(self,radius)
        Set accuracy radius of waypoints (how accurate it has to hit it).

    set_loiter_time(self,time)
        Set loiter time at waypoints (how long it should stay at the waypoint).
        time must be in decimal seconds (e.g. 3.5 s)
    set_max_horizontal_speed(self,speed)
        Set maximail horizontal speed.

    takeoff(self,lat,lon,alt)
        Take off at position (lat,lon) to altitude alt.

    write_list_2D(self,wpt_list)
        Write wpt_list to the file.
        wpt_list must be a list of type:
            [[lat1,lon1],[lat2,lon2],[lat3,lon3]

    write_list_3D(self,wpt_list)
        Write wpt_list to the file.
        wpt_list must be a list of type:
            [[lat1,lon1,alt1],[lat2,lon2,alt2],[lat3,lon3,alt3]

"""


# qgc constants
MAV_CMD_NAV_WAYPOINT = 16
MAV_CMD_NAV_TAKEOFF = 22


class export_to_qgc:
    def __init__(self,filename):
        self.qgc_current = 0
        self.qgc_frame_takeoff = 0 # 0 = abs 3 = relative
        self.qgc_frame_wpt = 0 # 0 = abs 3 = relative
        self.qgc_vert_vmax = 1.0
        self.qgc_radius = 3.5
        self.qgc_loiter = 5.0
        self.qgc_hori_vmax = 5.0
        self.qgc_yaw = 0.0
        self.qgc_altitude = 5
        self.qgc_dunno2 = 1
        self.file = filename
        f = open(filename,'w')
        f.write ('QGC WPL 120\n')
        f.close()

    def set_wpt_coordinate_frame(self,frame):
        self.qgc_frame_wpt = frame

    def set_takeoff_coordinate_frame(self,frame):
        self.qgc_frame_takeoff = frame

    def set_yaw_angle(self,angle):
        self.qgc_yaw = angle

    def set_max_vertical_speed(self,speed):
        self.qgc_vert_vmax = speed

    def set_accuracy_radius(self,radius):
        self.qgc_radius = radius

    def set_loiter_time(self,time):
        self.qgc_loiter = time

    def set_max_horizontal_speed(self,speed):
        self.qgc_hori_vmax = speed

    def takeoff(self,lat,lon,alt):
        f = open(self.file,'a')
        f.write ('%d\t%d\t%d\t%d\t%.2f\t%.0f\t%.2f\t%.2f\t%.8f\t%.8f\t%.3f\t%d\n' % (0, 1, self.qgc_frame_takeoff, MAV_CMD_NAV_TAKEOFF, self.qgc_radius, self.qgc_loiter*1000, self.qgc_yaw, self.qgc_vert_vmax, lat, lon, alt, self.qgc_dunno2))
        f.close()
    def write_list2D(self,wpt_list):
        f = open(self.file,'a')
        for i in range(len(wpt_list)):
            f.write ('%d\t%d\t%d\t%d\t%.2f\t%.0f\t%.2f\t%.2f\t%.8f\t%.8f\t%.3f\t%d\n' % (i+1, self.qgc_current, self.qgc_frame_wpt, MAV_CMD_NAV_WAYPOINT, self.qgc_radius, self.qgc_loiter*1000, self.qgc_hori_vmax, self.qgc_yaw, wpt_list[i][0], wpt_list[i][1], self.qgc_altitude, self.qgc_dunno2))
        f.close()

    def write_list3D(self,wpt_list):
        f = open(self.file,'a')
        for i in range(len(wpt_list)):
            f.write ('%d\t%d\t%d\t%d\t%.2f\t%.0f\t%.2f\t%.2f\t%.8f\t%.8f\t%.3f\t%d\n' % (i+1, self.qgc_current, self.qgc_frame_wpt, MAV_CMD_NAV_WAYPOINT, self.qgc_radius, self.qgc_loiter*1000, self.qgc_hori_vmax, self.qgc_yaw, wpt_list[i][0], wpt_list[i][1], wpt_list[i][2], self.qgc_dunno2))
        f.close()
