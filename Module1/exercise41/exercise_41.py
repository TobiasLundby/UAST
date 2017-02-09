#!/usr/bin/env python
#*****************************************************************************
# UTM projection conversion test
# Copyright (c) 2013-2016, Kjeld Jensen <kjeld@frobomind.org>
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
#*****************************************************************************
"""
This file contains a simple Python script to test the UTM conversion class.

Revision
2013-04-05 KJ First version
2015-03-09 KJ Minor update of the license text.
2016-01-16 KJ Corrected a minor problem with the library location reference.
2017-02-08 TL Modified to fit exercise
"""
# import utmconv class
from utm import utmconv
from math import pi, cos, acos, sin

# define test position
test_lat =  55.36732
test_lon = 10.43192
loc_add_m = 1000
print 'Test position [deg]:'
print '  latitude:  %.10f'  % (test_lat)
print '  longitude: %.10f'  % (test_lon)

# instantiate utmconv class
uc = utmconv()

# convert from geodetic to UTM
(hemisphere, zone, letter, easting, northing) = uc.geodetic_to_utm (test_lat,test_lon)
print '\nConverted from geodetic to UTM [m]'
print '  %d %c %.5fe %.5fn' % (zone, letter, easting, northing)

# Add 1km=1000m to both northing
print "\nAdding 1km to northing"
new_easting = easting+0
new_northing = northing+loc_add_m

# convert back from UTM to geodetic
(lat, lon) = uc.utm_to_geodetic (hemisphere, zone, new_easting, new_northing)
print '\nConverted back from UTM to geodetic [deg]:'
print '  latitude:  %.10f'  % (lat)
print '  longitude: %.10f'  % (lon)

# Convert lat and long to radians
test_lat_rad = test_lat*(pi/180)
test_lon_rad = test_lon*(pi/180)
lat_rad = lat*(pi/180)
lon_rad = lon*(pi/180)
# Calculate Great Circle Distance; Distance between points formula
d_rad=acos(sin(test_lat_rad)*sin(lat_rad)+cos(test_lat_rad)*cos(lat_rad)*cos(test_lon_rad-lon_rad))
d_nm=((180*60)/pi)*d_rad
d_km=d_nm*1.85200
print '++Great Circle Distance: %.10f'  % (d_km)
print '++Error [m]: %.10f'  % (loc_add_m-d_km*1000)

# Add 1km=1000m to both easting
print "\nAdding 1km to easting"
new_easting = easting+loc_add_m
new_northing = northing+0

# convert back from UTM to geodetic
(lat, lon) = uc.utm_to_geodetic (hemisphere, zone, new_easting, new_northing)
print '\nConverted back from UTM to geodetic [deg]:'
print '  latitude:  %.10f'  % (lat)
print '  longitude: %.10f'  % (lon)

# Convert lat and long to radians
test_lat_rad = test_lat*(pi/180)
test_lon_rad = test_lon*(pi/180)
lat_rad = lat*(pi/180)
lon_rad = lon*(pi/180)
# Calculate Great Circle Distance; Distance between points formula
d_rad=acos(sin(test_lat_rad)*sin(lat_rad)+cos(test_lat_rad)*cos(lat_rad)*cos(test_lon_rad-lon_rad))
d_nm=((180*60)/pi)*d_rad
d_km=d_nm*1.85200
print '++Great Circle Distance [km]: %.10f'  % (d_km)
print '++Error [m]: %.10f'  % (loc_add_m-d_km*1000)
