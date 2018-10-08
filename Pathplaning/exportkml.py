#!/usr/bin/python
# /****************************************************************************
# exportKML
# Copyright (c) 2014-2018, Kjeld Jensen <kj@kjen.dk>
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
# ****************************************************************************/
'''
2015-03-22 Kjeld Removed unnecessary trkptend() function
2015-11-18 Kjeld Added optional absolute altitude mode
2018-03-13 Kjeld Added a
'''


class kmlclass:

    def __init__(self):
        return

    def begin(self, fname, name, desc, width):
        self.f = open(fname, 'w')
        self.f.write('<?xml version="1.0" encoding="UTF-8"?>\n')
        self.f.write('<kml xmlns="http://www.opengis.net/kml/2.2">\n')
        self.f.write('<Document>\n')
        self.f.write('<name>%s</name>\n' % (name))
        self.f.write('<description>%s</description>\n' % (desc))
        self.f.write('<Style id="red">\n')
        self.f.write('  <LineStyle>\n')
        self.f.write('    <color>ff0000ff</color>\n')
        self.f.write('    <width>%.1f</width>\n' % (width))
        self.f.write('  </LineStyle>\n')
        self.f.write('</Style>\n')
        self.f.write('<Style id="green">\n')
        self.f.write('  <LineStyle>\n')
        self.f.write('    <color>ff00ff00</color>\n')
        self.f.write('    <width>%.1f</width>\n' % (width))
        self.f.write('  </LineStyle>\n')
        self.f.write('</Style>\n')
        self.f.write('<Style id="blue">\n')
        self.f.write('  <LineStyle>\n')
        self.f.write('    <color>ffff0000</color>\n')
        self.f.write('    <width>%.1f</width>\n' % (width))
        self.f.write('  </LineStyle>\n')
        self.f.write('</Style>\n')
        self.f.write('<Style id="cyan">\n')
        self.f.write('  <LineStyle>\n')
        self.f.write('    <color>ffffff00</color>\n')
        self.f.write('    <width>%.1f</width>\n' % (width))
        self.f.write('  </LineStyle>\n')
        self.f.write('</Style>\n')
        self.f.write('<Style id="yellow">\n')
        self.f.write('  <LineStyle>\n')
        self.f.write('    <color>ff00ffff</color>\n')
        self.f.write('    <width>%.1f</width>\n' % (width))
        self.f.write('  </LineStyle>\n')
        self.f.write('</Style>\n')
        self.f.write('<Style id="grey">\n')
        self.f.write('  <LineStyle>\n')
        self.f.write('    <color>ff888888</color>\n')
        self.f.write('    <width>%.1f</width>\n' % (width))
        self.f.write('  </LineStyle>\n')
        self.f.write('</Style>\n')
        return

    def trksegbegin(self, segname, segdesc, color, altitude):
        self.f.write('<Placemark>\n')
        self.f.write('<name>%s</name>\n' % (segname))
        self.f.write('<description>%s</description>\n' % (segdesc))
        self.f.write('<styleUrl>#%s</styleUrl>\n' % (color))
        self.f.write('<LineString>\n')
        if altitude == 'absolute':
            self.f.write('<altitudeMode>absolute</altitudeMode>\n')
        elif altitude == 'relativeToGround':
            self.f.write('<altitudeMode>relativeToGround</altitudeMode>\n')
        self.f.write('<coordinates>\n')
        return

    def trksegend(self):
        self.f.write('</coordinates>\n')
        self.f.write('</LineString>\n')
        self.f.write('</Placemark>\n')
        return

    def trkpt(self, lat, lon, ele):
        self.f.write('%013.10f,%014.10f,%.3f\n' % (lon, lat, ele))
        return

    def end(self):
        self.f.write('</Document>\n')
        self.f.write('</kml>')
        self.f.close()
        return


if __name__ == "__main__":
    print
    'Creating the file testfile.kml as an example on how tu use kmlclass'
    # width: defines the line width, use e.g. 0.1 - 1.0
    kml = kmlclass()
    kml.begin('testfile.kml', 'Example', 'Example on the use of kmlclass', 0.1)
    # color: use 'red' or 'green' or 'blue' or 'cyan' or 'yellow' or 'grey'
    # altitude: use 'absolute' or 'relativeToGround'
    kml.trksegbegin('', '', 'red', 'absolute')
    kml.trkpt(55.47, 10.33, 0.0)
    kml.trkpt(55.47, 10.34, 0.0)
    kml.trkpt(55.48, 10.34, 0.0)
    kml.trkpt(55.47, 10.33, 0.0)
    kml.trksegend()
    kml.end()

