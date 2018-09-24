# -*- coding: utf-8 -*-
"""
Created on Wed Jul 05 13:50:30 2017

@author: Preyas Shah
"""

# Import Statements
from datetime import datetime
from gps_pkg.gps import *
import csv
import sys
import math
from numpy import linalg as la
import numpy as np

# Point is a class having two x and y coordinates as objects
class Point(object):
    def __init__(self, x, y):
        self.x = x
        self.y = y

# Edge is a class containing two points
class Edge(object):
    def __init__(self, P1, P2):
        self.P1 = P1
        self.P2 = P2

# Rectange is a class having four end-points as objects
class Rectangle(object):
    def __init__(self, P1, P2, P3, P4):
        self.Points = (P1, P2, P3, P4)

#Function that projects a rectangle along any axis and returns the minimum and maximum values
def projectRectangle(axis, rectangleObject):
    dotProd = np.dot(axis, [rectangleObject.Points[0].x, rectangleObject.Points[0].y])
    minVal = dotProd
    maxVal = dotProd
    for i in range(1, len(rectangleObject.Points)):
        dotProd = np.dot(axis, [rectangleObject.Points[i].x, rectangleObject.Points[i].y])
        if(dotProd < minVal):
            minVal = dotProd
        elif(dotProd > maxVal):
            maxVal = dotProd
    #Return the minimum and maximum values
    return(minVal, maxVal)

# Determine if two rectangles are colliding
def isColliding(rect1, rect2):
    edges = []
    for i in range(0,4):
        edges.append(Edge(rect1.Points[i], rect1.Points[(i + 1) % 4]))
        
    for i in range(0,4):
        edges.append(Edge(rect2.Points[i], rect2.Points[(i + 1) % 4]))
    
    for edge in edges:
        # Axis is the perpendicular to the edge
        axis = [-(edge.P2.y - edge.P1.y), edge.P2.x - edge.P1.x]
        #Find the Normal
        normAxis = la.norm(axis)
        for i in range(0, len(axis)):
            axis[i] =  float(axis[i] / normAxis) # Divide by normal to get unit amplitude
                
        (minVal1, maxVal1) = projectRectangle(axis, rect1)
        (minVal2, maxVal2) = projectRectangle(axis, rect2)
        
        interval = 0.0 # Interval between two projections
        if(minVal1 < minVal2):
            interval = minVal2 - maxVal1
        else:
            interval = minVal1 - maxVal2
        #If during any iteration, interval is found to be positive, that means
        #rectanges are not intersecting, return true
        if(interval > 0):
            return False
    #All Projections are overlapping, both rectangles are intersecting, return true
    return True
          
def mainFunction():

    # Local Variables
    inputFileName = sys.argv[1]
    timeStamp = []
    v1_latitude = []
    v1_longitude = []
    v2_latitude = []
    v2_longitude = []
    threat_collision = []
    #Total number of meaningful records extracted from the csv file
    count = 0
    main_name,extention=inputFileName.split('.')
    #print 'main_name:',main_name
    #print 'extention:',extention
    out_kml_name=main_name+'.kml'
    with open(inputFileName, "r") as csvFile:
        csvHandler = csv.reader(csvFile)
        for row in csvHandler:
            # Skip the first row
            if(count == 0):
                pass
            else:
                try:
                    veh_id = row[0] # First row is just the heading information
                except:
                    continue
                if(veh_id == "v1"):
                    # If first vehicle, then store the longitude and latitude of v1
                    timeStamp.append(row[1])
                    v1_longitude.append(float(row[2]))
                    v1_latitude.append(float(row[3]))
                    threat_collision.append(False)
                else:
                    v2_longitude.append(float(row[2]))
                    v2_latitude.append(float(row[3])) 
                # Default value of threat collision probability
            count = count + 1
        #Total number of elements include the first row
        #deduct 1 from count
        count = count - 1
    for i in range(1, count/2):
        #try:
            time_init = datetime.strptime(timeStamp[i-1],"%Y-%m-%dT%H:%M:%SZ")
            time_fin  = datetime.strptime(timeStamp[i],"%Y-%m-%dT%H:%M:%SZ")
            time_diff = int((time_fin - time_init).total_seconds())
            
            # Now, calculate the speed and heading of the v1 and v2
            v1_distance  = GPSDistance(v1_latitude[i], v1_longitude[i], v1_latitude[i-1], v1_longitude[i-1])
            v2_distance  = GPSDistance(v2_latitude[i], v2_longitude[i], v2_latitude[i-1], v2_longitude[i-1])
            v1_heading   = BfromGPS(v1_latitude[i-1], v1_longitude[i-1], v1_latitude[i], v1_longitude[i])
            v2_heading   = BfromGPS(v2_latitude[i-1], v2_longitude[i-1], v2_latitude[i], v2_longitude[i])
            
            if(v1_heading < 0):
                v1_heading = v1_heading + 360
            
            if(v2_heading < 0):
                v2_heading = v2_heading + 360
            #Speed = distance/time
            v1_speed     = v1_distance/time_diff
            v2_speed     = v2_distance/time_diff
            
            # Now consider the first vehicle as a reference point
            x1 = 0.0
            y1 = 0.0
            x2 = XfromGPS(v1_latitude[i], v1_longitude[i], v2_latitude[i], v2_longitude[i])
            y2 = YfromGPS(v1_latitude[i], v1_longitude[i], v2_latitude[i], v2_longitude[i])
            
            r1_width = 4.0 # lane width is fix
            r1_length = v1_speed * 8
            # Determine the x and y coordinates of the Rectangle R1 points
            r1_P1x = x1 + 0.5 * r1_width * math.sin((v1_heading - 90) * math.pi / 180)
            r1_P1y = y1 + 0.5 * r1_width * math.cos((v1_heading - 90) * math.pi / 180)
            r1_P2x = r1_P1x + r1_length * math.sin(v1_heading * math.pi / 180)
            r1_P2y = r1_P1y + r1_length * math.cos(v1_heading * math.pi / 180)
            r1_P4x = x1 + 0.5 * r1_width * math.sin((v1_heading + 90) * math.pi / 180)
            r1_P4y = y1 + 0.5 * r1_width * math.cos((v1_heading + 90) * math.pi / 180)
            r1_P3x = r1_P4x + r1_length * math.sin(v1_heading * math.pi / 180)
            r1_P3y = r1_P4y + r1_length * math.cos(v1_heading * math.pi / 180)
            
            r2_width = 4.0
            r2_length = v2_speed * 8
            # Determine the x and y coordinates of the Rectangle R2 points
            r2_P1x = x2 + 0.5 * r2_width * math.sin((v2_heading - 90) * math.pi / 180)
            r2_P1y = y2 + 0.5 * r2_width * math.cos((v2_heading - 90) * math.pi / 180)
            r2_P2x = r2_P1x + r2_length * math.sin(v2_heading * math.pi / 180)
            r2_P2y = r2_P1y + r2_length * math.cos(v2_heading * math.pi / 180)
            r2_P4x = x2 + 0.5 * r2_width * math.sin((v2_heading + 90) * math.pi / 180)
            r2_P4y = y2 + 0.5 * r2_width * math.cos((v2_heading + 90) * math.pi / 180)
            r2_P3x = r2_P4x + r2_length * math.sin(v2_heading * math.pi / 180)
            r2_P3y = r2_P4y + r2_length * math.cos(v2_heading * math.pi / 180)
            
            #Initialize the Rectangle R1 Points
            r1_P1 = Point(r1_P1x, r1_P1y)
            r1_P2 = Point(r1_P2x, r1_P2y)
            r1_P3 = Point(r1_P3x, r1_P3y)
            r1_P4 = Point(r1_P4x, r1_P4y)
            
            #Initialize the Rectangle R2 Points
            r2_P1 = Point(r2_P1x, r2_P1y)
            r2_P2 = Point(r2_P2x, r2_P2y)
            r2_P3 = Point(r2_P3x, r2_P3y)
            r2_P4 = Point(r2_P4x, r2_P4y)
            
            #Initialize the Rectangle R1 and R2
            
            rect1 = Rectangle(r1_P1,r1_P2,r1_P3,r1_P4)
            rect2 = Rectangle(r2_P1,r2_P2,r2_P3,r2_P4)
            
            #If both rectanges are colliding, that means there is a danger of collision
            #Set threat_collision[i] to be True
            if(isColliding(rect1, rect2)):
                threat_collision[i] = True
        #except:
        #    continue
        
    with  open(out_kml_name, 'w') as kml_file:
    #kml_file.write('Hi there!')
        kml_file.write('''<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://earth.google.com/kml/2.2">
<!-- 

TimeStamp is recommended for Point.

Each Point represents a sample from a GPS.

-->

  <Document>
    <name>Points with TimeStamps</name>
	<Style id="sn_cabs3">
		<IconStyle>
			<color>ff0000ff</color>
			<scale>0.7</scale>
			<Icon>
				<href>http://maps.google.com/mapfiles/kml/shapes/cabs.png</href>
			</Icon>
			<hotSpot x="0.5" y="0" xunits="fraction" yunits="fraction"/>
		</IconStyle>
		<ListStyle>
		</ListStyle>
	</Style>
	<Style id="sh_cabs3">
		<IconStyle>
			<color>ff0000ff</color>
			<scale>0.816667</scale>
			<Icon>
				<href>http://maps.google.com/mapfiles/kml/shapes/cabs.png</href>
			</Icon>
			<hotSpot x="0.5" y="0" xunits="fraction" yunits="fraction"/>
		</IconStyle>
		<ListStyle>
		</ListStyle>
	</Style>
	<StyleMap id="msn_cabs3">
		<Pair>
			<key>normal</key>
			<styleUrl>#sn_cabs3</styleUrl>
		</Pair>
		<Pair>
			<key>highlight</key>
			<styleUrl>#sh_cabs3</styleUrl>
		</Pair>
	</StyleMap>

	<Style id="sn_cabs2">
		<IconStyle>
			<color>ffff0000</color>
			<scale>0.7</scale>
			<Icon>
				<href>http://maps.google.com/mapfiles/kml/shapes/cabs.png</href>
			</Icon>
			<hotSpot x="0.5" y="0" xunits="fraction" yunits="fraction"/>
		</IconStyle>
		<ListStyle>
		</ListStyle>
	</Style>
	<Style id="sh_cabs2">
		<IconStyle>
			<color>ffff0000</color>
			<scale>0.816667</scale>
			<Icon>
				<href>http://maps.google.com/mapfiles/kml/shapes/cabs.png</href>
			</Icon>
			<hotSpot x="0.5" y="0" xunits="fraction" yunits="fraction"/>
		</IconStyle>
		<ListStyle>
		</ListStyle>
	</Style>
	<StyleMap id="msn_cabs2">
		<Pair>
			<key>normal</key>
			<styleUrl>#sn_cabs2</styleUrl>
		</Pair>
		<Pair>
			<key>highlight</key>
			<styleUrl>#sh_cabs2</styleUrl>
		</Pair>
	</StyleMap>

	<StyleMap id="msn_cabs1">
		<Pair>
			<key>normal</key>
			<styleUrl>#sn_cabs1</styleUrl>
		</Pair>
		<Pair>
			<key>highlight</key>
			<styleUrl>#sh_cabs1</styleUrl>
		</Pair>
	</StyleMap>
	<Style id="sh_cabs1">
		<IconStyle>
			<color>ff00ff00</color>
			<scale>0.7</scale>
			<Icon>
				<href>http://maps.google.com/mapfiles/kml/shapes/cabs.png</href>
			</Icon>
			<hotSpot x="0.5" y="0" xunits="fraction" yunits="fraction"/>
		</IconStyle>
		<ListStyle>
		</ListStyle>
	</Style>
	<Style id="sn_cabs1">
		<IconStyle>
			<color>ff00ff00</color>
			<scale>0.7</scale>
			<Icon>
				<href>http://maps.google.com/mapfiles/kml/shapes/cabs.png</href>
			</Icon>
			<hotSpot x="0.5" y="0" xunits="fraction" yunits="fraction"/>
		</IconStyle>
		<ListStyle>
		</ListStyle>
	</Style>''')
        kml_file.write('\n')
        for i in range(0, count/2):
            if(threat_collision[i]) :
                kml_file.write('\t<Placemark><TimeStamp><when>'+timeStamp[i]+'</when></TimeStamp><styleUrl>#msn_cabs3</styleUrl><Point><coordinates>'+str(v1_longitude[i])+','+str(v1_latitude[i])+',0</coordinates></Point></Placemark>\n')
            else :
                kml_file.write('\t<Placemark><TimeStamp><when>'+timeStamp[i]+'</when></TimeStamp><styleUrl>#msn_cabs1</styleUrl><Point><coordinates>'+str(v1_longitude[i])+','+str(v1_latitude[i])+',0</coordinates></Point></Placemark>\n')
        kml_file.write('\n')
        for i in range(0, count/2):
            kml_file.write('\t<Placemark><TimeStamp><when>'+timeStamp[i]+'</when></TimeStamp><styleUrl>#msn_cabs2</styleUrl><Point><coordinates>'+str(v2_longitude[i])+','+str(v2_latitude[i])+',0</coordinates></Point></Placemark>\n')
            #print latitudes[i],longitudes[i]
        kml_file.write('''\t</Document>                                                                                                           
</kml>''')
mainFunction()
