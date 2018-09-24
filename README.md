# Intersection-Collision-Warning
To run the command, open a command line, go to the folder and type:
python ICW.py xxx.csv
The ICW application accepts the csv file formats as:
vehicle_id, timeStamp, longitude, latitude
It expects the first data row of csv file to be the heading of the data.
timeStamp,longitude,latitude
It uses 2D Polygon Detection Algorithm using Separating Axis Theorem to detect
the collision probability
