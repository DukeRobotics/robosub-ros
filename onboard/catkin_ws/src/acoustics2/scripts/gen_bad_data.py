import numpy as np
import csv

D = 1
v = 1500.0

with open('onboard/catkin_ws/src/acoustics2/fake_data/timing_points_square.csv2', 'w', newline='') as csvfile:
    csv_writer = csv.writer(csvfile)
    for y in range (-30, 30):
        for x in range (-30, 30):
            t1 = np.sqrt((x - 0)**2 + (y -0)**2)/v
            t2 = np.sqrt((x - D)**2 + (y - 0)**2)/v
            t3 = np.sqrt((x - 0)**2 + (y - D)**2)/v
            #store values into a csv
            csv_writer.writerow([x, y, t1, t2, t3])

        
