import numpy as np
import csv

D = 1
v = 1500.0

with open('onboard/catkin_ws/src/acoustics2/fake_data/lookup_table.csv2', 'w', newline='') as csvfile:
    csv_writer = csv.writer(csvfile)
    for y in np.arange(-30, 31, 1):
        for x in np.arange(-30, 31, 1):
            t1 = np.sqrt((x - 0)**2 + (y -0)**2)/v
            t2 = np.sqrt((x - D)**2 + (y - 0)**2)/v
            t3 = np.sqrt((x - 0)**2 + (y - D)**2)/v
            #store values into a csvx``
            scale = 1*10**(6)
            csv_writer.writerow([round(x, 3), round(y,3), round(t1,10), round(t2,10), round(t3,10)])

        
