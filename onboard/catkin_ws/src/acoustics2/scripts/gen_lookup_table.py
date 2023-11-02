import numpy as np
import csv

D = 1
v = 1500.0

with open('onboard/catkin_ws/src/acoustics2/fake_data/temp_data_2.csv2', 'w', newline='') as csvfile:
    csv_writer = csv.writer(csvfile)
    csv_writer.writerow(['theta', 'Dt1', 'Dt2'])
    for y in np.arange(-30, 31, 1):
        for x in np.arange(-30, 31, 1):
            t1 = np.sqrt((x - 0)**2 + (y -0)**2)/v
            t2 = np.sqrt((x - D)**2 + (y - 0)**2)/v
            t3 = np.sqrt((x - 0)**2 + (y - D)**2)/v
            #store values into a csv
            scale = 1*10**(6)
            angle = np.arctan2(y,x)
            csv_writer.writerow([round(angle, 4), round(t1-t2,7), round(t1-t3,7)])

        
