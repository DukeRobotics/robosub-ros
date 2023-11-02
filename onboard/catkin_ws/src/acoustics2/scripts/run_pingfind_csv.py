import time
import csv
from find_point_simple import PingFinder


pf = PingFinder()

with open('onboard/catkin_ws/src/acoustics2/fake_data/final_calc_square5.csv2', 'w', newline='') as csvfile:
    csv_writer = csv.writer(csvfile)
    with open('onboard/catkin_ws/src/acoustics2/fake_data/temp_data.csv2', 'r') as timing_points:
        csv_reader = csv.reader(timing_points)
        next(csv_reader)
        for row in csv_reader:
            Rx, Ry, t1, t2, t3 = map(float, row)
            start_time = time.time()

            x,y = pf.make_guess(t1,t2,t3)

            delta_time = time.time() - start_time   
            csv_writer.writerow([Rx, Ry, round(x,2), round(y,2), round(delta_time,3)])