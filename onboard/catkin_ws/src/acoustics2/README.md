# Acoustics

This package handles the calculations for locating an underwater acoustics pinger that is pinging at a certain frequency.

## Design Doc

- Get time of arrival of each ping from hydrophones topic (given as a ros topic or simulated data)
    - Given 4 times of arrival for four corners of the robot
    - Given distance between each corner
- Using time of arrival and distance data calculate estimated asimuth.
- Use current state information and direction to calculate an estimated position.
    - Average over a bunch of trials every 2 seconds.