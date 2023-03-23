# Sonar

The `soanar` package contains scripts which interface with the bluerobotics ping-360 sonar on the robot. Unlike other packages on the robot, the sonar acts on a call recieve system where once the sonar recieves a `sonar_request` message it will return a `sonar_response`. 

The sonar works by sending out a "ping" and waiting for a response. The ping-360 sends out a number of pings within the given range and returns an array of 8-bit integer values which represent the intensity of the echo. This array represents one line along the xy plane of the robot where each index in the array corresponds to a distance from the sonar sensor.

## Sonar Documentaiton

The bluerobotics ping-360 sonar uses the ping-python protocal to communicate with the sonar over serial usb. The sonar library takes in a few input parameters to complete a sonar sweep:
- transmit_duration: the length of each ping
- sample period: time between pings
- transmit_frequency: frequency which the ping is sent at (higher than acoustics so it doesnt interfere)
- number_of_samples: the number of indicies returned within the given range.
- BAUD_RATE: sampling rate of the sonar

All of these parameters control how far the sonar scans and how many data points there are within the scan. The `transmit_duration` and `sample period` directly control the range of the sonar scan by a linear constant. The `number_of_samples` controls how granular the information is within the scan. 

With a 5m scan, the paramters are:

    transmit_duration = 27
    sample_period = 222
    number_of_samples = 1200

With these parameters the data recieved from the soanr is an array of 1200 8-bit integers. The index of each element in the array is a fraction of the total distance scanned (ie index 600 is at position 2.5m) and the value at that distance is the relative echo intensity. The higher the intensity the more object there is at that location.

Taking multiple scans over a range of angles can generate an image which shows the objects surrounding the robot. Below example of an image which the sonar generates: 

![Sonar_Image](Sonar_Image.jpeg)

each row is an angle from -90 to 90 degrees and each column is a radial depth away from the robot. The yellow dot in the center is an object in the middle of the pool.

## Communication

To communcate with other branches the `sonar_publisher.py` creates a sonar object and then waits for CV to publish a request for a scan with a start angle and an end angle. Once the scan is complete it will publish the result to the sonar response topic.

## General Sonar Data

- Horizontal beam with: 2°
- Vertial beam with: 25°
- Min range: .75m
- Max range: 50m

For more information use these guides by bluerobotics:
- https://bluerobotics.com/store/sensors-sonars-cameras/sonar/ping360-sonar-r1-rp/
- https://bluerobotics.com/learn/understanding-and-using-scanning-sonars/