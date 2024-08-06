import rospy
from std_msgs.msg import Int8
from spike_detection import PipelineConfig, spikes_pipeline, get_first_hydrophones

CENTER_FREQUENCY = 35000
FREQ_RADIUS = 1000

spike_config = PipelineConfig()
config = PipelineConfig(CENTER_FREQUENCY - FREQ_RADIUS, CENTER_FREQUENCY + FREQ_RADIUS)
config.noise_floor_stdev_mult = 5.5

def localization_pub():

    # -1 means we are under the octagon
    # 0-3 inclusive are the 4 quadrants relative to the robot heading

    pub = rospy.Publisher('/acoustics/localization', Int8, queue_size=10)
    rospy.init_node('localization_pub', anonymous=True)

    while not rospy.is_shutdown():
        spikes = spikes_pipeline('./savefiles', config)
        firsts = get_first_hydrophones(spikes, config)
        
        times = [s.time for s in spikes[-1]]
        range = max(times) - min(times)
        if range < 1e-5:
            counts = -1
        else:
            counts = [0, 0, 0, 0]
            for i in firsts:
                counts[i] += 1
            localization = counts.index(max(counts))
            
        
        
        rospy.loginfo(localization)
        pub.publish(localization)

if __name__ == '__main__':
    try:
        localization_pub()
    except rospy.ROSInterruptException:
        pass

