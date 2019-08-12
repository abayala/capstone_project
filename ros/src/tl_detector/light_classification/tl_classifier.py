from styx_msgs.msg import TrafficLight
import semaphoreDetection as SD
import rospy

class TLClassifier(object):
    def __init__(self):
        # TODO load classifier
        # Not using classifier, so not needed

        pass

    def get_classification(self, image):
        """Determines the color of the traffic light in the image
        Args:
            image (cv::Mat): image containing the traffic light
        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """

        rospy.logdebug("Classification requested")

        numRed, numGreen, numAmbar = SD.findSemaphore(image)

        if numRed > 1:
            rospy.logwarn("RED light detected")
            return TrafficLight.RED

        elif numGreen > 1:
            rospy.logwarn("GREEN light detected")
            return TrafficLight.GREEN

        elif numAmbar > 1:
            rospy.logwarn("YELLOW light detected")
            return TrafficLight.YELLOW

        else:
            rospy.logwarn("UNKNOWN light")
            return TrafficLight.UNKNOWN
