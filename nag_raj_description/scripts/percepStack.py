import cv2 
import rospy
import message_filters
from sensor_msgs.msg import Image
import tf2_ros
import tf2_msgs.msg


class Perception:
    def __init__(self) -> None:
        # Initialisation of CV model

        
        sub_rgb = message_filters.Subscriber("/camera/color/image_raw2", Image)
        sub_depth = message_filters.Subscriber("/camera/depth/image_raw2", Image)
        ts = message_filters.ApproximateTimeSynchronizer([sub_depth, sub_rgb], queue_size=10, slop=0.5)
        ts.registerCallback(self.callback)

        self.pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)

        self.rgb_image, self.depth_image = None, None
        self.rgb_shape, self.depth_shape = None, None

        self.found=False
        
        
    def rgb_callback(self, rgb_message) :

        self.rgb_image = self.bridge.imgmsg_to_cv2(rgb_message, desired_encoding = "bgr8")
        self.rgb_shape = self.rgb_image.shape

    
    def depth_callback(self, depth_message) :
        self.depth_image = self.bridge.imgmsg_to_cv2(depth_message, desired_encoding=depth_message.encoding)
        self.depth_shape = self.depth_image.shape 

    def callback(self,depth_data, rgb_data):
        self.depth_callback(depth_data)
        self.rgb_callback(rgb_data)


def main():
    rospy.init_node("percepStack", anonymous=True)
    # try:
    ps = Perception()
    rospy.sleep(1)
    while True:
        ps.detect()
        
    # except Exception as e:
        # ##print("Error:", str(e))    




if __name__=="__main__" :
    main()
    rospy.spin()