import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO

def image_callback(data):
    # Initialize YOLO model
    model = YOLO("/home/aakshar/catkin_ws/src/flipkartGrid/perception/scripts/ml_models/qr_detect.pt")

    # Convert the ROS image message to an OpenCV image
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except Exception as e:
        print(e)
        return

    # Perform QR code detection using YOLO
    results = model.predict(cv_image, show=True, save=False, hide_labels=False, hide_conf=False, conf=0.5, save_txt=False, save_crop=False, line_thickness=2)

    masks = []
    bounding_boxes = []

    for result in results:
        for mask in result.masks:
            print(mask.xy[0])
            masks.append(mask.xy[0])
        for box in result.boxes:
            bounding_boxes.append(box.xyxy[0])
            for i in masks:
                for x, y in i:
                    cv2.circle(cv_image, (int(x), int(y)), 1, (0, 0, 255), 2)

    # Display the image with detected QR codes
    cv2.imshow("image", cv_image)
    cv2.waitKey(0)

def qr_code_detection_node():
    rospy.init_node('qr_code_detection_node', anonymous=True)
    rospy.Subscriber('/camera/rgb/image_rect_color', Image, image_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        qr_code_detection_node()
    except rospy.ROSInterruptException:
        pass