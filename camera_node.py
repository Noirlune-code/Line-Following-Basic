import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image as cam_img_msg

bridge = CvBridge()

latest_image = None

def image_callback(msg):
    global latest_image
    latest_image = msg


def save_image_file(event = None):
    global latest_image

    if latest_image is not None:
        try:
            image_data = bridge.imgmsg_to_cv2(latest_image, desired_encoding='bgr8')

            file_name = 'current_used_image.png'

            cv2.imwrite(file_name, image_data)

            rospy.loginfo("Image saved")

        except Exception as e:
            rospy.logerr(f"Failed to save image: {e}")


def main():

    rospy.init_node("camera_data_saver", anonymous=True)

    image_topic_name = "/webcam1/image_raw"

    rospy.Subscriber(image_topic_name, cam_img_msg, image_callback)

    rospy.Timer(rospy.Duration(0.5), save_image_file)

    rospy.spin()


if __name__ == "__main__":
    main()

