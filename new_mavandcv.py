import cv2
import numpy as np
import math
from cv_bridge import CvBridge
from pymavlink import mavutil
import rospy
from sensor_msgs.msg import Image as cam_img_msg
import time
i = 0 # Iteration for data loggging

k = 30 # value of k (height from origin to draw the line)

multiplier = 1.1# increase the value to scale up pixel to real distance ratio

time_sleep_after_command_sent = 0.5 # time to sleep after the command start

def generate_continuous_contour(contour):
    # Reshape the contour to (N, 2) - each point is (x, y)
    contour = contour.reshape(-1, 2)
    
    continuous_contour = []

    for i in range(len(contour) - 1):
        x1, y1 = contour[i]
        x2, y2 = contour[i + 1]

        # Linearly interpolate between the two points
        if x1 == x2:
            # If the x coordinates are the same, just take all y's between y1 and y2
            for y in range(min(y1, y2), max(y1, y2) + 1):
                continuous_contour.append([x1, y])
        else:
            # Otherwise, calculate the line equation: y = mx + b
            slope = (y2 - y1) / (x2 - x1)
            intercept = y1 - slope * x1

            # Iterate over all x coordinates between x1 and x2 (inclusive)
            for x in range(min(x1, x2), max(x1, x2) + 1):
                y = int(round(slope * x + intercept))
                continuous_contour.append([x, y])

    return np.array(continuous_contour)

def finding_top_two_contours(image_path):
    global i
    image1 = cv2.imread(image_path)
    resized_image = cv2.resize(image1, (640, 480))
    gray_image = cv2.cvtColor(resized_image, cv2.COLOR_BGR2GRAY)
    blurred_image = cv2.GaussianBlur(gray_image, (5, 5), 0)
    edges = cv2.Canny(blurred_image, 50, 150)
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=lambda contour: len(contour), reverse=True)

    #debug
    # cv2.drawContours(resized_image, contours, -1, (0, 255, 0), 2)
    # cv2.imwrite(f"all_contours_{i}.jpg", resized_image)
    # cv2.imshow("all_contours", resized_image)
    # cv2.waitKey(0)
    #debug

    contours_ned = None

    # #debug
    # cv2.drawContours(resized_image, contours_ned, -1, (0, 255, 0), 2)
    # cv2.imwrite(f"top_contours_{i}.jpg", resized_image)
    # cv2.imshow("top_contours", resized_image)
    # cv2.waitKey(0)
    #debug

    try:
        contours_ned = [contours[0], contours[1]]
        contours_ned[0] = generate_continuous_contour(contours_ned[0])
        contours_ned[1] = generate_continuous_contour(contours_ned[1])
    except Exception as E:
        
        print("Contours not found")

    return contours_ned

def finding_intersections(image_path, contours_found):
    global i
    global k 
    image = cv2.imread(image_path)
    height, width, channels = image.shape
    
    origin = (int(width/2), int(height/2))


    def finding_nearest_intersection(height , contour):
        intersection = None
        j = 0
        while True:
            for item in contour:
                if item[1] == height - j:
                    intersection = tuple(item)
                    return intersection
            j = j + 1
        


    one_intersection = finding_nearest_intersection(origin[1] - k, contours_found[0])
    two_intersection = finding_nearest_intersection(origin[1] - k,contours_found[1])

    #debug
    print(f"{i}) first_intersection: {one_intersection} , second_intersection = {two_intersection}")
    #debug

    return [tuple(one_intersection), tuple(two_intersection), tuple(origin)]

def get_altitude_h(connection):
    msg = connection.recv_match(type="LOCAL_POSITION_NED",blocking=True )
    altitude = round(msg.z, 2)
    return -altitude



def finding_midpoint(one_point, two_point):
    global i
    midpoint_x = int((one_point[0] + two_point[0]) / 2)
    midpoint_y = int((one_point[1] + two_point[1]) / 2)
    #debug
    print(f"{i}) midpoint_found = ({midpoint_x}, {midpoint_y})")
    #debug

    return (midpoint_x, midpoint_y)

bridge = CvBridge()

def image_callback(msg):
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imwrite('image_live.jpg', cv_image)
        # rospy.loginfo("Image saved as image_live.jpg")
        
    except Exception as e:
        rospy.logerr("Failed to process image: %s", str(e))

def give_xy_lr_theta(inital_posi, final_posi, scale_of_pixel_to_distance):
    global i
    global multiplier
    delx = final_posi[0] - inital_posi[0]
    dely = final_posi[1] - inital_posi[1]
    vel = scale_of_pixel_to_distance * multiplier
    NSvel = - vel* dely
    og_EWvel = vel * delx
    og_yaw_angle = math.atan(delx/dely)
    if 2 == 3:
        yaw_angle = 0
        EWvel = 0
    else:
        EWvel = og_EWvel
        yaw_angle = og_yaw_angle

    #debug
    print(f"{i}) forward/backward: {NSvel}, left/right: {EWvel}({og_EWvel}), rotation: {yaw_angle}({og_yaw_angle})")
    #debug

    return NSvel,EWvel,yaw_angle
    
def movement(NSvel,EWvel,yaw_angle):
        global time_sleep_after_command_sent
        global i
        the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, the_connection.target_system,
                                the_connection.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, int(0b010111111000), NSvel, EWvel, 0, NSvel, EWvel, 0, 0, 0, 0, yaw_angle, 0))
        
        #debug
        print(f"{i}Movement Command Sent")
        #debug
        time.sleep(time_sleep_after_command_sent)
        return 
    

def give_feed_to_ros_node(marked_image_data):
    global i 
    if i ==0:
        rospy.init_node('publisher_node', anonymous=True)
        image_pub = rospy.Publisher('/marked_image_data', cam_img_msg, queue_size=1)
        print("Publishing_initialised")
    ros_image = bridge.cv2_to_imgmsg(marked_image_data, encoding="bgr8")
    image_pub.publish(ros_image)
    return

the_connection = mavutil.mavlink_connection('udpin:localhost:14551')


the_connection.wait_heartbeat()
print("Heartbeat form system ( system %u component %u)" % (the_connection.target_system, the_connection.target_component))


rospy.init_node('drone_navigation', anonymous=True)
image_sub = rospy.Subscriber("/webcam1/image_raw", cam_img_msg, image_callback)

scale = get_altitude_h(the_connection)/554.25
print(scale)

j =0 
while True:
    try:
        
        imag_data = cv2.imread('image_live.jpg')
        # cv2.imshow("marked_image", imag_data)
        # cv2.waitKey(0)
        time.sleep(0.5)
        image = 'image_current.jpg'
        cv2.imwrite(image,imag_data)
        contours01 = finding_top_two_contours(image)
        if contours01 == None:
            print("Drone rassata bhatak gya")
            break
        points_list =finding_intersections(image, contours01)

        initial = points_list[2]
        
        final = finding_midpoint(points_list[0], points_list[1])


        #debug
        image_data = cv2.imread('image_current.jpg')
        cv2.circle(image_data, initial ,2 , (255, 0, 0),-1 )
        cv2.circle(image_data, final ,2, (0, 0, 255),-1 )
        # cv2.imshow("marked_image", image_data)
        # cv2.waitKey(0)
        # cv2.imwrite(f"marked_image{i}.jpg", image_data)
        #debug


        NSvel,EWvel,yaw_angle = give_xy_lr_theta(initial, final, scale)

        movement(NSvel,EWvel,yaw_angle)
        # time.sleep(1)
        # give_feed_to_ros_node(image_data)
        i = i + 1
    except Exception as E1:
        j = j + 1
        print(f"Image saving error {j}")

