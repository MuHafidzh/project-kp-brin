import os
import sys
import rospy

from queue import Queue
from datetime import timedelta

from custom_msg.msg import yolo
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from build.process import cv2, from_cv, draw, init

DISPLAY = True

def run_image(detector):
    if cv_image is not None:
        try:
            while True:
                yolo_msg = yolo()
                p = from_cv(model=detector, source=cv_image)

                class_id = []
                center_x = []
                center_y = []

                if p['det']:
                    print(f'Detect: {list(set(p["name"]))}')
                    for i, _ in enumerate(p['id']):
                        class_id.append(p['name'][i])
                        center_x.append(p['xywh'][i][0])
                        center_y.append(p['xywh'][i][1])

                yolo_msg.class_id = class_id
                yolo_msg.center_x = center_x
                yolo_msg.center_y = center_y
                pub.publish(yolo_msg)

                if DISPLAY:
                    draw(p['det'], cv_image)
                    cv2.namedWindow("Live")
                    cv2.imshow("Live", cv_image)
                    cv2.waitKey(1)
                
        except KeyboardInterrupt:
            cv2.destroyAllWindows()
            
    else:
        print("NO IMAGE")

def callback(msg):
    global cv_image
    try:
        cv_image = CvBridge().imgmsg_to_cv2(msg, desired_encoding='bgr8')

        Queue().put(cv_image)
    
    except CvBridgeError as e:
        rospy.logger('CvBridge Error: {0}'.format(e))

def loop(detector):
    run_image(detector)
    rospy.loginfo('Control Loop Running')

def main():
    global pub 
    pub = rospy.Publisher('yolo_topic', yolo, queue_size=10)
    detector = init()
    image_sub = rospy.Subscriber('/camera/color/image_raw', Image, callback)

    rospy.init_node('listener', anonymous=True)

    t = rospy.Time.now()
    CONTROL_LOOP = timedelta(seconds=.01)
    control_loop_duration = CONTROL_LOOP.total_seconds()
    control_loop = rospy.Timer(rospy.Duration(control_loop_duration), lambda event: loop(detector))

    rospy.spin()

if __name__ == '__main__':
    main()
