#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from fiducial_msgs.msg import FiducialTransformArray
from geometry_msgs.msg import Pose, Pose2D
from tf2_msgs.msg import TFMessage

class ArucoLocalizer:
    def __init__(self):
        rospy.init_node('aruco_localizer', anonymous=False)
        
        ODOM_TOPIC = rospy.get_param('/aruco_localization/out_odom_topic')
        ARUCO_TOPIC = rospy.get_param('/aruco_localization/in_aruco_topic')
        BASE_TO_CAM = rospy.get_param('/aruco_localization/base_to_cam')
        PARENT_TF = rospy.get_param('aruco_localization/parent_tf')
        CHILD_TF = rospy.get_param('aruco_localization/child_tf')
        self.parameters = rospy.get_param('/aruco_localization/markers/')
        
        
        self.tf_listener = rospy.Subscriber('/tf', TFMessage, self.tf_callback)
        self.odom_publisher = rospy.Publisher(ODOM_TOPIC, Odometry, queue_size=50)
        rospy.Subscriber(ARUCO_TOPIC, FiducialTransformArray, self.aruco_callback)
        
        self.arucos = {}
        
        self.base_to_cam = Pose()
        self.base_to_cam.position.x = BASE_TO_CAM['position']['x']
        self.base_to_cam.position.y = BASE_TO_CAM['position']['y']
        self.base_to_cam.position.z = BASE_TO_CAM['position']['z']
        self.base_to_cam.orientation.x = BASE_TO_CAM['orientation']['x']
        self.base_to_cam.orientation.y = BASE_TO_CAM['orientation']['y']
        self.base_to_cam.orientation.z = BASE_TO_CAM['orientation']['z']
        self.base_to_cam.orientation.w = BASE_TO_CAM['orientation']['w']
        
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = PARENT_TF
        self.odom_msg.child_frame_id = CHILD_TF
        
        self.fiducials = []
        
    def aruco_callback(self, msg):
        self.fiducials = msg.transforms
        
    def tf_callback(self, msg):
        self.tf_list = msg.transforms
        base_to_fid_pose = Pose()
        base_to_fid_pose.position.x = None
        base_to_fid_pose.position.y = None
        base_to_fid_pose.position.z = None
        
        for tf in self.tf_list:
            if tf.header.frame_id == 'camera_link' and 'fiducial' in tf.child_frame_id:
                base_to_fid_pose.position.x = tf.transform.translation.z + self.base_to_cam.position.x
                base_to_fid_pose.position.y = - tf.transform.translation.y + self.base_to_cam.position.y
                base_to_fid_pose.position.z = - tf.transform.translation.x + self.base_to_cam.position.z
                
                self.odom_msg.pose.pose.position.x = self.arucos[tf.child_frame_id].x - base_to_fid_pose.position.x
                self.odom_msg.pose.pose.position.y = self.arucos[tf.child_frame_id].y - base_to_fid_pose.position.y
                self.odom_msg.pose.pose.position.z = 0
                self.odom_msg.header.stamp = rospy.Time.now()
                self.odom_publisher.publish(self.odom_msg)
    
    def set_aruco_poses(self):
        keys = self.parameters.keys()
        for key in keys:
            pos = Pose2D()
            pos.x = self.parameters[key]['pose']['x']
            pos.y = self.parameters[key]['pose']['y']
            pos.theta = self.parameters[key]['pose']['theta']
            self.arucos.update({'fiducial_' + key[-1]: pos})
    
if __name__ == '__main__':
    try:
        ar_loc = ArucoLocalizer()
        ar_loc.set_aruco_poses()
        rospy.spin()
    except KeyboardInterrupt:
        rospy.signal_shutdown('KeyboardInterrupt')
    
    