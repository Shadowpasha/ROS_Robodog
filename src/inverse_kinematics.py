#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist,Pose,Quaternion,Point
from nav_msgs.msg import Odometry
import leg_kinematics
from tf.transformations import *

FL = leg_kinematics.leg(0.26,0.26,0.12,0.15,0,0)
FR = leg_kinematics.leg(0.26,0.26,0.12,0.15,1,0)
BL = leg_kinematics.leg(0.26,0.26,0.12,0.15,0,1)
BR = leg_kinematics.leg(0.26,0.26,0.12,0.15,1,1)

Zoffset = 0.0 
Xoffset = 0.0
Yoffset = 0.0
Pitchoffset = 0.0
Rolloffset = 0.0
Yawoffset = 0.0
step_X = 0.02
step_Height = 0.03
stage = 0
last_check =  0
xspeed = 0.0
yspeed = 0.0
x_dist = 0.0
y_dist = 0.0
yaw = 0.0

def callback(data):
    global Zoffset,Xoffset,Yoffset,Pitchoffset,Rolloffset,Yawoffset
    Xoffset = data.linear.x 
    Yoffset = data.linear.y 
    Yawoffset = data.angular.z * 0.08

if __name__ == '__main__':

    pub = rospy.Publisher('/position_controller/command', Float64MultiArray, queue_size=10)
    odompub = rospy.Publisher('/raw_odom',Odometry, queue_size=10)
    rospy.Subscriber("/cmd_vel", Twist, callback)
    rospy.init_node('inverse_kinematics', anonymous=True)
    rate = rospy.Rate(30) # 10hz
    while not rospy.is_shutdown():
        if(Xoffset != 0.0 or Yoffset != 0.0 or Yawoffset != 0.0):
            if(rospy.get_time() - last_check > 0.3):
 
                if stage == 0:
                    FL.calc_angles(0.0 + step_X*Xoffset,0.0 + step_X*Yoffset,0.16-step_Height,0.0,0.0,Yawoffset)
                    FR.calc_angles(0.0 - step_X*Xoffset,0.0 - step_X*Yoffset,0.16,0.0,0.0,-Yawoffset)
                    BL.calc_angles(0.0 - step_X*Xoffset,0.0 - step_X*Yoffset,0.16,0.0,0.0,-Yawoffset)
                    BR.calc_angles(0.0 + step_X*Xoffset,0.0 + step_X*Yoffset,0.16-step_Height,0.0,0.0,Yawoffset)
                    stage = 1
                elif stage == 1:
                    FL.calc_angles(0.0 + step_X*Xoffset,0.0 + step_X*Yoffset,0.16,0.0,0.0,Yawoffset)
                    FR.calc_angles(0.0 - step_X*Xoffset,0.0 - step_X*Yoffset,0.16,0.0,0.0,-Yawoffset)
                    BL.calc_angles(0.0 - step_X*Xoffset,0.0 - step_X*Yoffset,0.16,0.0,0.0,-Yawoffset)
                    BR.calc_angles(0.0 + step_X*Xoffset,0.0 + step_X*Yoffset,0.16,0.0,0.0,Yawoffset)
                    x_dist+= step_X*Xoffset
                    y_dist+= step_X*Yoffset
                    yaw+= Yawoffset
                    stage = 2
                elif stage == 2:
                    FL.calc_angles(0.0 - step_X*Xoffset,0.0 - step_X*Yoffset,0.16,0.0,0.0,-Yawoffset)
                    FR.calc_angles(0.0 + step_X*Xoffset,0.0 + step_X*Yoffset,0.16-step_Height,0.0,0.0,Yawoffset)
                    BL.calc_angles(0.0 + step_X*Xoffset,0.0 + step_X*Yoffset,0.16-step_Height,0.0,0.0,Yawoffset)
                    BR.calc_angles(0.0 - step_X*Xoffset,0.0 - step_X*Yoffset,0.16,0.0,0.0,-Yawoffset)
                    stage = 3
                elif stage == 3:
                    FL.calc_angles(0.0 - step_X*Xoffset,0.0 - step_X*Yoffset,0.16,0.0,0.0,-Yawoffset)
                    FR.calc_angles(0.0 + step_X*Xoffset,0.0 + step_X*Yoffset,0.16,0.0,0.0,Yawoffset)
                    BL.calc_angles(0.0 + step_X*Xoffset,0.0 + step_X*Yoffset,0.16,0.0,0.0,Yawoffset)
                    BR.calc_angles(0.0 - step_X*Xoffset,0.0 - step_X*Yoffset,0.16,0.0,0.0,-Yawoffset)
                    x_dist+= step_X*Xoffset
                    y_dist+= step_X*Yoffset
                    yaw+= Yawoffset
                    stage = 0
                
                last_check = rospy.get_time()
        else:
                FL.calc_angles(0.0,0.0,0.16,0.0,0.0,0.0)
                FR.calc_angles(0.0,0.0,0.16,0.0,0.0,0.0)
                BL.calc_angles(0.0,0.0,0.16,0.0,0.0,0.0)
                BR.calc_angles(0.0,0.0,0.16,0.0,0.0,0.0)
                stage == 0
        
        msg = Float64MultiArray()
        msg.data = [FL.shoulder_angle,FL.hip_angle,FL.knee_angle,FR.shoulder_angle, FR.hip_angle,FR.knee_angle,BL.shoulder_angle,BL.hip_angle,BL.knee_angle,BR.shoulder_angle,BR.hip_angle,BR.knee_angle]
        pub.publish(msg)
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "raw_odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose = Pose(Point(x_dist, y_dist, 0.), Quaternion(*quaternion_from_euler(0, 0, yaw)))
        odompub.publish(odom)
        rate.sleep()
