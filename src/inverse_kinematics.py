#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist,Pose,Quaternion,Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import leg_kinematics
from tf.transformations import *
import tf

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
step_X = 0.025
step_Height = 0.025
stage = 0
last_check =  0
xspeed = 0.0
yspeed = 0.0
x_dist = 0.0
y_dist = 0.0
yaw = 0.0
leg_fl_off,leg_fr_off,leg_bl_off,leg_br_off = 0.0,0.0,0.0,0.0
kpr,kir,kdr = 0.005,2.0,0.0001
kpp,kip,kdp = 0.005,2.0,0.0001
pre_time = 0.0
kpout_r,kiout_r,kdout_r = 0.0,0.0,0.0
kpout_p,kiout_p,kdout_p = 0.0,0.0,0.0
prev_roll_error = 0.0
prev_pitch_error = 0.0
roll,pitch,yaw = 0.0,0.0,0.0
roll_error,pitch_error = 0.0, 0.0

def callback(data):
    global Zoffset,Xoffset,Yoffset,Pitchoffset,Rolloffset,Yawoffset
    Xoffset = data.linear.x 
    Yoffset = data.linear.y 
    Yawoffset = data.angular.z * 0.08

def imu_callback(imu):
    global roll,pitch,yaw
    orientation_q = imu.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)

 

if __name__ == '__main__':

    pub = rospy.Publisher('/position_controller/command', Float64MultiArray, queue_size=10)
    odompub = rospy.Publisher('/odom_raw',Odometry, queue_size=10)
    odom_broadcaster = tf.TransformBroadcaster()
    rospy.Subscriber("/cmd_vel", Twist, callback)
    rospy.Subscriber("/imu", Imu, imu_callback)
    rospy.init_node('inverse_kinematics', anonymous=True)
    rate = rospy.Rate(30) # 10hz
    while not rospy.is_shutdown():
        if(Xoffset != 0.0 or Yoffset != 0.0 or Yawoffset != 0.0):
            if(rospy.get_time() - last_check > 0.3):
 
                if stage == 0:
                    FL.calc_angles(0.0 + step_X*Xoffset,0.0 + step_X*Yoffset,0.16-step_Height + leg_fl_off,0.0,0.0,Yawoffset)
                    FR.calc_angles(0.0 - step_X*Xoffset,0.0 - step_X*Yoffset,0.16 + leg_fr_off,0.0,0.0,-Yawoffset)
                    BL.calc_angles(0.0 - step_X*Xoffset,0.0 - step_X*Yoffset,0.16 + leg_bl_off,0.0,0.0,-Yawoffset)
                    BR.calc_angles(0.0 + step_X*Xoffset,0.0 + step_X*Yoffset,0.16-step_Height + leg_br_off,0.0,0.0,Yawoffset)
                    stage = 1
                elif stage == 1:
                    FL.calc_angles(0.0 + step_X*Xoffset,0.0 + step_X*Yoffset,0.16 + leg_fl_off,0.0,0.0,Yawoffset)
                    FR.calc_angles(0.0 - step_X*Xoffset,0.0 - step_X*Yoffset,0.16 + leg_fr_off,0.0,0.0,-Yawoffset)
                    BL.calc_angles(0.0 - step_X*Xoffset,0.0 - step_X*Yoffset,0.16 + leg_bl_off,0.0,0.0,-Yawoffset)
                    BR.calc_angles(0.0 + step_X*Xoffset,0.0 + step_X*Yoffset,0.16 + leg_br_off,0.0,0.0,Yawoffset)
                    x_dist+= (step_X*Xoffset)*1.4 * math.cos(yaw) + (step_X*Yoffset)*1.4 * -math.sin(yaw)
                    y_dist+= (step_X*Xoffset)*1.4 * math.sin(yaw) + (step_X*Yoffset)*1.4 * math.cos(yaw)
                    yaw+= (Yawoffset)*1.4
                    stage = 2
                elif stage == 2:
                    FL.calc_angles(0.0 - step_X*Xoffset,0.0 - step_X*Yoffset,0.16 + leg_fl_off,0.0,0.0,-Yawoffset)
                    FR.calc_angles(0.0 + step_X*Xoffset,0.0 + step_X*Yoffset,0.16-step_Height + leg_fr_off,0.0,0.0,Yawoffset)
                    BL.calc_angles(0.0 + step_X*Xoffset,0.0 + step_X*Yoffset,0.16-step_Height + leg_bl_off,0.0,0.0,Yawoffset)
                    BR.calc_angles(0.0 - step_X*Xoffset,0.0 - step_X*Yoffset,0.16 + leg_br_off,0.0,0.0,-Yawoffset)
                    stage = 3
                elif stage == 3:
                    FL.calc_angles(0.0 - step_X*Xoffset,0.0 - step_X*Yoffset,0.16 + leg_fl_off,0.0,0.0,-Yawoffset)
                    FR.calc_angles(0.0 + step_X*Xoffset,0.0 + step_X*Yoffset,0.16 + leg_fr_off,0.0,0.0,Yawoffset)
                    BL.calc_angles(0.0 + step_X*Xoffset,0.0 + step_X*Yoffset,0.16 + leg_bl_off,0.0,0.0,Yawoffset)
                    BR.calc_angles(0.0 - step_X*Xoffset,0.0 - step_X*Yoffset,0.16 + leg_br_off,0.0,0.0,-Yawoffset)
                    x_dist+= (step_X*Xoffset)*1.4 * math.cos(yaw) + (step_X*Yoffset)*1.4 * -math.sin(yaw)
                    y_dist+= (step_X*Xoffset)*1.4 * math.sin(yaw) + (step_X*Yoffset)*1.4 * math.cos(yaw)
                    yaw+= (Yawoffset)*1.4
                    stage = 0



                # if stage == 0:
                #     FL.calc_angles(0.0 + step_X*Xoffset*1.5,0.0 + step_X*Yoffset*1.5 - 0.02,0.16 + leg_fl_off,0.0,0.0,Yawoffset*1.5)
                #     FR.calc_angles(0.0 - step_X*Xoffset,0.0 - step_X*Yoffset- 0.02,0.16 + leg_fr_off,0.0,0.0,-Yawoffset)
                #     BL.calc_angles(0.0 + step_X*Xoffset,0.0 + step_X*Yoffset- 0.02,0.16 + leg_bl_off,0.0,0.0,Yawoffset)
                #     BR.calc_angles(0.0 - step_X*Xoffset*1.5,0.0 - step_X*Yoffset*1.5,0.16  + leg_br_off,0.0,0.0,-Yawoffset*1.5)
                #     stage = 1
                # elif stage == 1:
                #     FL.calc_angles(0.0 + step_X*Xoffset*1.5,0.0 + step_X*Yoffset*1.5- 0.02,0.16 + leg_fl_off,0.0,0.0,Yawoffset*1.5)
                #     FR.calc_angles(0.0 - step_X*Xoffset,0.0 - step_X*Yoffset- 0.02,0.16 + leg_fr_off,0.0,0.0,-Yawoffset)
                #     BL.calc_angles(0.0 + step_X*Xoffset,0.0 + step_X*Yoffset- 0.02,0.16 + leg_bl_off,0.0,0.0,+Yawoffset)
                #     BR.calc_angles(0.0 - step_X*Xoffset,0.0 - step_X*Yoffset,0.16 - step_Height + leg_br_off,0.0,0.0,-Yawoffset)
                #     stage = 2
                # elif stage == 2:
                #     FL.calc_angles(0.0 + step_X*Xoffset,0.0 + step_X*Yoffset- 0.02,0.16 + leg_fl_off,0.0,0.0,Yawoffset)
                #     FR.calc_angles(0.0 - step_X*Xoffset*1.5,0.0 - step_X*Yoffset*1.5- 0.02,0.16 + leg_fr_off,0.0,0.0,-Yawoffset*1.5)
                #     BL.calc_angles(0.0 - step_X*Xoffset,0.0 - step_X*Yoffset- 0.02,0.16 + leg_bl_off,0.0,0.0,-Yawoffset)
                #     BR.calc_angles(0.0 + step_X*Xoffset,0.0 + step_X*Yoffset*1.5,0.16 - step_Height + leg_br_off,0.0,0.0,Yawoffset)
                #     stage = 3
                # elif stage == 3:
                #     FL.calc_angles(0.0 + step_X*Xoffset,0.0 + step_X*Yoffset- 0.02,0.16 + leg_fl_off,0.0,0.0,Yawoffset)
                #     FR.calc_angles(0.0 - step_X*Xoffset*1.5,0.0 - step_X*Yoffset*1.5,0.16 + leg_fr_off,0.0,0.0,-Yawoffset*1.5)
                #     BL.calc_angles(0.0 - step_X*Xoffset,0.0 - step_X*Yoffset- 0.02,0.16 + leg_bl_off,0.0,0.0,-Yawoffset)
                #     BR.calc_angles(0.0 + step_X*Xoffset*1.5,0.0 + step_X*Yoffset*1.5- 0.02,0.16  + leg_br_off,0.0,0.0,Yawoffset*1.5)
                #     stage = 4
                # elif stage == 4:
                #     FL.calc_angles(0.0 + step_X*Xoffset,0.0 + step_X*Yoffset- 0.02,0.16 + leg_fl_off,0.0,0.0,Yawoffset)
                #     FR.calc_angles(0.0 - step_X*Xoffset,0.0 - step_X*Yoffset,0.16 - step_Height + leg_fr_off,0.0,0.0,-Yawoffset)
                #     BL.calc_angles(0.0 - step_X*Xoffset,0.0 - step_X*Yoffset- 0.02,0.16 + leg_bl_off,0.0,0.0,-Yawoffset)
                #     BR.calc_angles(0.0 + step_X*Xoffset*1.5,0.0 + step_X*Yoffset*1.5- 0.02,0.16  + leg_br_off,0.0,0.0,+Yawoffset*1.5)
                #     stage = 5
                # elif stage == 5:
                #     FL.calc_angles(0.0 - step_X*Xoffset,0.0 - step_X*Yoffset- 0.02,0.16 + leg_fl_off,0.0,0.0,-Yawoffset)
                #     FR.calc_angles(0.0 + step_X*Xoffset,0.0 + step_X*Yoffset,0.16 - step_Height + leg_fr_off,0.0,0.0,Yawoffset)
                #     BL.calc_angles(0.0 - step_X*Xoffset*1.5,0.0 - step_X*Yoffset*1.5- 0.02,0.16 + leg_bl_off,0.0,0.0,-Yawoffset*1.5)
                #     BR.calc_angles(0.0 + step_X*Xoffset,0.0 + step_X*Yoffset- 0.02,0.16  + leg_br_off,0.0,0.0,Yawoffset)
                #     stage = 6
                # elif stage == 6:
                #     FL.calc_angles(0.0 - step_X*Xoffset,0.0 - step_X*Yoffset,0.16 + leg_fl_off,0.0,0.0,-Yawoffset)
                #     FR.calc_angles(0.0 + step_X*Xoffset*1.5,0.0 + step_X*Yoffset*1.5,0.16 + leg_fr_off,0.0,0.0,+Yawoffset*1.5)
                #     BL.calc_angles(0.0 - step_X*Xoffset*1.5,0.0 - step_X*Yoffset*1.5,0.16 + leg_bl_off,0.0,0.0,-Yawoffset*1.5)
                #     BR.calc_angles(0.0 + step_X*Xoffset,0.0 + step_X*Yoffset,0.16  + leg_br_off,0.0,0.0,+Yawoffset)
                #     stage = 7
                # elif stage == 7:
                #     FL.calc_angles(0.0 - step_X*Xoffset,0.0 - step_X*Yoffset + 0.02,0.16 + leg_fl_off,0.0,0.0,-Yawoffset)
                #     FR.calc_angles(0.0 + step_X*Xoffset*1.5,0.0 + step_X*Yoffset*1.5+ 0.02,0.16 + leg_fr_off,0.0,0.0,Yawoffset*1.5)
                #     BL.calc_angles(0.0 - step_X*Xoffset,0.0 - step_X*Yoffset,0.16 - step_Height + leg_bl_off,0.0,0.0,-Yawoffset)
                #     BR.calc_angles(0.0 + step_X*Xoffset,0.0 + step_X*Yoffset+ 0.02,0.16  + leg_br_off,0.0,0.0,Yawoffset)
                #     stage = 8
                # elif stage == 8:
                #     FL.calc_angles(0.0 - step_X*Xoffset*1.5,0.0 - step_X*Yoffset*1.5+ 0.02,0.16 + leg_fl_off,0.0,0.0,-Yawoffset*1.5)
                #     FR.calc_angles(0.0 + step_X*Xoffset,0.0 + step_X*Yoffset,0.16 + leg_fr_off,0.0,0.0,Yawoffset)
                #     BL.calc_angles(0.0 + step_X*Xoffset,0.0 + step_X*Yoffset+ 0.02,0.16 - step_Height + leg_bl_off,0.0,0.0,Yawoffset)
                #     BR.calc_angles(0.0 - step_X*Xoffset,0.0 - step_X*Yoffset+ 0.02,0.16  + leg_br_off,0.0,0.0,-Yawoffset)
                #     stage = 9
                # elif stage == 9:
                #     FL.calc_angles(0.0 - step_X*Xoffset*1.5,0.0 - step_X*Yoffset*1.5,0.16 + leg_fl_off,0.0,0.0,-Yawoffset*1.5)
                #     FR.calc_angles(0.0 + step_X*Xoffset,0.0 + step_X*Yoffset+ 0.02,0.16 + leg_fr_off,0.0,0.0,Yawoffset)
                #     BL.calc_angles(0.0 + step_X*Xoffset*1.5,0.0 + step_X*Yoffset*1.5+ 0.02,0.16 + leg_bl_off,0.0,0.0,Yawoffset*1.5)
                #     BR.calc_angles(0.0 - step_X*Xoffset,0.0 - step_X*Yoffset+ 0.02,0.16  + leg_br_off,0.0,0.0,-Yawoffset)
                #     stage = 10
                # elif stage == 10:
                #     FL.calc_angles(0.0 - step_X*Xoffset,0.0 - step_X*Yoffset,0.16 - step_Height + leg_fl_off,0.0,0.0,-Yawoffset)
                #     FR.calc_angles(0.0 + step_X*Xoffset,0.0 + step_X*Yoffset+ 0.02,0.16 + leg_fr_off,0.0,0.0,Yawoffset)
                #     BL.calc_angles(0.0 + step_X*Xoffset*1.5,0.0 + step_X*Yoffset*1.5+ 0.02,0.16 + leg_bl_off,0.0,0.0,Yawoffset*1.5)
                #     BR.calc_angles(0.0 - step_X*Xoffset,0.0 - step_X*Yoffset+ 0.02,0.16  + leg_br_off,0.0,0.0,-Yawoffset)
                #     stage = 11
                # elif stage == 11:
                #     FL.calc_angles(0.0 + step_X*Xoffset,0.0 + step_X*Yoffset,0.16 - step_Height + leg_fl_off,0.0,0.0,Yawoffset)
                #     FR.calc_angles(0.0 - step_X*Xoffset,0.0 - step_X*Yoffset+ 0.02,0.16 + leg_fr_off,0.0,0.0,-Yawoffset)
                #     BL.calc_angles(0.0 + step_X*Xoffset,0.0 + step_X*Yoffset+ 0.02,0.16 + leg_bl_off,0.0,0.0,Yawoffset)
                #     BR.calc_angles(0.0 - step_X*Xoffset*1.5,0.0 - step_X*Yoffset*1.5+ 0.02,0.16  + leg_br_off,0.0,0.0,-Yawoffset*1.5)
                #     stage = 12
                # elif stage == 12:
                #     FL.calc_angles(0.0 + step_X*Xoffset*1.5,0.0 + step_X*Yoffset*1.5,0.16 + leg_fl_off,0.0,0.0,Yawoffset*1.5)
                #     FR.calc_angles(0.0 - step_X*Xoffset,0.0 - step_X*Yoffset,0.16 + leg_fr_off,0.0,0.0,-Yawoffset)
                #     BL.calc_angles(0.0 + step_X*Xoffset,0.0 + step_X*Yoffset,0.16 + leg_bl_off,0.0,0.0,Yawoffset)
                #     BR.calc_angles(0.0 - step_X*Xoffset*1.5,0.0 - step_X*Yoffset*1.5,0.16  + leg_br_off,0.0,0.0,-Yawoffset*1.5)
                #     stage = 0
                
                last_check = rospy.get_time()
        else:
                FL.calc_angles(0.0,0.0,0.16 + leg_fl_off,0.0,0.0,0.0)
                FR.calc_angles(0.0,0.0,0.16 + leg_fr_off,0.0,0.0,0.0)
                BL.calc_angles(0.0,0.0,0.16 + leg_bl_off,0.0,0.0,0.0)
                BR.calc_angles(0.0,0.0,0.16 + leg_br_off,0.0,0.0,0.0)
                stage == 0

        if(rospy.get_time() - pre_time > 0.001):

            roll_error = 0.0 - roll
        
            kpout_r = roll_error * kpr
            kiout_r = kiout_r + (((roll_error+prev_roll_error)/2.0) * kir * 0.001)
            kdout_r = ((roll_error - prev_roll_error)/0.001) * kdr
            k_total_r = kpout_r + kiout_r + kdout_r
            prev_roll_error = roll_error
            

            pitch_error = 0.0 - pitch
            kpout_p = pitch_error * kpp
            kiout_p = kiout_p + (((pitch_error+prev_pitch_error)/2.0) * kip * 0.001)
            kdout_p = ((pitch_error - prev_pitch_error)/0.001) * kdp
            k_total_p = kpout_p + kiout_p + kdout_p
            prev_pitch_error = pitch_error
            

            leg_fl_off = -k_total_p + k_total_r
            leg_fr_off = -k_total_p - k_total_r
            leg_bl_off = k_total_p + k_total_r
            leg_br_off = k_total_p - k_total_r

            # rospy.loginfo(leg_fl_off)
            
            # if(roll < -0.0472665):
            #     leg_fl_off = leg_fl_off+0.0005
            #     leg_bl_off = leg_bl_off+0.0005
            #     leg_fr_off = leg_fr_off-0.0005
            #     leg_br_off = leg_br_off-0.0005
            # elif(roll > 0.0472665):
            #     leg_fl_off = leg_fl_off-0.0005
            #     leg_bl_off = leg_bl_off-0.0005
            #     leg_fr_off = leg_fr_off+0.0005
            #     leg_br_off = leg_br_off+0.0005
            # elif(pitch > 0.0472665):
            #     leg_fl_off = leg_fl_off+0.0005
            #     leg_fr_off = leg_fr_off+0.0005
            #     leg_bl_off = leg_bl_off-0.0005
            #     leg_br_off = leg_br_off-0.0005
            # elif(pitch < -0.0472665):
            #     leg_fl_off = leg_fl_off-0.0005
            #     leg_fr_off = leg_fr_off-0.0005
            #     leg_bl_off = leg_bl_off+0.0005
            #     leg_br_off = leg_br_off+0.0005

            pre_time = rospy.get_time()
        
        msg = Float64MultiArray()
        msg.data = [FL.shoulder_angle,FL.hip_angle,FL.knee_angle,FR.shoulder_angle, FR.hip_angle,FR.knee_angle,BL.shoulder_angle,BL.hip_angle,BL.knee_angle,BR.shoulder_angle,BR.hip_angle,BR.knee_angle]
        pub.publish(msg)
        # odom = Odometry()
        # odom.header.stamp = rospy.Time.now()
        # odom.header.frame_id = "odom"
        # odom.child_frame_id = "base_link"
        # odom.pose.pose = Pose(Point(x_dist, y_dist, 0.), Quaternion(*quaternion_from_euler(0, 0, yaw)))
        # odompub.publish(odom)
        # odom_quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
        # odom_broadcaster.sendTransform(
        # (x_dist, y_dist, 0.),
        # odom_quat,
        # rospy.Time.now(),
        # "odom",
        # "odom_raw"
        # )

        rate.sleep()
