#!/usr/bin/env python
import math

class leg:


    def __init__(self, robot_length, robot_width, upleglen, lowleglen, side, front):
        self.upleglen = upleglen
        self.lowleglen = lowleglen
        self.robot_length = robot_length
        self.robot_width = robot_width
        self.side = side
        self.front = front
        
    def calc_angles(self, X, Y, Z, roll, pitch, yaw):


        if(self.side == 0 ):
         Yorigin =  Y + self.robot_width/2 
        else:
         Yorigin = Y - self.robot_width/2
        if(self.front == 0 ):
         Xorigin = X + self.robot_length/2 
        else:
         Xorigin = X - self.robot_length/2

        original_yaw = math.atan2(Yorigin,Xorigin)

        radius = math.sqrt(math.pow(Xorigin,2) + math.pow(Yorigin,2))

        new_yaw = original_yaw + yaw
        
        Y_yaw = radius * math.sin(new_yaw)
        X_yaw = radius * math.cos(new_yaw)
       

        if(self.side == 0 ):
         Y_yaw =  Y_yaw - self.robot_width/2 
        else:
         Y_yaw =  Y_yaw + self.robot_width/2 
        if(self.front == 0 ):
         X_yaw = X_yaw - self.robot_length/2
        else:
         X_yaw = X_yaw + self.robot_length/2


        offset_height_pitch = math.sin(pitch) * (self.robot_length/2.0)
        offset_width_pitch = math.cos(pitch) * (self.robot_length/2.0)

        if(self.front == 0):
            distance_to_gnd_pitch = Z + offset_height_pitch
            deltax = ((self.robot_length/2.0) + X_yaw)  - offset_width_pitch
        else:
            distance_to_gnd_pitch = Z - offset_height_pitch
            deltax = offset_width_pitch - ((self.robot_length/2.0) - X_yaw) 
       

        anglexoffset = math.atan(deltax/distance_to_gnd_pitch)
        distance_to_gnd_X = math.sqrt(math.pow(deltax,2) + math.pow(distance_to_gnd_pitch,2))

        
        pitchx = -math.sin(pitch - anglexoffset) * (distance_to_gnd_X)
        pitchz = math.cos(pitch - anglexoffset) * (distance_to_gnd_X)

        offset_height_roll = math.sin(roll) * (self.robot_width/2.0)
        offset_width_roll = math.cos(roll) * (self.robot_width/2.0)


        if(self.side == 0):
            distance_to_gnd_roll = pitchz + offset_height_roll
            deltay = ((self.robot_width/2.0) + Y_yaw)  - offset_width_roll
        else:
            distance_to_gnd_roll = pitchz - offset_height_roll
            deltay = offset_width_roll - ((self.robot_width/2.0) - Y_yaw)

        angleyoffset = math.atan(deltay/distance_to_gnd_roll)
        distance_to_gnd_Y = math.sqrt(math.pow(deltay,2) + math.pow(distance_to_gnd_roll,2))

        rolly = -math.sin(roll - angleyoffset) * (distance_to_gnd_Y)
        rollz = math.cos(roll - angleyoffset) * (distance_to_gnd_Y)


        angley = math.atan(rolly/rollz)
        ZY = math.sqrt(math.pow(rolly,2) + math.pow(rollz,2))
        anglex = math.atan(pitchx/ZY)
        ZX = math.sqrt(math.pow(pitchx,2) + math.pow(ZY,2))
        
        if(self.side == 0):
            self.shoulder_angle = 1.57 + angley
        else :
            self.shoulder_angle = 1.57 - angley
        
        self.hip_angle = 1.57 -  math.acos((math.pow(ZX,2)+math.pow(self.upleglen,2)-math.pow(self.lowleglen,2))/(2*ZX*self.upleglen)) + anglex 
        self.knee_angle = math.acos((math.pow(self.lowleglen,2)+math.pow(self.upleglen,2)-math.pow(ZX,2))/(2*self.lowleglen*self.upleglen))
        