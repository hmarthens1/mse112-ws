#!/usr/bin/env python3
# encoding: utf-8
# Inverse kinematics of a 4-DOF robot: Given the corresponding coordinates (X, Y, Z) and pitch angle, calculate the rotation angle of each joint
# 2020/07/20 Aiden
import logging
from math import *

# CRITICAL, ERROR, WARNING, INFO, DEBUG
logging.basicConfig(level=logging.ERROR)
logger = logging.getLogger(__name__)

class IK:
    # Count the servos from bottom to top
    # Count the servos from bottom to top
    l1 = 8.00    #The distance from the center of the robot chassis to the center axis of the second servo is 6.10cm
    l2 = 6.50   #The distance from the second servo to the third servo is 10.16cm
    l3 = 6.20    #The distance from the third servo to the fourth servo is 9.64cm
    l4 = 0.00    #No specific assignment is done here, and the value is reassigned according to the selection during initialization

    # Air pump model specific parameters
    l5 = 4.70  #The distance from the fourth servo to the top of the nozzle is 4.70cm
    l6 = 4.46  #The distance from the top of the nozzle to the nozzle is 4.46cm
    alpha = degrees(atan(l6 / l5))  #Calculate the angle between l5 and l4

    def __init__(self, arm_type): #According to different types of clamps, adapt parameters
        self.arm_type = arm_type
        if self.arm_type == 'pump': #If it is an air pump type robotic arm
            self.l4 = sqrt(pow(self.l5, 2) + pow(self.l6, 2))  #The fourth servo to the nozzle acts as the fourth link
        elif self.arm_type == 'arm':
            self.l4 = 10.0  #The distance from the fourth servo to the end of the robotic arm is 16.6cm. The end of the robotic arm refers to when the claws are fully closed.

    def setLinkLength(self, L1=l1, L2=l2, L3=l3, L4=l4, L5=l5, L6=l6):
        # Change the connecting rod length of the robot arm to adapt to robots of the same structure with different lengths
        self.l1 = L1
        self.l2 = L2
        self.l3 = L3
        self.l4 = L4
        self.l5 = L5
        self.l6 = L6
        if self.arm_type == 'pump':
            self.l4 = sqrt(pow(self.l5, 2) + pow(self.l6, 2))
            self.alpha = degrees(atan(self.l6 / self.l5))

    def getLinkLength(self):
        # Get the current setting of the connecting rod length
        if self.arm_type == 'pump':
            return {"L1":self.l1, "L2":self.l2, "L3":self.l3, "L4":self.l4, "L5":self.l5, "L6":self.l6}
        else:
            return {"L1":self.l1, "L2":self.l2, "L3":self.l3, "L4":self.l4}

    def getRotationAngle(self, coordinate_data, Alpha):
        # Given the specified coordinates and pitch angle, return the angle that each joint should rotate. If there is no solution, return False
         # coordinate_data is the coordinates of the end of the gripper. The coordinate unit is cm. It is passed in as a tuple, for example (0, 5, 10)
         # Alpha is the angle between the holder and the horizontal plane, in degrees

         # Let the end of the gripper be P(X, Y, Z), the coordinate origin be O, the origin is the projection of the gimbal center on the ground, and the projection of point P on the ground is P_
         # The intersection point of l1 and l2 is A, the intersection point of l2 and l3 is B, the intersection point of l3 and l4 is C
         # CD is perpendicular to PD, CD is perpendicular to the z-axis, then the pitch angle Alpha is the angle between DC and PC, AE is perpendicular to DP_, and E is on DP_, CF is perpendicular to AE, and F is on AE
         # Angle representation: For example, the angle between AB and BC is expressed as ABC

        X, Y, Z = coordinate_data
        if self.arm_type == 'pump':
            Alpha -= self.alpha
        #Find the rotation angle of the base
        theta6 = degrees(atan2(Y, X))
 
        P_O = sqrt(X*X + Y*Y) #P_distance to origin O
        CD = self.l4 * cos(radians(Alpha))
        PD = self.l4 * sin(radians(Alpha)) #When the pitch angle is positive, PD is positive, when the pitch angle is negative, PD is negative
        AF = P_O - CD
        CF = Z - self.l1 - PD
        AC = sqrt(pow(AF, 2) + pow(CF, 2))
        if round(CF, 4) < -self.l1:
            logger.debug('Height below0, CF(%s)<l1(%s)', CF, -self.l1)
            return False
        if self.l2 + self.l3 < round(AC, 4): #The sum of two sides is less than the third side
            logger.debug('Cannot form a connecting rod structure, l2(%s) + l3(%s) < AC(%s)', self.l2, self.l3, AC)
            return False

        #Looking for theta4
        cos_ABC = round((pow(self.l2, 2) + pow(self.l3, 2) - pow(AC, 2))/(2*self.l2*self.l3), 4) #Law of Cosines
        if abs(cos_ABC) > 1:
            logger.debug('Cannot form a connecting rod structure, abs(cos_ABC(%s)) > 1', cos_ABC)
            return False
        ABC = acos(cos_ABC) #Inverse trigonometry to find radians
        theta4 = 180.0 - degrees(ABC)

        #Find theta5
        CAF = acos(AF / AC)
        cos_BAC = round((pow(AC, 2) + pow(self.l2, 2) - pow(self.l3, 2))/(2*self.l2*AC), 4) #Law of Cosines
        if abs(cos_BAC) > 1:
            logger.debug('Cannot form a connecting rod structure, abs(cos_BAC(%s)) > 1', cos_BAC)
            return False
        if CF < 0:
            zf_flag = -1
        else:
            zf_flag = 1
        theta5 = degrees(CAF * zf_flag + acos(cos_BAC))

        #Find theta3
        theta3 = Alpha - theta5 + theta4
        if self.arm_type == 'pump':
            theta3 += self.alpha

        return {"theta3":theta3, "theta4":theta4, "theta5":theta5, "theta6":theta6} # Returns the angle dictionary if there is a solution
            
if __name__ == '__main__':
    ik = IK('arm')
    #ik.setLinkLength(L1=ik.l1 + 1.30, L4=ik.l4)
    print('连杆长度：', ik.getLinkLength())
    #print(ik.getRotationAngle((0, ik.l4, ik.l1 + ik.l2 + ik.l3), 0))
