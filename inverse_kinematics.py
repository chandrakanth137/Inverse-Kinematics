'''
The following program calculates the inverse kinematics of a 2 arm manipulator robot
It also verifies the joint angles by calculating the forward kinematics based on the calculated joint angles from inverse kinematics

Given the flexibilty, a single point can be achieved using 2 different positions for the link arms, so the inverse kinematics for both 
the configuration is calculated. [DOF_2_config1 & DOF_2_config2]
'''

from math import acos, sin, cos, degrees, atan2

'''
The following class represents one type of arm configuration to calculate inverse kinematics.
X - X coordinate of the end effector
Y - Y coordinate of the end effector
L1 - Length of the first link / arm
L2 - Length of the second link / arm
theta_1 - Joint angle of the first link / arm
theta_2 - Joint angle of the second link / arm 

The function "second_arm_joint_angle" calculates theta_2
The function "first_arm_joint_angle" calculates theta_1
The function "forward kinematics" calculates the forward kinematics based on the calculated joint angles from inverse kinematics
The function "calc_joint_angles" is the driver function for the class which calls the other functions and prints the value 
'''

class DOF_2_config1:
    def __init__(self, L1, L2, X, Y):
        self.X = X 
        self.Y = Y
        self.L1 = L1
        self.L2 = L2
        self.theta_1 = None
        self.theta_2 = None
    def second_arm_joint_angle(self):
        numerator = (self.X ** 2) + (self.Y ** 2) - (self.L1 ** 2) - (self.L2 ** 2)
        denominator = 2 * self.L1 * self.L2

        self.theta_2 = acos(numerator / denominator)

    def first_arm_joint_angle(self):
        temp1 = self.L1 + (self.L2 * cos(self.theta_2))
        temp2 = self.L2 * sin(self.theta_2)
        
        numerator = (self.Y * temp1) - (self.X * temp2)
        denominator =  (self.X * temp1) + (self.Y * temp2)

        self.theta_1 = atan2(numerator, denominator)
    
    def forward_kinematics(self):
        X = (self.L1 * cos(self.theta_1)) + (self.L2 * cos(self.theta_1 + self.theta_2))
        Y = (self.L1 * sin(self.theta_1)) + (self.L2 * sin(self.theta_1 + self.theta_2))
        return (X,Y)
    
    def calc_joint_angles(self):
        self.second_arm_joint_angle()
        self.first_arm_joint_angle()
        print("============================= Configuration 1 =============================================\n")
        print("-------------------------- Calculating Inverse Kinematics --------------------------------")
        print("The joint angle of the first arm: ", degrees(self.theta_1))
        print("The joint angle of the second arm: ", degrees(self.theta_2))
        print("--------------------------------------------------------------------------------------------\n")
        
        X,Y = self.forward_kinematics()
        print("-------------------------- Calculating Forward Kinematics ---------------------------------")
        print("Coordinate X: ", X)
        print("Coordinate Y: ", Y)
        print("---------------------------------------------------------------------------------------------")
        print("===========================================================================================\n")

class DOF_2_config2:
    def __init__(self, L1, L2, X, Y):
        self.X = X
        self.Y = Y
        self.L1 = L1
        self.L2 = L2
        self.theta_1 = None
        self.theta_2 = None

    def second_arm_joint_angle(self):
        numerator = (self.X ** 2) + (self.Y ** 2) - (self.L1 ** 2) - (self.L2 ** 2)
        denominator = 2 * self.L1 * self.L2

        self.theta_2 = -acos(numerator / denominator)

    def first_arm_joint_angle(self):
        temp1 = self.L1 + (self.L2 * cos(self.theta_2))
        temp2 = self.L2 * sin(self.theta_2)

        numerator = (self.Y * temp1) - (self.X * temp2)
        denominator =  (self.X * temp1) + (self.Y * temp2)

        self.theta_1 = atan2(numerator, denominator)
        
    def forward_kinematics(self):
        X = (self.L1 * cos(self.theta_1)) + (self.L2 * cos(self.theta_1 + self.theta_2))
        Y = (self.L1 * sin(self.theta_1)) + (self.L2 * sin(self.theta_1 + self.theta_2))
        return (X,Y)

    def calc_joint_angles(self):
        self.second_arm_joint_angle()
        self.first_arm_joint_angle()
        
        print("============================= Configuration 2 =============================================\n")
        print("-------------------------- Calculating Inverse Kinematics --------------------------------")
        print("The joint angle of the first arm: ", degrees(self.theta_1))
        print("The joint angle of the second arm: ", degrees(self.theta_2))
        print("--------------------------------------------------------------------------------------------\n")
        
        X,Y = self.forward_kinematics()
        print("-------------------------- Calculating Forward Kinematics ---------------------------------")
        print("Coordinate X: ", X)
        print("Coordinate Y: ", Y)
        print("---------------------------------------------------------------------------------------------")
        print("===========================================================================================\n")

if __name__ == '__main__':
    X = float(input("Enter the X Coordinate: "))
    Y = float(input("Enter the Y Coordinate: "))
    L1 = float(input("Enter the length of first arm: "))
    L2 = float(input("Enter the length of second arm: "))

    config1 = DOF_2_config1(L1,L2,X,Y)
    config2 = DOF_2_config2(L1,L2,X,Y)
    config1.calc_joint_angles()
    config2.calc_joint_angles()