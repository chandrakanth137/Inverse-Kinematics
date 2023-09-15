from math import acos, sin, cos, pi, degrees, atan2

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
        print(self.theta_1 / self.theta_2)
        print("--------------Calculating Inverse Kinematics for Type I Configuration-----------------------")
        print("The joint angle of the first arm: ", degrees(self.theta_1))
        print("The joint angle of the second arm: ", degrees(self.theta_2))
        print("--------------------------------------------------------------------------------------------")
        X,Y = self.forward_kinematics()
        print("--------------Calculating Forward Kinematics for Type I Configuration-----------------------")
        print("Coordinate X: ", X)
        print("Coordinate Y: ", Y)
        print("---------------------------------------------------------------------------------------------")

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
        
        print(self.theta_1 / self.theta_2)
        print("--------------Calculating Inverse Kinematics for Type II Configuration-----------------------")
        print("The joint angle of the first arm: ", degrees(self.theta_1))
        print("The joint angle of the second arm: ", degrees(self.theta_2))
        print("---------------------------------------------------------------------------------------------")
        X,Y = self.forward_kinematics()
        print("--------------Calculating Forward Kinematics for Type II Configuration-----------------------")
        print("Coordinate X: ", X)
        print("Coordinate Y: ", Y)
        print("---------------------------------------------------------------------------------------------")
        

if __name__ == '__main__':
    X = int(input("Enter the X Coordinate: "))
    Y = int(input("Enter the Y Coordinate: "))
    L1 = int(input("Enter the length of first arm: "))
    L2 = int(input("Enter the length of second arm: "))

    config1 = DOF_2_config1(L1,L2,X,Y)
    config2 = DOF_2_config2(L1,L2,X,Y)
    config1.calc_joint_angles()
    config2.calc_joint_angles()