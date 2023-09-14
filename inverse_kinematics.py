from math import atan, acos, sin, cos, pi, degrees, atan2

'''
Parameters
X
Y
L1
L2
theta_1
theta_2

X = L1 * Cos(theta_1) + L2 * Cos(theta_1 + theta_2) 
Y = L1 * Sin(theta_1) + L2 * Sin(theta_1 + theta_2)
'''

class DOF_2_config1:
	def __init__(self,L1,L2,X,Y):
		self.X = X
		self.Y = Y
		self.L1 = L1
		self.L2 = L2

	def second_arm_joint_angle(self):
		numerator = (self.X ** 2) + (self.Y ** 2) - (self.L1 ** 2) - (self.L2 ** 2)
		denominator = 2 * self.L1 * self.L2

		result = acos(numerator / denominator)
		return result

	def first_arm_joint_angle(self,theta_2):
		temp1 = self.L1 + (self.L2 * cos(theta_2))
		temp2 = self.L2 * sin(theta_2)
		numerator = (self.Y * temp1) - (self.X * temp2)
		denominator =  (self.X * temp1) + (self.Y * temp2)

		result = atan2(numerator , denominator)
		return result

	def calc_joint_angles(self):
		theta_2 = self.second_arm_joint_angle()
		theta_1 = self.first_arm_joint_angle(theta_2)

		print("--------------Calculating Inverse Kinematics for Type I Configuration-----------------------")
		print("The joint angle of the first arm: ", degrees(theta_1))
		print("The joint angle of the second arm: ", degrees(theta_2))
		print("--------------------------------------------------------------------------------------------\n")

class DOF_2_config2:
	def __init__(self,L1,L2,X,Y):
		self.X = X
		self.Y = Y
		self.L1 = L1
		self.L2 = L2

	def second_arm_joint_angle(self):
		numerator = (self.X ** 2) + (self.Y ** 2) - (self.L1 ** 2) - (self.L2 ** 2)
		denominator = 2 * self.L1 * self.L2

		result = -acos(numerator / denominator)
		return result

	def first_arm_joint_angle(self, theta_2):
		temp1 = self.L1 + (self.L2 * cos(theta_2))
		temp2 = self.L2 * sin(theta_2)

		numerator = (self.Y * temp1) - (self.X * temp2)
		denominator =  (self.X * temp1) + (self.Y * temp2)

		result = atan2(numerator, denominator)
		return result

	def calc_joint_angles(self):
		theta_2 = self.second_arm_joint_angle()
		theta_1 = self.first_arm_joint_angle(theta_2)

		print("--------------Calculating Inverse Kinematics for Type II Configuration-----------------------")
		print("The joint angle of the first arm: ", degrees(theta_1))
		print("The joint angle of the second arm: ", degrees(theta_2))
		print("---------------------------------------------------------------------------------------------\n")


# def second_arm_angle(X, Y, L1, L2):
# 	numerator = (X**2) + (Y**2) - (L1**2) - (L2 ** 2)
# 	denominator = 2 * L1 * L2
# 	result_rad = math.acos(numerator / denominator)
# 	print("Angle 2 in Radians: ", pi - result_rad)
# 	# result_deg = math.degrees(result_rad) 
	
# 	return result_rad


# def first_arm_angle(X, Y, L1, L2, theta_2):
# 	temp1 = L1 + L2 * math.cos(theta_2)
# 	temp2 = L2 * math.sin(theta_2)

# 	numerator = (Y * temp1) - (X * temp2)
# 	denominator =  (X * temp1) + (Y * temp2)

# 	# first_part = math.atan(Y / X)
# 	#second_part_num = L2 * math.sin(theta_2)
# 	# second_part_denom = L1 + (L2 * math.cos(theta_2))
# 	# second_part = math.atan(second_part_num / second_part_denom)
# 	# result_rad = first_part - second_part


# 	result_rad = math.atan(numerator / denominator)
# 	print("Angle 1 in Radians: ", pi - result_rad)
# 	#result_deg = math.degrees(result_rad)

# 	return result_rad


if __name__ == '__main__':
	X = int(input("Enter the X Coordinate: "))
	Y = int(input("Enter the Y Coordinate: "))
	L1 = int(input("Enter the length of first arm: "))
	L2 = int(input("Enter the length of second arm: "))

	config1 = DOF_2_config1(L1,L2,X,Y)
	config2 = DOF_2_config2(L1,L2,X,Y)
	config1.calc_joint_angles()
	config2.calc_joint_angles()



