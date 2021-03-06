# Determines the wheel angle and drive speed for motors

import math

def rad2deg(radians):
	return radians * (180/math.pi)

def deg2rad(degrees):
	return degrees * (math.pi/180)
		
##RC_Input is in degrees, velocity is speed unit and rotate_about is a bool##
class SteeringCalc:
    def __init__(self, RC_input, velocity):	
		#Direction variables		
        self.front_left_angle = 0
        self.front_right_angle = 0
        self.back_left_angle = 0
        self.back_right_angle = 0
        self.velocity_left = 0
        self.velocity_right = 0
        self.chassis_length = 0.512 
        self.chassis_width = 0.512 
        self.safety_constant = 2

    def turn_onself(self, controller):
        self.front_left_angle = 45
        self.front_right_angle = -45
        self.back_left_angle = -self.front_left_angle
        self.back_right_angle = -self.front_right_angle
        self.velocity_left = controller.left_joy_y
        self.velocity_right = -controller.left_joy_y

    def strafe(self, controller):
        angle = int(rad2deg(math.atan(controller.right_joy_x/(controller.right_joy_y + 0.01))))
        self.front_left_angle = angle
        self.back_left_angle = angle
        self.front_right_angle = self.front_left_angle
        self.back_right_angle = self.back_left_angle
        self.velocity_left = controller.left_joy_y
        self.velocity_right = controller.left_joy_y

    def bicycle(self, controller):
        angle_deg = controller.right_joy_x / 2
        angle_rad = deg2rad(angle_deg)

        self.origin_to_centre = 9999999 if angle_rad == 0 else 0.256 / math.tan(angle_rad)

        if angle_rad > 0:
            self.front_right_angle = angle_deg
            self.front_left_angle = rad2deg(math.atan(0.256/(self.origin_to_centre + 2*0.256)))
        else:
            self.front_left_angle = angle_deg
            self.front_right_angle = rad2deg(math.atan(0.256/(self.origin_to_centre - 2*0.256)))

        self.back_right_angle = -self.front_right_angle
        self.back_left_angle = -self.front_left_angle

        self.radius_left = math.sqrt((self.origin_to_centre + 0.256)**2 + 0.256**2)
        self.radius_right = math.sqrt((self.origin_to_centre - 0.256)**2 + 0.256**2)		
        
        self.velocity_left = controller.left_joy_y * self.safety_constant * abs((self.radius_left / self.origin_to_centre))
        self.velocity_right = controller.left_joy_y * self.safety_constant * abs((self.radius_right / self.origin_to_centre))
