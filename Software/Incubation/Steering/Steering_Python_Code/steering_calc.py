# Determines the wheel angle and drive speed for motors

import math

def rad2deg(radians):
	return radians * (180/math.pi)

def deg2rad(degrees):
	return degrees * (math.pi/180)
		
##RC_Input is in degrees, velocity is speed unit and rotate_about is a bool##
class SteeringCalc:
    def __init__(self, RC_input, velocity, rotate_about, strafe):	
		#Direction variables		
        self.front_left_angle = 0
        self.front_right_angle = 0
        self.back_left_angle = 0
        self.back_right_angle = 0
        self.velocity_left = 0
        self.velocity_right = 0
        self.chassis_length = 0.512 
        self.chassis_width = 0.512 

    def turn_onself(self, velocity):
        self.front_left_angle = 45
        self.front_right_angle = -45
        self.back_left_angle = -self.front_left_angle
        self.back_right_angle = -self.front_right_angle
        self.velocity_left = velocity
        self.velocity_right = -velocity

    def update_values(self, angle_deg, velocity):
        angle_rad = deg2rad(angle_deg)

        self.origin_to_centre = 9999999 if angle_rad == 0 else 0.256 / math.tan(angle_rad)

        if self.RC_input > 0:
            self.front_right_angle = RC_input
            self.front_left_angle = rad2deg(math.atan(0.256/(self.origin_to_centre + 2*0.256)))
        else:
            self.front_left_angle = RC_input
            self.front_right_angle = rad2deg(math.atan(0.256/(self.origin_to_centre - 2*0.256)))

        self.back_right_angle = -self.front_right_angle
        self.back_left_angle = -self.front_left_angle

        self.radius_left = math.sqrt((self.origin_to_centre + 0.256)**2 + 0.256**2)
        self.radius_right = math.sqrt((self.origin_to_centre - 0.256)**2 + 0.256**2)		
        
        self.velocity_left = velocity * abs((self.radius_left / self.origin_to_centre)) #Leftside (FL, BL)
        self.velocity_right = velocity * abs((self.radius_right / self.origin_to_centre)) #Rightside (FR, BR)
