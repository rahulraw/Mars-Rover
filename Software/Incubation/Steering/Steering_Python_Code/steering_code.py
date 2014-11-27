# Determines the wheel angle and drive speed for motors

import math

def rad2deg(radians):
	degrees = radians*(180/math.pi);
	return degrees;
		
##RC_Input is in degrees, velocity is speed unit and rotate_about is a bool##
class SteeringCalc:
    def __init__(self, RC_input, velocity, rotate_about, strafe):	
		#Direction variables		
        self.front_left_angle = 0
        self.front_right_angle = 0
        self.back_left_angle = 0
        self.back_right_angle = 0
        self.chassis_length = 0.512; 
        self.chassis_width = 0.512; 
        self.update_values(RC_input, velocity, rotate_about, strafe)
        		 


    def turn_onself(self):
        self.front_left_angle = 45;
        self.front_right_angle = -45;
        self.back_left_angle = -self.front_left_angle;
        self.back_right_angle = -self.front_right_angle;
        self.velocity_left = self.velocity;
        self.velocity_right = -self.velocity;

    def update_values(self, RC_input, velocity, rotate_about, strafe):
        print (">>>> RX_INPUT ", RC_input) 
        self.RC_input = RC_input*math.pi/180;	#converted to radians
        
        if self.RC_input == 0:
            self.origin_to_centre = 9999999
        else:
            self.origin_to_centre = 0.256/math.tan(self.RC_input)

        if self.RC_input > 0:
            self.front_right_angle = RC_input
            self.front_left_angle = rad2deg(math.atan(0.256/(self.origin_to_centre + 2*0.256)));
        else:
            self.front_left_angle = RC_input
            self.front_right_angle = rad2deg(math.atan(0.256/(self.origin_to_centre - 2*0.256)));

        self.back_right_angle = -self.front_right_angle;
        self.back_left_angle = -self.front_left_angle;
        print ("front left angle: ", self.front_left_angle)

        self.rotate_about = rotate_about;
        self.strafe = strafe;
		
	#Velocity variables		
        self.velocity = velocity;
        self.radius_left = math.sqrt((self.origin_to_centre+0.256)**2+0.256**2);
        self.radius_right = math.sqrt((self.origin_to_centre-0.256)**2+0.256**2);		
        
        self.velocity_left = velocity*abs((self.radius_left/self.origin_to_centre)); #Leftside (FL, BL)
        self.velocity_right = velocity*abs((self.radius_right/self.origin_to_centre)); #Rightside (FR, BR)

        if (self.rotate_about):
            self.turn_onself()
