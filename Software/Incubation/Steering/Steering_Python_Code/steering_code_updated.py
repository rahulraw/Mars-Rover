# Determines the wheel angle and drive speed for motors

#This algorithm uses a phantom wheel located at the centre to direct the rover.
#Cannot be greater than 45 deg.

#Limit RC_input to 40 degrees in arduino code.
#Limit velocity to 127 here when turning.

import math

def rad2deg(radians):
	degrees = radians*(180/math.pi);
	return degrees
		
##RC_Input is in degrees, velocity is speed unit and rotate_about is a bool##
class SteeringCalc:
    def __init__(self, RC_input, velocity, rotate_about, strafe):	
        self.front_left_angle = 0
        self.front_right_angle = 0
        self.back_left_angle = 0
        self.back_right_angle = 0
        self.chassis_length = 0.512 
        self.chassis_width = 0.512
        self.update_values(RC_input, velocity, rotate_about, strafe)
        	
    def update_values(self, RC_input, velocity, rotate_about, strafe):
        
        if (rotate_about):
            self.RC_input = RC_input
            self.velocity = velocity
            self.turn_onself()

        elif (strafe):
            self.RC_input = RC_input
            self.velocity = velocity
            self.strafe()

        else:    
            print (">>>RC_INPUT ", RC_input)
            self.RC_input = RC_input*math.pi/180
            if RC_input == 0:
                self.origin_to_centre = 999999
            else:
                self.origin_to_centre = 0.256/(math.tan(self.RC_input))		
     
            self.front_left_angle = rad2deg(math.atan(0.256/(self.origin_to_centre+0.256)))
            self.front_right_angle = rad2deg(math.atan(0.256/(self.origin_to_centre-0.256)))
            self.back_left_angle = -self.front_left_angle;
            self.back_right_angle = -self.front_right_angle

            self.rotate_about = rotate_about
            self.strafe = strafe
            
            #Velocity variables
            print (">>>velocity_Input ", velocity)		
            self.velocity = velocity
            self.radius_left = math.sqrt((self.origin_to_centre+0.256)**2+0.256**2)
            self.radius_right = math.sqrt((self.origin_to_centre-0.256)**2+0.256**2)		
            self.velocity_left = velocity*abs((self.radius_left/self.origin_to_centre)) #Leftside (FL, BL)
            self.velocity_right = velocity*abs((self.radius_right/self.origin_to_centre)) #Rightside (FR, BR)

            #Velocity assignment: Set the velocity to the side with greater radius, then calculate other side.
          
           # if self.radius_left > self.radius_right:
           #     self.velocity_left = self.velocity
           #     self.velocity_right = (self.radius_right/self.radius_left)*self.velocity_left
            
           # elif self.radius_right > self.radius_left:
           #     self.velocity_right = self.velocity
           #     self.velocity_left = (self.radius_left/self.radius_right)*self.velocity_right

           # else:
           #     self.velocity_right, self.velocity_left = self.velocity, self.velocity
           
           
        print "Original velocity value: %s" % self.velocity		
        print "Left wheel velocity value: %s" % self.velocity_left
        print "Left wheel velocity value: %s" % self.velocity_right

        print ""       

        print "Front right wheel angle: %s" % (self.front_right_angle)	
        print "Back right wheel angle: %s" % (self.back_right_angle)
        print "Front left wheel angle: %s" % (self.front_left_angle)
        print "Back left wheel angle: %s" % (self.back_left_angle)

    def turn_onself(self):
    	self.front_left_angle = 45
        self.front_right_angle = -45
        self.back_left_angle = -self.front_left_angle
        self.back_right_angle = -self.front_right_angle
        self.velocity_left = self.velocity
        self.velocity_right = -self.velocity

    def strafe(self):
        self.velocity_left, self.velocity_right = self.velocity, self.velocity
        self.front_left_angle = self.RC_input
        self.front_right_angle = self.front_left_angle
        self.back_right_angle, self.back_left_angle = self.front_left_angle, self.front_left_angle

