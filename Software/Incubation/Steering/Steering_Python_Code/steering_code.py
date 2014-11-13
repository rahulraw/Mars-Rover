import math

def cosine_formula(a, b, angle_C):
	c_squared = (a*a) + (b*b) - (2*a*b)*math.cos(angle_C);
	return math.sqrt(c_squared);

def sine_formula(B, angle_A, A):
	return math.asin(B*math.sin(angle_A)/A);
			
def convert_radians_to_degrees(radians):
	degrees = radians*(180/math.pi);
	return degrees;

#Given a quadratic ax**2 +bx +c = 0, solves for x.
def find_root (a,b,c):
	x = (b + math.sqrt(b**2-4*a*c))/(2*a)
	return x;

#Radius ratio for velocity of each wheel
def radius_ratio_formula(velocity, r1, r2):
	return velocity*r1/r2;
		
##RC_Input is in degrees, velocity is speed unit and rotate_about is a bool##
class SteeringCalc:
    def __init__(self, RC_input, velocity, rotate_about):	
		#Direction variables		
        self.RC_input = RC_input*math.pi/180;	
        self.chassis_length = 0.727/2*math.cos(math.pi/4); #CV?
        self.chassis_width = 0.727/2*math.sin(math.pi/4); #CV?
        self.origin_to_centre = None;
        self.centre_to_wheel = math.sqrt((self.chassis_length**2) + (self.chassis_width**2)); #CV?
        self.alpha = math.acos(self.chassis_width/self.centre_to_wheel);						
        self.origin_to_wheel = None;
        self.steering_angle = None;
        self.front_left_angle = None;
        self.front_right_angle = None;
        self.back_left_angle = None;
        self.back_right_angle = None;
        self.K1 = None;
        self.K2 = None;
        self.K3 = None;
        self.rotate_about = rotate_about;
		
		#Velocity variables		
        self.velocity = velocity;		
        self.velocity_left = None; #Leftside (FL, BL)
        self.velocity_right = None; #Rightside (FR, BR)
        self.radius_left = None;
        self.radius_right = None;
	
	#Calculate the constants K1, K2, K3. See derivation sheet.
    def __calc_Ks__(self):
        self.K1 = self.centre_to_wheel*math.sin(self.alpha);
        self.K2 = self.centre_to_wheel*self.centre_to_wheel;
        self.K3 = abs(2*self.centre_to_wheel*math.cos(self.alpha));

	#Calculate value of origin to centre.
    def __calc_origin_to_centre__(self):
	    if self.RC_input > 0:
		    self.origin_to_centre = find_root(1, self.K3, self.K2-(self.K1/math.sin(self.RC_input))**2);
	    elif self.RC_input < 0:#If the RC input is < 0, set the origin to centre < 0.
		    self.origin_to_centre = -find_root(1, self.K3, self.K2-(self.K1/math.sin(self.RC_input))**2);
	    else:
		    print "Error, OC is not in range!";
		    exit();
			 

    def __calc_origin_to_wheel__(self):
	    return cosine_formula(abs(self.origin_to_centre), self.centre_to_wheel, self.alpha);

    def __calc_steering_angle__(self):
	    return sine_formula(self.centre_to_wheel, self.alpha, self.origin_to_wheel);		


	#Calculate the angles for the right side of the rover.
    def __right__(self):
        self.__calc_Ks__();
        self.__calc_origin_to_centre__();
        if (self.rotate_about):
            self.alpha = math.acos(self.chassis_width/self.centre_to_wheel);
            self.front_right_angle = -math.asin(1/math.sqrt(2));
            self.back_right_angle = -self.front_right_angle;
		
        else:
            if self.origin_to_centre > self.chassis_width:		
                self.origin_to_wheel = self.__calc_origin_to_wheel__();
                self.radius_right = self.origin_to_wheel;
                self.front_right_angle = self.__calc_steering_angle__();
            
            elif self.origin_to_centre < -self.chassis_width:
                self.alpha = math.pi - math.acos(self.chassis_width/self.centre_to_wheel);
                self.origin_to_wheel = self.__calc_origin_to_wheel__();
                self.radius_right = self.origin_to_wheel;
                self.front_right_angle = -self.__calc_steering_angle__();			
			
            else:
                print("Error, OC is not in range!");
            self.back_right_angle = -self.front_right_angle;

	#Calculate the angles for the left side of the rover.
    def __left__(self):		
        self.__calc_Ks__();
        self.__calc_origin_to_centre__();		
        if(self.rotate_about):
            self.front_left_angle = math.asin(1/math.sqrt(2));
            self.back_left_angle = -self.front_left_angle;

        else:
            if self.origin_to_centre < -self.chassis_width:
                self.alpha = math.acos(self.chassis_width/self.centre_to_wheel);						
                self.origin_to_wheel = self.__calc_origin_to_wheel__();
                self.radius_left = self.origin_to_wheel;
                self.front_left_angle = -self.__calc_steering_angle__();

            elif self.origin_to_centre > self.chassis_width:
                self.alpha = math.pi - math.acos(self.chassis_width/self.centre_to_wheel);
                self.origin_to_wheel = self.__calc_origin_to_wheel__();
                self.radius_left = self.origin_to_wheel;
                self.front_left_angle = self.__calc_steering_angle__();
            else:
                print("Error, OC is not in range!");
            self.back_left_angle = -self.front_left_angle;


	#clockwise turn for turn_about = True.
    def clockwise_turn_about(self):
        self.velocity_left = self.velocity;
        self.velocity_right = -self.velocity;
	#counterclockwise turn for turn_about = True.
    def counterclockwise_turn_about(self):
        self.velocity_left = -self.velocity;
        self.velocity_right = self.velocity;

    def calc_velocity(self):
        if self.rotate_about: #Case where OC is at zero 
            if self.velocity > 0:				
                self.clockwise_turn_about();				
            if self.velocity < 0:				
                self.counterclockwise_turn_about();		
        else:
            self.velocity_left = radius_ratio_formula(self.velocity, self.radius_left, self.radius_right);
            self.velocity_right = radius_ratio_formula(self.velocity, self.radius_right, self.radius_left);	

	#Calculating function that calls all required functions to get angles for each wheel
    def calc_all(self, RC_input, velocity, rotate_about):
        self.RC_input = RC_input;
        self.velocity = velocity;
        self.rotate_about = rotate_about;
        
        self.__right__();
        self.__left__();
        self.calc_velocity();
		
        print "Original velocity value: %s" % self.velocity;		
        print "Left wheel velocity value: %s" % self.velocity_left;
        print "Left wheel velocity value: %s" % self.velocity_right;

        print "";
        
        self.front_right_angle = convert_radians_to_degrees(self.front_right_angle);        
        self.back_right_angle = convert_radians_to_degrees(self.back_right_angle);        
        self.front_left_angle = convert_radians_to_degrees(self.front_left_angle);        
        self.back_left_angle = convert_radians_to_degrees(self.back_left_angle);        

        print "Front right wheel angle: %s" % (self.front_right_angle);	
        print "Back right wheel angle: %s" % (self.back_right_angle);
        print "Front left wheel angle: %s" % (self.front_left_angle);
        print "Back left wheel angle: %s" % (self.back_left_angle);

				

