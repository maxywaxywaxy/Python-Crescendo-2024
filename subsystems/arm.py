# import our clamp function
from utils.math_functions import clamp, interpolation_array
from utils.pid import PID
# import math for cos functions
import math

class Arm:
    #initiating the arm values and stuff
    def __init__(self, _arm_motor_left_front, _arm_motor_left_back, _arm_motor_right_front, _arm_motor_right_back, _arm_imu):
        #references to the arm motors and the imu that were passed in.
        self.arm_motor_left_front = _arm_motor_left_front
        self.arm_motor_left_back = _arm_motor_left_back

        self.arm_motor_right_front = _arm_motor_right_front
        self.arm_motor_right_back = _arm_motor_right_back
        self.arm_imu = _arm_imu

        # proportional constant
        self.kp = 0.002

        # # init gravity compensation
        # self.gravity_compensation = 0

        # calculating gravity compensation
        self.gravity_comp = 0.14 * math.cos(self.get_arm_pitch() * math.pi / 180)

        #make previous error zero
        self.arm_val = 0

        # init desired position value
        self.desired_position = 87

        # init shooting override value
        self.shooting_override = False
        self.shooting_holding_value = 0
        # p=0.0027 too low starting point
        # p=0.0037 too low
        # p=0.0047

        self.arm_pid = PID(0.0047, 0.00002, 0.02048*5, 0)

    def set_speed(self, speed):
        self.arm_motor_left_front.set(speed)
        self.arm_motor_left_back.set(speed)

        self.arm_motor_right_front.set(speed)
        self.arm_motor_right_back.set(speed)

    #function to get the angle (pitch) of the arm
    def get_arm_pitch(self):
        #return self.arm_imu.get_pitch() + 87.2314
        return 87.2314 - self.arm_imu.get_pitch()
    
    # function to stop spinning the arm motors
    def stop(self):
        self.arm_motor_left_front.set(0)
        self.arm_motor_left_back.set(0)

        self.arm_motor_right_front.set(0)
        self.arm_motor_right_back.set(0)

    def kg_interpolation(self, value):
      arr = [ \
      [0, 0.17],\
      [34.2, 0.16],\
      [45.5, 0.14],\
      [90, 0.1]]


      return interpolation_array(value, arr)

    def k_down_interpolation(self, value):
        arr = [ \
            [0, 0.008],\
            [15, 0.008],\
            [22.5, 0.007],\
            [30, 0.005],\
            [60, 0.0033],\
            [90, 0.0013]]
        
        return interpolation_array(value, arr)
    
    def arm_to_angle(self, desired_angle):

        #if arm angle is out of bounds, return it.
        if desired_angle < -10 or desired_angle > 90:
            return

        # get our current arm angle
        current_angle = self.get_arm_pitch()

        # calculate the error that we pass into the PID controller
        error = desired_angle - current_angle

        # calculate proportional term
        #pid_value = self.arm_pid.keep_integral(error)
        k = self.kp

        #if the error is zero meaning that the arm has to move up
        if (error < 0):
            k = self.k_down_interpolation(current_angle)

        proportional = k * error
        pid = self.arm_pid.steer_pid(error)

        # calculate justified current arm angle in radians
        justifed_angle_radians = current_angle * math.pi / 180

        # #if esired angle isn't reached
        # if current_angle < desired_angle - 1 or current_angle > desired_angle + 1:
        #     #gravity taken into account through angle measure and interpolation array of the current angle
        #     self.gravity_compensation = math.cos(justifed_angle_radians) * self.kg_interpolation(current_angle)

        # calculate motor power
        motor_power = pid + self.gravity_comp


        # clamp our motor power so we don't move to fast
        motor_power_clamped = clamp(motor_power, -0.03, 0.25)

        # check if not shooting
        if not self.shooting_override:
            self.shooting_holding_value = motor_power_clamped


        #print(f"desired: {self.desired_position}, current: {current_angle}, pow: {motor_power_clamped}")
        #spin the motors based on calculated PID value or previously stored holding value
        if self.shooting_override:
            self.set_speed(self.shooting_holding_value)
        else:
            self.set_speed(motor_power_clamped)
    
    #not in use
    def arm_gravity_test(self, printout):
        # get the current angle from the IMU
        current_angle = self.get_arm_pitch()

         # calculate justified current arm angle in radians
        justifed_angle_radians = current_angle * math.pi / 180

        # # calculate gravity compensation
        # gravity_compensation = math.cos(justifed_angle_radians) * self.kg_interpolation(current_angle)

        # calculate motor power
        motor_power = clamp(self.gravity_comp, -0.2, 0.23)

        # spin motors
        self.set_speed(motor_power)

        #if printout:
           # print(f"angle: {self.get_arm_pitch()}, kg: {self.kg_interpolation(current_angle)}, motor power: {motor_power}")
    
    #make the arm fall slower when reaching pitch = 0
    
    #THIS IS NOT A SOFT DROP. DONUT USE. This is one of the only ways to slam the arm.
    def soft_drop(self):
        #if(25>self.get_arm_pitch()>2):

        #Approach one - use cosine
        """
       
            math_val = math.cos(self.get_arm_pitch() * (math.pi / 180))
            print(math_val / 5)
            if(25>self.get_arm_pitch() > 10):
                print(1/(math_val*5))
                self.set_speed(1/(math_val*6))
            elif (10>=self.get_arm_pitch() > 2):
                print(1/(math_val*10))
                self.set_speed(1/(math_val*10))
        """
    
        #approach two - use diff between angles (speed) to determine needed motor offset
        """
        if (25 > self.get_arm_pitch() > 10):
            self.set_speed(angle_diff * .05)
        elif (10 >= self.get_arm_pitch() > 3):
            self.set_speed(angle_diff * .1)
        """

        #approach three - set intervals and set speed depending on interval    
        # if (25 > self.get_arm_pitch > 5):
        #     self.set_speed(0.25)
        #     print("setting speed to .25")
        # elif (5 >= self.get_arm_pitch() > 2.5):
        #     self.set_speed(0.125)
        #     print("setting speed to .125")
        # else:
        #     self.set_speed(0)

        #approach four - some square root thingy
        """
        sqrt_val = sqrt(arm_pitch)
        #value will be between 5 and ~1.7
        multiply value by a scalar
            scalar should start at 1/25 when val is near 5
            should end at 1/17 when value is near 1.7
        """

        #approach five - use gravity compensation
        if (self.get_arm_pitch() > 2.5):
            self.arm_to_angle(2.5)
        else:
            self.set_speed(0)