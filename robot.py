# import necessary libraries for interfacing with joysticks, driverstations, and hardware from different manufacturers
import wpilib
import phoenix5
import rev

import math

# import subsystems
from subsystems.drive import Drive
from subsystems.shooter import Shooter
from subsystems.networking import NetworkReciever
from subsystems.intake import Intake
from subsystems.arm import Arm
from subsystems.imu import IMU
from subsystems.climb import Climb
from commands.auto_drive import autoDrive

# import commands
from commands.amp_align import AmpAlign
from commands.descend import Descend
from commands.auto_intake import AutoIntake

# import autonomous code
from commands.autonomous import Autonomous

# import our constants which serve as "settings" for our robot/code, mainly IDs for CAN devices - motors, IMUs, and controllers
from utils import constants

# create our base robot class
class MyRobot(wpilib.TimedRobot):
    # initialize motors and sensors - create references to physical parts of our robot
    def robotInit(self):
            # create reference to our intake motor
        #self.intake_motor = rev.CANSparkMax(constants.INTAKE_MOTOR_ID, rev.CANSparkLowLevel.MotorType.kBrushless)
        print("Robot init 1")
        self.intake_top_motor = rev.CANSparkMax(constants.INTAKE_TOP_MOTOR_ID, rev.CANSparkLowLevel.MotorType.kBrushless)
        self.intake_bot_motor = rev.CANSparkMax(constants.INTAKE_BOT_MOTOR_ID, rev.CANSparkLowLevel.MotorType.kBrushless)
        print("Robot init 2")
        # create reference to our Falcon 500 motors for driving
        self.front_right = phoenix5._ctre.WPI_TalonFX(constants.FRONT_RIGHT_ID)
        self.front_left = phoenix5._ctre.WPI_TalonFX(constants.FRONT_LEFT_ID)
        self.back_left = phoenix5._ctre.WPI_TalonFX(constants.BACK_LEFT_ID)
        self.back_right = phoenix5._ctre.WPI_TalonFX(constants.BACK_RIGHT_ID)

        # invert the motors on the right side of our robot
        self.front_right.setInverted(True)
        self.back_right.setInverted(True)
        
        print("Robot init 3")
        #create a reference to our drivimg IMU
        self.drive_imu_motor_controller = phoenix5._ctre.WPI_TalonSRX(constants.IMU_ID)
        self.drive_imu = IMU(self.drive_imu_motor_controller)
        print("Robot init 4")

        #create reference to our Falcon motors
        self.shooter_upper_motor = phoenix5._ctre.WPI_TalonFX(constants.SHOOTER_UPPER_MOTOR_ID)
        self.shooter_lower_motor = phoenix5._ctre.WPI_TalonFX(constants.SHOOTER_LOWER_MOTOR_ID)

        self.shooter_upper_motor.setInverted(True)
        self.shooter_lower_motor.setInverted(True)

        #reference to the two arm motors that move it up and down
        self.arm_motor_left_front = rev.CANSparkMax(constants.ARM_LEFT_FRONT_ID, rev.CANSparkLowLevel.MotorType.kBrushless)
        self.arm_motor_left_back = rev.CANSparkMax(constants.ARM_LEFT_BACK_ID, rev.CANSparkLowLevel.MotorType.kBrushless)
        
        self.arm_motor_right_front = rev.CANSparkMax(constants.ARM_RIGHT_FRONT_ID, rev.CANSparkLowLevel.MotorType.kBrushless)
        self.arm_motor_right_back = rev.CANSparkMax(constants.ARM_RIGHT_BACK_ID, rev.CANSparkLowLevel.MotorType.kBrushless)

        # invert left side motors
        self.arm_motor_left_front.setInverted(True)
        self.arm_motor_left_back.setInverted(True)

        # reference to our arm IMU (also used for field-oriented)
        self.arm_imu_motor_controller = phoenix5._ctre.WPI_TalonSRX(constants.ARM_IMU_ID)
        self.arm_imu = IMU(self.arm_imu_motor_controller)

        # reference to climb motors
        self.climb_motor_left_front = phoenix5._ctre.WPI_TalonSRX(constants.CLIMB_MOTOR_LEFT_FRONT_ID)
        self.climb_motor_right_front = phoenix5._ctre.WPI_TalonSRX(constants.CLIMB_MOTOR_RIGHT_FRONT_ID)
        self.climb_motor_left_back = phoenix5._ctre.WPI_TalonSRX(constants.CLIMB_MOTOR_LEFT_BACK_ID)
        self.climb_motor_right_back = phoenix5._ctre.WPI_TalonSRX(constants.CLIMB_MOTOR_RIGHT_BACK_ID)
        
        # instances of our subsystems - passing in references to motors, sensors, etc.
        self.arm = Arm(self.arm_motor_left_front, self.arm_motor_left_back, self.arm_motor_right_front, self.arm_motor_right_back, self.arm_imu)
        self.intake = Intake(self.intake_top_motor, self.intake_bot_motor)
        self.drive = Drive(self.front_right, self.front_left, self.back_left, self.back_right, self.drive_imu)
        self.shooter = Shooter(self.shooter_lower_motor, self.shooter_upper_motor)
        self.climb = Climb(self.climb_motor_left_front, self.climb_motor_right_front, self.climb_motor_left_back, self.climb_motor_right_back)

        # instance of networking class to recieve information from raspberry pis
        self.networking = NetworkReciever()

        # create instance of our driver controller (flight stick) and operator controller (xbox controller)
        self.driver_controller = wpilib.Joystick(constants.DRIVER_CONTROLLER_ID)
        self.operator_controller = wpilib.XboxController(constants.OPERATOR_CONTROLLER_ID)

        #create instances of autonomous abilities for our robot
        self.amp_align = AmpAlign(self.drive, self.networking)
        self.descend = Descend(self.arm)
        self.auto_drive = autoDrive(self.drive, self.networking)
        self.auto_intake = AutoIntake(self.drive, self.descend, self.intake, self.networking, self.arm_imu)

        # switch to turn on or off drive
        self.enable_drive = True

        self.angle_two = 60

        #arm angle timer initiation
        self.arm_timer = 0

        #testing variable
        #self.up = False
    # setup before our robot transitions to autonomous
    def autonomousInit(self):
        # create instance of our autonomous code
        self.autonomous = Autonomous(self.drive, self.arm, self.shooter, self.intake)
        self.autonomous.stage = self.autonomous.IDLE

    # ran every 20 ms during autonomous mode
    def autonomousPeriodic(self):

        #calculating gravity compensation
        self.arm.gravity_comp = 0.14 * math.cos(self.arm.get_arm_pitch() * math.pi / 180)
        self.autonomous.two_note_auto()
         
    # setup before our robot transitions to teleop (where we control with a joystick or custom controller)
    def teleopInit(self):
        
        self.descend.descending = False
        self.descend.stage = self.descend.IDLE
        self.amp_align.stage = self.amp_align.IDLE
        self.arm.start_angle = self.arm_imu.get_pitch()

        if abs(self.arm.get_arm_pitch() - 80) < 10:
            self.arm.desired_position = 86
        
    # ran every 20 ms during teleop
    def teleopPeriodic(self):
        
        #calculating gravity compensation
        # 0.14 too light
        # 0.21 seems right, arm stays still
        self.arm.gravity_comp = 0.21 * math.cos(self.arm.get_arm_pitch() * math.pi / 180)

        #print(f"desired: {self.arm.desired_position}, current: {self.arm.get_arm_pitch()}"

        # get control buttons
        climb_down_button_pressed = self.operator_controller.getAButton() #hook moves down #THIS IS ACTUALLY BUTTON A # was B
        climb_up_button_pressed = self.operator_controller.getXButton() #hook moves up #THIS IS ACTUALLY BUTTON X # A
        shoot_button_pressed = self.operator_controller.getRightTriggerAxis() == 1
        amp_align_button_pressed = self.operator_controller.getYButton()
        amp_shoot_button_pressed = self.operator_controller.getBButton() #THIS IS ACTUALLY BUTTON B # X
        intake_button_pressed = self.operator_controller.getRightBumper()
        outtake_button_pressed = self.operator_controller.getLeftBumper()
        amp_blocking_position_button_pressed = self.operator_controller.getPOV() == 0
        # # inside_chassis_position_button_pressed = self.operator_controller.getPOV() == 90
        # auto_get_note = self.operator_controller.getPOV() == 90
        intake_position_button_pressed = self.operator_controller.getPOV() == 180
        shooting_position_button_pressed = self.operator_controller.getLeftTriggerAxis() == 1
        arm_up_button_pressed = self.operator_controller.getPOV() == 270
        sensor_intake_button_pressed = self.operator_controller.getPOV == 360

        #under_stage_button_pressed = self.driver_controller.getTriggerPressed()
        reset_drive_imu_button_pressed = self.driver_controller.getRawButton(11)
        
        # ---------- INTAKE ----------
        if intake_button_pressed:
            self.intake.intake_spin(0.5)
            #self.auto_intake.auto_intake_with_sensors()
        
        elif sensor_intake_button_pressed:
            self.auto_intake.auto_intake_with_sensors()
                         
        elif outtake_button_pressed:
            self.intake.intake_spin(-0.5)

        else:
            self.intake.stop()

        # ---------- CLIMB ----------
        if climb_up_button_pressed:
            self.climb.climb_spin(0.3)

        elif climb_down_button_pressed:
            self.climb.climb_spin(-0.3)

        else:
            self.climb.stop()
        
        # ---------- SHOOTER ----------
        if shoot_button_pressed:
            self.shooter.shooter_spin(1)

        else:
            if amp_shoot_button_pressed:
                #self.climb.climb_spin(-0.3)
                self.shooter.shooter_spin(0.025)


            else:
                self.shooter.stop()

        #------------ ARM -------------
        #desired positions: up, down, shooting angle
        if amp_blocking_position_button_pressed:
            # PREVIOUSLY 85
            self.arm.desired_position = 67
            # self.arm.arm_to_angle(self.arm.desired_position)
        
            # 0.25 = too strong
            # 0.14 = 
            # 0.125 = slightly too weak
        elif arm_up_button_pressed:
            self.arm.desired_position = 80
        # elif inside_chassis_position_button_pressed:  
        #     self.arm.desired_position = 40
        # v MOVED v
        # elif auto_get_note:
        #     auto_turning = self.auto_drive.go_to_note()
        elif shooting_position_button_pressed:
            self.arm.desired_position = 20
            
        elif intake_position_button_pressed:
            # self.arm.soft_drop()
            # self.arm.desired_position = 0
            self.arm.desired_position = max(0, self.arm.desired_position - 1) # should move about 50 degress/second
        """"
        if auto_get_note:
            auto_turning = self.auto_drive.go_to_note()
        else:
            auto_turning = 0
           """ 
       # else:
            #self.arm.desired_position = self.arm.get_arm_pitch()
        # if (not intake_position_button_pressed):
        if (self.arm.desired_position > 0) or (self.arm.get_arm_pitch()>5):
            self.arm.arm_to_angle(self.arm.desired_position)
            # self.arm.set_speed(self.arm.gravity_comp)
        else:
            self.arm.set_speed(0) # let the arm sit down without power

        self.arm_timer = self.arm_timer + 1
        if(self.arm_timer % 25 == 0):
            print("arm angle = ", self.arm.get_arm_pitch(), self.arm_imu.getYawPitchRoll()[1],"destination angle = ", self.arm.desired_position, " ", self.arm.arm_pid.integral, " ", self.arm.start_angle)
        #ARM TESTS
        """
        if arm_up_button_pressed:
            print (self.arm.get_arm_pitch())
            self.arm.arm_to_angle(80)
            #if (self.arm.get_arm_pitch() < 60):
                #print( "M") # self.arm.set_speed(0.14 * math.cos(self.arm.get_arm_pitch() * math.pi / 180))

        #elif intake_position_button_pressed:
           #print (self.arm.get_pitch())
           #self.arm.arm_to_angle()

        elif inside_chassis_position_button_pressed:
            print (self.arm.get_arm_pitch())
            self.arm.arm_to_angle(40)

        elif shooting_position_button_pressed:
            print(self.arm.get_arm_pitch())
            self.arm.arm_to_angle(20)

        # drop the arm softly to not damage it
        elif (intake_position_button_pressed):
            #print(self.arm.get_arm_pitch())
            #implementing soft drop (testing w/ gravity)
            #angle_one = self.angle_two
            #self.angle_two = self.arm.get_arm_pitch()
            #angle_diff = angle_one - self.angle_two
            self.arm.soft_drop()
            
            #if(angle_diff > 0 and angle_diff < 10):
                #self.arm.soft_drop(angle_diff)

        elif not amp_blocking_position_button_pressed:
            if (self.arm.get_arm_pitch() > 70):
                self.arm.set_speed(-0.05)
            else:
                self.arm.set_speed(0.06)
 
        """
        # check if drive is enabled
        if self.enable_drive:
            # get the x and y axis of the left joystick on our controller
            joystick_x = self.driver_controller.getX()

            # rember that y joystick is inverted
            # multiply by -1;
            # "up" on the joystick is -1 and "down" is 1
            joystick_y = self.driver_controller.getY() * -1

            # get the twist of our driver joystick
            joystick_turning = self.driver_controller.getZ()

            # run field oriented drive based on joystick values and note direction
           # self.drive.field_oriented_drive(joystick_x, joystick_y, joystick_turning + auto_turning)
            self.drive.field_oriented_drive(joystick_x, joystick_y, joystick_turning)
            
            # if we click button 11 on the flight stick, reset the IMU yaw
            if reset_drive_imu_button_pressed:
                self.drive_imu.reset_yaw()
        
        
# run our robot code
if __name__ == "__main__":
    wpilib.run(MyRobot)

# the command that deploys our code to our robot:
#py -3 -m robotpy deploy
