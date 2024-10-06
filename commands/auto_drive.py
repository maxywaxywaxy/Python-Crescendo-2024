from subsystems.drive import Drive
from subsystems.networking import NetworkReciever

#import ntcore
from math import sin

class autoDrive:
    def __init__(self, _drive : Drive, _networking : NetworkReciever):
        self.drive = _drive
        self.networking = _networking
    
    def go_to_note(self):
        # run every time
        self.notedata = self.networking.get_note_data
        # notedata[0,1,2]
        #         [x,y,d]
        # camera window size: x= -320,320    y= -240,240

        # change values to fit [-1,1] range
        #kevin: range and division don't match
        note_distance = self.notedata[0] / 320  # from robot positon
        note_offset = self.notedata[1] / 240    # from robot direction


        # self.shooter.shooter_spin(1)
        # tank_drive(self, left_joystick, right_joystick)
        # mecanum_drive_robot_oriented(self, joystick_x, joystick_y, joystick_turning)

        # if needed: https://www.desmos.com/calculator/cbyz2i6qkd

        # what does this do?
        error_margin = 0.1
        low_power = 0.15

        if (note_offset >= -error_margin and note_offset < 0):
            turning_speed = -low_power

        elif (note_offset >= 0 and note_offset <= error_margin):
            turning_speed = low_power

        else:
            turning_speed = note_offset * 0.5

        drive_speed = 0

        self.drive.mecanum_drive_robot_oriented(0, -drive_speed, turning_speed)