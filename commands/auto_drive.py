from subsystems.drive import Drive
from subsystems.networking import NetworkReciever

import ntcore
from math import sin

class autoDrive:
    def __init__(self, _drive : Drive, _networking : NetworkReciever):
        self.drive = _drive
        self.networking = _networking
    
    def auto_drive(self):
        # run every time
        self.notedata = self.networking.get_note_data
        # notedata[0,1,2]
        #         [x,y,d]
        # camera window size: x= -320,320    y= -240,240

        # change values to fit [-1,1] range
        note_distance = self.notedata[0] / 640  # from robot positon
        note_offset = self.notedata[1] / 480    # from robot direction


        # self.shooter.shooter_spin(1)
        # tank_drive(self, left_joystick, right_joystick)
        # mecanum_drive_robot_oriented(self, joystick_x, joystick_y, joystick_turning)

        # if needed: https://www.desmos.com/calculator/cbyz2i6qkd

        error_margin = 0.1

        if (note_offset >= -error_margin):

            if (note_offset < 0):
                turning_speed = -error_margin

            elif (note_offset <= error_margin):
                turning_speed = error_margin

            else:
                turning_speed = sin(note_offset) * 1/2

        else:
            turning_speed = sin(note_offset) * 1/2

            
        self.drive.mecanum_drive_robot_oriented(self, 1, 0, turning_speed)