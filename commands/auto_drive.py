from subsystems.drive import Drive
from subsystems.networking import NetworkReciever

import ntcore

class autoDrive:
    def __init__(self, _drive : Drive, _networking : NetworkReciever):
        self.drive = _drive
        self.networking = _networking
        notedata = self.networking.get_note_data
        # notedata[0,1,2] 
    
    def auto_drive(self):
        # self.shooter.shooter_spin(1)
        # tank_drive(self, left_joystick, right_joystick):