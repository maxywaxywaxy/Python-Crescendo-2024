from subsystems.drive import Drive
from subsystems.networking import NetworkReciever

class AmpAlign:
    def __init__(self, _drive : Drive, _networking : NetworkReciever):
        self.drive = _drive
        self.networking = _networking

        #initializing stages
        self.IDLE = 0
        self.MOVING = 1
        self.FINISHED = 2
        #sets stage to idle to begin with
        self.stage = self.IDLE

    #main method that actually aligns the robot to the amp
    def amp_align(self):

        #if stage is idle, sets stage to moving
        if self.stage == self.IDLE:
            self.stage = self.MOVING

        #if stage is moving:
        elif self.stage == self.MOVING:
            #grabs apriltag data from networking
            apriltag_data = self.networking.get_apriltag_data()

            #fourth slot in the array checked for "TRUE"
            sees_apriltag = apriltag_data[4]

            #if no apriltag seen, ends method
            if not sees_apriltag:
                return
            
            #if there is an apriltag:
            #grabs apriltag ID
            apriltag_id = apriltag_data[3]

            #if not desired apriltag id (4 or 8), ends method
            if not (apriltag_x == 4 or apriltag_x == 8):
                return

            #grabs x variable of the apriltag
            apriltag_x = apriltag_data[0]

            #if close enough to the apriltag, we stop and go to finished
            if abs(apriltag_x) < 20:
                self.stage = self.FINISHED

            #if not close enough, we move
            else:
                if apriltag_x < -20:
                    self.drive.mecanum_drive_robot_oriented(-0.2, 0, 0)
                elif apriltag_x > 20:
                    self.drive.mecanum_drive_robot_oriented(0.2, 0, 0)

        #if finished, we stop moving.
        elif self.stage == self.FINISHED:
            self.drive.tank_drive(0, 0)

