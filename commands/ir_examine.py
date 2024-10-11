from subsystems.networking import NetworkReciever
from wpilib import DigitalInput

class IR_TEST:
    def __init__ (self, _networking: NetworkReciever):
        self.networking = _networking
        self.IR_Loading = DigitalInput(0)
        self.IR_Ready = DigitalInput(1)

    def test(self):
        if self.IR_Loading.get() == 0:
            print("loading broken")

        elif self.IR_Ready.get() == 0:
            print("ready broken")

        else:
            print ("Nothing broken")