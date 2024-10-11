from subsystems.networking import NetworkReciever
from wpilib import DigitalInput
class IRTest:
    def __init__(self, _newtorking: NetworkReciever):
        self.networking = _newtorking
        self.IR_Loading = DigitalInput(0)
        self.IR_Ready = DigitalInput(1)
    
    def test(self):
        if self.IR_Loading.get() == 0: 
            print ("loading broken")
        
        elif self.IR_Ready.get() == 0:
            print ("ready broken")

        else:
            print ("nothing broken")