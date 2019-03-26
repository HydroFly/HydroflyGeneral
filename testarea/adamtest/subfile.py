import numpy as np

def functioner():
    print("Called functioner")


class Vehicle:
    def __init(self):
        self.FlightMode = 0
        self.Conditions = [0, 0, 0, 0]
       
    def ArmCheck(self, IsArmed):
        self.Conditions = [IsArmed, IsArmed, IsArmed, IsArmed]
        return  np.prod(self.Conditions)


class State:
    def __init__(self):
        self.currentTime = 0
        self.running = 0 


