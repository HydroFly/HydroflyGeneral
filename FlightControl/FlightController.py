import time
import maxSonarTTY
import Adafruit_ADS1x15 
import utilities as utils
from numpy import *
#import custom PID object


#Things to think about:
# - Flightmode value location
# - Flightmode triggering
# - Flight mission profile definition location
#review all these parameters. Put in better place maybe?
gravity = -9.81
rho_water = 997
pipe_height = 0.381
pressure = 800 #psi. CHANGE TO METRIC!
ue = sqrt(2 * (pressure / rho_water + gravity * pipe_height))
dt = 0.02 #change to dt from time()


class HydroflyVehicle:
    def __init__(self, State): #load a spec sheet instead?
        self.FlightMode = State.FlightMode


    def run(self, State):
        height_cv = height_PID.get_cv(target_height, State.position[2])
        target_d_mass = mass_tot * exp((gravity * dt / ue) - target_dv / ue)
        m_dot_target = (mass_tot - target_d_mass) / dt

        duty_cycle = (m_dot_target / m_dot_max)
        if duty_cycle < 0:
            duty_cycle = 0
        elif duty_cycle >1:
            duty_cycle = 1;

        #Duty_Cycle Adjustment to appropriate System Capability?
        
        return duty_cycle
    #where do we keep track of system mass change?


    def ModeController(self, mode): #might not be optimum.
        if State.FlightMode == 1:
            #do something
            pass
        elif State.FlightMode == 2:
            pass
            #do something else
        elif State.FlightMode == 3:
            #do landing thing
            pass
        elif State.FlightMode == 4:
            #be manually controlled
            pass
        else:
            #abort
            pass
                

class HydroflyState:
    'Class to Hold and Calc. State Variables'
    def __init__(self):
        self.FlightMode = 0
        self.pressure = [0,0,0]
        self.theTime = time.time()
        self.currentTime = 0
        self.terminator = 0
        self.velocity = [0,0,0]
        self.position = [0,0,0]
        self.orientation = [0,0,0]

    def updateState(self, PreviousState, flightmode, adc, gain, flag, serialPort):
        self.theTime = time.time()
        dt = PreviousState.theTime = self.theTime
        self.FlightMode = flightmode

        self.pressure[0] = utils.voltToPressure(utils.valToVolt(adc.read_adc(0, gain), gain))
        self.pressure[1] = utils.voltToPressure(utils.valToVolt(adc.read_adc(1, gain), gain))
        self.pressure[2] = utils.voltToPressure(utils.valToVolt(adc.read_adc(2, gain), gain))
        self.terminator = flag
        self.orientation[0] = 0.0
        self.orientation[1] = 0.0
        self.orientation[2] = maxSonarTTY.measure(serialPort)
        self.velocity[0] = 0.0
        self.velocity[1] = 0.0
        self.velocity[2] = ((self.orientation[2] - PreviousState.orientation[2])/dt)

