# I don't know why the heck the PID caller is called get_cv. Might change to some standard name later. 
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

#Is a modecontroller function necessary? Best Practice?
#Where should abort function be?
#Define threading for when redline conditions occur. aka: prod(conditions)/= 1


#review values below cuz I couldn't be bothered to be accurate tonight
gravity = -9.81
rho_water = 997
pipe_height = 0.381
#pressure = 800 #psi. CHANGE TO METRIC!
pressure = 5500000 #pascals
ue = sqrt(2 * (pressure / rho_water + gravity * pipe_height))
print(ue)
dt = 0.02 #change to dt from time()
nozzle_area = 0.3
mass_water = 12
mass_dry = 6
tuning_time = 0.2 #can be estimated by how long each flight control cycle takes...? maybe?

m_dot_max = 997*nozzle_area * ue


class HydroflyVehicle:
    def __init__(self): #load a spec sheet instead?
        self.FlightMode = 0
        self.TargetHeight = 0
        self.Conditions = [0,0,0,0]

        self.Height_PID = PIDController(1,0,1,dt)
        self.Velocity_PID = PIDController(1,1,1, dt)

    def ArmCheck(self, HeightCheck, OrientationCheck, PressureCheck, SwitchCheck):
        self.conditions = [HeightCheck, OrientationCheck, PressureCheck, SwitchCheck]
        return self.conditions #returns 1 if all are good to go
            

    def run(self, State):

        height_cv = self.Height_PID.get_cv(self.TargetHeight, State.orientation[2])
        target_dv = 2 * (height_cv - State.velocity[2] * tuning_time)/(tuning_time ** 2)

        target_d_mass = State.mass_tot * exp((gravity * dt / ue) - target_dv / ue)
        m_dot_target = (State.mass_tot - target_d_mass) / dt

        duty_cycle = (m_dot_target / m_dot_max)
        if duty_cycle < 0:
            duty_cycle = 0
        elif duty_cycle >1:
            duty_cycle = 1;
        print("DutyCycle: ",duty_cycle, "target_dv: ",target_dv, "target_d_mass: ", target_d_mass)
        #print("DutyCycle: ",duty_cycle, "height_cv: ",height_cv , "TargetHeight", self.TargetHeight, "State.Position", State.orientation[2] )

        #Duty_Cycle Adjustment to appropriate System Capability?
        return duty_cycle
    
    def Abort(self):
        print("Aborting! And does nothing for now =)")

    def ModeController(self): #might not be optimum. Maybe make it an interrupt somehow?
        if self.FlightMode == 1:
            self.TargetHeight = 2 #aim for height of 2 inches for testing
            #PIDs already initialized in constructor

            #do something
            pass
        elif self.FlightMode == 2:
            pass
            #do something else
        elif self.FlightMode == 3:
            #do landing thing
            pass
        elif self.FlightMode == 4:
            #be manually controlled
            pass
        else:
            #abort
            pass


class PIDController:
    def __init__(self, kp, ki, kd, dt): #instead of dt, use last action time? then calc dt from current time? ****see below
        self.KP = kp
        self.KI = ki
        self.KD = kd
        self.prevError = 0
        self.integral = 0
        self.dt = dt
        self.times_cleaned = 0

    def get_cv(self, target, current): # ****feed that time value here
        error = target - current
        self.integral += error * self.dt
        derivative = (error - self.prevError) * self.dt
        self.prevError = error
        return self.KP * error + self.KI * self.integral + self.KD *derivative

    def clean(self):
        self.prevError = 0
        self.integral = 0
        self.times_cleaned += 1
        

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
        self.heightCorr = 0 #height correction #will change to array once we get 3 ultrasonic sensors
        self.mass_tot = mass_dry + mass_water

    def initialization(self, serialPort):
        print("Running Ultrasonic Sensor ",49, " times.")
        for x in range(0, 49):
            self.heightCorr += maxSonarTTY.measure(serialPort)
        self.heightCorr/=49
        print(self.heightCorr)
        print("Height Initialized")
        time.sleep(0.5)


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
        self.orientation[2] = maxSonarTTY.measure(serialPort) - self.heightCorr
        self.velocity[0] = 0.0
        self.velocity[1] = 0.0
        self.velocity[2] = ((self.orientation[2] - PreviousState.orientation[2])/dt)

