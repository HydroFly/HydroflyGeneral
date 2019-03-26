# I don't know why the heck the PID caller is called get_cv. Might change to some standard name later. 
import time
import maxSonarTTY
import Adafruit_ADS1x15 
import utilities as utils
from numpy import *


### DEFINE PROJECT CONSTANTS ### 
GRAVITY = -9.81
RHO_WATER = 997
pipe_height = 0.381
pressure = 5515806 #pascals (800 psi)
ue = sqrt(2 * (pressure / RHO_WATER + GRAVITY * pipe_height))
#dt = 0.02 #change to dt from time()
nozzle_diam = 0.006 # m (6 mm)
nozzle_area = pi * ((nozzle_diam/2) ** 2)
mass_water = 4.2 # kg
mass_dry = 8.1 # kg
tuning_time = 0.2 #can be estimated by how long each flight control cycle takes...? maybe?
m_dot_max = 997 *nozzle_area * ue # mass flow rate, kg/s

delta_t =0.2

class HydroflyVehicle:
    def __init__(self): #load a spec sheet instead?
        self.flight_mode = 0
        self.TargetHeight = 0
        self.Conditions = [0,0,0,0]

        self.Height_PID = PIDController(1,0,1,delta_t)
        self.Velocity_PID = PIDController(1,1,1, delta_t)

        self.RedlinePressure = 1000 # psi
        self.RedlineHeight = 3.5 # meters
        self.RedlineOrientation = [5.0, 5.0, 5.0] # degrees, + or -

        self.previousTime = time.time() #last time command was sent

#    def arm_check(self, HeightCheck, OrientationCheck, PressureCheck, SwitchCheck):
#        self.conditions = [HeightCheck, OrientationCheck, PressureCheck, SwitchCheck]
#        return self.conditions #returns 1 if all are good to go
            

    def run(self, State):
        #instead of opening for fraction of dt, figure out 
        #how to define certain time? (1second?)
        #and figure out fraction of 1 second needed to for system to flow
        #water out. aka a percentage.
        #have something time this either with delay or something
        #since we're having this in main while loop, maybe create a timer thread
        #timer() is a thread already?? use that? 
        #figure this out somehow.
        #
        #flexible/variable future-dt would work too. math should be the same

        dt = time.time() - self.previousTime
        height_cv = self.Height_PID.get_cv(self.TargetHeight, State.position[2])
        target_dv = 2 * (height_cv - State.velocity[2] * tuning_time)/(tuning_time ** 2)

        target_d_mass = State.mass_tot * exp((GRAVITY * dt / ue) - target_dv / ue)
        m_dot_target = (State.mass_tot - target_d_mass) / dt
        duty_cycle = (m_dot_target / m_dot_max)
        if duty_cycle < 0:
            duty_cycle = 0
        elif duty_cycle >1:
            duty_cycle = 1;
        #print("DutyCycle: ",duty_cycle, "target_dv: ",target_dv, "target_d_mass: ", target_d_mass, "dt : ", dt)

        #Duty_Cycle Adjustment to appropriate System Capability?
        self.previousTime = time.time()
        return duty_cycle
    
    def abort(self, State):
        State.terminator = 1
        print("Aborting! And does nothing for now =)")

    def mode_controller(self,State): 
        if self.flight_mode == 0:
            print("Still in calibration phase.")
        elif self.flight_mode == 1: # ascent 

            self.TargetHeight = 2 #aim for height of 2 meters for testing
            #PIDs already initialized in constructor
            pass
        elif self.flight_mode == 2: # hover
            pass
        elif self.flight_mode == 3: # descent
            pass
        elif self.flight_mode == 4: # abort
            #print("About to call abort from mode controller")
            self.abort(State)
        else:
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

    #cv: corrective value
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
    def __init__(self, serialPort):
        self.theTime_prev = time.time()
        self.position_prev = [0,0,0]
        self.flight_mode_prev = 0

        self.theTime = time.time()
        self.flight_mode = 0
        self.pressure = [0,0,0]
        self.terminator = 0
        self.future_terminator = [0, 0, 0] #logging, stateupdate, solenoidcontrol
        self.velocity = [0,0,0]
        self.position = [0,0,0]
        self.orientation = [0,0,0]
        self.height_corr = self.initialization(serialPort)
        
        #height correction - will change to array once we get 3 ultrasonic sensors
        self.mass_tot = mass_dry + mass_water
        
    def initialization(self, serialPort):
        print("Running Ultrasonic Sensor ",50, " times.")
        height_corr =0
        for x in range(0, 49):
            height_corr += maxSonarTTY.measure(serialPort)
        height_corr/=50
        print("Height At Initilization: ", height_corr)
        return height_corr


    def update_state(self, flightmode, adc, gain, serialPort, datafile):
        while (self.terminator==0):
            self.theTime= time.time()
            dt = time.time() - self.theTime_prev
            self.flight_mode = flightmode

            self.pressure[0] = utils.volt_to_pressure(utils.val_to_volt(adc.read_adc(0, gain), gain))
            self.pressure[1] = utils.volt_to_pressure(utils.val_to_volt(adc.read_adc(1, gain), gain))
            self.pressure[2] = utils.volt_to_pressure(utils.val_to_volt(adc.read_adc(2, gain), gain))
            self.position[0] = 0.0
            self.position[1] = 0.0
            self.position[2] = 0.0254*(maxSonarTTY.measure(serialPort) - self.height_corr)
            self.velocity[0] = 0.0
            self.velocity[1] = 0.0
            self.velocity[2] = ((self.position[2] - self.position_prev[2])/dt)
            self.position_prev = self.position
            self.theTime_prev = self.theTime

            #self.logdata(datafile)

    def check_state(self, TheVehicle):
        conditions = [True, True, True, True] #instantiate local variable
        while (self.terminator==0):
            #print("Checking State")
            conditions[0] = self.position[2] < TheVehicle.RedlineHeight
            conditions[1] = (self.orientation[0] < TheVehicle.RedlineOrientation[0]) and (self.orientation[1] < TheVehicle.RedlineOrientation[1]) and (self.orientation[2] < TheVehicle.RedlineOrientation[2]) 
            conditions[2] = True
            conditions[3] = True
            #print(conditions, prod(conditions))
            time.sleep(0.1)
            if prod(conditions) !=1:
                if (conditions[0] == False):
                    print("Height Exceeded Max Height of: ", TheVehicle.RedlineHeight) 
                TheVehicle.flight_mode = 4
                TheVehicle.mode_controller(self)
        
    def log_data(self,datafile):
        print("About to log, baby!")
        datafile.write(str(self.pressure[0]) + "," + str(self.pressure[1]) + "," + str(self.position[2])+ "\n")

