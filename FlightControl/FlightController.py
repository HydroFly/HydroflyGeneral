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
nozzle_diam = 0.006 # m (6 mm)
nozzle_area = pi * ((nozzle_diam/2) ** 2)
mass_water = 4.2 # kg
mass_dry = 8.1 # kg
global m_dot_max
m_dot_max = 997 *nozzle_area * ue # mass flow rate, kg/s
print("INIT: mass flow rate: ", m_dot_max)
delta_t =0.25
actuation_delay = 0.1 #Thomas's 100ms delay :3

print("\t *** Select Mode: ***")
print("\t 0_ Sensor Data")
print("\t 1_ TestMode: simulation.")
print("\t 2_ TestMOde: Static Thrust Test \n")

TEST_MODE = int(input("Enter Mode Number: "))
#TEST_MODE=0

class HydroflyVehicle:
    def __init__(self,openedFile): #load a spec sheet instead? #also create with pointer to state?
        datafile = openedFile #sets up the file to add data. Handler
        self.flight_mode = 0
        self.TargetHeight = 0
        self.TargetVelocity =0
        self.Conditions = [0,0,0,0] #redline conditions. height, orientation, pressure, offboard switch

        self.Height_PID = PIDController(1,0,1, delta_t) #making this smaller improves results
        self.Velocity_PID = PIDController(1,1,1, delta_t)

        self.RedlinePressure = 1000 # psi
        self.RedlineHeight = 3.0 # meters
        self.RedlineOrientation = [5.0, 5.0, 5.0] # degrees, + or -

        self.previousTime = time.time() #last time command was sent
        self.hover_endtime = 0

        self.dt = 0

        self.solenoid_change_time = 0 # last time solenoid valve condition changed
        self.solenoid_delay = .1 #(s) 100ms: time for solenoid to open or close

    def run(self, State):
        if State.time_start == 0:
            State.time_start = time.time() 
        self.dt = time.time() - self.previousTime
        height_cv = self.Height_PID.get_cv(self.TargetHeight, State.position[2])
        target_dv = 2 * (height_cv - State.velocity[2] * delta_t)/(delta_t ** 2)
        target_d_mass = State.mass_tot * exp((GRAVITY * delta_t / ue) - target_dv / ue)
        m_dot_target = (State.mass_tot - target_d_mass) / delta_t
        percent_open = (m_dot_target / m_dot_max) #imagine a segment being extrapolated on a curve

        if percent_open < 0:
            percent_open = 0
        elif percent_open >1:
            percent_open = 1
        
            
        if TEST_MODE == 2:
            percent_open = 1

        self.time_openUntil = time.time() + delta_t*percent_open
        print("RN: percent_open", percent_open, "time_openUntil", self.time_openUntil)

    def control(self, State):
        if State.terminator[0] == 0:
            if (State.solenoid_state == False and (time.time() <= self.time_openUntil)):
                State.solenoid_state = True
                time.sleep(actuation_delay)
                print("CN: Turned LED ON! ")
            elif (time.time() < self.time_openUntil):
                print("CN: Led ON Already. Keep ON!")
            elif (time.time() >= self.time_openUntil and State.solenoid_state==True):
                State.solenoid_state = False
                time.sleep(actuation_delay) #actuation delay
                print("CN: Turning LED OFF")
            elif (time.time() >= self.time_openUntil and State.solenoid_state==False):
                print("CN: LED OFF Already. Keep OFF!")
                pass
                #self.run(State) < could i call run again then control smartly?
            else:
                print("CN: Weird Condition")
        else:
            print("CN: terminator = 1", State.solenoid_state)


    def abort(self, State): #later, let mode controller set terminator function. This will only close valve and prepare shutdown
        print("Aborting! Setting terminator[0] = 1 and locking solenoid valve")
        State.terminator[0] = 1
        State.solenoid_state = False
        #mode_controller(4)

    def mode_controller(self, changed_mode): 
        self.flight_mode = changed_mode
        if self.flight_mode == 0:
            print("Still in calibration phase.")
        elif self.flight_mode == 1: # ascent 
            self.TargetHeight = 2.0 #aim for height of 2 meters for testing
            #PIDs already initialized in constructor
            pass
        elif self.flight_mode == 2: # hover
            self.hover_endtime = time.time() + 3
            pass
        elif self.flight_mode == 3: # descent
            self.TargetHeight = 0.0
            #self.TargetVelocity = -0.5 #-.5m/s
            #clean_PID()
            pass
        elif self.flight_mode == 4:
            
            pass
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
        derivative = (error - self.prevError)/self.dt
        self.prevError = error
        return self.KP * error + self.KI * self.integral + self.KD *derivative


    def clean(self):
        self.prevError = 0
        self.integral = 0
        self.times_cleaned += 1
        

class HydroflyState:
    'Class to Hold and Calc. State Variables'
    def __init__(self, serialPort, adc, gain, datafile):
        self.datafile = datafile
        self.theTime_prev = time.time()
        self.position_prev = [0,0,0]
        self.flight_mode_prev = 0

        self.theTime = time.time()
        self.time_start = 0
        self.time_no_water = 0
        self.time_no_altitude = 0

        self.flight_mode = 0
        self.pressure = [0,0,0]
        self.terminator = [0,0]

        self.velocity = [0,0,0]
        self.position = [0,0,0]
        self.orientation = [0,0,0]
        self.height_corr = self.us_initialization(serialPort)
        self.press_corr = self.press_initialization(adc, gain)

        self.solenoid_state = False # solenoid status
        self.solenoid_time = self.theTime_prev # last time solenoid was opened or closed 
        self.mass_tot = mass_dry + mass_water

        self.position_model = [0,0,0]
        self.velocity_model = [0,0,0]
        self.mass_water_model = mass_water
        self.pressure_model = [0,0,0] #calculated later by pv=nrt


    def us_initialization(self, serialPort):
        print("Running Ultrasonic Sensor ",10, " times.")
        height_corr =0
        for x in range(0, 9):
            height_corr += maxSonarTTY.measure(serialPort)
        height_corr/=10
        print("Height At Initilization: ", height_corr)
        return height_corr

    def press_initialization(self, adc, gain):
        print("Running Pressure Sensor ",10, " times.")
        press_corr = [0,0,0]
        for x in range(0, 9):
            press_corr[0] += utils.volt_to_pressure(utils.val_to_volt(adc.read_adc(0, gain), gain))
            press_corr[1] += utils.volt_to_pressure(utils.val_to_volt(adc.read_adc(0, gain), gain))
            press_corr[2] += utils.volt_to_pressure(utils.val_to_volt(adc.read_adc(0, gain), gain))

        press_corr[0] = 14.47 - (press_corr[0]/10)
        press_corr[1] = 14.47 - (press_corr[1]/10)
        press_corr[2] = 14.47 - (press_corr[2]/10)
        return press_corr



    def update_state(self, adc, gain, serialPort, TheVehicle):
        while (self.terminator[0]==0 or self.terminator[1]==0):
            self.theTime= time.time()
            dt = time.time() - self.theTime_prev
            self.flight_mode = TheVehicle.flight_mode

            self.pressure[0] = utils.volt_to_pressure(utils.val_to_volt(adc.read_adc(0, gain), gain)) + self.press_corr[0]
            self.pressure[1] = utils.volt_to_pressure(utils.val_to_volt(adc.read_adc(1, gain), gain)) + self.press_corr[1] 
            self.pressure[2] = utils.volt_to_pressure(utils.val_to_volt(adc.read_adc(2, gain), gain)) + self.press_corr[2]

            self.mass_water_model -= m_dot_max * dt * self.solenoid_state
            mass_tot_new = self.mass_tot - m_dot_max * dt * self.solenoid_state

            if (self.mass_water_model <=0):
                print("US: Out of Water")
                self.terminator[0] = 1
                mass_tot_new = mass_dry
                if self.time_no_water == 0: #if this is the first time through this loop, mark the time
                    self.time_no_water = time.time()


            if (TEST_MODE != 1): #==0
                self.position[0] = 0.0
                self.position[1] = 0.0
                self.position[2] = 0.0254*(maxSonarTTY.measure(serialPort) - self.height_corr)
                self.velocity[0] = 0.0
                self.velocity[1] = 0.0
                self.velocity[2] = ((self.position[2] - self.position_prev[2])/dt)
            elif (TEST_MODE == 1):
                dv = GRAVITY * dt + (ue * log(self.mass_tot / mass_tot_new))
                self.velocity_model[2] += dv
                self.position_model[2] += self.velocity_model[2]*dt

                if (self.position_model[2] <= 0):
                    self.velocity_model[2] = 0 #review algorithm order of this assignment
                    self.position_model[2] = 0
                    if self.time_no_altitude == 0 and self.mass_water_model <= 0: #if first time through loop, mark the time
                        self.time_no_altitude = time.time()
                        print("Time H20 runs out: ", (self.time_no_water - self.time_start))
                        print("Time hits ground: ", (self.time_no_altitude - self.time_start))
                        TheVehicle.abort(self)
                self.velocity[2] = self.velocity_model[2]
                self.position[2] = self.position_model[2]

            self.mass_tot = mass_tot_new
            self.position_prev = self.position
            self.theTime_prev = self.theTime
            print("US: velocity: ", self.velocity[2],  "height ", self.position[2], "Water Remaining: ", self.mass_water_model)
            self.datafile.write(","+str(self.theTime)+","+str(dt)+","+str(self.position[0])+","+str(self.position[1])+","+str(self.position[2])+","+str(self.velocity[0])+","+str(self.velocity[1])+","+str(self.velocity[2])+","+str(self.pressure[0])+","+str(self.pressure[1])+","+str(self.pressure[2])+"\n")


    def check_state(self, TheVehicle): #check safety
        conditions = [True, True, True, True] #instantiate local variable
        while (sum(self.terminator)==0):
            print("CS: Checking State")
            conditions[0] = self.position[2] < TheVehicle.RedlineHeight
            conditions[1] = (self.orientation[0] < TheVehicle.RedlineOrientation[0]) and (self.orientation[1] < TheVehicle.RedlineOrientation[1]) and (self.orientation[2] < TheVehicle.RedlineOrientation[2]) 
            conditions[2] = True
            conditions[3] = True
            time.sleep(0.05) #changed from 0.1. Performance improved.
            if prod(conditions) !=1:
                if (conditions[0] == False):
                    print("CS: Height Exceeded Max Height of: ", TheVehicle.RedlineHeight) 
                elif (conditions[1] == False):
                    print("CS: Height Exceeded Max Orientation of: ", TheVehicle.RedlineOrientation) 
                TheVehicle.abort(self)

