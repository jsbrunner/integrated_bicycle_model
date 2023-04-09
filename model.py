# -*- coding: utf-8 -*-


#%%
from mesa import Agent, Model
from mesa.time import SimultaneousActivation
from mesa.space import ContinuousSpace
from mesa.datacollection import DataCollector
import random
#%%
# The model.py file specifies the scenario-related parameters and design the agent and model class.

# "..." means more things to be added here


# In the current setup, all bicycles move at 4 m/s and keep a 0.5 m lateral distance with the edge.
# The bike lane is 300 m long and 2 m wide.
# Normally, the entire object (bicycle and cyclist) always stay within the 2-m-wide lane space.
# In highly congested situation or queueing situation, cyclists may utilize more lateral space to overtake or stop.
# Therefore, on both sides of the bike lane, there are 0.5-m-wide extra lateral spaces which may be occupied by the bicycle handlebar area.

#%%

''' INPUTS TO THE MODEL ''' # write them as inputs to the model later

# Scenario-related  parameters (inputs may be changed when calling the library's functions)
random.seed(4) # Random seed for the scenario, note that for initial testings, it is better to use the same random seed so that the results are the same
# print(random.gauss(4, 2))
Demand = [750,100] # Inflow demand (bicycle/h), each value represents the demand of half an hour (Hence, right now this is a one hour scenario with 150 bicycles in each half-an-hour.)

v0_mean = 4 # m/s mean for distribution of desired speed
v0_sd = 1 # m/s standard deviation of desired speed
v_lat_max = 0.5 # m/s maximum lateral speed

p_mean = 1 # m distance from edge
p_sd = 0.2 # m st. deviation for distribution of p

a_des = 1.4 # feasible relaxation time for acceleration
b_max = 1 # m/s**2 maximum braking force 
''' Relaxation time for braking -> look what the NDM needs '''

dt = 1 # simulation time step length in seconds 


# In this case, we first assume bicycles are generated with a same interval (uniformly distributed) according to the demand.
# Automatically generated scenario-related  parameters
Interval = [int(3600 / Demand[i]) for i in range(len(Demand))] # Time interval in each half-an-hour
# print(Interval)
Inflow_time = [] # time points that bicycles enter the bike lane
for i in range(len(Demand)):
    Inflow_time.extend(list(range(0 + 1800 * i, 1800 * (i+1), Interval[i])))
''' Stochasticity desired for the inflow (not equal interval) '''

''' make a variable for the time step length, in case we want to change it later '''

#%%
# Agent class
class Bicycle(Agent):
    
    ''' INITIALIZATION AND VARIABLES '''
    
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)
        
        # Fixed attributes
        self.unique_id = unique_id
        self.length = 2 # bicycle length
        self.width = 0.8 # bicycle width
        self.v0 = random.gauss(v0_mean, v0_sd) # distribution of desired speed
        self.p = random.uniform(p_mean-p_sd, p_mean+p_sd) # distribution of desired lateral position
        self.a_des = a_des # fixed value for the desired/feasible acceleration
        self.b_max = b_max # fixed value for the maximum deceleration/braking
        self.omega_max = v_lat_max # fixed value for the maximum lateral speed
        self.omega_des = ... # fixed value for the desired lateral speed
        # further:
        self.zeta = ... # parameter for lateral stabilization with speed
        self.alpha = ... # scale length of safety region
        self.beta = ... # scale width of safety region
        self.gamma = 0.95 # passing threshold
        self.lambd = ... # coefficient for consideration range (caution with the var name)
        # OPTIONAL
        self.phi = ... # length of necessary 'spatial gain'
        self.eta = ... # coefficient for backward view
        self.psi = ... # coefficient for far border of anticipation range
        
        # Dynamic attributes (these following values initialize the simulation)
        self.pos = (0,self.p) # Current position, a "tuple" type position variable is required by Mesa and used in its other built-in function
        self.speed = self.v0 # Current (actual) longitudinal speed
        self.balance = 0 # additional width for stabilization (add value on both sides)
        self.sr_width = 0.2 # width of the safety region
        self.sr_length = 6 # length of the safety region
        self.cr_length = 50 # consideration range length
        self.ar_length = 0 # length of the anticipation range
        self.bw_length = 10 # length of the backwards view
        self.acc = 0 # actual longitudinal acceleration/braking for the current time step
        self.v_lat = 0 # actual lateral speed for the current time step
        self.next_coords = [0,0] # Attribute which stores the determined next coordinates
        self.cat1_cyclists = [] # list of significantly slower cyclists in consideration range
        self.cat2_cyclists = [] # list of slightly slower cyclists in consideration range
        self.des_lat_pos = 0 # desired lateral position
        self.trajectory = [] # list including the coordinates for the desired path (therefore also implicitly the moving angle)
        self.leader = 0
        self.v0_density = self.v0 # would account for the perception of density/congestion downstream
    
    
    ''' GET FUNCTIONS '''
    
    def getPos(self):
        return [self.pos[0],self.pos[1]]
    def getSpeed(self):
        return self.speed
    #...more functions may be necessary
    
    
    ''' AUXILIARY FUNCTIONS '''
    
    # Find neighbors or any individual agent which influence its own behavior (haven't been tested)
    ''' Functions by Ying-Chuan
    def findLeaders(self):
        neighbors = self.model.space.get_neighbors(self.pos,self.look_ahead_dist,False)
        leaders = [l for l in neighbors if l.getPos()[0] > self.pos[0]]
        return leaders '''

    '''
    def findFollowers(self):
        neighbors = self.model.space.get_neighbors(self.pos,self.look_back_dist,False)
        followers = [l for l in neighbors if l.getPos()[0] < self.pos[0]]
        return followers
    '''
    
    def findCat1(self):
        neighbors = self.model.space.get_neighbors(self.pos,self.cr_length+10,False) # add 10 metres so that the circular radius does not matter anymore
        leaders = [l for l in neighbors if l.getPos()[0] > self.pos[0] and l.getPos()[0] < (self.pos[0]+self.cr_length)] # obtain cyclists in consideration range
        self.cat1_cyclists = [l for l in leaders if l.getSpeed() <= (self.gamma*self.v0)] # obtain cat1 cyclists
    
    def findCat2(self):
        ...
    
    '''
    def findClosestLeader(self):
        ...
        # 1. get_neighbors as in functions before (but with longer distance than fov)
        # 2. reduce list to the neighbors in front (and within fov)
        # 3. loop over list and find the closest agent
        # 4. return closest agent
        # leaders = self.findLeaders()
        # get closest leader
        # print(leaders)
    '''    
    
    
    ''' UPDATE FUNCTIONS '''
    
    # Calculate and update the attributes in the next step
    # Determine and update the next coordinates
    def calPos(self): # some more parameters to be added
        self.next_coords[0] = self.pos[0] + self.speed * dt # to be modified
        self.next_coords[1] = self.des_lat_pos # self.p # self.pos[1] # to be modified
    def calSpeed(self): # some more parameters to be added
        self.speed = self.v0 # to be modified
    
    
    ''' LEVEL FUNCTIONS '''
    
    # LEVEL 1: Desired lateral position
    def findLatPos(self): 
        # find cat1 cyclists in consideration range
        self.findCat1()
        # print(self.cat1_cyclists)
        if len(self.cat1_cyclists)==0:
            self.des_lat_pos = self.p
            # print("no cat. 1 leader")
        else:
            # self.des_lat_pos = 2
            # print(self.des_lat_pos)
            # obtain envelope
            envelope = [] # envelope is a list with coordinate points
            blocked_space = [] # list the touples with lateral positions of cat1 cyclists
            # obtain the 
            # 
            for i in self.cat1_cyclists:
                # blocked_space.append([i, i.getPos()[1]-0.4, i.getPos()[1]+0.4]) # change 0.4 to the actual width including stabilization
                blocked_space.append([i, i.getPos()[1]+0.4]) # left position 
            
            most_left = 0
            for j in blocked_space:
                if j[1] > most_left:
                    most_left = j[1]
            
            self.des_lat_pos = (most_left + 3)/2
            ''' Condition if there is enough space between the most left cyclist and the path edge '''
            
            # sort blocked_space tuples from left to right
            # print(blocked_space)
            # print(blocked_space.sort(key=lambda a: a[2]))
            # if all space is blocked:
                # look at where there is the most space
            
        
        # return/or determine 'des_lat_pos'
        
    # LEVEL 2: Moving angle and leader
    def findTraj(self):  
        ...
    
    # LEVEL 3: NDM
    def findAcc(self): 
        ...
    
    
    ''' STEP AND ADVANCE FUNCTIONS '''
    
    # Read surroundings and determine next coordinates after they all take actions (Note that the agent hasn't really moved when this function is called)
    def step(self):
        ''' CALL LEVEL FUNCTIONS '''
        self.findLatPos() # model level 1
        # self.findTraj() # moving angle and identify leader, level 2
        # self.findAcc() # level 3
        
        ''' CALL UPDATE FUNCTIONS '''
        self.calPos()
        self.calSpeed()
        # update regions:
        # calFOV
        # calSafetyRegion
        # calSize
        
    
    # Take (physical) actions, this function would be called automatically after the step() function
    def advance(self):
        self.model.space.move_agent(self,self.next_coords) # update on the canvas
        self.pos = (self.next_coords[0],self.next_coords[1]) # update self attributes
        # print("Bicycle ",self.unique_id,"move to x = ",self.pos[0]," y = ",self.pos[1] - 0.5)
        # clear bicycles which finish the trip
        if self.pos[0] >= 300:
            self.model.to_be_removed.append(self)

#%%
# Model class
class BikeLane(Model):
    def __init__(self):
        super().__init__()
        self.schedule = SimultaneousActivation(self)
        
        # Create the canvas, which is a 300-m-long bike lane, 2 m wide with 0.5 m extra lateral spaces on both sides
        self.space = ContinuousSpace(300.1, 3, torus=True) # Changed the torus=False here: otherwise, there will be an error because agents are 'out of bounds'
        
        # Initialize model variables
        self.time_step = 0
        self.inflow_count = 1 # The number of bicycle in the vertical queue that will enter
        self.n_agents = 0  # Current number of agents (bicycles) on the entire bike lane
        self.initial_coords = (0,1) # bicycles enter 0.5 m from the path edge
        self.to_be_removed = [] # A list storing bicycles which finish the trip at the time step and to be removed
        
        # Data collection functions, collect positions of every bicycle at every step, namely trajectories
        self.datacollector = DataCollector(agent_reporters={"Position": "pos", "Speed": "speed"})
    
    def deduct(self):
        self.n_agents = self.n_agents - 1
    
    def step(self):
        # Execute agents' functions, including both step and advance
        self.schedule.step()
        # Remove out of bound agents
        for b in self.to_be_removed:
            #print("Remove Bicycle ",b.unique_id)
            self.schedule.remove(b)
            self.space.remove_agent(b)
        self.deduct() # reduce n_agents by 1
        self.to_be_removed = []
        # Add bicycle agents at certain time steps
        if self.inflow_count < len(Inflow_time):
            if self.time_step == Inflow_time[self.inflow_count]:
                b = Bicycle(self.inflow_count, self)
                self.schedule.add(b)
                self.space.place_agent(b, self.initial_coords)
                self.inflow_count += 1
                self.n_agents += 1
        # Update the time
        self.time_step += 1
        # Execute data collector
        self.datacollector.collect(self)
    

#%%




